/* 온라인 보정과 모델 입력 변환을 openpilot 참조식과 대조한다: OnlineCalibrator ↔ calibrationd.py,
 * calibration_service의 저장·복원·수동 보정, app_config 환경 변수, 투영 행렬과 YUV6 워프 ↔
 * openpilot OpenCL 워프. */
#include "app_config.h"
#include "calibration_service.h"
#include "utils_math.h"
#include "model_input_transform.h"
#include "calibration_online.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr int kModelW = 512;
constexpr int kModelH = 256;
constexpr int kHalfW = kModelW / 2;
constexpr int kHalfH = kModelH / 2;
constexpr int kPlaneSize = kHalfW * kHalfH;
constexpr int kYuv6Floats = 6 * kPlaneSize;

constexpr double kPi = 3.14159265358979323846264338327950288;
// openpilot calibrationd.py 상수
constexpr double kMinSpeedFilter = 15.0 * 0.44704;
constexpr double kMaxVelAngleStd = 0.25 * kPi / 180.0;
constexpr double kMaxYawRateFilter = 2.0 * kPi / 180.0;
constexpr double kMaxAllowedSpread = 2.0 * kPi / 180.0;
constexpr double kPitchMin = -0.09074112085129739;
constexpr double kPitchMax = 0.14907572052989657;
constexpr double kYawMin = -0.06912048084718224;
constexpr double kYawMax = 0.06912048084718235;
constexpr double kSanityMargin = 0.005;
constexpr int kBlockSize = 100;
constexpr int kInputsNeeded = 5;
constexpr int kInputsWanted = 50;

void matmul3d(const double *a, const double *b, double *out)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k)
                sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

void matmul34d(const double *a3, const double *b34, double *out34)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k)
                sum += a3[r * 3 + k] * b34[k * 4 + c];
            out34[r * 4 + c] = sum;
        }
    }
}

// rot = Rz(yaw)·Ry(pitch)·Rx(roll), openpilot rot_from_euler과 같은 순서
void rot_from_euler_ref(const double rpy[3], double *rot)
{
    const double cr = std::cos(rpy[0]);
    const double sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]);
    const double sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]);
    const double sy = std::sin(rpy[2]);

    const double rx[9] = {
        1.0, 0.0, 0.0,
        0.0, cr, -sr,
        0.0, sr, cr,
    };
    const double ry[9] = {
        cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp,
    };
    const double rz[9] = {
        cy, -sy, 0.0,
        sy, cy, 0.0,
        0.0, 0.0, 1.0,
    };

    double tmp[9];
    matmul3d(ry, rx, tmp);
    matmul3d(rz, tmp, rot);
}

void euler_from_rot_ref(const double *rot, double rpy[3])
{
    rpy[0] = std::atan2(rot[2 * 3 + 1], rot[2 * 3 + 2]);
    rpy[1] = std::asin(std::max(-1.0, std::min(1.0, -rot[2 * 3 + 0])));
    rpy[2] = std::atan2(rot[1 * 3 + 0], rot[0 * 3 + 0]);
}

void compose_rpy_ref(const double base[3], const double observed[3], double out[3])
{
    double base_rot[9];
    double observed_rot[9];
    double composed[9];
    rot_from_euler_ref(base, base_rot);
    rot_from_euler_ref(observed, observed_rot);
    matmul3d(base_rot, observed_rot, composed);
    euler_from_rot_ref(composed, out);
}

bool finite3(const double v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

/* openpilot calibrationd.py의 Calibrator를 double로 옮긴 참조 구현. */
struct RefCalibrator {
    double rpys[kInputsWanted][3] = {};
    double rpy[3] = {};
    double spread[3] = {};
    double old_rpy[3] = {};
    double old_rpy_weight = 0.0;
    int idx = 0;
    int block_idx = 0;
    int valid_blocks = 0;
    CalibrationStatus status = CalibrationStatus::Uncalibrated;
    uint64_t accepted = 0;
    uint64_t rejected = 0;

    void reset(const double init[3], int blocks, const double *smooth_from = nullptr)
    {
        valid_blocks = std::max(0, std::min(blocks, kInputsWanted));
        for (int i = 0; i < 3; ++i)
            rpy[i] = std::isfinite(init[i]) ? init[i] : 0.0;
        for (int b = 0; b < kInputsWanted; ++b) {
            for (int i = 0; i < 3; ++i)
                rpys[b][i] = rpy[i];
        }
        idx = 0;
        block_idx = 0;
        if (smooth_from) {
            for (int i = 0; i < 3; ++i)
                old_rpy[i] = smooth_from[i];
            old_rpy_weight = 1.0;
        } else {
            old_rpy[0] = old_rpy[1] = old_rpy[2] = 0.0;
            old_rpy_weight = 0.0;
        }
        update_status();
    }

    void smooth_rpy(double out[3]) const
    {
        if (old_rpy_weight > 0.0) {
            for (int i = 0; i < 3; ++i)
                out[i] = old_rpy_weight * old_rpy[i] + (1.0 - old_rpy_weight) * rpy[i];
        } else {
            for (int i = 0; i < 3; ++i)
                out[i] = rpy[i];
        }
    }

    static bool is_valid(const double rpy_in[3])
    {
        return kPitchMin < rpy_in[1] && rpy_in[1] < kPitchMax &&
               kYawMin < rpy_in[2] && rpy_in[2] < kYawMax;
    }

    static void sanity_clip(double rpy_in[3])
    {
        if (!finite3(rpy_in)) {
            rpy_in[0] = 0.0;
            rpy_in[1] = 0.0;
            rpy_in[2] = 0.0;
            return;
        }
        rpy_in[1] = std::max(kPitchMin - kSanityMargin, std::min(kPitchMax + kSanityMargin, rpy_in[1]));
        rpy_in[2] = std::max(kYawMin - kSanityMargin, std::min(kYawMax + kSanityMargin, rpy_in[2]));
    }

    void update_status()
    {
        double sum[3] = {};
        double min_v[3] = {};
        double max_v[3] = {};
        int valid_count = 0;

        for (int b = 0; b < valid_blocks; ++b) {
            if (b == block_idx) continue;
            if (valid_count == 0) {
                for (int i = 0; i < 3; ++i)
                    min_v[i] = max_v[i] = rpys[b][i];
            }
            for (int i = 0; i < 3; ++i) {
                sum[i] += rpys[b][i];
                min_v[i] = std::min(min_v[i], rpys[b][i]);
                max_v[i] = std::max(max_v[i], rpys[b][i]);
            }
            ++valid_count;
        }

        if (valid_count > 0) {
            for (int i = 0; i < 3; ++i) {
                rpy[i] = sum[i] / valid_count;
                spread[i] = std::fabs(max_v[i] - min_v[i]);
            }
        } else {
            spread[0] = spread[1] = spread[2] = 0.0;
        }

        if (valid_blocks < kInputsNeeded) {
            status = CalibrationStatus::Uncalibrated;
        } else if (is_valid(rpy)) {
            status = CalibrationStatus::Calibrated;
        } else {
            status = CalibrationStatus::Invalid;
        }

        const double max_spread = std::max(spread[0], std::max(spread[1], spread[2]));
        if (status == CalibrationStatus::Calibrated && max_spread > kMaxAllowedSpread) {
            const int last_block = (block_idx + kInputsWanted - 1) % kInputsWanted;
            const double smooth_from[3] = {rpy[0], rpy[1], rpy[2]};
            reset(rpys[last_block], kInputsNeeded, smooth_from);
        }
    }

    bool update(const PoseObservation &pose, double v_ego = 20.0)
    {
        old_rpy_weight = std::max(0.0, old_rpy_weight - 1.0 / 400.0);

        const double trans[3] = {pose.trans[0], pose.trans[1], pose.trans[2]};
        const double rot[3] = {pose.rot[0], pose.rot[1], pose.rot[2]};
        const double trans_std[3] = {pose.trans_std[0], pose.trans_std[1], pose.trans_std[2]};
        const bool valid_numbers = finite3(trans) && finite3(rot) && finite3(trans_std) &&
            std::isfinite(v_ego);
        const bool straight_and_fast = valid_numbers &&
            v_ego > kMinSpeedFilter &&
            trans[0] > kMinSpeedFilter &&
            std::fabs(rot[2]) < kMaxYawRateFilter;
        const bool certain_if_calib = valid_numbers &&
            ((std::atan2(trans_std[1], trans[0]) < kMaxVelAngleStd) ||
             (valid_blocks < kInputsNeeded));

        if (!straight_and_fast || !certain_if_calib) {
            ++rejected;
            return false;
        }

        const double observed_rpy[3] = {
            0.0,
            -std::atan2(trans[2], trans[0]),
            std::atan2(trans[1], trans[0]),
        };
        double base[3];
        double new_rpy[3];
        smooth_rpy(base);
        compose_rpy_ref(base, observed_rpy, new_rpy);
        sanity_clip(new_rpy);

        if (!finite3(new_rpy)) {
            ++rejected;
            return false;
        }

        for (int i = 0; i < 3; ++i)
            rpys[block_idx][i] =
                (idx * rpys[block_idx][i] + (kBlockSize - idx) * new_rpy[i]) /
                static_cast<double>(kBlockSize);

        idx = (idx + 1) % kBlockSize;
        ++accepted;
        if (idx == 0) {
            ++block_idx;
            valid_blocks = std::min(kInputsWanted, std::max(block_idx, valid_blocks));
            block_idx %= kInputsWanted;
        }
        update_status();
        return true;
    }
};

PoseObservation make_pose(float tx = 20.0f, float ty = 0.2f, float tz = -0.4f,
                          float yaw_rate = 0.0f, float trans_std_y = 0.01f)
{
    PoseObservation pose{};
    pose.trans[0] = tx;
    pose.trans[1] = ty;
    pose.trans[2] = tz;
    pose.rot[0] = 0.0f;
    pose.rot[1] = 0.0f;
    pose.rot[2] = yaw_rate;
    pose.trans_std[0] = 0.01f;
    pose.trans_std[1] = trans_std_y;
    pose.trans_std[2] = 0.01f;
    pose.rot_std[0] = 0.01f;
    pose.rot_std[1] = 0.01f;
    pose.rot_std[2] = 0.01f;
    return pose;
}

void compare_snapshot(const OnlineCalibrator::Snapshot &actual, const RefCalibrator &expected,
                      const char *label)
{
    EXPECT_EQ(actual.valid_blocks, expected.valid_blocks) << std::string(label) + " valid_blocks";
    EXPECT_EQ(actual.block_sample_count, expected.idx)
        << std::string(label) + " block_sample_count";
    EXPECT_EQ(static_cast<int>(actual.status), static_cast<int>(expected.status))
        << std::string(label) + " status";
    EXPECT_NEAR(actual.accepted_samples, expected.accepted, 0.0)
        << std::string(label) + " accepted";
    EXPECT_NEAR(actual.rejected_samples, expected.rejected, 0.0)
        << std::string(label) + " rejected";
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual.rpy[i], expected.rpy[i], 1e-5) << std::string(label) + " rpy";
        EXPECT_NEAR(actual.spread[i], expected.spread[i], 1e-5) << std::string(label) + " spread";
    }
}

TEST(CalibrationEquivalence, OnlineCalibrator)
{
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;

    PoseObservation low_speed = make_pose(6.0f);
    EXPECT_FALSE(actual.update(low_speed, 20.0f).accepted) << "카메라 속도가 낮은 표본은 버린다";

    PoseObservation low_vehicle_speed = make_pose(20.0f, 0.0f, 0.0f, 0.01f, 0.0f);
    EXPECT_FALSE(actual.update(low_vehicle_speed, 2.0f).accepted)
        << "CAN vEgo가 낮은 표본은 버린다";
    EXPECT_FALSE(ref.update(low_speed)) << "참조식도 저속 표본을 버린다";
    EXPECT_FALSE(ref.update(low_vehicle_speed, 2.0)) << "참조식도 CAN vEgo가 낮은 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "low_speed_reject");

    PoseObservation high_yaw = make_pose();
    high_yaw.rot[2] = static_cast<float>(3.0 * kPi / 180.0);
    EXPECT_FALSE(actual.update(high_yaw, 20.0f).accepted) << "요레이트가 큰 표본은 버린다";
    EXPECT_FALSE(ref.update(high_yaw)) << "참조식도 요레이트가 큰 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "high_yaw_reject");

    PoseObservation nan_pose = make_pose();
    nan_pose.trans[0] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(actual.update(nan_pose, 20.0f).accepted) << "NaN 표본은 버린다";
    EXPECT_FALSE(ref.update(nan_pose)) << "참조식도 NaN 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "nan_reject");

    const PoseObservation accepted_pose = make_pose();
    for (int i = 0; i < kBlockSize; ++i) {
        EXPECT_TRUE(actual.update(accepted_pose, 20.0f).accepted) << "블록 표본을 받아들인다";
        EXPECT_TRUE(ref.update(accepted_pose)) << "참조식도 블록 표본을 받아들인다";
    }
    compare_snapshot(actual.snapshot(), ref, "one_block");
    EXPECT_EQ(actual.snapshot().valid_blocks, 1) << "유효 블록 1개";

    for (int i = 0; i < 4 * kBlockSize; ++i) {
        actual.update(accepted_pose, 20.0f);
        ref.update(accepted_pose);
    }
    compare_snapshot(actual.snapshot(), ref, "five_blocks");
    EXPECT_EQ(static_cast<int>(actual.snapshot().status),
              static_cast<int>(CalibrationStatus::Calibrated))
        << "블록 5개가 차면 보정 완료";

    PoseObservation uncertain = make_pose();
    uncertain.trans_std[1] = 1.0f;
    EXPECT_FALSE(actual.update(uncertain, 20.0f).accepted)
        << "보정 뒤에도 trans 표준편차가 큰 표본은 버린다";
    EXPECT_FALSE(ref.update(uncertain)) << "참조식도 보정 뒤 trans 표준편차가 큰 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "uncertain_after_calib");
}

ParsedModelOutput parsed_from_pose(const PoseObservation &pose)
{
    ParsedModelOutput output{};
    output.valid = true;
    output.has_pose = true;
    output.pose = pose;
    return output;
}

TEST(CalibrationEquivalence, CalibrationService)
{
    constexpr const char *kTestParamsDir = "params/work";
    constexpr const char *kTestCalibration = "params/work/calibration.json";
    std::remove(kTestCalibration);
    setenv("K230_PARAMS_DIR", kTestParamsDir, 1);

    OnlineCalibrator restored_calibrator;
    const float restored_rpy[3] = {0.0f, deg_to_rad(2.0f), deg_to_rad(-0.75f)};
    EXPECT_TRUE(restored_calibrator.restore(restored_rpy, 12))
        << "저장된 유효 보정값을 복원한다";
    float restored_output[3] = {};
    restored_calibrator.output_rpy(restored_output);
    EXPECT_NEAR(restored_output[1], restored_rpy[1], 1e-7)
        << "복원한 pitch로 보정기를 초기화한다";
    EXPECT_NEAR(restored_output[2], restored_rpy[2], 1e-7) << "복원한 yaw로 보정기를 초기화한다";
    EXPECT_EQ(restored_calibrator.snapshot().valid_blocks, 12) << "복원한 유효 블록 수";

    AppConfig auto_config;
    auto_config.calibration_auto = true;
    auto_config.manual_calibration = false;
    auto_config.log_calibration = false;
    CalibrationService service(auto_config);

    const ParsedModelOutput output = parsed_from_pose(make_pose());
    for (int i = 0; i < kBlockSize; ++i)
        service.update(output, 20.0f);

    float input_rpy[3] = {};
    service.input_rpy(input_rpy);
    const OnlineCalibrator::Snapshot snapshot = service.snapshot();
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(input_rpy[i], snapshot.rpy[i], 1e-7)
            << "자동 모드의 input_rpy는 온라인 스냅샷을 따른다";

    AppConfig manual_config;
    manual_config.calibration_auto = true;
    manual_config.manual_calibration = true;
    manual_config.manual_roll = deg_to_rad(0.4f);
    manual_config.manual_pitch = deg_to_rad(1.0f);
    manual_config.manual_yaw = deg_to_rad(-0.5f);
    CalibrationService manual(manual_config);
    for (int i = 0; i < 5 * kBlockSize; ++i)
        manual.update(output, 20.0f);

    float manual_rpy[3] = {};
    manual.input_rpy(manual_rpy);
    EXPECT_NEAR(manual_rpy[0], manual_config.manual_roll, 1e-7) << "수동 roll이 우선한다";
    EXPECT_NEAR(manual_rpy[1], manual_config.manual_pitch, 1e-7) << "수동 pitch가 우선한다";
    EXPECT_NEAR(manual_rpy[2], manual_config.manual_yaw, 1e-7) << "수동 yaw가 우선한다";
    EXPECT_EQ(manual.snapshot().valid_blocks, 5)
        << "수동 보정도 저장된 보정값으로 남는다";

    AppConfig restored_config;
    restored_config.calibration_auto = true;
    restored_config.manual_calibration = false;
    CalibrationService restored(restored_config);
    float persisted_rpy[3] = {};
    restored.input_rpy(persisted_rpy);
    EXPECT_NEAR(persisted_rpy[0], manual_config.manual_roll, 1e-7) << "저장된 roll을 다시 읽는다";
    EXPECT_NEAR(persisted_rpy[1], manual_config.manual_pitch, 1e-7) << "저장된 pitch를 다시 읽는다";
    EXPECT_NEAR(persisted_rpy[2], manual_config.manual_yaw, 1e-7) << "저장된 yaw를 다시 읽는다";
    EXPECT_EQ(static_cast<int>(restored.snapshot().status),
              static_cast<int>(CalibrationStatus::Calibrated))
        << "다시 읽은 보정은 보정 완료 상태다";

    unsetenv("K230_PARAMS_DIR");
    std::remove(kTestCalibration);
}

TEST(CalibrationEquivalence, AppConfigEnvFeedback)
{
    unsetenv("SUPERCOMBO_CALIB_ROLL_DEG");
    unsetenv("SUPERCOMBO_CALIB_PITCH_DEG");
    unsetenv("SUPERCOMBO_CALIB_YAW_DEG");

    setenv("SUPERCOMBO_CALIB_PITCH_DEG", "1.25", 1);
    setenv("SUPERCOMBO_CALIB_YAW_DEG", "-0.75", 1);
    AppConfig fallback = AppConfig::from_env_defaults();
    EXPECT_TRUE(fallback.manual_calibration)
        << "수동 보정 환경 변수를 주면 수동 모드가 켜진다";
    EXPECT_NEAR(fallback.manual_pitch, deg_to_rad(1.25f), 1e-7) << "수동 pitch 환경 변수 해석";
    EXPECT_NEAR(fallback.manual_yaw, deg_to_rad(-0.75f), 1e-7) << "수동 yaw 환경 변수 해석";
    EXPECT_EQ(fallback.nv12_width, kDefaultAiWidth) << "ISP 출력 폭 기본값은 오버스캔 폭";
    EXPECT_EQ(fallback.nv12_height, kDefaultAiHeight) << "ISP 출력 높이 기본값은 오버스캔 높이";
    EXPECT_NEAR(fallback.input_warp_fx, kDefaultInputWarpFx, 1e-5)
        << "ISP 출력 크기로 K230 카메라 fx를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_fy, kDefaultInputWarpFy, 1e-5)
        << "ISP 출력 크기로 K230 카메라 fy를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_cx, kDefaultInputWarpCx, 1e-5)
        << "ISP 출력 크기로 K230 카메라 cx를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_cy, kDefaultInputWarpCy, 1e-5)
        << "ISP 출력 크기로 K230 카메라 cy를 맞춘다";

    unsetenv("SUPERCOMBO_CALIB_ROLL_DEG");
    unsetenv("SUPERCOMBO_CALIB_PITCH_DEG");
    unsetenv("SUPERCOMBO_CALIB_YAW_DEG");
}

/* 모델 프레임 → 카메라 영상 투영 행렬의 참조식(openpilot get_warp_matrix 경로를 double로). */
void projection_reference(float roll, float pitch, float yaw, float fx, float fy, float cx, float cy,
                          float height, double *projection,
                          ModelFrame model_frame = ModelFrame::MedModel)
{
    const double ground_from_medmodel_frame[9] = {
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00,
       -1.09890110e-03, 0.00000000e+00, 2.81318681e-01,
       -1.84808520e-20, 9.00738606e-04, -4.28751576e-02,
    };
    const double ground_from_sbigmodel_frame[9] = {
        0.00000000e+00,  7.31372216e-19,  1.00000000e+00,
       -2.19780220e-03,  4.11497335e-19,  5.62637363e-01,
       -5.46146580e-20,  1.80147721e-03, -2.73464241e-01,
    };
    const double k[9] = {
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0,
    };
    const double rpy[3] = {roll, pitch, yaw};
    double rot[9];
    rot_from_euler_ref(rpy, rot);

    double device_from_road[9];
    for (int row = 0; row < 3; ++row) {
        device_from_road[row * 3 + 0] = rot[row * 3 + 0];
        device_from_road[row * 3 + 1] = -rot[row * 3 + 1];
        device_from_road[row * 3 + 2] = -rot[row * 3 + 2];
    }

    double view_from_road[9];
    for (int col = 0; col < 3; ++col) {
        view_from_road[0 * 3 + col] = device_from_road[1 * 3 + col];
        view_from_road[1 * 3 + col] = device_from_road[2 * 3 + col];
        view_from_road[2 * 3 + col] = device_from_road[0 * 3 + col];
    }

    const double extrinsic[12] = {
        view_from_road[0], view_from_road[1], view_from_road[2], 0.0,
        view_from_road[3], view_from_road[4], view_from_road[5], height,
        view_from_road[6], view_from_road[7], view_from_road[8], 0.0,
    };
    double camera_frame_from_road[12];
    matmul34d(k, extrinsic, camera_frame_from_road);

    double camera_frame_from_ground[9];
    for (int row = 0; row < 3; ++row) {
        camera_frame_from_ground[row * 3 + 0] = camera_frame_from_road[row * 4 + 0];
        camera_frame_from_ground[row * 3 + 1] = camera_frame_from_road[row * 4 + 1];
        camera_frame_from_ground[row * 3 + 2] = camera_frame_from_road[row * 4 + 3];
    }
    const double *ground_from_model_frame = model_frame == ModelFrame::SmallBigModel
        ? ground_from_sbigmodel_frame
        : ground_from_medmodel_frame;
    matmul3d(camera_frame_from_ground, ground_from_model_frame, projection);
}

// Y 평면 투영을 UV 평면(scale 0.5)으로 옮긴다
void transform_scale_buffer_ref(const double *in, double scale, double *out)
{
    const double transform_out[9] = {
        1.0 / scale, 0.0, 0.5,
        0.0, 1.0 / scale, 0.5,
        0.0, 0.0, 1.0,
    };
    const double transform_in[9] = {
        scale, 0.0, -0.5 * scale,
        0.0, scale, -0.5 * scale,
        0.0, 0.0, 1.0,
    };
    double tmp[9];
    matmul3d(in, transform_out, tmp);
    matmul3d(transform_in, tmp, out);
}

uint8_t clamp_u8(int value)
{
    return static_cast<uint8_t>(std::min(255, std::max(0, value)));
}

/* openpilot modeld transform.cl의 warpPerspective 한 픽셀(INTER_BITS 5, 계수 15비트 고정소수점). */
uint8_t warp_sample_opencl_ref(const uint8_t *src, int src_w, int src_h, int stride_bytes,
                               int bytes_per_pixel, int channel, const double *m, int dx, int dy)
{
    constexpr int kInterBits = 5;
    constexpr int kInterTabSize = 1 << kInterBits;
    constexpr int kCoefBits = 15;
    constexpr int kCoefScale = 1 << kCoefBits;

    const double x0 = m[0] * dx + m[1] * dy + m[2];
    const double y0 = m[3] * dx + m[4] * dy + m[5];
    const double w = m[6] * dx + m[7] * dy + m[8];
    const double scale = w != 0.0 ? static_cast<double>(kInterTabSize) / w : 0.0;
    const int x_fixed = static_cast<int>(std::rint(x0 * scale));
    const int y_fixed = static_cast<int>(std::rint(y0 * scale));
    const int sx = static_cast<int>(std::floor(static_cast<double>(x_fixed) / kInterTabSize));
    const int sy = static_cast<int>(std::floor(static_cast<double>(y_fixed) / kInterTabSize));
    const int ax = x_fixed - sx * kInterTabSize;
    const int ay = y_fixed - sy * kInterTabSize;
    const double tabx = static_cast<double>(ax) / kInterTabSize;
    const double taby = static_cast<double>(ay) / kInterTabSize;
    const int weights[4] = {
        static_cast<int>(std::lrint((1.0 - taby) * (1.0 - tabx) * kCoefScale)),
        static_cast<int>(std::lrint((1.0 - taby) * tabx * kCoefScale)),
        static_cast<int>(std::lrint(taby * (1.0 - tabx) * kCoefScale)),
        static_cast<int>(std::lrint(taby * tabx * kCoefScale)),
    };
    const int xs[4] = {sx, sx + 1, sx, sx + 1};
    const int ys[4] = {sy, sy, sy + 1, sy + 1};

    int64_t sum = 0;
    for (int i = 0; i < 4; ++i) {
        int value = 0;
        if (xs[i] >= 0 && xs[i] < src_w && ys[i] >= 0 && ys[i] < src_h) {
            value = src[ys[i] * stride_bytes + xs[i] * bytes_per_pixel + channel];
        }
        sum += static_cast<int64_t>(value) * weights[i];
    }
    return clamp_u8(static_cast<int>((sum + (1 << (kCoefBits - 1))) >> kCoefBits));
}

// 위치마다 값이 다른 합성 NV12
void fill_nv12(std::vector<uint8_t> &nv12, int width, int height)
{
    uint8_t *y_plane = nv12.data();
    uint8_t *uv_plane = y_plane + width * height;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x)
            y_plane[y * width + x] = static_cast<uint8_t>((x * 3 + y * 5 + (x * y) / 17) & 0xff);
    }
    for (int y = 0; y < height / 2; ++y) {
        for (int x = 0; x < width / 2; ++x) {
            uv_plane[y * width + x * 2] = static_cast<uint8_t>((64 + x * 2 + y * 3) & 0xff);
            uv_plane[y * width + x * 2 + 1] = static_cast<uint8_t>((192 + x * 5 + y) & 0xff);
        }
    }
}

// 워프 없이 NV12 512x256을 openpilot YUV6 순서(Y00, Y10, Y01, Y11, U, V)로 싼다
void pack_direct_openpilot_order(const uint8_t *nv12, float *out)
{
    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + kModelW * kModelH;
    float *y00_plane = out;
    float *y10_plane = y00_plane + kPlaneSize;
    float *y01_plane = y10_plane + kPlaneSize;
    float *y11_plane = y01_plane + kPlaneSize;
    float *u_plane = y11_plane + kPlaneSize;
    float *v_plane = u_plane + kPlaneSize;

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        const uint8_t *y0 = y_plane + (y2 * 2) * kModelW;
        const uint8_t *y1 = y0 + kModelW;
        const uint8_t *uv = uv_plane + y2 * kModelW;
        for (int x2 = 0; x2 < kHalfW; ++x2) {
            const int dst = y2 * kHalfW + x2;
            y00_plane[dst] = static_cast<float>(y0[x2 * 2 + 0]);
            y10_plane[dst] = static_cast<float>(y1[x2 * 2 + 0]);
            y01_plane[dst] = static_cast<float>(y0[x2 * 2 + 1]);
            y11_plane[dst] = static_cast<float>(y1[x2 * 2 + 1]);
            u_plane[dst] = static_cast<float>(uv[x2 * 2 + 0]);
            v_plane[dst] = static_cast<float>(uv[x2 * 2 + 1]);
        }
    }
}

// OpenCL 참조 샘플러로 워프한 뒤 YUV6로 싼다
void warp_pack_opencl_ref(const uint8_t *nv12, int src_w, int src_h,
                          const double *projection_y, float *out)
{
    double projection_uv[9];
    transform_scale_buffer_ref(projection_y, 0.5, projection_uv);

    const uint8_t *y_src = nv12;
    const uint8_t *uv_src = nv12 + src_w * src_h;
    float *y00_plane = out;
    float *y10_plane = y00_plane + kPlaneSize;
    float *y01_plane = y10_plane + kPlaneSize;
    float *y11_plane = y01_plane + kPlaneSize;
    float *u_plane = y11_plane + kPlaneSize;
    float *v_plane = u_plane + kPlaneSize;

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        for (int x2 = 0; x2 < kHalfW; ++x2) {
            const int dst = y2 * kHalfW + x2;
            const int ox = x2 * 2;
            const int oy = y2 * 2;
            y00_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox, oy);
            y10_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox, oy + 1);
            y01_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox + 1, oy);
            y11_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox + 1, oy + 1);
            u_plane[dst] = warp_sample_opencl_ref(uv_src, src_w / 2, src_h / 2, src_w, 2, 0,
                                                  projection_uv, x2, y2);
            v_plane[dst] = warp_sample_opencl_ref(uv_src, src_w / 2, src_h / 2, src_w, 2, 1,
                                                  projection_uv, x2, y2);
        }
    }
}

struct DiffStats {
    double mean = 0.0;
    double max = 0.0;
    double inner_mean = 0.0;
    double inner_max = 0.0;
};

// YUV6 두 개의 절대 오차. inner는 가장자리 8픽셀을 뺀 영역
DiffStats diff_stats(const std::vector<float> &a, const std::vector<float> &b)
{
    DiffStats stats;
    double sum = 0.0;
    double inner_sum = 0.0;
    int count = 0;
    int inner_count = 0;
    for (int plane = 0; plane < 6; ++plane) {
        for (int y = 0; y < kHalfH; ++y) {
            for (int x = 0; x < kHalfW; ++x) {
                const int idx = plane * kPlaneSize + y * kHalfW + x;
                const double d = std::fabs(a[idx] - b[idx]);
                sum += d;
                stats.max = std::max(stats.max, d);
                ++count;
                if (x >= 8 && x < kHalfW - 8 && y >= 8 && y < kHalfH - 8) {
                    inner_sum += d;
                    stats.inner_max = std::max(stats.inner_max, d);
                    ++inner_count;
                }
            }
        }
    }
    stats.mean = sum / count;
    stats.inner_mean = inner_sum / inner_count;
    return stats;
}

TEST(CalibrationEquivalence, ProjectionAndYuv6)
{
    AppConfig camera_config;
    ModelInputTransform camera_transform(camera_config);
    float camera_projection[9];
    double camera_reference[9];
    camera_transform.projection_matrix(camera_projection);
    projection_reference(0.0, 0.0, 0.0, camera_config.input_warp_fx,
                         camera_config.input_warp_fy, camera_config.input_warp_cx,
                         camera_config.input_warp_cy, camera_config.input_warp_height,
                         camera_reference);
    for (int i = 0; i < 9; ++i)
        EXPECT_NEAR(camera_projection[i], camera_reference[i], 1e-4)
            << "기본 투영은 K230 카메라 내부 파라미터를 쓴다";

    ModelInputTransform sbig_camera_transform(camera_config, ModelFrame::SmallBigModel);
    float sbig_camera_projection[9];
    double sbig_camera_reference[9];
    sbig_camera_transform.projection_matrix(sbig_camera_projection);
    projection_reference(0.0, 0.0, 0.0, camera_config.input_warp_fx,
                         camera_config.input_warp_fy, camera_config.input_warp_cx,
                         camera_config.input_warp_cy, camera_config.input_warp_height,
                         sbig_camera_reference, ModelFrame::SmallBigModel);
    for (int i = 0; i < 9; ++i)
        EXPECT_NEAR(sbig_camera_projection[i], sbig_camera_reference[i], 1e-4)
            << "sbig 투영은 openpilot 가상 카메라를 쓴다";

    const std::array<std::array<float, 3>, 5> cases = {{
        {{0.0f, 0.0f, 0.0f}},
        {{0.0f, deg_to_rad(1.5f), 0.0f}},
        {{0.0f, deg_to_rad(-1.5f), 0.0f}},
        {{0.0f, 0.0f, deg_to_rad(1.0f)}},
        {{0.0f, deg_to_rad(1.1f), deg_to_rad(-0.8f)}},
    }};

    for (const auto &rpy : cases) {
        AppConfig config;
        config.manual_roll = rpy[0];
        config.manual_pitch = rpy[1];
        config.manual_yaw = rpy[2];
        ModelInputTransform transform(config);
        float actual[9];
        double expected[9];
        transform.projection_matrix(actual);
        projection_reference(rpy[0], rpy[1], rpy[2], config.input_warp_fx,
                             config.input_warp_fy, config.input_warp_cx,
                             config.input_warp_cy, config.input_warp_height, expected);
        for (int i = 0; i < 9; ++i) {
            const double tolerance = std::max(1e-4, std::fabs(expected[i]) * 1e-5);
            EXPECT_NEAR(actual[i], expected[i], tolerance) << "투영 행렬";
        }
    }

    std::vector<uint8_t> nv12(kModelW * kModelH * 3 / 2);
    std::vector<float> direct(kYuv6Floats, 0.0f);
    std::vector<float> warped(kYuv6Floats, 0.0f);
    std::vector<float> ref(kYuv6Floats, 0.0f);
    std::vector<float> sbig_warped(kYuv6Floats, 0.0f);
    std::vector<float> sbig_ref(kYuv6Floats, 0.0f);
    fill_nv12(nv12, kModelW, kModelH);
    pack_direct_openpilot_order(nv12.data(), direct.data());

    AppConfig identity_config;
    identity_config.input_warp_fx = 910.0f;
    identity_config.input_warp_fy = 910.0f;
    identity_config.input_warp_cx = 256.0f;
    identity_config.input_warp_cy = 47.6f;
    ModelInputTransform identity(identity_config);
    identity.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    DiffStats identity_diff = diff_stats(direct, warped);
    EXPECT_NEAR(identity_diff.max, 0.0, 0.0)
        << "rpy가 0이면 워프한 YUV6가 직접 패킹과 비트까지 같다";

    AppConfig pitch_config;
    pitch_config.input_warp_fx = kDefaultModelFx;
    pitch_config.input_warp_fy = kDefaultModelFy;
    pitch_config.input_warp_cx = kDefaultModelCx;
    pitch_config.input_warp_cy = kDefaultModelCy;
    pitch_config.manual_pitch = deg_to_rad(1.5f);
    pitch_config.manual_yaw = deg_to_rad(-0.6f);
    ModelInputTransform pitched(pitch_config);
    pitched.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    double projection_y[9];
    projection_reference(0.0f, pitch_config.manual_pitch, pitch_config.manual_yaw,
                         pitch_config.input_warp_fx, pitch_config.input_warp_fy,
                         pitch_config.input_warp_cx, pitch_config.input_warp_cy,
                         pitch_config.input_warp_height, projection_y);
    warp_pack_opencl_ref(nv12.data(), kModelW, kModelH, projection_y, ref.data());
    DiffStats warp_diff = diff_stats(ref, warped);
    EXPECT_LT(warp_diff.mean, 1.0) << "워프 평균 절대 오차(openpilot OpenCL 참조 대비)";
    EXPECT_LT(warp_diff.inner_max, 8.0)
        << "워프 안쪽 최대 오차(openpilot OpenCL 참조 대비)";

    ModelInputTransform sbig_pitched(pitch_config, ModelFrame::SmallBigModel);
    sbig_pitched.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, sbig_warped);
    projection_reference(0.0f, pitch_config.manual_pitch, pitch_config.manual_yaw,
                         pitch_config.input_warp_fx, pitch_config.input_warp_fy,
                         pitch_config.input_warp_cx, pitch_config.input_warp_cy,
                         pitch_config.input_warp_height, projection_y,
                         ModelFrame::SmallBigModel);
    warp_pack_opencl_ref(nv12.data(), kModelW, kModelH, projection_y, sbig_ref.data());
    DiffStats sbig_warp_diff = diff_stats(sbig_ref, sbig_warped);
    EXPECT_LT(sbig_warp_diff.mean, 1.0) << "sbig 워프 평균 절대 오차(openpilot OpenCL 참조 대비)";
    EXPECT_LT(sbig_warp_diff.inner_max, 8.0)
        << "sbig 워프 안쪽 최대 오차(openpilot OpenCL 참조 대비)";

    constexpr int kSourceW = 640;
    constexpr int kSourceH = 360;
    std::vector<uint8_t> source_nv12(kSourceW * kSourceH * 3 / 2);
    std::vector<float> compact(kYuv6Floats, 0.0f);
    std::vector<float> opencl(kYuv6Floats, 0.0f);
    fill_nv12(source_nv12, kSourceW, kSourceH);
    AppConfig source_config;
    const std::array<std::array<float, 3>, 6> rpy_cases = {{
        {{0.0f, 0.0f, 0.0f}},
        {{0.0f, deg_to_rad(-0.75f), deg_to_rad(1.1f)}},
        {{deg_to_rad(0.5f), deg_to_rad(2.0f), deg_to_rad(-2.5f)}},
        {{deg_to_rad(-0.5f), deg_to_rad(-3.0f), deg_to_rad(3.5f)}},
        {{0.0f, deg_to_rad(8.0f), deg_to_rad(-3.9f)}},
        {{0.0f, deg_to_rad(-5.0f), deg_to_rad(3.9f)}},
    }};
    DiffStats opencl_worst;
    for (ModelFrame frame : {ModelFrame::MedModel, ModelFrame::SmallBigModel}) {
        ModelInputTransform transform(source_config, frame);
        for (const auto &rpy : rpy_cases) {
            transform.set_calibration(rpy[0], rpy[1], rpy[2]);
            float projection[9];
            transform.projection_matrix(projection);
            transform.nv12_to_yuv6_warped_scalar(
                source_nv12.data(), kSourceW, kSourceH, compact.data());
            double projection_opencl[9];
            for (int i = 0; i < 9; ++i)
                projection_opencl[i] = projection[i];
            warp_pack_opencl_ref(source_nv12.data(), kSourceW, kSourceH,
                                 projection_opencl, opencl.data());
            const DiffStats opencl_diff = diff_stats(compact, opencl);
            opencl_worst.mean = std::max(opencl_worst.mean, opencl_diff.mean);
            opencl_worst.inner_max =
                std::max(opencl_worst.inner_max, opencl_diff.inner_max);
        }
    }
    EXPECT_LT(opencl_worst.mean, 1.0) << "640x360 고정소수점 워프 평균 절대 오차(openpilot OpenCL 참조 대비)";
    EXPECT_LT(opencl_worst.inner_max, 8.0)
        << "640x360 고정소수점 워프 안쪽 최대 오차(openpilot OpenCL 참조 대비)";
}

} // namespace
