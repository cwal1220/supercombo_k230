#include "app_config.h"
#include "calibration_service.h"
#include "model_input_transform.h"
#include "online_calibrator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

int g_failures = 0;

void fail(const std::string &message)
{
    std::fprintf(stderr, "FAIL: %s\n", message.c_str());
    ++g_failures;
}

void expect_true(bool condition, const std::string &message)
{
    if (!condition) fail(message);
}

void expect_near(double actual, double expected, double tolerance, const std::string &message)
{
    if (std::fabs(actual - expected) > tolerance) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s actual=%.9g expected=%.9g tolerance=%.3g",
                      message.c_str(), actual, expected, tolerance);
        fail(buf);
    }
}

void expect_equal_int(int actual, int expected, const std::string &message)
{
    if (actual != expected) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s actual=%d expected=%d", message.c_str(), actual, expected);
        fail(buf);
    }
}

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
        old_rpy_weight = std::min(0.0, old_rpy_weight - 1.0 / 400.0);

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
    expect_equal_int(actual.valid_blocks, expected.valid_blocks, std::string(label) + " valid_blocks");
    expect_equal_int(actual.block_sample_count, expected.idx, std::string(label) + " block_sample_count");
    expect_equal_int(static_cast<int>(actual.status), static_cast<int>(expected.status),
                     std::string(label) + " status");
    expect_near(actual.accepted_samples, expected.accepted, 0.0, std::string(label) + " accepted");
    expect_near(actual.rejected_samples, expected.rejected, 0.0, std::string(label) + " rejected");
    for (int i = 0; i < 3; ++i) {
        expect_near(actual.rpy[i], expected.rpy[i], 1e-5, std::string(label) + " rpy");
        expect_near(actual.spread[i], expected.spread[i], 1e-5, std::string(label) + " spread");
    }
}

void test_online_calibrator()
{
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;

    PoseObservation low_speed = make_pose(6.0f);
    expect_true(!actual.update(low_speed, 20.0f).accepted, "low camera speed sample must reject");

    PoseObservation low_vehicle_speed = make_pose(20.0f, 0.0f, 0.0f, 0.01f, 0.0f);
    expect_true(!actual.update(low_vehicle_speed, 2.0f).accepted,
                "low CAN vEgo sample must reject");
    expect_true(!ref.update(low_speed), "reference low speed sample must reject");
    expect_true(!ref.update(low_vehicle_speed, 2.0), "reference low CAN vEgo sample must reject");
    compare_snapshot(actual.snapshot(), ref, "low_speed_reject");

    PoseObservation high_yaw = make_pose();
    high_yaw.rot[2] = static_cast<float>(3.0 * kPi / 180.0);
    expect_true(!actual.update(high_yaw, 20.0f).accepted, "high yaw-rate sample must reject");
    expect_true(!ref.update(high_yaw), "reference high yaw-rate sample must reject");
    compare_snapshot(actual.snapshot(), ref, "high_yaw_reject");

    PoseObservation nan_pose = make_pose();
    nan_pose.trans[0] = std::numeric_limits<float>::quiet_NaN();
    expect_true(!actual.update(nan_pose, 20.0f).accepted, "NaN sample must reject");
    expect_true(!ref.update(nan_pose), "reference NaN sample must reject");
    compare_snapshot(actual.snapshot(), ref, "nan_reject");

    const PoseObservation accepted_pose = make_pose();
    for (int i = 0; i < kBlockSize; ++i) {
        expect_true(actual.update(accepted_pose, 20.0f).accepted, "accepted block sample");
        expect_true(ref.update(accepted_pose), "reference accepted block sample");
    }
    compare_snapshot(actual.snapshot(), ref, "one_block");
    expect_equal_int(actual.snapshot().valid_blocks, 1, "one block valid count");

    for (int i = 0; i < 4 * kBlockSize; ++i) {
        actual.update(accepted_pose, 20.0f);
        ref.update(accepted_pose);
    }
    compare_snapshot(actual.snapshot(), ref, "five_blocks");
    expect_equal_int(static_cast<int>(actual.snapshot().status),
                     static_cast<int>(CalibrationStatus::Calibrated),
                     "five accepted blocks should calibrate");

    PoseObservation uncertain = make_pose();
    uncertain.trans_std[1] = 1.0f;
    expect_true(!actual.update(uncertain, 20.0f).accepted, "high trans std must reject after calibration");
    expect_true(!ref.update(uncertain), "reference high trans std must reject after calibration");
    compare_snapshot(actual.snapshot(), ref, "uncertain_after_calib");

    std::printf("calibrator: constants mph=%.4f yaw_limit_deg=2.0 std_limit_deg=0.25 blocks=%d/%d/%d spread_deg=2.0\n",
                kMinSpeedFilter, kBlockSize, kInputsNeeded, kInputsWanted);
}

ParsedModelOutput parsed_from_pose(const PoseObservation &pose)
{
    ParsedModelOutput output{};
    output.valid = true;
    output.has_pose = true;
    output.pose = pose;
    return output;
}

void test_calibration_service()
{
    constexpr const char *kTestParamsDir = "params/work";
    constexpr const char *kTestCalibration = "params/work/calibration.json";
    std::remove(kTestCalibration);
    setenv("K230_PARAMS_DIR", kTestParamsDir, 1);

    OnlineCalibrator restored_calibrator;
    const float restored_rpy[3] = {0.0f, deg_to_rad(2.0f), deg_to_rad(-0.75f)};
    expect_true(restored_calibrator.restore(restored_rpy, 12),
                "valid persisted calibration restores");
    float restored_output[3] = {};
    restored_calibrator.output_rpy(restored_output);
    expect_near(restored_output[1], restored_rpy[1], 1e-7,
                "restored pitch initializes calibrator");
    expect_near(restored_output[2], restored_rpy[2], 1e-7,
                "restored yaw initializes calibrator");
    expect_equal_int(restored_calibrator.snapshot().valid_blocks, 12,
                     "restored valid block count");

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
        expect_near(input_rpy[i], snapshot.rpy[i], 1e-7, "auto input_rpy follows online snapshot");

    AppConfig manual_config;
    manual_config.calibration_auto = true;
    manual_config.manual_calibration = true;
    manual_config.manual_pitch = deg_to_rad(1.0f);
    manual_config.manual_yaw = deg_to_rad(-0.5f);
    CalibrationService manual(manual_config);
    for (int i = 0; i < 5 * kBlockSize; ++i)
        manual.update(output, 20.0f);

    float manual_rpy[3] = {};
    manual.input_rpy(manual_rpy);
    expect_near(manual_rpy[0], 0.0, 1e-7, "manual roll wins");
    expect_near(manual_rpy[1], manual_config.manual_pitch, 1e-7, "manual pitch wins");
    expect_near(manual_rpy[2], manual_config.manual_yaw, 1e-7, "manual yaw wins");
    expect_equal_int(manual.snapshot().valid_blocks, 5,
                     "manual override is represented as persisted calibration");

    AppConfig restored_config;
    restored_config.calibration_auto = true;
    restored_config.manual_calibration = false;
    CalibrationService restored(restored_config);
    float persisted_rpy[3] = {};
    restored.input_rpy(persisted_rpy);
    expect_near(persisted_rpy[1], manual_config.manual_pitch, 1e-7,
                "persisted pitch reloads");
    expect_near(persisted_rpy[2], manual_config.manual_yaw, 1e-7,
                "persisted yaw reloads");
    expect_equal_int(static_cast<int>(restored.snapshot().status),
                     static_cast<int>(CalibrationStatus::Calibrated),
                     "persisted calibration reloads as calibrated");

    unsetenv("K230_PARAMS_DIR");
    std::remove(kTestCalibration);

    std::printf("calibration_service: restore, online feedback and manual override OK\n");
}

void test_app_config_env_feedback()
{
    unsetenv("SUPERCOMBO_NV12_WIDTH");
    unsetenv("SUPERCOMBO_NV12_HEIGHT");
    unsetenv("SUPERCOMBO_CALIB_ROLL_DEG");
    unsetenv("SUPERCOMBO_CALIB_PITCH_DEG");
    unsetenv("SUPERCOMBO_CALIB_YAW_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_ROLL_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_PITCH_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_YAW_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_FX");
    unsetenv("SUPERCOMBO_INPUT_WARP_FY");
    unsetenv("SUPERCOMBO_INPUT_WARP_CX");
    unsetenv("SUPERCOMBO_INPUT_WARP_CY");

    setenv("SUPERCOMBO_CALIB_PITCH_DEG", "1.25", 1);
    setenv("SUPERCOMBO_CALIB_YAW_DEG", "-0.75", 1);
    AppConfig fallback = AppConfig::from_env_defaults();
    expect_true(fallback.manual_calibration, "manual calibration env should enable manual override");
    expect_near(fallback.manual_pitch, deg_to_rad(1.25f), 1e-7, "manual pitch env parse");
    expect_near(fallback.manual_yaw, deg_to_rad(-0.75f), 1e-7, "manual yaw env parse");
    expect_near(fallback.input_warp_pitch, fallback.manual_pitch, 1e-7, "input warp pitch falls back to manual calibration");
    expect_near(fallback.input_warp_yaw, fallback.manual_yaw, 1e-7, "input warp yaw falls back to manual calibration");
    expect_equal_int(fallback.nv12_width, kDefaultAiWidth,
                     "ISP output defaults to overscan width");
    expect_equal_int(fallback.nv12_height, kDefaultAiHeight,
                     "ISP output defaults to overscan height");
    expect_near(fallback.input_warp_fx, kDefaultInputWarpFx, 1e-5,
                "ISP output scales K230 camera fx");
    expect_near(fallback.input_warp_fy, kDefaultInputWarpFy, 1e-5,
                "ISP output scales K230 camera fy");
    expect_near(fallback.input_warp_cx, kDefaultInputWarpCx, 1e-5,
                "ISP output scales K230 camera cx");
    expect_near(fallback.input_warp_cy, kDefaultInputWarpCy, 1e-5,
                "ISP output scales K230 camera cy");
    setenv("SUPERCOMBO_NV12_WIDTH", "512", 1);
    setenv("SUPERCOMBO_NV12_HEIGHT", "256", 1);
    AppConfig model_sized = AppConfig::from_env_defaults();
    expect_near(model_sized.input_warp_fx, default_input_warp_fx(512), 1e-5,
                "model-sized source scales K230 camera fx");
    expect_near(model_sized.input_warp_fy, default_input_warp_fy(256), 1e-5,
                "model-sized source scales K230 camera fy");
    expect_near(model_sized.input_warp_cx, default_input_warp_cx(512), 1e-5,
                "model-sized source scales K230 camera cx");
    expect_near(model_sized.input_warp_cy, default_input_warp_cy(256), 1e-5,
                "model-sized source scales K230 camera cy");
    unsetenv("SUPERCOMBO_NV12_WIDTH");
    unsetenv("SUPERCOMBO_NV12_HEIGHT");

    setenv("SUPERCOMBO_INPUT_WARP_PITCH_DEG", "0.50", 1);
    setenv("SUPERCOMBO_INPUT_WARP_FX", "433.53", 1);
    AppConfig explicit_input = AppConfig::from_env_defaults();
    expect_near(explicit_input.manual_pitch, deg_to_rad(1.25f), 1e-7, "manual pitch remains calibration env");
    expect_near(explicit_input.input_warp_pitch, deg_to_rad(0.50f), 1e-7, "explicit input warp pitch wins");
    expect_near(explicit_input.input_warp_fx, 433.53, 1e-5, "explicit input warp fx wins");

    unsetenv("SUPERCOMBO_CALIB_ROLL_DEG");
    unsetenv("SUPERCOMBO_CALIB_PITCH_DEG");
    unsetenv("SUPERCOMBO_CALIB_YAW_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_ROLL_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_PITCH_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_YAW_DEG");
    unsetenv("SUPERCOMBO_INPUT_WARP_FX");

    std::printf("app_config: input-warp/manual calibration env precedence OK\n");
}

void projection_reference(float roll, float pitch, float yaw, float fx, float fy, float cx, float cy,
                          float height, double *projection)
{
    const double ground_from_medmodel_frame[9] = {
        0.00000000e+00, 0.00000000e+00, 1.00000000e+00,
       -1.09890110e-03, 0.00000000e+00, 2.81318681e-01,
       -1.84808520e-20, 9.00738606e-04, -4.28751576e-02,
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
    matmul3d(camera_frame_from_ground, ground_from_medmodel_frame, projection);
}

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

void fill_nv12(std::vector<uint8_t> &nv12)
{
    uint8_t *y_plane = nv12.data();
    uint8_t *uv_plane = y_plane + kModelW * kModelH;
    for (int y = 0; y < kModelH; ++y) {
        for (int x = 0; x < kModelW; ++x)
            y_plane[y * kModelW + x] = static_cast<uint8_t>((x * 3 + y * 5 + (x * y) / 17) & 0xff);
    }
    for (int y = 0; y < kHalfH; ++y) {
        for (int x = 0; x < kHalfW; ++x) {
            uv_plane[y * kModelW + x * 2 + 0] = static_cast<uint8_t>((64 + x * 2 + y * 3) & 0xff);
            uv_plane[y * kModelW + x * 2 + 1] = static_cast<uint8_t>((192 + x * 5 + y) & 0xff);
        }
    }
}

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

void warp_pack_opencl_ref(const uint8_t *nv12, const double *projection_y, float *out)
{
    double projection_uv[9];
    transform_scale_buffer_ref(projection_y, 0.5, projection_uv);

    const uint8_t *y_src = nv12;
    const uint8_t *uv_src = nv12 + kModelW * kModelH;
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
            y00_plane[dst] = warp_sample_opencl_ref(y_src, kModelW, kModelH, kModelW, 1, 0,
                                                    projection_y, ox, oy);
            y10_plane[dst] = warp_sample_opencl_ref(y_src, kModelW, kModelH, kModelW, 1, 0,
                                                    projection_y, ox, oy + 1);
            y01_plane[dst] = warp_sample_opencl_ref(y_src, kModelW, kModelH, kModelW, 1, 0,
                                                    projection_y, ox + 1, oy);
            y11_plane[dst] = warp_sample_opencl_ref(y_src, kModelW, kModelH, kModelW, 1, 0,
                                                    projection_y, ox + 1, oy + 1);
            u_plane[dst] = warp_sample_opencl_ref(uv_src, kHalfW, kHalfH, kModelW, 2, 0,
                                                  projection_uv, x2, y2);
            v_plane[dst] = warp_sample_opencl_ref(uv_src, kHalfW, kHalfH, kModelW, 2, 1,
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

void test_projection_and_yuv6()
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
        expect_near(camera_projection[i], camera_reference[i], 1e-4,
                    "default projection uses K230 camera intrinsics");

    const std::array<std::array<float, 3>, 5> cases = {{
        {{0.0f, 0.0f, 0.0f}},
        {{0.0f, deg_to_rad(1.5f), 0.0f}},
        {{0.0f, deg_to_rad(-1.5f), 0.0f}},
        {{0.0f, 0.0f, deg_to_rad(1.0f)}},
        {{0.0f, deg_to_rad(1.1f), deg_to_rad(-0.8f)}},
    }};

    for (const auto &rpy : cases) {
        AppConfig config;
        config.input_warp_roll = rpy[0];
        config.input_warp_pitch = rpy[1];
        config.input_warp_yaw = rpy[2];
        ModelInputTransform transform(config);
        float actual[9];
        double expected[9];
        transform.projection_matrix(actual);
        projection_reference(rpy[0], rpy[1], rpy[2], config.input_warp_fx,
                             config.input_warp_fy, config.input_warp_cx,
                             config.input_warp_cy, config.input_warp_height, expected);
        for (int i = 0; i < 9; ++i) {
            const double tolerance = std::max(1e-4, std::fabs(expected[i]) * 1e-5);
            expect_near(actual[i], expected[i], tolerance, "projection matrix");
        }
    }

    std::vector<uint8_t> nv12(kModelW * kModelH * 3 / 2);
    std::vector<float> direct(kYuv6Floats, 0.0f);
    std::vector<float> warped(kYuv6Floats, 0.0f);
    std::vector<float> ref(kYuv6Floats, 0.0f);
    fill_nv12(nv12);
    pack_direct_openpilot_order(nv12.data(), direct.data());

    AppConfig identity_config;
    identity_config.input_warp_fx = 910.0f;
    identity_config.input_warp_fy = 910.0f;
    identity_config.input_warp_cx = 256.0f;
    identity_config.input_warp_cy = 47.6f;
    ModelInputTransform identity(identity_config);
    identity.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    DiffStats identity_diff = diff_stats(direct, warped);
    expect_near(identity_diff.max, 0.0, 0.0, "zero-rpy warped YUV6 must match direct pack exactly");

    AppConfig pitch_config;
    pitch_config.input_warp_fx = kDefaultModelFx;
    pitch_config.input_warp_fy = kDefaultModelFy;
    pitch_config.input_warp_cx = kDefaultModelCx;
    pitch_config.input_warp_cy = kDefaultModelCy;
    pitch_config.input_warp_pitch = deg_to_rad(1.5f);
    pitch_config.input_warp_yaw = deg_to_rad(-0.6f);
    ModelInputTransform pitched(pitch_config);
    pitched.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    double projection_y[9];
    projection_reference(0.0f, pitch_config.input_warp_pitch, pitch_config.input_warp_yaw,
                         pitch_config.input_warp_fx, pitch_config.input_warp_fy,
                         pitch_config.input_warp_cx, pitch_config.input_warp_cy,
                         pitch_config.input_warp_height, projection_y);
    warp_pack_opencl_ref(nv12.data(), projection_y, ref.data());
    DiffStats warp_diff = diff_stats(ref, warped);
    expect_true(warp_diff.mean < 1.0, "non-zero warp mean abs diff vs openpilot OpenCL reference");
    expect_true(warp_diff.inner_max < 8.0, "non-zero warp inner max diff vs openpilot OpenCL reference");

    std::printf("projection/yuv6: matrix_tol<=1e-4 identity_max=%.1f med_mean=%.3f\n",
                identity_diff.max, warp_diff.mean);
}

} // namespace

int main()
{
    test_online_calibrator();
    test_calibration_service();
    test_app_config_env_feedback();
    test_projection_and_yuv6();

    if (g_failures != 0) {
        std::fprintf(stderr, "verify_calibration_equivalence: %d failure(s)\n", g_failures);
        return 1;
    }

    std::printf("verify_calibration_equivalence: PASS\n");
    return 0;
}
