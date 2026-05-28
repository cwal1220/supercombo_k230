#include "online_calibrator.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kMinSpeedFilter = 15.0f * 0.44704f;
constexpr float kMaxVelAngleStd = 0.25f * kPi / 180.0f;
constexpr float kMaxYawRateFilter = 2.0f * kPi / 180.0f;
constexpr float kMaxAllowedSpread = 2.0f * kPi / 180.0f;
constexpr float kPitchMin = -0.09074112085129739f;
constexpr float kPitchMax = 0.14907572052989657f;
constexpr float kYawMin = -0.06912048084718224f;
constexpr float kYawMax = 0.06912048084718235f;
constexpr float kSanityMargin = 0.005f;

bool finite3(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

} // namespace

const char *calibration_status_name(CalibrationStatus status)
{
    switch (status) {
    case CalibrationStatus::Uncalibrated:
        return "uncalibrated";
    case CalibrationStatus::Calibrated:
        return "calibrated";
    case CalibrationStatus::Invalid:
        return "invalid";
    }
    return "unknown";
}

OnlineCalibrator::OnlineCalibrator()
{
    reset_to_rpy(snapshot_.rpy, 0);
}

OnlineCalibrator::UpdateResult OnlineCalibrator::update(const PoseObservation &pose)
{
    UpdateResult result;
    result.snapshot = snapshot_;

    const bool valid_numbers = finite3(pose.trans) && finite3(pose.rot) && finite3(pose.trans_std);
    const bool straight_and_fast = valid_numbers &&
        pose.trans[0] > kMinSpeedFilter &&
        std::fabs(pose.rot[2]) < kMaxYawRateFilter;
    const bool certain_if_calib = valid_numbers &&
        ((std::atan2(pose.trans_std[1], pose.trans[0]) < kMaxVelAngleStd) ||
         (snapshot_.valid_blocks < kInputsNeeded));

    if (!straight_and_fast || !certain_if_calib) {
        ++snapshot_.rejected_samples;
        result.snapshot = snapshot_;
        return result;
    }

    float observed_rpy[3] = {
        0.0f,
        -std::atan2(pose.trans[2], pose.trans[0]),
        std::atan2(pose.trans[1], pose.trans[0]),
    };
    sanity_clip(observed_rpy);
    if (!finite3(observed_rpy)) {
        ++snapshot_.rejected_samples;
        result.snapshot = snapshot_;
        return result;
    }

    for (int i = 0; i < 3; ++i)
        block_sum_[i] += observed_rpy[i];
    ++block_sample_count_;
    ++snapshot_.accepted_samples;
    snapshot_.block_sample_count = block_sample_count_;
    result.accepted = true;

    if (block_sample_count_ >= kBlockSize) {
        for (int i = 0; i < 3; ++i)
            block_rpys_[block_pos_][i] = block_sum_[i] / static_cast<float>(block_sample_count_);

        block_pos_ = (block_pos_ + 1) % kInputsWanted;
        ++completed_blocks_total_;
        std::memset(block_sum_, 0, sizeof(block_sum_));
        block_sample_count_ = 0;
        snapshot_.block_sample_count = 0;
        update_status();
        result.block_completed = true;
    }

    result.snapshot = snapshot_;
    return result;
}

OnlineCalibrator::Snapshot OnlineCalibrator::snapshot() const
{
    return snapshot_;
}

void OnlineCalibrator::reset_to_rpy(const float rpy[3], int valid_blocks)
{
    const int clamped_valid_blocks = std::max(0, std::min(valid_blocks, kInputsWanted));
    for (int b = 0; b < kInputsWanted; ++b) {
        for (int i = 0; i < 3; ++i)
            block_rpys_[b][i] = rpy[i];
    }
    std::memset(block_sum_, 0, sizeof(block_sum_));
    block_sample_count_ = 0;
    block_pos_ = clamped_valid_blocks % kInputsWanted;
    completed_blocks_total_ = clamped_valid_blocks;
    for (int i = 0; i < 3; ++i) {
        snapshot_.rpy[i] = rpy[i];
        snapshot_.spread[i] = 0.0f;
    }
    snapshot_.valid_blocks = clamped_valid_blocks;
    snapshot_.block_sample_count = 0;
    snapshot_.status = clamped_valid_blocks >= kInputsNeeded && is_calibration_valid(rpy)
        ? CalibrationStatus::Calibrated
        : CalibrationStatus::Uncalibrated;
}

void OnlineCalibrator::update_status()
{
    const int valid_blocks = std::min(completed_blocks_total_, kInputsWanted);
    snapshot_.valid_blocks = valid_blocks;
    if (valid_blocks <= 0) {
        const float zero[3] = {0.0f, 0.0f, 0.0f};
        reset_to_rpy(zero, 0);
        return;
    }

    float sum[3] = {};
    float min_v[3] = {block_rpys_[0][0], block_rpys_[0][1], block_rpys_[0][2]};
    float max_v[3] = {block_rpys_[0][0], block_rpys_[0][1], block_rpys_[0][2]};
    for (int b = 0; b < valid_blocks; ++b) {
        for (int i = 0; i < 3; ++i) {
            const float value = block_rpys_[b][i];
            sum[i] += value;
            min_v[i] = std::min(min_v[i], value);
            max_v[i] = std::max(max_v[i], value);
        }
    }

    for (int i = 0; i < 3; ++i) {
        snapshot_.rpy[i] = sum[i] / static_cast<float>(valid_blocks);
        snapshot_.spread[i] = std::fabs(max_v[i] - min_v[i]);
    }

    if (valid_blocks < kInputsNeeded) {
        snapshot_.status = CalibrationStatus::Uncalibrated;
    } else if (is_calibration_valid(snapshot_.rpy)) {
        snapshot_.status = CalibrationStatus::Calibrated;
    } else {
        snapshot_.status = CalibrationStatus::Invalid;
    }

    const float max_spread = std::max(snapshot_.spread[0], std::max(snapshot_.spread[1], snapshot_.spread[2]));
    if (snapshot_.status == CalibrationStatus::Calibrated && max_spread > kMaxAllowedSpread) {
        const int last_block = (block_pos_ + kInputsWanted - 1) % kInputsWanted;
        reset_to_rpy(block_rpys_[last_block], kInputsNeeded);
    }
}

bool OnlineCalibrator::is_calibration_valid(const float rpy[3]) const
{
    return kPitchMin < rpy[1] && rpy[1] < kPitchMax &&
           kYawMin < rpy[2] && rpy[2] < kYawMax;
}

void OnlineCalibrator::sanity_clip(float rpy[3]) const
{
    if (!finite3(rpy)) {
        rpy[0] = 0.0f;
        rpy[1] = 0.0f;
        rpy[2] = 0.0f;
        return;
    }
    rpy[0] = 0.0f;
    rpy[1] = std::max(kPitchMin - kSanityMargin, std::min(kPitchMax + kSanityMargin, rpy[1]));
    rpy[2] = std::max(kYawMin - kSanityMargin, std::min(kYawMax + kSanityMargin, rpy[2]));
}
