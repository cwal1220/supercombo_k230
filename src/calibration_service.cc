#include "calibration_service.h"

#include <cstdio>

CalibrationService::CalibrationService(const AppConfig &config)
    : projection_mode_(config.projection_mode),
      auto_enabled_(config.calibration_auto),
      manual_override_(config.manual_calibration),
      log_enabled_(config.log_calibration)
{
    fixed_rpy_[0] = config.manual_roll;
    fixed_rpy_[1] = config.manual_pitch;
    fixed_rpy_[2] = config.manual_yaw;
    set_fixed_projection();
    last_snapshot_ = calibrator_.snapshot();

    const char *mode = can_apply_online() ? "auto-pose-shadow-unvalid" : "manual";
    std::fprintf(stderr, "calibration mode=%s overlay_auto=%d manual_override=%d projection=%s\n",
                 mode, can_apply_online() ? 1 : 0, manual_override_ ? 1 : 0,
                 projection_mode_name(projection_mode_));
    std::fprintf(stderr, "projection calibration deg roll=%.3f pitch=%.3f yaw=%.3f\n",
                 rad_to_deg(fixed_rpy_[0]), rad_to_deg(fixed_rpy_[1]), rad_to_deg(fixed_rpy_[2]));
}

void CalibrationService::set_fixed_projection()
{
    projection_ = make_projection_state(projection_mode_, fixed_rpy_[0], fixed_rpy_[1], fixed_rpy_[2]);
}

void CalibrationService::input_rpy(float rpy[3]) const
{
    const float *src = can_apply_online() ? last_snapshot_.rpy : fixed_rpy_;
    rpy[0] = src[0];
    rpy[1] = src[1];
    rpy[2] = src[2];
}

const char *CalibrationService::mode_name(const OnlineCalibrator::Snapshot &snapshot) const
{
    if (!can_apply_online()) return "manual";
    return snapshot.status == CalibrationStatus::Calibrated
        ? "auto-pose-valid"
        : "auto-pose-shadow-unvalid";
}

void CalibrationService::update(const ParsedModelOutput &output)
{
    if (!can_apply_online()) return;
    if (!output.has_pose) return;

    const OnlineCalibrator::UpdateResult result = calibrator_.update(output.pose);
    last_snapshot_ = result.snapshot;

    if (result.snapshot.status == CalibrationStatus::Calibrated) {
        projection_ = make_projection_state(projection_mode_,
                                            result.snapshot.rpy[0],
                                            result.snapshot.rpy[1],
                                            result.snapshot.rpy[2]);
    } else if (result.snapshot.status == CalibrationStatus::Invalid) {
        set_fixed_projection();
    }

    maybe_log(result);
}

void CalibrationService::maybe_log(const OnlineCalibrator::UpdateResult &result)
{
    if (!log_enabled_) return;

    const auto now = std::chrono::steady_clock::now();
    const bool first = last_log_.time_since_epoch().count() == 0;
    const bool elapsed = first || now - last_log_ >= std::chrono::seconds(1);
    const bool status_changed = result.snapshot.status != last_status_ ||
        result.snapshot.valid_blocks != last_valid_blocks_;
    if (!elapsed && !result.block_completed && !status_changed) return;

    last_log_ = now;
    last_status_ = result.snapshot.status;
    last_valid_blocks_ = result.snapshot.valid_blocks;

    std::fprintf(stderr,
                 "\ncalib mode=%s status=%s accepted=%llu rejected=%llu "
                 "validBlocks=%d blockSamples=%d last=%s rpy_deg=(%.3f %.3f %.3f) "
                 "spread_deg=(%.3f %.3f %.3f)\n",
                 mode_name(result.snapshot),
                 calibration_status_name(result.snapshot.status),
                 static_cast<unsigned long long>(result.snapshot.accepted_samples),
                 static_cast<unsigned long long>(result.snapshot.rejected_samples),
                 result.snapshot.valid_blocks,
                 result.snapshot.block_sample_count,
                 result.accepted ? "accepted" : "rejected",
                 rad_to_deg(result.snapshot.rpy[0]),
                 rad_to_deg(result.snapshot.rpy[1]),
                 rad_to_deg(result.snapshot.rpy[2]),
                 rad_to_deg(result.snapshot.spread[0]),
                 rad_to_deg(result.snapshot.spread[1]),
                 rad_to_deg(result.snapshot.spread[2]));
}
