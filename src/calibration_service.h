#ifndef CALIBRATION_SERVICE_H
#define CALIBRATION_SERVICE_H

#include "app_config.h"
#include "model_output.h"
#include "online_calibrator.h"
#include "projection.h"

#include <chrono>

class CalibrationService {
public:
    explicit CalibrationService(const AppConfig &config);

    void update(const ParsedModelOutput &output);
    ProjectionState projection() const { return projection_; }
    OnlineCalibrator::Snapshot snapshot() const { return last_snapshot_; }
    void input_rpy(float rpy[3]) const;
    bool can_apply_online() const { return auto_enabled_ && !manual_override_; }

private:
    const char *mode_name(const OnlineCalibrator::Snapshot &snapshot) const;
    void maybe_log(const OnlineCalibrator::UpdateResult &result);
    void set_fixed_projection();

    ProjectionMode projection_mode_ = ProjectionMode::Legacy;
    bool auto_enabled_ = true;
    bool manual_override_ = false;
    bool log_enabled_ = false;
    float fixed_rpy_[3] = {};

    OnlineCalibrator calibrator_;
    OnlineCalibrator::Snapshot last_snapshot_{};
    ProjectionState projection_;

    std::chrono::steady_clock::time_point last_log_{};
    CalibrationStatus last_status_ = CalibrationStatus::Uncalibrated;
    int last_valid_blocks_ = -1;
};

#endif
