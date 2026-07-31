#pragma once

#include <string>

struct K7DrivingParams {
  int model_timeout_ms = 250;
  int vehicle_state_timeout_ms = 500;
  int inactive_release_ms = 3000;
  float mdps_speed_spoof_kph = 60.0f;
  float lane_change_min_speed_kph = 30.0f;
  int manual_steer_disable_frames = 50;
  int driver_torque_threshold = 170;
  float max_lateral_jerk = 5.0f;
  float max_lateral_accel = 3.3f;
};

bool load_k7_driving_params_json(const std::string &path,
                                 K7DrivingParams *params,
                                 std::string *error);
