#pragma once

#include <string>

struct DrivingParams {
  int model_timeout_ms = 250;
  int vehicle_state_timeout_ms = 500;
  int inactive_release_ms = 3000;
  float mdps_speed_spoof_kph = 60.0f;
  float lane_change_min_speed_kph = 30.0f;
  int manual_steer_disable_frames = 50;
  int driver_torque_threshold = 170;
};

bool load_driving_params_json(const std::string &path,
                              DrivingParams *params,
                              std::string *error);
