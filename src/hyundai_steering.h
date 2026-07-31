#pragma once

struct HyundaiSteeringLimits {
  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  int steer_driver_allowance = 50;
  int steer_driver_multiplier = 2;
  int steer_driver_factor = 1;
};

int apply_hyundai_steer_torque_limits(int desired_torque, int last_torque, int driver_torque,
                                      const HyundaiSteeringLimits &limits = HyundaiSteeringLimits{});

float mdps_speed_for_lkas(float cluster_speed, bool lkas_active, bool is_mph,
                          float spoof_speed_kph = 60.0f);
