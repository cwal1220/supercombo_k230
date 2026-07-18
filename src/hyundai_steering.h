#pragma once

struct HyundaiSteeringLimits {
  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  int steer_driver_allowance = 50;
  int steer_driver_multiplier = 2;
  int steer_driver_factor = 1;
};

struct SteeringGateInput {
  bool path_usable = false;
  bool engaged = false;
  bool cruise_active = false;
  bool panda_ready = false;
  bool steering_fault = false;
  bool blinker_on = false;
  bool no_smart_mdps = false;
  float v_ego_mps = 0.0f;
  float min_enable_speed_mps = 0.0f;
};

int apply_hyundai_steer_torque_limits(int desired_torque, int last_torque, int driver_torque,
                                      const HyundaiSteeringLimits &limits = HyundaiSteeringLimits{});

bool steering_gate_allows(const SteeringGateInput &input);
float mdps_speed_for_lkas(float cluster_speed, bool lkas_active, bool is_mph);
