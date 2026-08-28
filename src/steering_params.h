#pragma once

#include "hyundai_steering.h"

#include <string>

struct EffectiveSteerLimits {
  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
};

struct SteeringParams {
  bool enabled = true;

  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  int steer_driver_allowance = 50;
  int steer_driver_multiplier = 2;
  int steer_driver_factor = 1;
  int steering_pressed_threshold = 150;

  int torque_max_lat_accel_raw = 22;
  int torque_kp_raw = 10;
  int torque_kf_raw = 10;
  int torque_ki_raw = 1;
  int torque_friction_raw = 100;
  bool torque_use_angle = true;
  int torque_output_sign = -1;

  int smooth_steer_method = 0;
  float smooth_max_steering_angle_deg = 90.0f;
  float smooth_max_driver_angle_wait = 0.002f;
  float smooth_max_steer_angle_wait = 0.001f;
  float smooth_driver_angle_wait = 0.001f;

  float steer_ratio = 16.8f;
  float tire_stiffness_factor = 1.0f;
  float steer_actuator_delay = 0.46f;
  float max_steering_angle_deg = 90.0f;
  bool avoid_lkas_fault_enabled = true;
  float avoid_lkas_fault_max_angle_deg = 85.0f;
  int avoid_lkas_fault_max_frames = 89;
  bool no_smart_mdps = false;
  bool turn_steering_disable = false;
  float angle_offset_deg = 0.0f;
  float roll_rad = 0.0f;
  /* openpilot latAccelOffset(m/s^2). 상수 횡가속 편향을 FF에서 뺀다.
   * +y=오른쪽 관례라 양수 = 우측 쏠림 보정. fit 도구 출력을 그대로 넣는다. */
  float torque_lat_accel_offset = 0.0f;
  /* ESP12 실측으로 추정한 도로 편경사(뱅크)를 FF에서 실시간 보정한다.
   * 켜면 상수 offset이 커버 못 하는 커브별 편경사까지 잡는다. */
  bool live_bank_compensation = true;
  float mass_kg = 1816.0f;
  float wheelbase_m = 2.855f;
  float center_to_front_ratio = 0.4f;
  float steer_ratio_rear = 0.0f;
  float camera_offset_m = -0.06f;
  float path_offset_m = 0.0f;
  float min_steer_speed_mps = 0.3f;

  float torque_max_lat_accel() const;
  float torque_kp() const;
  float torque_kf() const;
  float torque_ki() const;
  float torque_friction() const;
  float center_to_front_m() const;
  EffectiveSteerLimits effective_steer_limits() const;
  HyundaiSteeringLimits hyundai_limits(const EffectiveSteerLimits &limits) const;
};

// steering_params.json을 읽어 SteeringParams에 반영한다.
bool load_steering_params_json(const std::string &path,
                               SteeringParams *params,
                               std::string *error);
