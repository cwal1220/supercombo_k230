#pragma once

#include "hyundai_steering.h"

#include <string>

struct EffectiveSteerLimits {
  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  bool variable_steer_max = false;
  bool variable_steer_delta = false;
  bool steer_more_active = false;
  float model_speed_kph = 0.0f;
};

struct K7SteeringParams {
  bool enabled = true;

  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  int steer_driver_allowance = 50;
  int steer_driver_multiplier = 2;
  int steer_driver_factor = 1;
  bool variable_steer_max = false;
  bool variable_steer_delta = false;
  int steer_max_base = 384;
  int steer_delta_up_base = 3;
  int steer_delta_down_base = 7;

  int torque_max_lat_accel_raw = 27;
  int torque_kp_raw = 10;
  int torque_kf_raw = 10;
  int torque_ki_raw = 1;
  int torque_friction_raw = 65;
  bool torque_use_angle = true;
  int torque_angle_deadzone_raw = 10;
  int torque_output_sign = -1;

  int smooth_steer_method = 0;
  float smooth_max_steering_angle_deg = 90.0f;
  float smooth_max_driver_angle_wait = 0.002f;
  float smooth_max_steer_angle_wait = 0.001f;
  float smooth_driver_angle_wait = 0.001f;

  float steer_ratio = 15.5f;
  float tire_stiffness_factor = 0.85f;
  float steer_actuator_delay = 0.36f;
  float steer_limit_timer = 1.0f;
  float max_steering_angle_deg = 90.0f;
  bool avoid_lkas_fault_enabled = false;
  float avoid_lkas_fault_max_angle_deg = 85.0f;
  int avoid_lkas_fault_max_frames = 90;
  bool avoid_lkas_fault_beyond = false;
  bool no_smart_mdps = false;
  bool turn_steering_disable = false;
  bool ldws_car_fix = false;
  float angle_offset_deg = 0.8f;
  float roll_rad = 0.0f;
  float mass_kg = 1816.0f;
  float wheelbase_m = 2.855f;
  float center_to_front_ratio = 0.4f;
  float steer_ratio_rear = 0.0f;
  bool invert_steer = false;
  float min_steer_speed_mps = 0.3f;

  float torque_max_lat_accel() const;
  float torque_kp() const;
  float torque_kf() const;
  float torque_ki() const;
  float torque_friction() const;
  float steering_angle_deadzone_deg() const;
  float center_to_front_m() const;
  EffectiveSteerLimits effective_steer_limits(float speed_kph,
                                              float steering_angle_deg = 0.0f,
                                              float speed_mps = -1.0f,
                                              bool driver_guard = false) const;
  HyundaiSteeringLimits hyundai_limits(const EffectiveSteerLimits &limits) const;
};

class OpenpilotTorqueController {
public:
  // PID와 saturation 상태를 초기화한다.
  void reset();

  // openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다.
  int update(bool active,
             float speed_mps,
             float desired_curvature,
             float steering_angle_deg,
             bool steering_pressed,
             bool steering_rate_limited,
             const K7SteeringParams &params,
             float yaw_rate_rad_s = 0.0f,
             bool yaw_rate_valid = false);

  // 현재 조향각/속도에서 차량 모델 기반 실제 curvature를 추정한다.
  float estimate_actual_curvature(float speed_mps,
                                  float steering_angle_deg,
                                  const K7SteeringParams &params,
                                  float yaw_rate_rad_s = 0.0f,
                                  bool yaw_rate_valid = false);

  float normalized_output() const { return normalized_output_; }
  float error() const { return error_; }
  float feedforward() const { return feedforward_; }
  float actual_curvature() const { return actual_curvature_; }
  bool saturated() const { return saturated_; }

private:
  // 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
  void update_vehicle_model(const K7SteeringParams &params);

  // 조향각과 속도에서 실제 curvature를 계산한다.
  float vehicle_model_curvature(float steering_angle_rad,
                                float speed_mps,
                                float roll_rad,
                                const K7SteeringParams &params);

  // PID 한 스텝을 계산한다.
  float pid_update(float error,
                   float feedforward,
                   bool freeze_integrator,
                   const K7SteeringParams &params);

  float p_ = 0.0f;
  float i_ = 0.0f;
  float f_ = 0.0f;
  float sat_count_ = 0.0f;
  float slip_factor_ = 0.0f;
  float inv_slip_factor_ = 0.0f;
  float last_mass_kg_ = -1.0f;
  float last_wheelbase_m_ = -1.0f;
  float last_center_to_front_m_ = -1.0f;
  float last_tire_stiffness_factor_ = -1.0f;
  float last_steer_ratio_ = -1.0f;
  float last_steer_ratio_rear_ = -1.0f;
  float normalized_output_ = 0.0f;
  float error_ = 0.0f;
  float feedforward_ = 0.0f;
  float actual_curvature_ = 0.0f;
  bool saturated_ = false;
};

// steering_params.json을 읽어 K7SteeringParams에 반영한다.
bool load_k7_steering_params_json(const std::string &path,
                                  K7SteeringParams *params,
                                  std::string *error);
