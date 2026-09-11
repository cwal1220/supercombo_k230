#pragma once

#include <string>

/* k230_controlsd가 함께 읽는 런타임 파라미터. params/steering.json과
 * params/driving.json이 각각의 출처다. CAN 계층은 이 헤더에 의존하지만
 * 반대 방향은 없다 — 토크 제한 변환은 hyundai_can.h가 제공한다. */

/* 기본값은 params/steering.json(K7 YG HEV 실차 검증값)과 일치시킨다. 로드 실패는
 * controlsd가 throw하므로 이 값은 파일 폴백이 아니라, JSON에 키가 빠졌을 때와
 * 호스트 체크·리플레이 도구가 실제로 쓰는 값이다. */
struct SteeringParams {
  bool enabled = true;

  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  int steer_driver_allowance = 50;
  int steer_driver_multiplier = 2;
  int steer_driver_factor = 1;
  int steering_pressed_threshold = 150;

  int torque_max_lat_accel_raw = 40;
  int torque_kp_raw = 20;
  int torque_kf_raw = 9;
  int torque_ki_raw = 3;
  int torque_friction_raw = 100;
  bool torque_use_angle = true;
  int torque_output_sign = -1;

  float steer_ratio = 16.8f;
  float tire_stiffness_factor = 1.0f;
  float steer_actuator_delay = 0.34f;
  float max_steering_angle_deg = 90.0f;
  bool avoid_lkas_fault_enabled = true;
  float avoid_lkas_fault_max_angle_deg = 85.0f;
  int avoid_lkas_fault_max_frames = 89;
  float angle_offset_deg = -0.7f;
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
  float path_offset_m = 0.0f;
  float min_steer_speed_mps = 1.0f;

  float torque_max_lat_accel() const;
  float torque_kp() const;
  float torque_kf() const;
  float torque_ki() const;
  float torque_friction() const;
  float center_to_front_m() const;
};

struct DrivingParams {
  int model_timeout_ms = 250;
  int vehicle_state_timeout_ms = 500;
  int inactive_release_ms = 3000;
  float mdps_speed_spoof_kph = 60.0f;
  float lane_change_min_speed_kph = 30.0f;
  int driver_torque_threshold = 170;
};

// params/steering.json을 읽어 SteeringParams에 반영한다.
bool load_steering_params_json(const std::string &path,
                               SteeringParams *params,
                               std::string *error);

// params/driving.json을 읽어 DrivingParams에 반영한다.
bool load_driving_params_json(const std::string &path,
                              DrivingParams *params,
                              std::string *error);
