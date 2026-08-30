#include "openpilot_torque_controller.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common_utils.h"

namespace {

constexpr float kGravity = 9.8f;
constexpr float kDtCtrl = 0.01f;
constexpr float kFrictionThreshold = 0.2f;
// 저크 선행 마찰: 0.19s 앞의 요청 저크를 1.2Hz LP로 걸러 미리 반영한다.
constexpr int kJerkLookaheadFrames = 19;
constexpr float kJerkGain = 0.3f;
constexpr float kJerkFilterAlpha =
    1.0f / (1.0f + 1.0f / (2.0f * 3.14159265f * 1.2f * 0.01f));
constexpr float kStdCargoKg = 136.0f;
constexpr float kCivicMass = 1326.0f + kStdCargoKg;
constexpr float kCivicWheelbase = 2.70f;
constexpr float kCivicCenterToFront = kCivicWheelbase * 0.4f;
constexpr float kCivicCenterToRear = kCivicWheelbase - kCivicCenterToFront;
constexpr float kCivicTireStiffnessFront = 192150.0f;
constexpr float kCivicTireStiffnessRear = 202500.0f;

float apply_deadzone(float error, float deadzone) {
  if (error > deadzone) return error - deadzone;
  if (error < -deadzone) return error + deadzone;
  return 0.0f;
}

std::pair<float, float> scale_tire_stiffness(float mass, float wheelbase,
                                             float center_to_front,
                                             float tire_stiffness_factor) {
  const float center_to_rear = wheelbase - center_to_front;
  float front = kCivicTireStiffnessFront * tire_stiffness_factor * mass / kCivicMass;
  front *= (center_to_rear / wheelbase) / (kCivicCenterToRear / kCivicWheelbase);
  float rear = kCivicTireStiffnessRear * tire_stiffness_factor * mass / kCivicMass;
  rear *= (center_to_front / wheelbase) / (kCivicCenterToFront / kCivicWheelbase);
  return {front, rear};
}

}  // namespace

// PID와 saturation 상태를 초기화한다.
void OpenpilotTorqueController::reset() {
  p_ = 0.0f;
  i_ = 0.0f;
  f_ = 0.0f;
  normalized_output_ = 0.0f;
  error_ = 0.0f;
  feedforward_ = 0.0f;
  actual_curvature_ = 0.0f;
  // 요청 버퍼/저크 필터는 매 프레임 갱신되므로 비우지 않는다
}

// openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다.
int OpenpilotTorqueController::update(bool active,
                                      float speed_mps,
                                      float desired_curvature,
                                      float steering_angle_deg,
                                      bool steering_pressed,
                                      bool steering_rate_limited,
                                      const SteeringParams &params,
                                      float yaw_rate_rad_s,
                                      bool yaw_rate_valid,
                                      float road_bank_lat_accel) {
  const float actual_curvature = estimate_actual_curvature(
      speed_mps, steering_angle_deg, params, yaw_rate_rad_s, yaw_rate_valid);

  const float speed_sq = speed_mps * speed_mps;
  const float desired_lat_accel = desired_curvature * speed_sq;
  const float actual_lat_accel = actual_curvature * speed_sq;

  /* 버퍼/저크 필터는 active와 무관하게 매 프레임 갱신한다. inactive에서
   * 얼리면 재engage 첫 delay 구간에 낡은 요청과 비교되어 토크가 튄다. */
  request_head_ = (request_head_ + 1) % kRequestBufferLen;
  lat_accel_request_[request_head_] =
      std::isfinite(desired_lat_accel) ? desired_lat_accel : 0.0f;
  curvature_request_[request_head_] =
      std::isfinite(desired_curvature) ? desired_curvature : 0.0f;
  const int delay_frames = clamp_int(
      static_cast<int>(params.steer_actuator_delay / kDtCtrl) + 1,
      1, kRequestBufferLen);
  const auto at = [this](int back) {
    return (request_head_ - back + 2 * kRequestBufferLen) % kRequestBufferLen;
  };
  const float expected_lat_accel = lat_accel_request_[at(delay_frames - 1)];
  const float expected_curvature = curvature_request_[at(delay_frames - 1)];

  const int lookahead_back = clamp_int(delay_frames - 1 - kJerkLookaheadFrames,
                                       1, kRequestBufferLen - 2);
  const float raw_jerk = (lat_accel_request_[at(lookahead_back - 1)] -
                          lat_accel_request_[at(lookahead_back + 1)]) /
                         (2.0f * kDtCtrl);
  jerk_filtered_ += kJerkFilterAlpha * (raw_jerk - jerk_filtered_);

  if (!params.enabled || !active || speed_mps < params.min_steer_speed_mps) {
    reset();
    actual_curvature_ = actual_curvature;
    error_ = desired_curvature - actual_curvature;
    return 0;
  }

  const float curvature_deadzone = 0.0f;  // 조향각 deadzone 0 고정

  const float lat_accel_deadzone = curvature_deadzone * speed_sq;

  // openpilot LOW_SPEED_X/Y: 저속에서는 곡률 오차를 세게 반영하고 고속에서는
  // 줄여서 사행을 막는다. (구 포크의 500/500/200 평탄 곡선을 대체)
  const float low_speed_scale = interp(speed_mps, {0.0f, 10.0f, 20.0f, 30.0f},
                                       {15.0f, 13.0f, 10.0f, 5.0f});
  const float low_speed_factor = low_speed_scale * low_speed_scale;
  const float setpoint = expected_lat_accel + low_speed_factor * expected_curvature;
  const float measurement = actual_lat_accel + low_speed_factor * actual_curvature;
  const float error = setpoint - measurement;

  float feedforward = desired_lat_accel - params.roll_rad * kGravity;
  // 상수 편향(offset)은 FF에서 뺀다. bank = -g*sin(도로기울기)이므로
  // 중력의 횡가속 기여(-bank)를 빼려면 bank를 더한다 (2026-08-30 부호 수정)
  feedforward -= params.torque_lat_accel_offset;
  if (params.live_bank_compensation) feedforward += road_bank_lat_accel;
  const float friction = interp(
      apply_deadzone(error + kJerkGain * jerk_filtered_, lat_accel_deadzone),
      {-kFrictionThreshold, kFrictionThreshold},
      {-params.torque_friction(), params.torque_friction()});
  feedforward += friction / params.torque_kf();

  const bool freeze_integrator = steering_rate_limited || steering_pressed || speed_mps < 5.0f;
  const float pid_output = pid_update(error, feedforward, freeze_integrator, params);

  const int sign = params.torque_output_sign >= 0 ? 1 : -1;
  normalized_output_ = clamp_float(static_cast<float>(sign) * pid_output, -1.0f, 1.0f);
  error_ = error;
  feedforward_ = feedforward;
  actual_curvature_ = actual_curvature;
  return static_cast<int>(std::lround(normalized_output_ * static_cast<float>(params.steer_max)));
}

// 현재 조향각/속도에서 차량 모델 기반 실제 curvature를 추정한다.
float OpenpilotTorqueController::estimate_actual_curvature(float speed_mps,
                                                           float steering_angle_deg,
                                                           const SteeringParams &params,
                                                           float yaw_rate_rad_s,
                                                           bool yaw_rate_valid) {
  actual_curvature_vm_ = 0.0f;
  actual_curvature_yaw_ = 0.0f;
  if (!std::isfinite(speed_mps) || speed_mps < params.min_steer_speed_mps) return 0.0f;
  /* vm 경로의 부호 반전은 openpilot latcontrol_torque와 동일하다. */
  const float actual_curvature_vm = -vehicle_model_curvature(
      deg_to_rad(steering_angle_deg - params.angle_offset_deg),
      speed_mps,
      params.roll_rad,
      params);
  actual_curvature_vm_ = actual_curvature_vm;
  float actual_curvature_yaw = actual_curvature_vm;
  if (yaw_rate_valid && std::isfinite(yaw_rate_rad_s)) {
    /* K7 ESP12 YAW_RATE는 제어 관례와 부호가 반대라 반전한다. 2026-08-15
     * 실차 수동 주행의 커브 53샘플에서 curveVm/curveYaw 부호가 96% 반대,
     * 크기 비율 평균 1.15로 확인했다. */
    actual_curvature_yaw = -yaw_rate_rad_s / std::max(speed_mps, 1e-3f);
  }
  actual_curvature_yaw_ = actual_curvature_yaw;
  if (params.torque_use_angle) return actual_curvature_vm;
  return interp(speed_mps, {2.0f, 5.0f}, {actual_curvature_vm, actual_curvature_yaw});
}

// 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
void OpenpilotTorqueController::update_vehicle_model(const SteeringParams &params) {
  const float center_to_front = params.center_to_front_m();
  if (std::fabs(last_mass_kg_ - params.mass_kg) < 1e-3f &&
      std::fabs(last_wheelbase_m_ - params.wheelbase_m) < 1e-4f &&
      std::fabs(last_center_to_front_m_ - center_to_front) < 1e-4f &&
      std::fabs(last_tire_stiffness_factor_ - params.tire_stiffness_factor) < 1e-4f &&
      std::fabs(last_steer_ratio_ - params.steer_ratio) < 1e-4f &&
      std::fabs(last_steer_ratio_rear_ - params.steer_ratio_rear) < 1e-4f) {
    return;
  }
  const float center_to_rear = params.wheelbase_m - center_to_front;
  const auto stiffness = scale_tire_stiffness(
      params.mass_kg, params.wheelbase_m, center_to_front, params.tire_stiffness_factor);
  const float denominator =
      params.wheelbase_m * params.wheelbase_m * stiffness.first * stiffness.second;
  slip_factor_ = std::fabs(denominator) < 1e-9f
      ? 0.0f
      : params.mass_kg * (stiffness.first * center_to_front -
                          stiffness.second * center_to_rear) / denominator;
  inv_slip_factor_ = std::fabs(slip_factor_) < 1e-6f ? 0.0f : 1.0f / slip_factor_;
  last_mass_kg_ = params.mass_kg;
  last_wheelbase_m_ = params.wheelbase_m;
  last_center_to_front_m_ = center_to_front;
  last_tire_stiffness_factor_ = params.tire_stiffness_factor;
  last_steer_ratio_ = params.steer_ratio;
  last_steer_ratio_rear_ = params.steer_ratio_rear;
}

// 조향각과 속도에서 실제 curvature를 계산한다.
float OpenpilotTorqueController::vehicle_model_curvature(float steering_angle_rad,
                                                         float speed_mps,
                                                         float roll_rad,
                                                         const SteeringParams &params) {
  update_vehicle_model(params);
  float denom = 1.0f - slip_factor_ * speed_mps * speed_mps;
  if (std::fabs(denom) < 1e-6f) denom = denom >= 0.0f ? 1e-6f : -1e-6f;
  const float curvature_factor =
      (1.0f - params.steer_ratio_rear) / denom / params.wheelbase_m;
  float roll_comp = 0.0f;
  if (inv_slip_factor_ != 0.0f) {
    const float roll_denom = inv_slip_factor_ - speed_mps * speed_mps;
    if (std::fabs(roll_denom) >= 1e-6f) {
      roll_comp = kGravity * roll_rad / roll_denom;
    }
  }
  return curvature_factor * steering_angle_rad / params.steer_ratio + roll_comp;
}

// PID 한 스텝을 계산한다.
float OpenpilotTorqueController::pid_update(float error,
                                            float feedforward,
                                            bool freeze_integrator,
                                            const SteeringParams &params) {
  p_ = error * params.torque_kp();
  f_ = feedforward * params.torque_kf();
  const float next_i = i_ + error * params.torque_ki() * kDtCtrl;
  const float control_with_i = p_ + next_i + f_;
  if (((error >= 0.0f && (control_with_i <= 1.0f || next_i < 0.0f)) ||
       (error <= 0.0f && (control_with_i >= -1.0f || next_i > 0.0f))) &&
      !freeze_integrator) {
    i_ = next_i;
  }
  return clamp_float(p_ + i_ + f_, -1.0f, 1.0f);
}
