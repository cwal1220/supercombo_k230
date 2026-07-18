#include "steering_params.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "json_utils.h"

namespace {

constexpr float kGravity = 9.8f;
constexpr float kDtCtrl = 0.01f;
constexpr float kFrictionThreshold = 0.2f;
constexpr float kStdCargoKg = 136.0f;
constexpr float kCivicMass = 1326.0f + kStdCargoKg;
constexpr float kCivicWheelbase = 2.70f;
constexpr float kCivicCenterToFront = kCivicWheelbase * 0.4f;
constexpr float kCivicCenterToRear = kCivicWheelbase - kCivicCenterToFront;
constexpr float kCivicTireStiffnessFront = 192150.0f;
constexpr float kCivicTireStiffnessRear = 202500.0f;

float clamp_float(float value, float lo, float hi) {
  if (!std::isfinite(value)) return lo;
  return std::min(std::max(value, lo), hi);
}

int clamp_int(int value, int lo, int hi) {
  return std::min(std::max(value, lo), hi);
}

float interp(float x, std::initializer_list<float> xp, std::initializer_list<float> fp) {
  const auto xs = xp.begin();
  const auto fs = fp.begin();
  const size_t n = xp.size();
  if (n == 0 || fp.size() != n) return 0.0f;
  if (x <= xs[0]) return fs[0];
  if (x >= xs[n - 1]) return fs[n - 1];
  for (size_t idx = 1; idx < n; ++idx) {
    if (x <= xs[idx]) {
      const float low_x = xs[idx - 1];
      const float high_x = xs[idx];
      const float low_y = fs[idx - 1];
      const float high_y = fs[idx];
      if (std::fabs(high_x - low_x) < 1e-6f) return high_y;
      return low_y + (x - low_x) * (high_y - low_y) / (high_x - low_x);
    }
  }
  return fs[n - 1];
}

float apply_deadzone(float error, float deadzone) {
  if (error > deadzone) return error - deadzone;
  if (error < -deadzone) return error + deadzone;
  return 0.0f;
}

float radians(float deg) {
  return deg * 0.017453292519943295f;
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

void parse_optional_bool(const std::string &text, const std::string &key, bool *field) {
  bool value = false;
  if (parse_json_bool_value(text, key, &value)) *field = value;
}

void parse_optional_float(const std::string &text, const std::string &key,
                          float lo, float hi, float *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value)) {
    *field = clamp_float(value, lo, hi);
  }
}

void parse_optional_int(const std::string &text, const std::string &key,
                        int lo, int hi, int *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value)) {
    *field = clamp_int(static_cast<int>(std::lround(value)), lo, hi);
  }
}

}  // namespace

float K7SteeringParams::torque_max_lat_accel() const {
  return std::max(0.1f, static_cast<float>(torque_max_lat_accel_raw) * 0.1f);
}

float K7SteeringParams::torque_kp() const {
  return static_cast<float>(torque_kp_raw) * 0.1f / torque_max_lat_accel();
}

float K7SteeringParams::torque_kf() const {
  return std::max(1e-6f, static_cast<float>(torque_kf_raw) * 0.1f / torque_max_lat_accel());
}

float K7SteeringParams::torque_ki() const {
  return static_cast<float>(torque_ki_raw) * 0.1f / torque_max_lat_accel();
}

float K7SteeringParams::torque_friction() const {
  return static_cast<float>(torque_friction_raw) * 0.001f;
}

float K7SteeringParams::steering_angle_deadzone_deg() const {
  return static_cast<float>(torque_angle_deadzone_raw) * 0.1f;
}

float K7SteeringParams::center_to_front_m() const {
  return wheelbase_m * center_to_front_ratio;
}

EffectiveSteerLimits K7SteeringParams::effective_steer_limits(
    float speed_kph, float steering_angle_deg, float speed_mps, bool driver_guard) const {
  const float speed = std::isfinite(speed_kph) ? std::fabs(speed_kph) : 0.0f;
  const float ego_speed_mps =
      speed_mps >= 0.0f && std::isfinite(speed_mps) ? std::fabs(speed_mps) : speed / 3.6f;
  const bool steer_more_active =
      avoid_lkas_fault_beyond &&
      avoid_lkas_fault_enabled &&
      std::fabs(steering_angle_deg) > avoid_lkas_fault_max_angle_deg * 0.5f &&
      ego_speed_mps <= 12.5f &&
      !driver_guard;
  float next_steer_max = 0.0f;
  float next_delta_up = 0.0f;
  float next_delta_down = 0.0f;
  if (steer_more_active) {
    next_steer_max = static_cast<float>(steer_max);
    next_delta_up = static_cast<float>(steer_delta_up);
    next_delta_down = static_cast<float>(steer_delta_down);
  } else if (ego_speed_mps > 8.3f) {
    next_steer_max = variable_steer_max
        ? interp(speed, {30.0f, 100.0f, 255.0f},
                 {static_cast<float>(steer_max), static_cast<float>(steer_max_base),
                  static_cast<float>(steer_max_base)})
        : static_cast<float>(steer_max_base);
    next_delta_up = variable_steer_delta
        ? interp(speed, {30.0f, 100.0f, 255.0f},
                 {static_cast<float>(steer_delta_up), static_cast<float>(steer_delta_up_base),
                  static_cast<float>(steer_delta_up_base)})
        : static_cast<float>(steer_delta_up_base);
    next_delta_down = variable_steer_delta
        ? interp(speed, {30.0f, 100.0f, 255.0f},
                 {static_cast<float>(steer_delta_down), static_cast<float>(steer_delta_down_base),
                  static_cast<float>(steer_delta_down_base)})
        : static_cast<float>(steer_delta_down_base);
  } else {
    next_steer_max = static_cast<float>(steer_max_base);
    next_delta_up = static_cast<float>(steer_delta_up_base);
    next_delta_down = static_cast<float>(steer_delta_down_base);
  }

  EffectiveSteerLimits limits;
  limits.steer_max = clamp_int(static_cast<int>(std::lround(next_steer_max)), 0, steer_max);
  limits.steer_delta_up = clamp_int(static_cast<int>(std::lround(next_delta_up)), 0,
                                   std::max(steer_delta_up, steer_delta_up_base));
  limits.steer_delta_down = clamp_int(static_cast<int>(std::lround(next_delta_down)), 0,
                                     std::max(steer_delta_down, steer_delta_down_base));
  limits.variable_steer_max = variable_steer_max;
  limits.variable_steer_delta = variable_steer_delta;
  limits.steer_more_active = steer_more_active;
  limits.model_speed_kph = speed;
  return limits;
}

HyundaiSteeringLimits K7SteeringParams::hyundai_limits(
    const EffectiveSteerLimits &limits) const {
  HyundaiSteeringLimits out;
  out.steer_max = limits.steer_max;
  out.steer_delta_up = limits.steer_delta_up;
  out.steer_delta_down = limits.steer_delta_down;
  out.steer_driver_allowance = steer_driver_allowance;
  out.steer_driver_multiplier = steer_driver_multiplier;
  out.steer_driver_factor = steer_driver_factor;
  return out;
}

// PID와 saturation 상태를 초기화한다.
void OpenpilotTorqueController::reset() {
  p_ = 0.0f;
  i_ = 0.0f;
  f_ = 0.0f;
  sat_count_ = 0.0f;
  normalized_output_ = 0.0f;
  error_ = 0.0f;
  feedforward_ = 0.0f;
  actual_curvature_ = 0.0f;
  saturated_ = false;
}

// openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다.
int OpenpilotTorqueController::update(bool active,
                                      float speed_mps,
                                      float desired_curvature,
                                      float steering_angle_deg,
                                      bool steering_pressed,
                                      bool steering_rate_limited,
                                      const K7SteeringParams &params,
                                      float yaw_rate_rad_s,
                                      bool yaw_rate_valid) {
  const float actual_curvature = estimate_actual_curvature(
      speed_mps, steering_angle_deg, params, yaw_rate_rad_s, yaw_rate_valid);
  if (!params.enabled || !active || speed_mps < params.min_steer_speed_mps) {
    reset();
    actual_curvature_ = actual_curvature;
    error_ = desired_curvature - actual_curvature;
    return 0;
  }

  float curvature_deadzone = 0.0f;
  if (params.torque_use_angle) {
    curvature_deadzone = std::fabs(vehicle_model_curvature(
        radians(params.steering_angle_deadzone_deg()), speed_mps, 0.0f, params));
  }

  const float speed_sq = speed_mps * speed_mps;
  const float desired_lat_accel = desired_curvature * speed_sq;
  const float actual_lat_accel = actual_curvature * speed_sq;
  const float lat_accel_deadzone = curvature_deadzone * speed_sq;
  const float low_speed_factor = interp(speed_mps, {0.0f, 10.0f, 20.0f},
                                        {500.0f, 500.0f, 200.0f});
  const float setpoint = desired_lat_accel + low_speed_factor * desired_curvature;
  const float measurement = actual_lat_accel + low_speed_factor * actual_curvature;
  const float error = setpoint - measurement;
  float feedforward = desired_lat_accel - params.roll_rad * kGravity;
  const float friction = interp(apply_deadzone(error, lat_accel_deadzone),
                                {-kFrictionThreshold, kFrictionThreshold},
                                {-params.torque_friction(), params.torque_friction()});
  feedforward += friction / params.torque_kf();

  const bool freeze_integrator = steering_rate_limited || steering_pressed || speed_mps < 5.0f;
  const float pid_output = pid_update(error, feedforward, freeze_integrator, params);
  if (1.0f - std::fabs(pid_output) < 1e-3f &&
      speed_mps > 10.0f && !steering_rate_limited && !steering_pressed) {
    sat_count_ += kDtCtrl;
  } else {
    sat_count_ -= kDtCtrl;
  }
  sat_count_ = clamp_float(sat_count_, 0.0f, params.steer_limit_timer);
  saturated_ = sat_count_ > params.steer_limit_timer - 1e-3f;

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
                                                           const K7SteeringParams &params,
                                                           float yaw_rate_rad_s,
                                                           bool yaw_rate_valid) {
  if (!std::isfinite(speed_mps) || speed_mps < params.min_steer_speed_mps) return 0.0f;
  const float actual_curvature_vm = -vehicle_model_curvature(
      radians(steering_angle_deg - params.angle_offset_deg),
      speed_mps,
      params.roll_rad,
      params);
  if (params.torque_use_angle) return actual_curvature_vm;
  float actual_curvature_yaw = actual_curvature_vm;
  if (yaw_rate_valid && std::isfinite(yaw_rate_rad_s)) {
    actual_curvature_yaw = yaw_rate_rad_s / std::max(speed_mps, 1e-3f);
  }
  return interp(speed_mps, {2.0f, 5.0f}, {actual_curvature_vm, actual_curvature_yaw});
}

// 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
void OpenpilotTorqueController::update_vehicle_model(const K7SteeringParams &params) {
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
                                                         const K7SteeringParams &params) {
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
                                            const K7SteeringParams &params) {
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

// steering_params.json을 읽어 K7SteeringParams에 반영한다.
bool load_k7_steering_params_json(const std::string &path,
                                  K7SteeringParams *params,
                                  std::string *error) {
  if (!params) return false;
  std::ifstream file(path);
  if (!file.is_open()) {
    if (error) *error = "open failed";
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  try {
    parse_optional_bool(text, "enabled", &params->enabled);
    parse_optional_int(text, "steer_max", 0, 384, &params->steer_max);
    parse_optional_int(text, "steer_delta_up", 0, 20, &params->steer_delta_up);
    parse_optional_int(text, "steer_delta_down", 0, 30, &params->steer_delta_down);
    parse_optional_int(text, "steer_driver_allowance", 0, 300, &params->steer_driver_allowance);
    parse_optional_int(text, "steer_driver_multiplier", 0, 10, &params->steer_driver_multiplier);
    parse_optional_int(text, "steer_driver_factor", 0, 5, &params->steer_driver_factor);
    parse_optional_bool(text, "variable_steer_max", &params->variable_steer_max);
    parse_optional_bool(text, "variable_steer_delta", &params->variable_steer_delta);
    parse_optional_int(text, "steer_max_base", 0, 384, &params->steer_max_base);
    parse_optional_int(text, "steer_delta_up_base", 0, 20, &params->steer_delta_up_base);
    parse_optional_int(text, "steer_delta_down_base", 0, 30, &params->steer_delta_down_base);
    parse_optional_int(text, "torque_max_lat_accel_raw", 1, 80, &params->torque_max_lat_accel_raw);
    parse_optional_int(text, "torque_kp_raw", 0, 100, &params->torque_kp_raw);
    parse_optional_int(text, "torque_kf_raw", 0, 100, &params->torque_kf_raw);
    parse_optional_int(text, "torque_ki_raw", 0, 100, &params->torque_ki_raw);
    parse_optional_int(text, "torque_friction_raw", 0, 300, &params->torque_friction_raw);
    parse_optional_bool(text, "torque_use_angle", &params->torque_use_angle);
    parse_optional_int(text, "torque_angle_deadzone_raw", 0, 50, &params->torque_angle_deadzone_raw);
    parse_optional_int(text, "torque_output_sign", -1, 1, &params->torque_output_sign);
    params->torque_output_sign = params->torque_output_sign >= 0 ? 1 : -1;
    parse_optional_int(text, "smooth_steer_method", 0, 1, &params->smooth_steer_method);
    parse_optional_float(text, "smooth_max_steering_angle_deg", 0.0f, 180.0f,
                         &params->smooth_max_steering_angle_deg);
    parse_optional_float(text, "smooth_max_driver_angle_wait", 0.0f, 1.0f,
                         &params->smooth_max_driver_angle_wait);
    parse_optional_float(text, "smooth_max_steer_angle_wait", 0.0f, 1.0f,
                         &params->smooth_max_steer_angle_wait);
    parse_optional_float(text, "smooth_driver_angle_wait", 0.0f, 1.0f,
                         &params->smooth_driver_angle_wait);
    parse_optional_float(text, "steer_ratio", 8.0f, 25.0f, &params->steer_ratio);
    parse_optional_float(text, "tire_stiffness_factor", 0.2f, 2.0f, &params->tire_stiffness_factor);
    parse_optional_float(text, "steer_actuator_delay", 0.01f, 1.0f, &params->steer_actuator_delay);
    parse_optional_float(text, "steer_limit_timer", 0.1f, 3.0f, &params->steer_limit_timer);
    parse_optional_float(text, "max_steering_angle_deg", 0.0f, 360.0f,
                         &params->max_steering_angle_deg);
    parse_optional_bool(text, "avoid_lkas_fault_enabled", &params->avoid_lkas_fault_enabled);
    parse_optional_float(text, "avoid_lkas_fault_max_angle_deg", 1.0f, 180.0f,
                         &params->avoid_lkas_fault_max_angle_deg);
    parse_optional_int(text, "avoid_lkas_fault_max_frames", 0, 300,
                       &params->avoid_lkas_fault_max_frames);
    parse_optional_bool(text, "avoid_lkas_fault_beyond", &params->avoid_lkas_fault_beyond);
    parse_optional_bool(text, "no_smart_mdps", &params->no_smart_mdps);
    parse_optional_bool(text, "turn_steering_disable", &params->turn_steering_disable);
    parse_optional_bool(text, "ldws_car_fix", &params->ldws_car_fix);
    parse_optional_float(text, "angle_offset_deg", -10.0f, 10.0f, &params->angle_offset_deg);
    parse_optional_float(text, "roll_rad", -0.2f, 0.2f, &params->roll_rad);
    parse_optional_float(text, "mass_kg", 1000.0f, 2600.0f, &params->mass_kg);
    parse_optional_float(text, "wheelbase_m", 2.0f, 3.5f, &params->wheelbase_m);
    parse_optional_float(text, "center_to_front_ratio", 0.2f, 0.7f, &params->center_to_front_ratio);
    parse_optional_float(text, "steer_ratio_rear", -0.5f, 0.5f, &params->steer_ratio_rear);
    parse_optional_bool(text, "invert_steer", &params->invert_steer);
    parse_optional_float(text, "min_steer_speed_mps", 0.0f, 5.0f, &params->min_steer_speed_mps);
  } catch (const std::exception &exc) {
    if (error) *error = exc.what();
    return false;
  }
  return true;
}
