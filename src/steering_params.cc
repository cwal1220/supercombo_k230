#include "steering_params.h"

#include "common_utils.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "json_utils.h"

float SteeringParams::torque_max_lat_accel() const {
  return std::max(0.1f, static_cast<float>(torque_max_lat_accel_raw) * 0.1f);
}

float SteeringParams::torque_kp() const {
  return static_cast<float>(torque_kp_raw) * 0.1f / torque_max_lat_accel();
}

float SteeringParams::torque_kf() const {
  return std::max(1e-6f, static_cast<float>(torque_kf_raw) * 0.1f / torque_max_lat_accel());
}

float SteeringParams::torque_ki() const {
  return static_cast<float>(torque_ki_raw) * 0.1f / torque_max_lat_accel();
}

float SteeringParams::torque_friction() const {
  return static_cast<float>(torque_friction_raw) * 0.001f;
}

float SteeringParams::center_to_front_m() const {
  return wheelbase_m * center_to_front_ratio;
}

EffectiveSteerLimits SteeringParams::effective_steer_limits() const {
  EffectiveSteerLimits limits;
  limits.steer_max = steer_max;
  limits.steer_delta_up = steer_delta_up;
  limits.steer_delta_down = steer_delta_down;
  return limits;
}

HyundaiSteeringLimits SteeringParams::hyundai_limits(
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

// steering_params.json을 읽어 SteeringParams에 반영한다.
bool load_steering_params_json(const std::string &path,
                               SteeringParams *params,
                               std::string *error) {
  if (!params) return false;
  std::ifstream file(path);
  if (!file.is_open()) {
    if (error) *error = "open failed";
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  try {
    parse_json_optional_bool(text, "enabled", &params->enabled);
    parse_json_optional_int(text, "steer_max", 0, 384, &params->steer_max);
    parse_json_optional_int(text, "steer_delta_up", 0, 20, &params->steer_delta_up);
    parse_json_optional_int(text, "steer_delta_down", 0, 30, &params->steer_delta_down);
    parse_json_optional_int(text, "steer_driver_allowance", 0, 300, &params->steer_driver_allowance);
    parse_json_optional_int(text, "steer_driver_multiplier", 0, 10, &params->steer_driver_multiplier);
    parse_json_optional_int(text, "steer_driver_factor", 0, 5, &params->steer_driver_factor);
    parse_json_optional_int(text, "steering_pressed_threshold", 0, 500,
                       &params->steering_pressed_threshold);
    parse_json_optional_int(text, "torque_max_lat_accel_raw", 1, 80, &params->torque_max_lat_accel_raw);
    parse_json_optional_int(text, "torque_kp_raw", 0, 100, &params->torque_kp_raw);
    parse_json_optional_int(text, "torque_kf_raw", 0, 100, &params->torque_kf_raw);
    parse_json_optional_int(text, "torque_ki_raw", 0, 100, &params->torque_ki_raw);
    parse_json_optional_int(text, "torque_friction_raw", 0, 300, &params->torque_friction_raw);
    parse_json_optional_bool(text, "torque_use_angle", &params->torque_use_angle);
    parse_json_optional_int(text, "torque_output_sign", -1, 1, &params->torque_output_sign);
    params->torque_output_sign = params->torque_output_sign >= 0 ? 1 : -1;
    parse_json_optional_int(text, "smooth_steer_method", 0, 1, &params->smooth_steer_method);
    parse_json_optional_float(text, "smooth_max_steering_angle_deg", 0.0f, 180.0f,
                         &params->smooth_max_steering_angle_deg);
    parse_json_optional_float(text, "smooth_max_driver_angle_wait", 0.0f, 1.0f,
                         &params->smooth_max_driver_angle_wait);
    parse_json_optional_float(text, "smooth_max_steer_angle_wait", 0.0f, 1.0f,
                         &params->smooth_max_steer_angle_wait);
    parse_json_optional_float(text, "smooth_driver_angle_wait", 0.0f, 1.0f,
                         &params->smooth_driver_angle_wait);
    parse_json_optional_float(text, "steer_ratio", 8.0f, 25.0f, &params->steer_ratio);
    parse_json_optional_float(text, "tire_stiffness_factor", 0.2f, 2.0f, &params->tire_stiffness_factor);
    parse_json_optional_float(text, "steer_actuator_delay", 0.01f, 1.0f, &params->steer_actuator_delay);
    parse_json_optional_float(text, "max_steering_angle_deg", 0.0f, 360.0f,
                         &params->max_steering_angle_deg);
    parse_json_optional_bool(text, "avoid_lkas_fault_enabled", &params->avoid_lkas_fault_enabled);
    parse_json_optional_float(text, "avoid_lkas_fault_max_angle_deg", 1.0f, 180.0f,
                         &params->avoid_lkas_fault_max_angle_deg);
    parse_json_optional_int(text, "avoid_lkas_fault_max_frames", 0, 300,
                       &params->avoid_lkas_fault_max_frames);
    parse_json_optional_bool(text, "no_smart_mdps", &params->no_smart_mdps);
    parse_json_optional_bool(text, "turn_steering_disable", &params->turn_steering_disable);
    parse_json_optional_float(text, "angle_offset_deg", -10.0f, 10.0f, &params->angle_offset_deg);
    parse_json_optional_float(text, "roll_rad", -0.2f, 0.2f, &params->roll_rad);
    parse_json_optional_float(text, "torque_lat_accel_offset", -1.0f, 1.0f, &params->torque_lat_accel_offset);
    parse_json_optional_bool(text, "live_bank_compensation", &params->live_bank_compensation);
    parse_json_optional_float(text, "mass_kg", 1000.0f, 2600.0f, &params->mass_kg);
    parse_json_optional_float(text, "wheelbase_m", 2.0f, 3.5f, &params->wheelbase_m);
    parse_json_optional_float(text, "center_to_front_ratio", 0.2f, 0.7f, &params->center_to_front_ratio);
    parse_json_optional_float(text, "steer_ratio_rear", -0.5f, 0.5f, &params->steer_ratio_rear);
    parse_json_optional_float(text, "camera_offset_m", -1.0f, 1.0f, &params->camera_offset_m);
    parse_json_optional_float(text, "path_offset_m", -1.0f, 1.0f, &params->path_offset_m);
    parse_json_optional_float(text, "min_steer_speed_mps", 0.0f, 5.0f, &params->min_steer_speed_mps);
  } catch (const std::exception &exc) {
    if (error) *error = exc.what();
    return false;
  }
  return true;
}
