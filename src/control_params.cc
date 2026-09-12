#include "control_params.h"

#include "common_utils.h"
#include "json_utils.h"

#include <algorithm>

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

bool load_steering_params_json(const std::string &path,
                               SteeringParams *params,
                               std::string *error) {
  if (!params) return false;
  return load_json_param_file(path, [params](const std::string &text) {
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
    parse_json_optional_float(text, "angle_offset_deg", -10.0f, 10.0f, &params->angle_offset_deg);
    parse_json_optional_float(text, "torque_lat_accel_offset", -1.0f, 1.0f, &params->torque_lat_accel_offset);
    parse_json_optional_bool(text, "live_bank_compensation", &params->live_bank_compensation);
    parse_json_optional_float(text, "mass_kg", 1000.0f, 2600.0f, &params->mass_kg);
    parse_json_optional_float(text, "wheelbase_m", 2.0f, 3.5f, &params->wheelbase_m);
    parse_json_optional_float(text, "center_to_front_ratio", 0.2f, 0.7f, &params->center_to_front_ratio);
    parse_json_optional_float(text, "steer_ratio_rear", -0.5f, 0.5f, &params->steer_ratio_rear);
    parse_json_optional_float(text, "path_offset_m", -1.0f, 1.0f, &params->path_offset_m);
    parse_json_optional_float(text, "min_steer_speed_mps", 0.0f, 5.0f, &params->min_steer_speed_mps);
  }, error);
}

bool load_driving_params_json(const std::string &path,
                              DrivingParams *params,
                              std::string *error) {
  if (!params) return false;
  return load_json_param_file(path, [params](const std::string &text) {
    parse_json_optional_int(text, "model_timeout_ms", 50, 2000,
                       &params->model_timeout_ms);
    parse_json_optional_int(text, "vehicle_state_timeout_ms", 50, 2000,
                       &params->vehicle_state_timeout_ms);
    parse_json_optional_int(text, "inactive_release_ms", 0, 5000,
                       &params->inactive_release_ms);
    parse_json_optional_float(text, "mdps_speed_spoof_kph", 30.0f, 100.0f,
                         &params->mdps_speed_spoof_kph);
    parse_json_optional_float(text, "lane_change_min_speed_kph", 0.0f, 80.0f,
                         &params->lane_change_min_speed_kph);
    parse_json_optional_int(text, "driver_torque_threshold", 0, 500,
                       &params->driver_torque_threshold);
    parse_json_optional_bool(text, "laneless_mode", &params->laneless_mode);
  }, error);
}
