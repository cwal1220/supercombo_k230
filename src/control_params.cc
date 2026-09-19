#include "control_params.h"

#include "utils_json.h"

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

namespace {

constexpr JsonBoolField<SteeringParams> kSteeringBools[] = {
    {"enabled", &SteeringParams::enabled},
    {"torque_use_angle", &SteeringParams::torque_use_angle},
    {"avoid_lkas_fault_enabled", &SteeringParams::avoid_lkas_fault_enabled},
    {"live_bank_compensation", &SteeringParams::live_bank_compensation},
};
constexpr JsonIntField<SteeringParams> kSteeringInts[] = {
    {"steer_max", 0, 384, &SteeringParams::steer_max},
    {"steer_delta_up", 0, 20, &SteeringParams::steer_delta_up},
    {"steer_delta_down", 0, 30, &SteeringParams::steer_delta_down},
    {"steer_driver_allowance", 0, 300, &SteeringParams::steer_driver_allowance},
    {"steer_driver_multiplier", 0, 10, &SteeringParams::steer_driver_multiplier},
    {"steer_driver_factor", 0, 5, &SteeringParams::steer_driver_factor},
    {"steering_pressed_threshold", 0, 500, &SteeringParams::steering_pressed_threshold},
    {"torque_max_lat_accel_raw", 1, 80, &SteeringParams::torque_max_lat_accel_raw},
    {"torque_kp_raw", 0, 100, &SteeringParams::torque_kp_raw},
    {"torque_kf_raw", 0, 100, &SteeringParams::torque_kf_raw},
    {"torque_ki_raw", 0, 100, &SteeringParams::torque_ki_raw},
    {"torque_friction_raw", 0, 300, &SteeringParams::torque_friction_raw},
    {"torque_output_sign", -1, 1, &SteeringParams::torque_output_sign},
    {"avoid_lkas_fault_max_frames", 0, 300, &SteeringParams::avoid_lkas_fault_max_frames},
};
constexpr JsonFloatField<SteeringParams> kSteeringFloats[] = {
    {"steer_ratio", 8.0f, 25.0f, &SteeringParams::steer_ratio},
    {"tire_stiffness_factor", 0.2f, 2.0f, &SteeringParams::tire_stiffness_factor},
    {"steer_actuator_delay", 0.01f, 1.0f, &SteeringParams::steer_actuator_delay},
    {"max_steering_angle_deg", 0.0f, 360.0f, &SteeringParams::max_steering_angle_deg},
    {"avoid_lkas_fault_max_angle_deg", 1.0f, 180.0f, &SteeringParams::avoid_lkas_fault_max_angle_deg},
    {"angle_offset_deg", -10.0f, 10.0f, &SteeringParams::angle_offset_deg},
    {"torque_lat_accel_offset", -1.0f, 1.0f, &SteeringParams::torque_lat_accel_offset},
    {"mass_kg", 1000.0f, 2600.0f, &SteeringParams::mass_kg},
    {"wheelbase_m", 2.0f, 3.5f, &SteeringParams::wheelbase_m},
    {"center_to_front_ratio", 0.2f, 0.7f, &SteeringParams::center_to_front_ratio},
    {"steer_ratio_rear", -0.5f, 0.5f, &SteeringParams::steer_ratio_rear},
    {"path_offset_m", -1.0f, 1.0f, &SteeringParams::path_offset_m},
    {"min_steer_speed_mps", 0.0f, 5.0f, &SteeringParams::min_steer_speed_mps},
    {"torque_low_speed_gain", 0.1f, 1.0f, &SteeringParams::torque_low_speed_gain},
};

constexpr JsonBoolField<DrivingParams> kDrivingBools[] = {
    {"laneless_mode", &DrivingParams::laneless_mode},
};
constexpr JsonIntField<DrivingParams> kDrivingInts[] = {
    {"model_timeout_ms", 50, 2000, &DrivingParams::model_timeout_ms},
    {"vehicle_state_timeout_ms", 50, 2000, &DrivingParams::vehicle_state_timeout_ms},
    {"inactive_release_ms", 0, 5000, &DrivingParams::inactive_release_ms},
    {"driver_torque_threshold", 0, 500, &DrivingParams::driver_torque_threshold},
};
constexpr JsonFloatField<DrivingParams> kDrivingFloats[] = {
    {"mdps_speed_spoof_kph", 30.0f, 100.0f, &DrivingParams::mdps_speed_spoof_kph},
    {"lane_change_min_speed_kph", 0.0f, 80.0f, &DrivingParams::lane_change_min_speed_kph},
};

}  // namespace

bool load_steering_params_json(const std::string &path,
                               SteeringParams *params,
                               std::string *error) {
  if (!params) return false;
  return load_json_param_file(path, [params](const std::string &text) {
    parse_json_fields(text, kSteeringBools, params);
    parse_json_fields(text, kSteeringInts, params);
    parse_json_fields(text, kSteeringFloats, params);
    // 0은 허용하지 않는다: 부호는 +1 아니면 -1.
    params->torque_output_sign = params->torque_output_sign >= 0 ? 1 : -1;
  }, error);
}

bool load_driving_params_json(const std::string &path,
                              DrivingParams *params,
                              std::string *error) {
  if (!params) return false;
  return load_json_param_file(path, [params](const std::string &text) {
    parse_json_fields(text, kDrivingBools, params);
    parse_json_fields(text, kDrivingInts, params);
    parse_json_fields(text, kDrivingFloats, params);
  }, error);
}
