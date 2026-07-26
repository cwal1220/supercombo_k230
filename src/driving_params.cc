#include "driving_params.h"

#include "json_utils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace {

float clamp_float(float value, float lo, float hi) {
  if (!std::isfinite(value)) return lo;
  return std::min(std::max(value, lo), hi);
}

int clamp_int(float value, int lo, int hi) {
  return std::min(std::max(static_cast<int>(std::lround(value)), lo), hi);
}

void parse_optional_float(const std::string &text, const std::string &key,
                          float lo, float hi, float *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value))
    *field = clamp_float(value, lo, hi);
}

void parse_optional_int(const std::string &text, const std::string &key,
                        int lo, int hi, int *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value))
    *field = clamp_int(value, lo, hi);
}

}  // namespace

bool load_k7_driving_params_json(const std::string &path,
                                 K7DrivingParams *params,
                                 std::string *error) {
  if (!params) return false;
  std::ifstream file(path);
  if (!file.is_open()) {
    if (error) *error = "open failed";
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  try {
    parse_optional_int(text, "model_timeout_ms", 50, 2000,
                       &params->model_timeout_ms);
    parse_optional_int(text, "vehicle_state_timeout_ms", 50, 2000,
                       &params->vehicle_state_timeout_ms);
    parse_optional_int(text, "inactive_release_ms", 0, 5000,
                       &params->inactive_release_ms);
    parse_optional_float(text, "mdps_speed_spoof_kph", 30.0f, 100.0f,
                         &params->mdps_speed_spoof_kph);
    parse_optional_float(text, "lane_change_min_speed_kph", 0.0f, 80.0f,
                         &params->lane_change_min_speed_kph);
    parse_optional_int(text, "manual_steer_disable_frames", 0, 500,
                       &params->manual_steer_disable_frames);
    parse_optional_int(text, "driver_torque_threshold", 0, 500,
                       &params->driver_torque_threshold);
    parse_optional_float(text, "max_lateral_jerk", 0.1f, 20.0f,
                         &params->max_lateral_jerk);
    parse_optional_float(text, "max_lateral_accel", 0.5f, 5.0f,
                         &params->max_lateral_accel);
    parse_optional_float(text, "max_curvature", 0.01f, 0.5f,
                         &params->max_curvature);
  } catch (const std::exception &exc) {
    if (error) *error = exc.what();
    return false;
  }
  return true;
}
