#include "driving_params.h"

#include "common_utils.h"
#include "json_utils.h"

#include <fstream>
#include <iterator>
#include <stdexcept>

bool load_driving_params_json(const std::string &path,
                              DrivingParams *params,
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
    parse_json_optional_int(text, "manual_steer_disable_frames", 0, 500,
                       &params->manual_steer_disable_frames);
    parse_json_optional_int(text, "driver_torque_threshold", 0, 500,
                       &params->driver_torque_threshold);
  } catch (const std::exception &exc) {
    if (error) *error = exc.what();
    return false;
  }
  return true;
}
