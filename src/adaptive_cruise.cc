#include "adaptive_cruise.h"

#include "common_utils.h"
#include "json_utils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace {

constexpr double kAcceleratorReleaseDelayS = 0.5;

constexpr int kCruiseButtonResume = 1;
constexpr int kCruiseButtonSet = 2;
constexpr float kMphToKph = 1.609344f;
constexpr float kDisplayStep = 2.0f;
constexpr float kMinimumSpeedKph = 30.0f;
constexpr float kMinimumSpeedMph = 20.0f;

bool valid_set_speed(float speed_kph) {
  return std::isfinite(speed_kph) && speed_kph > 0.0f && speed_kph < 300.0f;
}

void parse_optional_float(const std::string &text, const std::string &key,
                          float minimum, float maximum, float *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value))
    *field = clamp_float(value, minimum, maximum);
}

void parse_optional_int(const std::string &text, const std::string &key,
                        int minimum, int maximum, int *field) {
  float value = 0.0f;
  if (parse_json_float_value(text, key, &value)) {
    *field = std::clamp(static_cast<int>(std::lround(value)), minimum, maximum);
  }
}

bool valid_vision_lead(const K7AdaptiveCruiseInput &input,
                       const K7AdaptiveCruiseConfig &config) {
  return input.vision_lead_valid &&
         std::isfinite(input.vision_lead_probability) &&
         input.vision_lead_probability >= config.lead_probability_threshold &&
         std::isfinite(input.vision_lead_distance_m) &&
         std::isfinite(input.vision_lead_relative_speed_mps) &&
         input.vision_lead_distance_m >= 1.0f &&
         input.vision_lead_distance_m <= 150.0f &&
         std::fabs(input.vision_lead_relative_speed_mps) <= 40.0f;
}

}  // namespace

bool load_k7_adaptive_cruise_params_json(
    const std::string &path, K7AdaptiveCruiseConfig *config,
    std::string *error) {
  if (!config) return false;
  std::ifstream file(path);
  if (!file.is_open()) {
    if (error) *error = "open failed";
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  try {
    parse_json_bool_value(text, "enabled", &config->enabled);
    parse_optional_float(text, "lead_probability_threshold", 0.2f, 0.99f,
                         &config->lead_probability_threshold);
    parse_optional_float(text, "standstill_gap_m", 2.0f, 20.0f,
                         &config->standstill_gap_m);
    parse_optional_float(text, "following_time_s", 0.8f, 4.0f,
                         &config->following_time_s);
    parse_optional_float(text, "gap_correction_gain", 0.05f, 1.0f,
                         &config->gap_correction_gain);
    parse_optional_float(text, "max_slowdown_correction_mps", 0.5f, 10.0f,
                         &config->max_slowdown_correction_mps);
    parse_optional_float(text, "max_speedup_correction_mps", 0.0f, 5.0f,
                         &config->max_speedup_correction_mps);
    parse_optional_float(text, "deceleration_rate_kph_per_s", 0.5f, 5.0f,
                         &config->deceleration_rate_kph_per_s);
    parse_optional_float(text, "lead_hold_s", 0.1f, 2.0f,
                         &config->lead_hold_s);
    parse_optional_float(text, "lead_restore_delay_s", 1.0f, 10.0f,
                         &config->lead_restore_delay_s);
    parse_optional_float(text, "command_interval_s", 0.5f, 5.0f,
                         &config->command_interval_s);
    parse_optional_int(text, "button_pulse_frames", 1, 10,
                       &config->button_pulse_frames);
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

K7AdaptiveCruiseController::K7AdaptiveCruiseController(
    K7AdaptiveCruiseConfig config)
    : config_(config) {}

void K7AdaptiveCruiseController::update_config(
    const K7AdaptiveCruiseConfig &config) {
  config_ = config;
}

float K7AdaptiveCruiseController::minimum_speed_kph(bool speed_unit_mph) const {
  return speed_unit_mph ? kMinimumSpeedMph * kMphToKph : kMinimumSpeedKph;
}

float K7AdaptiveCruiseController::display_step_kph(bool speed_unit_mph) const {
  return kDisplayStep * (speed_unit_mph ? kMphToKph : 1.0f);
}

void K7AdaptiveCruiseController::capture_driver_set_speed(float speed_kph,
                                                          double now_s) {
  if (!valid_set_speed(speed_kph)) return;
  session_valid_ = true;
  maximum_speed_kph_ = speed_kph;
  commanded_speed_kph_ = speed_kph;
  last_command_s_ = now_s;
  last_set_command_s_ = -1.0;
  command_button_ = 0;
  command_frames_remaining_ = 0;
}

void K7AdaptiveCruiseController::update_vision_lead(
    const K7AdaptiveCruiseInput &input) {
  if (!input.vision_lead_updated || !valid_vision_lead(input, config_)) return;

  const bool reacquired = last_valid_lead_s_ < 0.0 ||
                          input.now_s - last_valid_lead_s_ > config_.lead_hold_s;
  if (reacquired) {
    filtered_lead_distance_m_ = input.vision_lead_distance_m;
    filtered_lead_relative_speed_mps_ =
        input.vision_lead_relative_speed_mps;
  } else {
    const float distance_alpha =
        input.vision_lead_distance_m < filtered_lead_distance_m_ ? 0.45f : 0.2f;
    filtered_lead_distance_m_ +=
        distance_alpha * (input.vision_lead_distance_m - filtered_lead_distance_m_);
    filtered_lead_relative_speed_mps_ +=
        0.3f * (input.vision_lead_relative_speed_mps -
                filtered_lead_relative_speed_mps_);
  }
  last_valid_lead_s_ = input.now_s;
}

K7AdaptiveCruiseOutput K7AdaptiveCruiseController::update(
    const K7AdaptiveCruiseInput &input) {
  const bool accelerator_override =
      input.gas_pressed || input.driver_accelerator_override;
  if (accelerator_override) last_accelerator_override_s_ = input.now_s;

  const bool driver_button_pressed =
      input.driver_button != 0 && previous_driver_button_ == 0;
  const bool driver_main_pressed =
      input.driver_main_button != 0 && previous_driver_main_button_ == 0;
  const bool cruise_activated =
      input.cruise_active && !previous_cruise_active_;
  const float minimum_kph = minimum_speed_kph(input.speed_unit_mph);
  const float step_kph = display_step_kph(input.speed_unit_mph);

  if (!input.enabled || driver_main_pressed) {
    session_valid_ = false;
    maximum_speed_kph_ = 0.0f;
    commanded_speed_kph_ = 0.0f;
    last_valid_lead_s_ = -1.0;
    last_set_command_s_ = -1.0;
  }

  if (input.enabled && cruise_activated &&
      (!session_valid_ || input.driver_button == kCruiseButtonSet)) {
    capture_driver_set_speed(
        std::max(minimum_kph, input.driver_set_speed_kph), input.now_s);
  }

  if (session_valid_ && input.cruise_active && driver_button_pressed &&
      !cruise_activated) {
    if (input.driver_button == kCruiseButtonSet) {
      last_set_command_s_ = input.now_s;
      commanded_speed_kph_ =
          std::max(minimum_kph, commanded_speed_kph_ - step_kph);
      if (valid_set_speed(input.driver_set_speed_kph))
        maximum_speed_kph_ = input.driver_set_speed_kph;
      maximum_speed_kph_ = std::max(maximum_speed_kph_, commanded_speed_kph_);
    } else if (input.driver_button == kCruiseButtonResume) {
      if (valid_set_speed(input.driver_set_speed_kph))
        maximum_speed_kph_ = input.driver_set_speed_kph;
      commanded_speed_kph_ =
          std::min(maximum_speed_kph_, commanded_speed_kph_ + step_kph);
    }
  }

  update_vision_lead(input);

  const bool lead_valid = session_valid_ && last_valid_lead_s_ >= 0.0 &&
                          input.now_s >= last_valid_lead_s_ &&
                          input.now_s - last_valid_lead_s_ <= config_.lead_hold_s;
  float target_speed_kph = commanded_speed_kph_;
  if (session_valid_ && lead_valid) {
    const float ego_speed_mps = std::max(0.0f, input.ego_speed_kph / 3.6f);
    const float lead_speed_mps =
        std::max(0.0f, ego_speed_mps + filtered_lead_relative_speed_mps_);
    const float desired_gap_m =
        config_.standstill_gap_m + config_.following_time_s * ego_speed_mps;
    const float slowdown_response_s =
        step_kph / config_.deceleration_rate_kph_per_s;
    const float prediction_horizon_s =
        std::min(2.0f, config_.command_interval_s + 0.5f * slowdown_response_s);
    const float predicted_lead_distance_m = std::max(
        1.0f, filtered_lead_distance_m_ +
                  std::min(0.0f, filtered_lead_relative_speed_mps_) *
                      prediction_horizon_s);
    const float gap_correction_mps = clamp_float(
        (predicted_lead_distance_m - desired_gap_m) *
            config_.gap_correction_gain,
        -config_.max_slowdown_correction_mps,
        config_.max_speedup_correction_mps);
    target_speed_kph = clamp_float(
        (lead_speed_mps + gap_correction_mps) * 3.6f,
        minimum_kph, maximum_speed_kph_);
  } else if (session_valid_ && last_valid_lead_s_ >= 0.0 &&
             input.now_s - last_valid_lead_s_ >=
                 config_.lead_restore_delay_s) {
    target_speed_kph = maximum_speed_kph_;
  }

  const bool active = input.enabled && session_valid_ && input.cruise_active;
  const bool accelerator_released =
      !accelerator_override &&
      (last_accelerator_override_s_ < 0.0 ||
       input.now_s - last_accelerator_override_s_ >=
           kAcceleratorReleaseDelayS);
  const bool command_allowed = active && input.controls_ready &&
                               !input.brake_pressed && accelerator_released &&
                               input.driver_button == 0 && !driver_main_pressed;
  if (!command_allowed) {
    command_button_ = 0;
    command_frames_remaining_ = 0;
  }

  int output_button = 0;
  if (command_allowed && command_frames_remaining_ > 0) {
    output_button = command_button_;
    --command_frames_remaining_;
  } else if (command_allowed) {
    const float command_deadband_kph = step_kph * 0.75f;
    const bool wants_set =
        target_speed_kph <= commanded_speed_kph_ - command_deadband_kph &&
        commanded_speed_kph_ > minimum_kph + 0.1f;
    const bool wants_resume =
        target_speed_kph >= commanded_speed_kph_ + command_deadband_kph &&
        commanded_speed_kph_ < maximum_speed_kph_ - 0.1f;
    const double command_elapsed_s = input.now_s - last_command_s_;
    const double set_interval_s = std::max(
        config_.command_interval_s,
        step_kph / config_.deceleration_rate_kph_per_s);
    const bool set_ready =
        last_command_s_ < 0.0 ||
        command_elapsed_s >=
            (last_set_command_s_ < 0.0 ? config_.command_interval_s
                                       : set_interval_s);
    const bool resume_ready =
        (last_command_s_ < 0.0 ||
         command_elapsed_s >= config_.command_interval_s) &&
        (last_set_command_s_ < 0.0 ||
         input.now_s - last_set_command_s_ >= config_.lead_restore_delay_s);

    if (wants_set && set_ready) {
      command_button_ = kCruiseButtonSet;
      commanded_speed_kph_ =
          std::max(minimum_kph, commanded_speed_kph_ - step_kph);
      last_set_command_s_ = input.now_s;
    } else if (wants_resume && resume_ready) {
      command_button_ = kCruiseButtonResume;
      commanded_speed_kph_ =
          std::min(maximum_speed_kph_, commanded_speed_kph_ + step_kph);
    } else {
      command_button_ = 0;
    }
    if (command_button_ != 0) {
      last_command_s_ = input.now_s;
      command_frames_remaining_ = config_.button_pulse_frames - 1;
      output_button = command_button_;
    }
  }

  previous_cruise_active_ = input.cruise_active;
  previous_driver_button_ = input.driver_button;
  previous_driver_main_button_ = input.driver_main_button;

  K7AdaptiveCruiseOutput output;
  output.session_valid = session_valid_;
  output.active = active;
  output.lead_valid = lead_valid;
  output.maximum_speed_kph = maximum_speed_kph_;
  output.commanded_speed_kph = commanded_speed_kph_;
  output.target_speed_kph = target_speed_kph;
  output.command_button = output_button;
  return output;
}
