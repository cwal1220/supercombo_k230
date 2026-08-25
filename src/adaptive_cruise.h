#pragma once

#include <string>

struct AdaptiveCruiseConfig {
  bool enabled = true;
  float lead_probability_threshold = 0.5f;
  float standstill_gap_m = 5.0f;
  float following_time_s = 1.8f;
  float gap_correction_gain = 0.25f;
  float max_slowdown_correction_mps = 4.0f;
  float max_speedup_correction_mps = 2.5f;
  float deceleration_rate_kph_per_s = 1.5f;
  float lead_hold_s = 0.6f;
  float lead_restore_delay_s = 2.0f;
  float command_interval_s = 1.0f;
  int button_pulse_frames = 5;
};

bool load_adaptive_cruise_params_json(
    const std::string &path, AdaptiveCruiseConfig *config,
    std::string *error);

struct AdaptiveCruiseInput {
  double now_s = 0.0;
  bool enabled = true;
  bool controls_ready = false;
  bool cruise_active = false;
  bool brake_pressed = false;
  bool gas_pressed = false;
  bool driver_accelerator_override = false;
  bool speed_unit_mph = false;
  int driver_button = 0;
  int driver_main_button = 0;
  float ego_speed_kph = 0.0f;
  float driver_set_speed_kph = 0.0f;
  bool vision_lead_updated = false;
  bool vision_lead_valid = false;
  float vision_lead_probability = 0.0f;
  float vision_lead_distance_m = 0.0f;
  float vision_lead_relative_speed_mps = 0.0f;
};

struct AdaptiveCruiseOutput {
  bool session_valid = false;
  bool active = false;
  bool lead_valid = false;
  float maximum_speed_kph = 0.0f;
  float commanded_speed_kph = 0.0f;
  float target_speed_kph = 0.0f;
  int command_button = 0;
};

class AdaptiveCruiseController {
public:
  explicit AdaptiveCruiseController(
      AdaptiveCruiseConfig config = AdaptiveCruiseConfig());

  void update_config(const AdaptiveCruiseConfig &config);
  AdaptiveCruiseOutput update(const AdaptiveCruiseInput &input);

private:
  void capture_driver_set_speed(float speed_kph, double now_s);
  void update_vision_lead(const AdaptiveCruiseInput &input);
  float minimum_speed_kph(bool speed_unit_mph) const;
  float display_step_kph(bool speed_unit_mph) const;

  AdaptiveCruiseConfig config_;
  bool session_valid_ = false;
  bool previous_cruise_active_ = false;
  int previous_driver_button_ = 0;
  int previous_driver_main_button_ = 0;
  float maximum_speed_kph_ = 0.0f;
  float commanded_speed_kph_ = 0.0f;
  float filtered_lead_distance_m_ = 0.0f;
  float filtered_lead_relative_speed_mps_ = 0.0f;
  double last_valid_lead_s_ = -1.0;
  double last_command_s_ = -1.0;
  double last_set_command_s_ = -1.0;
  double last_accelerator_override_s_ = -1.0;
  int command_button_ = 0;
  int command_frames_remaining_ = 0;
};
