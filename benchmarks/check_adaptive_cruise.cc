#include "adaptive_cruise.h"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

bool near(float value, float expected) {
  return std::fabs(value - expected) < 0.001f;
}

K7AdaptiveCruiseInput base_input(double now_s) {
  K7AdaptiveCruiseInput input;
  input.now_s = now_s;
  input.controls_ready = true;
  input.cruise_active = true;
  input.ego_speed_kph = 80.0f;
  input.driver_set_speed_kph = 80.0f;
  input.vision_lead_probability = 0.9f;
  return input;
}

K7AdaptiveCruiseOutput activate(K7AdaptiveCruiseController *controller) {
  K7AdaptiveCruiseInput input = base_input(0.0);
  input.driver_button = 2;
  return controller->update(input);
}

void verify_close_lead_follows_measured_deceleration_rate() {
  K7AdaptiveCruiseController controller;
  K7AdaptiveCruiseOutput output = activate(&controller);
  require(output.session_valid && output.active &&
              near(output.maximum_speed_kph, 80.0f) &&
              near(output.commanded_speed_kph, 80.0f),
          "first SET must capture maximum and commanded speed");

  K7AdaptiveCruiseInput input = base_input(0.01);
  controller.update(input);
  input.now_s = 0.5;
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 12.0f;
  input.vision_lead_relative_speed_mps = -4.0f;
  output = controller.update(input);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 80.0f),
          "adaptive command must wait one second after SET");

  input.now_s = 1.0;
  output = controller.update(input);
  require(output.command_button == 2 && near(output.commanded_speed_kph, 78.0f) &&
              output.target_speed_kph < output.commanded_speed_kph,
          "close slower lead must issue one SET command");

  input.vision_lead_updated = false;
  for (int frame = 1; frame < 5; ++frame) {
    input.now_s = 1.0 + frame * 0.01;
    output = controller.update(input);
    require(output.command_button == 2,
            "SET command must be emitted as a five-frame pulse");
  }
  input.now_s = 1.05;
  output = controller.update(input);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 78.0f),
          "button pulse must stop after five frames");

  input.now_s = 1.9;
  input.vision_lead_updated = true;
  output = controller.update(input);
  require(output.command_button == 0,
          "adaptive commands must be rate limited to one start per second");
  input.now_s = 2.0;
  output = controller.update(input);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 78.0f),
          "the next SET must wait for the measured deceleration response");
  input.now_s = 2.34;
  output = controller.update(input);
  require(output.command_button == 2 && near(output.commanded_speed_kph, 76.0f),
          "persistent close lead must request the next step after its response time");
}

void verify_closing_lead_prediction_and_resume_delay() {
  K7AdaptiveCruiseController controller;
  activate(&controller);

  K7AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 50.0f;
  input.vision_lead_relative_speed_mps = -5.0f;
  K7AdaptiveCruiseOutput output = controller.update(input);
  require(output.target_speed_kph < 64.0f,
          "closing lead distance must be predicted at command response time");
  require(output.command_button == 2 && near(output.commanded_speed_kph, 78.0f),
          "predicted closing lead must request SET");

  input.vision_lead_updated = false;
  for (int frame = 1; frame < 5; ++frame) {
    input.now_s = 1.0 + frame * 0.01;
    controller.update(input);
  }

  input.now_s = 2.0;
  input.vision_lead_updated = true;
  input.vision_lead_distance_m = 150.0f;
  input.vision_lead_relative_speed_mps = 40.0f;
  output = controller.update(input);
  require(output.target_speed_kph > output.commanded_speed_kph &&
              output.command_button == 0,
          "RES must not immediately reverse a recent SET command");

  input.now_s = 3.0;
  output = controller.update(input);
  require(output.command_button == 1 && near(output.commanded_speed_kph, 80.0f),
          "RES may restore speed after the configured recovery delay");
}

void verify_lead_loss_holds_then_restores_maximum() {
  K7AdaptiveCruiseController controller;
  activate(&controller);
  K7AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 10.0f;
  input.vision_lead_relative_speed_mps = -5.0f;
  K7AdaptiveCruiseOutput output = controller.update(input);
  require(output.command_button == 2 && near(output.commanded_speed_kph, 78.0f),
          "lead must lower current command before restore test");
  input.vision_lead_updated = false;
  for (int frame = 1; frame < 5; ++frame) {
    input.now_s = 1.0 + frame * 0.01;
    controller.update(input);
  }

  input.vision_lead_updated = true;
  input.vision_lead_valid = false;
  input.now_s = 2.0;
  output = controller.update(input);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 78.0f),
          "short lead loss must hold the reduced setting");

  input.now_s = 3.01;
  output = controller.update(input);
  require(output.command_button == 1 && near(output.commanded_speed_kph, 80.0f),
          "stable lead loss must restore toward the captured maximum");
}

void verify_driver_and_pedal_gates() {
  K7AdaptiveCruiseController controller;
  activate(&controller);
  K7AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 8.0f;
  input.vision_lead_relative_speed_mps = -6.0f;
  input.brake_pressed = true;
  K7AdaptiveCruiseOutput output = controller.update(input);
  require(output.command_button == 0, "brake must block adaptive button output");

  input.brake_pressed = false;
  input.gas_pressed = true;
  input.now_s = 2.0;
  output = controller.update(input);
  require(output.command_button == 0, "gas pedal must block adaptive button output");

  input.gas_pressed = false;
  input.now_s = 2.49;
  output = controller.update(input);
  require(output.command_button == 0,
          "adaptive output must wait after the gas pedal is released");

  input.now_s = 2.5;
  output = controller.update(input);
  require(output.command_button == 2,
          "adaptive output may resume after the gas release delay");

  input.now_s = 2.6;
  input.driver_accelerator_override = true;
  output = controller.update(input);
  require(output.command_button == 0,
          "TCS driver override must cancel an active button pulse");

  input.driver_accelerator_override = false;
  input.driver_button = 1;
  input.driver_set_speed_kph = 82.0f;
  input.now_s = 3.1;
  output = controller.update(input);
  require(output.command_button == 0 && near(output.maximum_speed_kph, 82.0f),
          "driver RES must update maximum and suppress automatic output");
}

void verify_session_reset_and_minimum_speed() {
  K7AdaptiveCruiseController controller;
  K7AdaptiveCruiseInput input = base_input(0.0);
  input.driver_button = 2;
  input.driver_set_speed_kph = 20.0f;
  K7AdaptiveCruiseOutput output = controller.update(input);
  require(output.session_valid && near(output.maximum_speed_kph, 30.0f),
          "metric SET speed must not fall below the stock cruise minimum");

  input.driver_button = 0;
  input.driver_main_button = 1;
  input.now_s = 0.1;
  output = controller.update(input);
  require(!output.session_valid && !output.active &&
              output.command_button == 0,
          "main button must clear the adaptive session");

  K7AdaptiveCruiseController imperial_controller;
  input = base_input(0.0);
  input.speed_unit_mph = true;
  input.driver_button = 2;
  input.driver_set_speed_kph = 25.0f;
  output = imperial_controller.update(input);
  require(output.session_valid &&
              near(output.maximum_speed_kph, 20.0f * 1.609344f),
          "imperial SET speed must use the 20 mph stock cruise minimum");

  input.enabled = false;
  input.driver_button = 0;
  input.now_s = 0.1;
  output = imperial_controller.update(input);
  require(!output.session_valid && output.command_button == 0,
          "disabled adaptive cruise must clear its session and output");
}

void verify_runtime_config_update() {
  K7AdaptiveCruiseConfig config;
  config.lead_probability_threshold = 0.8f;
  K7AdaptiveCruiseController controller(config);
  activate(&controller);

  K7AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_probability = 0.7f;
  input.vision_lead_distance_m = 8.0f;
  input.vision_lead_relative_speed_mps = -6.0f;
  K7AdaptiveCruiseOutput output = controller.update(input);
  require(!output.lead_valid && output.command_button == 0,
          "lead below the configured probability must be ignored");

  config.lead_probability_threshold = 0.6f;
  config.command_interval_s = 0.5f;
  config.button_pulse_frames = 1;
  controller.update_config(config);
  input.now_s = 1.01;
  output = controller.update(input);
  require(output.lead_valid && output.command_button == 2,
          "runtime config update must apply without recreating the controller");

  input.vision_lead_updated = false;
  input.now_s = 1.02;
  output = controller.update(input);
  require(output.command_button == 0,
          "one-frame runtime pulse must end on the following control tick");
}

void verify_repository_config_loads() {
  K7AdaptiveCruiseConfig config;
  std::string error;
  require(load_k7_adaptive_cruise_params_json(
              "params/yg_adaptive_cruise.json", &config, &error),
          error.c_str());
  require(config.enabled && near(config.following_time_s, 1.8f) &&
              near(config.standstill_gap_m, 5.0f) &&
              near(config.deceleration_rate_kph_per_s, 1.5f) &&
              near(config.command_interval_s, 1.0f) &&
              config.button_pulse_frames == 5,
          "repository adaptive cruise defaults do not match the runtime schema");
}

}  // namespace

int main() {
  try {
    verify_close_lead_follows_measured_deceleration_rate();
    verify_closing_lead_prediction_and_resume_delay();
    verify_lead_loss_holds_then_restores_maximum();
    verify_driver_and_pedal_gates();
    verify_session_reset_and_minimum_speed();
    verify_runtime_config_update();
    verify_repository_config_loads();
    std::puts("K7_ADAPTIVE_CRUISE_OK");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "check_adaptive_cruise: %s\n", error.what());
    return 1;
  }
}
