#include "departure_alert.h"

#include <stdexcept>

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

DepartureAlertInput stopped_input(double now_s) {
  DepartureAlertInput input;
  input.now_s = now_s;
  input.vehicle_valid = true;
  input.gear = 5;
  input.speed_mps = 0.0f;
  return input;
}

void verify_lead_departure() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i <= 20; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.1);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 5.0f + (i % 2) * 0.2f;
    input.lead_relative_speed_mps = 0.2f;
    output = detector.update(input);
  }
  require(output.type == DepartureAlertType::none,
          "stationary lead jitter must not trigger an alert");
  require(output.lead_armed, "stable lead must arm the detector");

  for (int i = 21; i <= 25; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.1);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 5.6f;
    input.lead_relative_speed_mps = 1.0f;
    output = detector.update(input);
  }
  require(output.type == DepartureAlertType::lead_departed,
          "departing lead must trigger an alert");
  require(output.event_id == 1, "first departure alert event id");

  for (int i = 26; i <= 50; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.1);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 10.0f;
    input.lead_relative_speed_mps = 3.0f;
    output = detector.update(input);
  }
  require(output.event_id == 1,
          "one stop cycle must not trigger duplicate lead alerts");
}

void verify_green_light() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i < 60; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    input.stop_line_valid = true;
    input.stop_line_probability = 0.8f;
    input.stop_line_distance_m = 3.0f;
    output = detector.update(input);
  }
  require(!output.green_light_armed,
          "green-light detection must not arm before three seconds");

  DepartureAlertInput armed_input = stopped_input(3.0);
  armed_input.model_updated = true;
  armed_input.model_valid = true;
  armed_input.plan_distance_m = 4.0f;
  armed_input.stop_line_valid = true;
  armed_input.stop_line_probability = 0.8f;
  armed_input.stop_line_distance_m = 3.0f;
  output = detector.update(armed_input);
  require(output.green_light_armed,
          "stable stopped model state must arm at three seconds");

  for (int i = 61; i <= 68; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 26.0f;
    input.stop_line_valid = false;
    input.stop_line_probability = 0.2f;
    input.stop_line_distance_m = 20.0f;
    output = detector.update(input);
  }
  require(output.type == DepartureAlertType::green_light,
          "an armed detector must trigger when the model path opens");
  require(output.event_id == 1, "first green-light alert event id");
}

void verify_three_second_display_uses_total_stop_time() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i < 60; ++i)
    output = detector.update(stopped_input(i * 0.05));

  DepartureAlertInput input = stopped_input(3.0);
  input.model_updated = true;
  input.model_valid = true;
  input.plan_distance_m = 4.0f;
  input.stop_line_valid = true;
  input.stop_line_probability = 0.8f;
  input.stop_line_distance_m = 3.0f;
  output = detector.update(input);
  require(output.green_light_armed,
          "traffic signal must display after three seconds of total stop time");
}

void verify_signal_change_has_priority_over_lead_departure() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i <= 60; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 5.0f;
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    input.stop_line_valid = true;
    input.stop_line_probability = 0.8f;
    input.stop_line_distance_m = 3.0f;
    output = detector.update(input);
  }
  require(output.green_light_armed,
          "a tracked lead must not suppress the red traffic signal");

  for (int i = 61; i <= 68; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 5.6f;
    input.lead_relative_speed_mps = 1.0f;
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 26.0f;
    output = detector.update(input);
  }
  require(output.type == DepartureAlertType::green_light,
          "signal change must take priority over a departing lead");
  require(output.event_id == 1, "priority alert must emit one event");
}

void verify_distant_stop_line_does_not_arm() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i <= 100; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    input.stop_line_valid = true;
    input.stop_line_probability = 0.8f;
    input.stop_line_distance_m = 5.1f;
    output = detector.update(input);
  }
  require(!output.green_light_armed,
          "raw stop-line distance above the C2 threshold must not arm");
}

}  // namespace

int main() {
  verify_lead_departure();
  verify_green_light();
  verify_three_second_display_uses_total_stop_time();
  verify_signal_change_has_priority_over_lead_departure();
  verify_distant_stop_line_does_not_arm();
  return 0;
}
