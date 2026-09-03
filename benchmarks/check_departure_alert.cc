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

/* 정차하고 모델이 여기서 멈추겠다고 계획하면 3 s 뒤 무장한다. 길이
 * 열리면(plan > 10 m, 0.3 s) 알림이 뜬다. */
void verify_green_light() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i < 60; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    output = detector.update(input);
  }
  require(!output.green_light_armed,
          "green-light detection must not arm before three seconds");

  DepartureAlertInput armed_input = stopped_input(3.0);
  armed_input.model_updated = true;
  armed_input.model_valid = true;
  armed_input.plan_distance_m = 4.0f;
  output = detector.update(armed_input);
  require(output.green_light_armed,
          "stable stopped model state must arm at three seconds");

  for (int i = 61; i <= 68; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 9.0f;
    output = detector.update(input);
  }
  require(output.type == DepartureAlertType::none,
          "a plan below the open threshold must not trigger");

  for (int i = 69; i <= 76; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 11.0f;
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
  output = detector.update(input);
  require(output.green_light_armed,
          "traffic signal must display after three seconds of total stop time");
}

/* 정체(앞차 있음)에서도 무장은 된다(094 lead 확률은 앞차 유무를 가르지
 * 못한다). plan이 닫힌 채 앞차만 출발하면 lead_departed가 뜬다. */
void verify_queue_keeps_lead_alert() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i <= 60; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 6.0f;
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 3.0f;
    output = detector.update(input);
  }
  require(output.green_light_armed,
          "a lead in front must not block arming");

  for (int i = 61; i <= 70; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 7.0f;
    input.lead_relative_speed_mps = 1.0f;
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 3.0f;
    output = detector.update(input);
  }
  require(output.type == DepartureAlertType::lead_departed,
          "lead departure must still alert without stop lines");
}

/* 무장 뒤 앞차가 끼어들어도 무장은 유지된다. 그 앞차가 출발하며 plan도
 * 같은 프레임에 열리면 더 구체적인 사유인 lead_departed가 뜬다. */
void verify_lead_departure_wins_when_plan_opens() {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i <= 61; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    output = detector.update(input);
  }
  require(output.green_light_armed,
          "an empty stop must arm the signal alert first");

  for (int i = 62; i <= 90; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 6.0f;
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    output = detector.update(input);
  }
  require(output.green_light_armed,
          "a lead cutting in must keep the signal alert armed");

  for (int i = 91; i <= 100; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 7.0f;
    input.lead_relative_speed_mps = 1.0f;
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 30.0f;
    output = detector.update(input);
  }
  require(output.type == DepartureAlertType::lead_departed,
          "a departing lead must win over the plan opening in the same frame");
}

}  // namespace

int main() {
  verify_lead_departure();
  verify_green_light();
  verify_three_second_display_uses_total_stop_time();
  verify_queue_keeps_lead_alert();
  verify_lead_departure_wins_when_plan_opens();
  return 0;
}
