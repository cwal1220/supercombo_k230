/* DepartureAlertDetector: 정차 중 앞차 출발(lead_departed)과 신호 대기 뒤 길이 열릴 때
 * (green_light) 알림. 입력은 0.05~0.1 s 틱으로 합성한다. */
#include "departure_alert.h"

#include <gtest/gtest.h>

namespace {

DepartureAlertInput stopped_input(double now_s) {
  DepartureAlertInput input;
  input.now_s = now_s;
  input.vehicle_valid = true;
  input.gear = 5;
  input.speed_mps = 0.0f;
  return input;
}

TEST(DepartureAlert, LeadDeparture) {
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
  ASSERT_EQ(output.type, DepartureAlertType::none)
      << "멈춘 앞차의 거리 떨림으로는 알림이 뜨지 않는다";
  ASSERT_TRUE(output.lead_armed) << "안정된 앞차가 검출기를 무장한다";

  for (int i = 21; i <= 25; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.1);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 5.6f;
    input.lead_relative_speed_mps = 1.0f;
    output = detector.update(input);
  }
  ASSERT_EQ(output.type, DepartureAlertType::lead_departed)
      << "앞차가 출발하면 알림이 뜬다";
  ASSERT_EQ(output.event_id, 1u) << "첫 출발 알림의 이벤트 id";

  for (int i = 26; i <= 50; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.1);
    input.lead_updated = true;
    input.lead_valid = true;
    input.lead_distance_m = 10.0f;
    input.lead_relative_speed_mps = 3.0f;
    output = detector.update(input);
  }
  ASSERT_EQ(output.event_id, 1u) << "정차 한 번에 앞차 알림은 한 번만 뜬다";
}

/* 정차하고 모델이 여기서 멈추겠다고 계획하면 3 s 뒤 무장한다. 길이
 * 열리면(plan > 10 m, 0.3 s) 알림이 뜬다. */
TEST(DepartureAlert, GreenLight) {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i < 60; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    output = detector.update(input);
  }
  ASSERT_FALSE(output.green_light_armed)
      << "신호 대기 검출은 3초 전에는 무장하지 않는다";

  DepartureAlertInput armed_input = stopped_input(3.0);
  armed_input.model_updated = true;
  armed_input.model_valid = true;
  armed_input.plan_distance_m = 4.0f;
  output = detector.update(armed_input);
  ASSERT_TRUE(output.green_light_armed) << "정차 상태가 3초 이어지면 무장한다";

  for (int i = 61; i <= 68; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 9.0f;
    output = detector.update(input);
  }
  ASSERT_EQ(output.type, DepartureAlertType::none)
      << "plan이 열림 기준보다 짧으면 알림이 뜨지 않는다";

  for (int i = 69; i <= 76; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 11.0f;
    output = detector.update(input);
  }
  ASSERT_EQ(output.type, DepartureAlertType::green_light)
      << "무장한 뒤 모델 경로가 열리면 알림이 뜬다";
  ASSERT_EQ(output.event_id, 1u) << "첫 신호 알림의 이벤트 id";
}

TEST(DepartureAlert, ThreeSecondDisplayUsesTotalStopTime) {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i < 60; ++i)
    output = detector.update(stopped_input(i * 0.05));

  DepartureAlertInput input = stopped_input(3.0);
  input.model_updated = true;
  input.model_valid = true;
  input.plan_distance_m = 4.0f;
  output = detector.update(input);
  ASSERT_TRUE(output.green_light_armed)
      << "모델 입력 없이 서 있던 시간까지 합쳐 정차 3초면 무장한다";
}

/* 정체(앞차 있음)에서도 무장은 된다(094 lead 확률은 앞차 유무를 가르지
 * 못한다). plan이 닫힌 채 앞차만 출발하면 lead_departed가 뜬다. */
TEST(DepartureAlert, QueueKeepsLeadAlert) {
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
  ASSERT_TRUE(output.green_light_armed) << "앞차가 있어도 무장한다";

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
  ASSERT_EQ(output.type, DepartureAlertType::lead_departed)
      << "plan이 닫힌 채여도 앞차 출발 알림은 뜬다";
}

/* 무장 뒤 앞차가 끼어들어도 무장은 유지된다. 그 앞차가 출발하며 plan도
 * 같은 프레임에 열리면 더 구체적인 사유인 lead_departed가 뜬다. */
TEST(DepartureAlert, LeadDepartureWinsWhenPlanOpens) {
  DepartureAlertDetector detector;
  DepartureAlertOutput output;

  for (int i = 0; i <= 61; ++i) {
    DepartureAlertInput input = stopped_input(i * 0.05);
    input.model_updated = true;
    input.model_valid = true;
    input.plan_distance_m = 4.0f;
    output = detector.update(input);
  }
  ASSERT_TRUE(output.green_light_armed) << "앞차 없는 정차에서 먼저 신호 알림을 무장한다";

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
  ASSERT_TRUE(output.green_light_armed) << "앞차가 끼어들어도 신호 알림 무장은 유지된다";

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
  ASSERT_EQ(output.type, DepartureAlertType::lead_departed)
      << "같은 프레임에 plan도 열리면 앞차 출발이 이긴다";
}

}  // namespace
