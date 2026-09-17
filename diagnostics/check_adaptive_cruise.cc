#include "adaptive_cruise.h"
#include "check_harness.h"

#include <cmath>

namespace {

/* 버튼을 누르면 차량의 설정 속도가 실제로 움직인다. 예전 픽스처는 이걸
 * 모사하지 않고 driver_set_speed_kph를 고정한 채 컨트롤러 내부 추측값만
 * 검사해서, 설정 속도 읽기 동기화가 없던 시절의 버그를 통과시켰다. */
struct Vehicle {
  float set_speed_kph = 80.0f;
  float cluster_speed_kph = 80.0f;
  float step_kph = 2.0f;
  /* 클러스터가 휠보다 높게 읽는 비(K7 실측 1.066). */
  float over_read = 1.066f;
  int previous_button = 0;
  int previous_driver_button = 0;
  bool cruise_on = false;
  /* 이 값마다 버튼 하나를 흘린다. 0이면 흘리지 않는다. */
  int drop_every = 0;
  int presses = 0;

  /* 버튼은 펄스의 상승 엣지당 한 번만 먹는다. 스톡 스토크와 같다. */
  void apply(int button) {
    if (button == 0 || previous_button != 0) { previous_button = button; return; }
    previous_button = button;
    ++presses;
    if (drop_every != 0 && presses % drop_every == 0) return;
    if (button == 2) set_speed_kph = std::fmax(30.0f, set_speed_kph - step_kph);
    if (button == 1) set_speed_kph += step_kph;
  }
  /* 운전자 버튼도 같은 차량에 작용한다. 예전 픽스처는 이걸 모사하지 않아
   * 운전자 조작 시나리오를 손으로 꾸며야 했고, 그래서 결함을 놓쳤다. */
  void driver_press(int button) {
    if (button != 0 && previous_driver_button == 0) {
      /* 고정형 크루즈와 같다: 꺼져 있을 때의 SET은 현재 속도를 잡고,
       * 켜져 있을 때의 SET은 한 스텝 낮춘다 (vehicle_can.cc의 추정기와 동일). */
      if (button == 2) {
        set_speed_kph = cruise_on
            ? std::fmax(30.0f, set_speed_kph - step_kph)
            : std::fmax(30.0f, cluster_speed_kph);
        cruise_on = true;
      }
      if (button == 1) {
        if (cruise_on) set_speed_kph += step_kph;
        cruise_on = true;
      }
    }
    previous_driver_button = button;
  }
  /* 클러스터 속도는 설정 속도를 따라간다. 즉시가 아니다. */
  void settle(float dt_s) {
    cluster_speed_kph += (set_speed_kph - cluster_speed_kph) * dt_s * 0.8f;
  }
  void snap() { cluster_speed_kph = set_speed_kph; }
  float wheel_kph() const { return cluster_speed_kph / over_read; }
};

AdaptiveCruiseInput base_input(double now_s) {
  AdaptiveCruiseInput input;
  input.now_s = now_s;
  input.controls_ready = true;
  input.cruise_active = true;
  input.ego_speed_kph = 80.0f;
  input.cluster_speed_kph = 80.0f;
  input.driver_set_speed_kph = 80.0f;
  input.vision_lead_probability = 0.9f;
  return input;
}

/* 한 틱: 차량이 보고하는 설정 속도를 넣고, 나온 버튼을 차량에 반영한다. */
AdaptiveCruiseOutput tick(AdaptiveCruiseController *controller,
                          AdaptiveCruiseInput input, Vehicle *vehicle) {
  /* driver_set_speed_kph 는 차량 보고가 아니라 운전자 조작만 반영하는 추정치다
   * (이 차에는 SCC가 없다). 우리 명령은 여기에 절대 나타나지 않는다. */
  vehicle->driver_press(input.driver_button);
  input.driver_set_speed_kph = vehicle->set_speed_kph;
  input.ego_speed_kph = vehicle->wheel_kph();
  input.cluster_speed_kph = vehicle->cluster_speed_kph;
  const AdaptiveCruiseOutput output = controller->update(input);
  vehicle->apply(output.command_button);
  return output;
}

AdaptiveCruiseOutput activate(AdaptiveCruiseController *controller,
                              Vehicle *vehicle) {
  AdaptiveCruiseInput input = base_input(0.0);
  input.driver_button = 2;
  return tick(controller, input, vehicle);
}

void verify_close_lead_follows_measured_deceleration_rate() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  AdaptiveCruiseOutput output = activate(&controller, &vehicle);
  require(output.session_valid && output.active &&
              near(output.maximum_speed_kph, 80.0f, 0.001f) &&
              near(output.commanded_speed_kph, 80.0f, 0.001f),
          "first SET must capture maximum and commanded speed");

  AdaptiveCruiseInput input = base_input(0.01);
  tick(&controller, input, &vehicle);
  input.now_s = 0.5;
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 12.0f;
  input.vision_lead_relative_speed_mps = -4.0f;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 80.0f, 0.001f),
          "adaptive command must wait one second after SET");

  input.now_s = 1.0;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 2 && near(output.commanded_speed_kph, 78.0f, 0.001f) &&
              output.target_speed_kph < output.commanded_speed_kph,
          "close slower lead must issue one SET command");

  input.vision_lead_updated = false;
  for (int frame = 1; frame < 5; ++frame) {
    input.now_s = 1.0 + frame * 0.01;
    output = tick(&controller, input, &vehicle);
    require(output.command_button == 2,
            "SET command must be emitted as a five-frame pulse");
  }
  input.now_s = 1.05;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 78.0f, 0.001f),
          "button pulse must stop after five frames");

  input.now_s = 1.9;
  input.vision_lead_updated = true;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0,
          "adaptive commands must be rate limited to one start per second");
  input.now_s = 2.0;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 78.0f, 0.001f),
          "the next SET must wait for the measured deceleration response");
  input.now_s = 2.34;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 2 && near(output.commanded_speed_kph, 76.0f, 0.001f),
          "persistent close lead must request the next step after its response time");
}

void verify_closing_lead_prediction_and_resume_delay() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  activate(&controller, &vehicle);

  AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 50.0f;
  input.vision_lead_relative_speed_mps = -5.0f;
  AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
  require(output.target_speed_kph < 64.0f,
          "closing lead distance must be predicted at command response time");
  require(output.command_button == 2 && near(output.commanded_speed_kph, 78.0f, 0.001f),
          "predicted closing lead must request SET");

  input.vision_lead_updated = false;
  for (int frame = 1; frame < 5; ++frame) {
    input.now_s = 1.0 + frame * 0.01;
    tick(&controller, input, &vehicle);
  }

  input.now_s = 2.0;
  input.vision_lead_updated = true;
  input.vision_lead_distance_m = 150.0f;
  input.vision_lead_relative_speed_mps = 40.0f;
  output = tick(&controller, input, &vehicle);
  require(output.target_speed_kph > output.commanded_speed_kph &&
              output.command_button == 0,
          "RES must not immediately reverse a recent SET command");

  input.now_s = 3.0;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 1 && near(output.commanded_speed_kph, 80.0f, 0.001f),
          "RES may restore speed after the configured recovery delay");
}

void verify_lead_loss_holds_then_restores_maximum() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  activate(&controller, &vehicle);
  AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 10.0f;
  input.vision_lead_relative_speed_mps = -5.0f;
  AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
  require(output.command_button == 2 && near(output.commanded_speed_kph, 78.0f, 0.001f),
          "lead must lower current command before restore test");
  input.vision_lead_updated = false;
  for (int frame = 1; frame < 5; ++frame) {
    input.now_s = 1.0 + frame * 0.01;
    tick(&controller, input, &vehicle);
  }

  input.vision_lead_updated = true;
  input.vision_lead_valid = false;
  input.now_s = 2.0;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0 && near(output.commanded_speed_kph, 78.0f, 0.001f),
          "short lead loss must hold the reduced setting");

  input.now_s = 3.01;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 1 && near(output.commanded_speed_kph, 80.0f, 0.001f),
          "stable lead loss must restore toward the captured maximum");
}

void verify_driver_and_pedal_gates() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  activate(&controller, &vehicle);
  AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_distance_m = 8.0f;
  input.vision_lead_relative_speed_mps = -6.0f;
  input.brake_pressed = true;
  AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
  require(output.command_button == 0, "brake must block adaptive button output");

  input.brake_pressed = false;
  input.gas_pressed = true;
  input.now_s = 2.0;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0, "gas pedal must block adaptive button output");

  input.gas_pressed = false;
  input.now_s = 2.49;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0,
          "adaptive output must wait after the gas pedal is released");

  input.now_s = 2.5;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 2,
          "adaptive output may resume after the gas release delay");

  input.now_s = 2.6;
  input.driver_accelerator_override = true;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0,
          "TCS driver override must cancel an active button pulse");

  /* 운전자가 RES로 설정 속도를 올린다. 천장은 버튼을 뗀 뒤 차량이 정착하고
   * 나서 실측 속도로 잡는다 — 누르는 도중의 값은 아직 이동 중이다. */
  input.driver_accelerator_override = false;
  input.vision_lead_valid = false;      // 선행차를 치워 천장 동작만 본다
  input.driver_button = 1;
  input.now_s = 3.1;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0,
          "driver button must suppress automatic output while it is held");

  input.driver_button = 0;
  for (int frame = 1; frame <= 600; ++frame) {
    input.now_s = 3.1 + frame * 0.01;
    output = tick(&controller, input, &vehicle);
    vehicle.settle(0.01f);
  }
  /* 지수 수렴이라 정착 판정 시점에 한 스텝의 1/6 정도가 남는다. 명령 단위가
   * 2 km/h이므로 실질 영향은 없다. */
  /* 앞 단계에서 자동 SET-이 한 번 나가 80 -> 78이 됐으므로 운전자 RES 후는 80이다. */
  require(near(output.maximum_speed_kph, 80.0f, 0.5f) &&
              near(output.commanded_speed_kph, 80.0f, 0.5f),
          "driver RES must raise the ceiling once the car has settled");
}

void verify_session_reset_and_minimum_speed() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  vehicle.set_speed_kph = 20.0f;
  vehicle.cluster_speed_kph = 20.0f;
  AdaptiveCruiseInput input = base_input(0.0);
  input.driver_button = 2;
  AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
  require(output.session_valid && near(output.maximum_speed_kph, 30.0f, 0.001f),
          "metric SET speed must not fall below the stock cruise minimum");

  input.driver_button = 0;
  input.driver_main_button = 1;
  input.now_s = 0.1;
  output = tick(&controller, input, &vehicle);
  require(!output.session_valid && !output.active &&
              output.command_button == 0,
          "main button must clear the adaptive session");

  AdaptiveCruiseController imperial_controller;
  Vehicle imperial_vehicle;
  imperial_vehicle.set_speed_kph = 25.0f;
  imperial_vehicle.cluster_speed_kph = 25.0f;
  input = base_input(0.0);
  input.speed_unit_mph = true;
  input.driver_button = 2;
  output = tick(&imperial_controller, input, &imperial_vehicle);
  require(output.session_valid &&
              near(output.maximum_speed_kph, 20.0f * 1.609344f, 0.001f),
          "imperial SET speed must use the 20 mph stock cruise minimum");

  input.enabled = false;
  input.driver_button = 0;
  input.now_s = 0.1;
  output = tick(&imperial_controller, input, &imperial_vehicle);
  require(!output.session_valid && output.command_button == 0,
          "disabled adaptive cruise must clear its session and output");
}

void verify_runtime_config_update() {
  AdaptiveCruiseConfig config;
  config.lead_probability_threshold = 0.8f;
  AdaptiveCruiseController controller(config);
  Vehicle vehicle;
  activate(&controller, &vehicle);

  AdaptiveCruiseInput input = base_input(1.0);
  input.vision_lead_updated = true;
  input.vision_lead_valid = true;
  input.vision_lead_probability = 0.7f;
  input.vision_lead_distance_m = 8.0f;
  input.vision_lead_relative_speed_mps = -6.0f;
  AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
  require(!output.lead_valid && output.command_button == 0,
          "lead below the configured probability must be ignored");

  config.lead_probability_threshold = 0.6f;
  config.command_interval_s = 0.5f;
  config.button_pulse_frames = 1;
  controller.update_config(config);
  input.now_s = 1.01;
  output = tick(&controller, input, &vehicle);
  require(output.lead_valid && output.command_button == 2,
          "runtime config update must apply without recreating the controller");

  input.vision_lead_updated = false;
  input.now_s = 1.02;
  output = tick(&controller, input, &vehicle);
  require(output.command_button == 0,
          "one-frame runtime pulse must end on the following control tick");
}

/* 이 차에는 SCC가 없어 설정 속도를 CAN으로 읽을 수 없다(2026-09-13 주행 실측:
 * SCC11/SCC12 수신 0건). driver_set_speed_kph 는 운전자 조작만 반영하는
 * 추정치이고 우리 명령은 절대 나타나지 않는다. 아래 회귀는 그 전제 위에서
 * 실제로 보고된 증상과 리뷰에서 나온 결함을 각각 고정한다. */

/* 정속 추종: 차간이 맞으면 설정 속도를 건드리지 않아야 한다. 예전에는 차간·
 * 상대속도(휠 기준)를 설정 속도(클러스터 기준)와 그대로 비교해, 실측 6.6%
 * 차이만으로 SET-이 끝없이 나가 30 km/h 바닥까지 내려갔다. */
void verify_steady_following_issues_no_commands() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  activate(&controller, &vehicle);
  int commands = 0;
  AdaptiveCruiseOutput output;
  for (int i = 1; i <= 12000; ++i) {
    AdaptiveCruiseInput input = base_input(i * 0.01);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    const float wheel_mps = vehicle.wheel_kph() / 3.6f;
    input.vision_lead_distance_m = 5.0f + 1.8f * wheel_mps;
    input.vision_lead_relative_speed_mps = 0.0f;
    output = tick(&controller, input, &vehicle);
    vehicle.settle(0.01f);
    if (output.command_button != 0) ++commands;
  }
  require(near(vehicle.set_speed_kph, 80.0f, 0.001f),
          "steady following must leave the car's set speed alone");
  require(commands == 0, "steady following must issue no button commands");
  require(output.display_scale > 1.03f && output.display_scale < 1.10f,
          "the cluster/wheel scale must be learned from the two speed inputs");
}

/* 해제 뒤 재설정: 이전 주행의 천장이 남으면 선행차가 없는데도 RES+로 속도를
 * 끌어올린다. 고속도로 100 -> 브레이크 -> 시내 60 재설정이 그 경로였다. */
void verify_reengage_does_not_inherit_the_old_ceiling() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  vehicle.set_speed_kph = 100.0f;
  vehicle.cluster_speed_kph = 100.0f;
  activate(&controller, &vehicle);
  double t = 0.0;
  for (int i = 0; i < 300; ++i) {
    AdaptiveCruiseInput input = base_input(t += 0.01);
    input.vision_lead_updated = true;
    tick(&controller, input, &vehicle);
  }
  for (int i = 0; i < 100; ++i) {          // 브레이크: 크루즈 해제
    AdaptiveCruiseInput input = base_input(t += 0.01);
    input.cruise_active = false;
    input.brake_pressed = true;
    tick(&controller, input, &vehicle);
  }
  vehicle.cruise_on = false;
  vehicle.set_speed_kph = 60.0f;
  vehicle.cluster_speed_kph = 60.0f;
  AdaptiveCruiseInput set_input = base_input(t += 0.01);
  set_input.driver_button = 2;
  tick(&controller, set_input, &vehicle);
  AdaptiveCruiseOutput output;
  for (int i = 0; i < 9000; ++i) {         // 90초, 선행차 없음
    AdaptiveCruiseInput input = base_input(t += 0.01);
    input.vision_lead_updated = true;
    output = tick(&controller, input, &vehicle);
    vehicle.settle(0.01f);
  }
  require(near(output.maximum_speed_kph, 60.0f, 1.0f),
          "re-engaging must capture a new ceiling, not inherit the old one");
  require(near(vehicle.set_speed_kph, 60.0f, 1.0f),
          "re-engaging at a lower speed must not be pushed back up");
}

/* 운전자가 SET-을 길게 눌러 속도를 내리는 것은 일상적인 조작이다. 이전 구현은
 * 버튼 누른 순간에 천장 재설정을 무장해 0.5초 뒤 중간값에 굳히고, 그 뒤 운전자가
 * 더 내린 분을 RES+로 되돌렸다. */
void verify_held_driver_set_is_not_undone() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  vehicle.set_speed_kph = 100.0f;
  vehicle.cluster_speed_kph = 100.0f;
  activate(&controller, &vehicle);
  double t = 0.0;
  for (int i = 0; i < 300; ++i) {
    AdaptiveCruiseInput input = base_input(t += 0.01);
    input.vision_lead_updated = true;
    tick(&controller, input, &vehicle);
  }
  for (int i = 0; i < 400; ++i) {          // 4초 길게 누름: 차량이 연속 감속
    AdaptiveCruiseInput input = base_input(t += 0.01);
    input.driver_button = 2;
    input.vision_lead_updated = true;
    tick(&controller, input, &vehicle);
    vehicle.set_speed_kph = std::fmax(60.0f, vehicle.set_speed_kph - 0.1f);
    vehicle.cluster_speed_kph = vehicle.set_speed_kph;
  }
  AdaptiveCruiseOutput output;
  for (int i = 0; i < 9000; ++i) {
    AdaptiveCruiseInput input = base_input(t += 0.01);
    input.vision_lead_updated = true;
    output = tick(&controller, input, &vehicle);
    vehicle.settle(0.01f);
  }
  require(near(vehicle.set_speed_kph, 60.0f, 1.0f),
          "a held driver SET- must not be pushed back up afterwards");
  require(near(output.maximum_speed_kph, 60.0f, 1.0f),
          "the ceiling must follow where the driver actually left the car");
}

/* 차량이 버튼을 무시하면(펄스 유실, 배선 문제) 예전에는 분당 45펄스를 영원히
 * 쐈다. 듣지 않는다고 판단하면 물러나야 한다. */
void verify_backs_off_when_the_car_ignores_commands() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  activate(&controller, &vehicle);
  vehicle.drop_every = 1;                  // 모든 버튼을 흘린다
  int presses = 0, previous = 0;
  for (int i = 1; i <= 6000; ++i) {
    AdaptiveCruiseInput input = base_input(i * 0.01);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    input.vision_lead_distance_m = 12.0f;
    input.vision_lead_relative_speed_mps = -5.0f;
    const AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
    if (output.command_button != 0 && previous == 0) ++presses;
    previous = output.command_button;
  }
  require(presses > 0, "an unresponsive car must still be tried at first");
  require(presses <= 20,
          "an unresponsive car must not be pressed indefinitely");
}

/* 클러스터/휠 비가 학습 범위를 벗어나면 두 척도를 비교할 근거가 없다.
 * 그 상태에서 판단하면 원래 버그가 그대로 재현된다. */
void verify_no_commands_without_a_learned_scale() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  vehicle.over_read = 1.60f;               // 범위 밖
  activate(&controller, &vehicle);
  int commands = 0;
  for (int i = 1; i <= 6000; ++i) {
    AdaptiveCruiseInput input = base_input(i * 0.01);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    const float wheel_mps = vehicle.wheel_kph() / 3.6f;
    input.vision_lead_distance_m = 5.0f + 1.8f * wheel_mps;
    input.vision_lead_relative_speed_mps = 0.0f;
    if (tick(&controller, input, &vehicle).command_button != 0) ++commands;
    vehicle.settle(0.01f);
  }
  require(commands == 0,
          "an implausible cluster/wheel ratio must block all commands");
  require(near(vehicle.set_speed_kph, 80.0f, 0.001f),
          "an implausible cluster/wheel ratio must leave the set speed alone");
}

/* 차량 스텝이 2 km/h가 아니면(롱프레스형 10 km/h) 예전에는 SET-/RES+가 번갈아
 * 나오며 설정 속도가 무한 왕복했다. */
void verify_no_limit_cycle_with_a_coarse_vehicle_step() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  vehicle.step_kph = 10.0f;
  activate(&controller, &vehicle);
  int presses = 0, previous = 0;
  for (int i = 1; i <= 6000; ++i) {
    AdaptiveCruiseInput input = base_input(i * 0.01);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    const float wheel_mps = vehicle.wheel_kph() / 3.6f;
    input.vision_lead_distance_m = 5.0f + 1.8f * wheel_mps;
    input.vision_lead_relative_speed_mps = 0.0f;
    const AdaptiveCruiseOutput output = tick(&controller, input, &vehicle);
    if (output.command_button != 0 && previous == 0) ++presses;
    previous = output.command_button;
    vehicle.settle(0.01f);
  }
  require(presses <= 2, "a coarse vehicle step must not produce a limit cycle");
}

/* 느린 선행차는 실제로 따라가야 한다 — 위 안전장치들이 기능 자체를 죽이지
 * 않았는지 확인한다. */
void verify_slower_lead_is_actually_followed() {
  AdaptiveCruiseController controller;
  Vehicle vehicle;
  activate(&controller, &vehicle);
  float gap = 60.0f;
  const float lead_mps = 50.0f / 3.6f;
  for (int i = 1; i <= 18000; ++i) {       // 180초
    AdaptiveCruiseInput input = base_input(i * 0.01);
    const float ego_mps = vehicle.wheel_kph() / 3.6f;
    gap = std::fmax(1.0f, gap + (lead_mps - ego_mps) * 0.01f);
    input.vision_lead_updated = true;
    input.vision_lead_valid = true;
    input.vision_lead_distance_m = gap;
    input.vision_lead_relative_speed_mps = lead_mps - ego_mps;
    tick(&controller, input, &vehicle);
    vehicle.settle(0.01f);
  }
  const float ego_kph = vehicle.wheel_kph();
  require(near(ego_kph, 50.0f, 3.0f),
          "a slower lead must actually be followed at its speed");
  const float desired_gap = 5.0f + 1.8f * ego_kph / 3.6f;
  require(near(gap, desired_gap, 5.0f),
          "a slower lead must be followed at roughly the desired gap");
}

void verify_repository_config_loads() {
  AdaptiveCruiseConfig config;
  std::string error;
  require(load_adaptive_cruise_params_json(
              "params/adaptive_cruise.json", &config, &error),
          error.c_str());
  require(config.enabled && near(config.following_time_s, 1.8f, 0.001f) &&
              near(config.standstill_gap_m, 5.0f, 0.001f) &&
              near(config.deceleration_rate_kph_per_s, 1.5f, 0.001f) &&
              near(config.command_interval_s, 1.0f, 0.001f) &&
              config.button_pulse_frames == 5,
          "repository adaptive cruise defaults do not match the runtime schema");
}

}  // namespace

int main() {
  return run_checks("ADAPTIVE_CRUISE_OK", [] {
    verify_close_lead_follows_measured_deceleration_rate();
    verify_closing_lead_prediction_and_resume_delay();
    verify_lead_loss_holds_then_restores_maximum();
    verify_driver_and_pedal_gates();
    verify_session_reset_and_minimum_speed();
    verify_runtime_config_update();
    verify_steady_following_issues_no_commands();
    verify_reengage_does_not_inherit_the_old_ceiling();
    verify_held_driver_set_is_not_undone();
    verify_backs_off_when_the_car_ignores_commands();
    verify_no_commands_without_a_learned_scale();
    verify_no_limit_cycle_with_a_coarse_vehicle_step();
    verify_slower_lead_is_actually_followed();
    verify_repository_config_loads();
  });
}
