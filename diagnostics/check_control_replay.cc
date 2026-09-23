#include "can_replay.h"
#include "check_harness.h"
#include "control_fixtures.h"
#include "control_holds.h"
#include "hyundai_can.h"
#include "ipc_messages.h"
#include "lateral_controller.h"
#include "model_output.h"
#include "lateral_path.h"
#include "lateral_torque.h"
#include "vehicle_can.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

LateralPath replay_path() {
  LateralPath path;
  path.left_valid = true;
  path.right_valid = true;
  path.usable_for_steering = true;
  path.point_count = 30;
  path.reach_m = 60.0f;
  return path;
}

LateralTarget replay_target() {
  LateralTarget target;
  target.valid = true;
  target.mpc_solution_valid = true;
  for (int i = 0; i < kLateralControlN; ++i) {
    target.psis[i] = 0.0008f * 20.0f * (10.0f * i * i / (32.0f * 32.0f));
    target.curvatures[i] = 0.0008f;
  }
  return target;
}

/* lag 보상 곡률의 독립 전사본. 한계값은 lateral_controller.h에서 그대로
 * 가져온다 — 숫자를 복제하면 구현이 바뀔 때 이 검증이 조용히 썩는다.
 * 픽스처 target은 capture_timestamp_ns=0이라 plan 나이 보정은 0이다. */
float reference_lag_adjusted_curvature(const LateralTarget &target, float speed_mps,
                                       float actuator_delay, float prev_curvature) {
  const float delay = std::max(0.01f, actuator_delay);
  float psi = target.psis[kLateralControlN - 1];
  if (delay <= 0.0f) {
    psi = target.psis[0];
  } else {
    for (int i = 1; i < kLateralControlN; ++i) {
      if (delay <= model_t_idx(i)) {
        const float p = (delay - model_t_idx(i - 1)) /
                        (model_t_idx(i) - model_t_idx(i - 1));
        psi = target.psis[i - 1] + p * (target.psis[i] - target.psis[i - 1]);
        break;
      }
    }
  }
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  const float current = target.curvatures[0];
  float desired = current + 2.0f * (psi / (speed * delay) - current);
  const float rate_limit = kMaxLateralJerk / (speed * speed);
  desired = std::clamp(desired,
                       prev_curvature - rate_limit * kCurvatureRateWindowS,
                       prev_curvature + rate_limit * kCurvatureRateWindowS);
  const float accel_speed = std::max(speed, 1.0f);
  desired = std::clamp(desired,
                       -kMaxLateralAccel / (accel_speed * accel_speed),
                       kMaxLateralAccel / (accel_speed * accel_speed));
  return std::clamp(desired, -kMaxCurvature, kMaxCurvature);
}

void verify_mdps_speed_spoof() {
  HyundaiCanConfig config;
  config.main_bus = 0;
  config.mdps_bus = 1;
  config.send_lkas_on_scc_bus = false;
  HyundaiLkas11Values lkas;
  HyundaiClu11Values clu;
  clu.speed_decimal = 0.375f;
  HyundaiLkasCommand command;
  command.steer_req = true;
  const auto frames = build_lateral_can_frames(
      lkas, clu, command, config, true, 20.0f, false, 1);
  require(frames.size() == 3, "low-speed frame schedule");
  std::array<uint8_t, 4> bytes = {};
  std::copy_n(frames[2].data.begin(), bytes.size(), bytes.begin());
  const HyundaiClu11Values decoded = decode_clu11(bytes);
  require(frames[2].address == kHyundaiClu11Address && frames[2].bus == 1,
          "MDPS CLU11 bus");
  require(std::fabs(decoded.speed - 60.0f) < 0.001f,
          "MDPS CLU11 60 kph spoof");
  require(std::fabs(decoded.speed_decimal - 0.375f) < 0.001f,
          "MDPS CLU11 decimal preservation");

  config.mdps_speed_spoof_kph = 72.0f;
  const auto custom_frames = build_lateral_can_frames(
      lkas, clu, command, config, true, 20.0f, false, 1);
  std::copy_n(custom_frames[2].data.begin(), bytes.size(), bytes.begin());
  require(std::fabs(decode_clu11(bytes).speed - 72.0f) < 0.001f,
          "configured MDPS speed spoof");
}

void verify_lca11() {
  std::array<uint8_t, 8> bytes{};
  bytes[1] = 1;
  bytes[2] = 2;
  const Lca11Values decoded = decode_lca11(bytes);
  require(decoded.left_blindspot && decoded.right_blindspot,
          "LCA11 blind-spot decoding");

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiLca11Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  require(vehicle.left_blindspot && vehicle.right_blindspot,
          "LCA11 vehicle-state update");
}

void verify_whl_spd11() {
  std::array<uint8_t, 8> bytes{};
  const auto set_speed_raw = [&bytes](int start_bit, float speed_kph) {
    set_signal_le(&bytes, start_bit, 14,
                  static_cast<uint32_t>(std::lround(speed_kph / 0.03125f)));
  };
  set_speed_raw(0, 40.0f);
  set_speed_raw(16, 41.0f);
  set_speed_raw(32, 39.0f);
  set_speed_raw(48, 40.5f);

  const WhlSpd11Values decoded = decode_whl_spd11(bytes);
  require(std::fabs(decoded.speed_fl_kph - 40.0f) < 0.001f &&
              std::fabs(decoded.speed_fr_kph - 41.0f) < 0.001f &&
              std::fabs(decoded.speed_rl_kph - 39.0f) < 0.001f &&
              std::fabs(decoded.speed_rr_kph - 40.5f) < 0.001f,
          "WHL_SPD11 wheel speed decoding");

  VehicleCanState vehicle;
  vehicle.cluster_speed_raw = 72.0f;
  update_vehicle_can_state(&vehicle, kHyundaiWhlSpd11Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  require(vehicle.whl_spd11_time_s == 1.0 &&
              std::fabs(vehicle_speed_kph(vehicle, 1.2) - 40.125f) < 0.001f,
          "WHL_SPD11 vehicle speed average");
  /* 클러스터로 대체하지 않는다: 도메인이 달라 최소 조향 속도 게이트가
   * 뒤집힌다. 대신 낡은 휠 속도는 vehicle_state_fresh에서 막힌다. */
  require(!std::isfinite(vehicle_speed_kph(vehicle, 1.6)),
          "stale WHL_SPD11 must not fall back to CLU speed");
  require(!vehicle_state_fresh(vehicle, 1.6, 0.5),
          "stale WHL_SPD11 must block control");
}

void verify_tpms11() {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 11, 2, 2);
  bytes[2] = 23;
  bytes[3] = 24;
  bytes[4] = 25;
  bytes[5] = 26;
  const Tpms11Values decoded = decode_tpms11(bytes);
  require(decoded.unit == 2 &&
              std::fabs(decoded.pressure_fl - 2.3f) < 0.001f &&
              std::fabs(decoded.pressure_fr - 2.4f) < 0.001f &&
              std::fabs(decoded.pressure_rl - 2.5f) < 0.001f &&
              std::fabs(decoded.pressure_rr - 2.6f) < 0.001f &&
              !decoded.warning,
          "TPMS11 pressure decoding");

  bytes[0] |= 1U << 4;
  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiTpms11Address, bytes, 6,
                           kPowertrainBus, 1.0);
  require(tpms_state_fresh(vehicle, 2.0) && vehicle.tpms_warning,
          "TPMS11 vehicle-state update");
  require(!tpms_state_fresh(vehicle, 7.0),
          "TPMS11 freshness timeout");

  bytes[2] = 0xff;
  update_vehicle_can_state(&vehicle, kHyundaiTpms11Address, bytes, 6,
                           kPowertrainBus, 8.0);
  require(tpms_state_fresh(vehicle, 8.0) &&
              vehicle.tpms_pressure_fl == 0.0f &&
              vehicle.tpms_pressure_fr > 0.0f,
          "TPMS11 unavailable wheel pressure");
}

void verify_tcs15() {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 29, 3, 2);
  const Tcs15Values decoded = decode_tcs15(bytes);
  require(decoded.brake_hold && !decoded.esp_disabled,
          "TCS15 active Auto Hold decoding");

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiTcs15Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  require(vehicle.brake_hold && vehicle.tcs15_time_s == 1.0,
          "TCS15 Auto Hold vehicle-state update");

  set_signal_le(&bytes, 29, 3, 3);
  update_vehicle_can_state(&vehicle, kHyundaiTcs15Address, bytes,
                           bytes.size(), kPowertrainBus, 1.1);
  require(!vehicle.brake_hold, "TCS15 ready state is not active Auto Hold");
}

void verify_tcs13_driver_override() {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 45, 2, 2);
  const Tcs13Values decoded = decode_tcs13(bytes);
  require(decoded.driver_override == 2,
          "TCS13 driver accelerator override decoding");

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  require(vehicle.driver_override == 2,
          "TCS13 driver accelerator override vehicle-state update");
}

void verify_scc11() {
  std::array<uint8_t, 8> bytes{};
  bytes[0] = 1;
  bytes[1] = 88;
  set_signal_le(&bytes, 22, 2, 1);
  set_signal_le(&bytes, 33, 11, 54);
  set_signal_le(&bytes, 44, 12, 1715);
  const Scc11Values decoded = decode_scc11(bytes);
  require(decoded.main_mode && std::fabs(decoded.set_speed_raw - 88.0f) < 0.001f &&
              decoded.object_valid &&
              std::fabs(decoded.object_distance_m - 5.4f) < 0.001f &&
              std::fabs(decoded.object_relative_speed_mps - 1.5f) < 0.001f,
          "SCC11 cruise state decoding");

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiScc11Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  require(vehicle.cruise_main &&
              std::fabs(vehicle.cruise_set_speed_raw - 88.0f) < 0.001f &&
              vehicle.radar_lead_valid &&
              std::fabs(vehicle.radar_lead_distance_m - 5.4f) < 0.001f &&
              std::fabs(vehicle.radar_lead_relative_speed_mps - 1.5f) < 0.001f,
          "SCC11 vehicle-state update");
  require(std::fabs(cruise_set_speed_kph(vehicle) - 88.0f) < 0.001f,
          "SCC11 metric set speed");
  vehicle.speed_unit_mph = true;
  require(std::fabs(cruise_set_speed_kph(vehicle) - 141.622272f) < 0.001f,
          "SCC11 imperial set speed conversion");
}

void update_clu11(VehicleCanState *vehicle, float speed, int button,
                  bool unit_mph, double now_s) {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 0, 3, static_cast<uint32_t>(button));
  set_signal_le(&bytes, 8, 9,
                static_cast<uint32_t>(std::lround(speed * 2.0f)));
  set_signal_le(&bytes, 17, 1, unit_mph ? 1U : 0U);
  update_vehicle_can_state(vehicle, kHyundaiClu11Address, bytes, 4,
                           kPowertrainBus, now_s);
}

void release_cruise_button(VehicleCanState *vehicle, float speed,
                           bool unit_mph, double now_s) {
  update_clu11(vehicle, speed, 0, unit_mph, now_s);
}

void verify_fixed_cruise_speed_estimate() {
  VehicleCanState vehicle;
  update_clu11(&vehicle, 64.0f, 2, false, 1.0);
  require(vehicle.cruise_active &&
              std::fabs(cruise_set_speed_kph(vehicle) - 64.0f) < 0.001f,
          "fixed cruise SET must latch cluster speed");

  release_cruise_button(&vehicle, 64.0f, false, 1.1);
  update_clu11(&vehicle, 64.0f, 1, false, 1.2);
  require(std::fabs(cruise_set_speed_kph(vehicle) - 66.0f) < 0.001f,
          "fixed cruise RES must increment active target");
  update_clu11(&vehicle, 64.0f, 1, false, 1.3);
  require(std::fabs(cruise_set_speed_kph(vehicle) - 66.0f) < 0.001f,
          "held cruise button must not repeat without release");

  release_cruise_button(&vehicle, 64.0f, false, 1.4);
  update_clu11(&vehicle, 64.0f, 2, false, 1.5);
  require(std::fabs(cruise_set_speed_kph(vehicle) - 64.0f) < 0.001f,
          "fixed cruise SET must decrement active target");
  release_cruise_button(&vehicle, 64.0f, false, 1.6);
  update_clu11(&vehicle, 64.0f, 4, false, 1.7);
  require(!vehicle.cruise_active &&
              std::fabs(cruise_set_speed_kph(vehicle) - 64.0f) < 0.001f,
          "fixed cruise CANCEL must preserve target");
  release_cruise_button(&vehicle, 70.0f, false, 1.8);
  update_clu11(&vehicle, 70.0f, 2, false, 1.9);
  require(vehicle.cruise_active &&
              std::fabs(cruise_set_speed_kph(vehicle) - 70.0f) < 0.001f,
          "fixed cruise SET after cancel must latch current speed");

  std::array<uint8_t, 8> tcs13{};
  set_signal_le(&tcs13, 55, 1, 1);
  update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13,
                           tcs13.size(), kPowertrainBus, 2.0);
  require(!vehicle.cruise_active, "brake must cancel estimated cruise activity");
  set_signal_le(&tcs13, 55, 1, 0);
  update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13,
                           tcs13.size(), kPowertrainBus, 2.1);
  release_cruise_button(&vehicle, 60.0f, false, 2.2);
  update_clu11(&vehicle, 60.0f, 1, false, 2.3);
  require(vehicle.cruise_active &&
              std::fabs(cruise_set_speed_kph(vehicle) - 70.0f) < 0.001f,
          "RES after brake must restore estimated target");

  release_cruise_button(&vehicle, 70.0f, false, 2.4);
  std::array<uint8_t, 8> clu_main{};
  set_signal_le(&clu_main, 3, 1, 1);
  set_signal_le(&clu_main, 8, 9, 140);
  update_vehicle_can_state(&vehicle, kHyundaiClu11Address, clu_main, 4,
                           kPowertrainBus, 2.5);
  require(!vehicle.cruise_active &&
              std::fabs(cruise_set_speed_kph(vehicle) - 70.0f) < 0.001f,
          "cruise MAIN press must deactivate and preserve target");

  VehicleCanState imperial;
  update_clu11(&imperial, 40.0f, 2, true, 1.0);
  require(std::fabs(cruise_set_speed_kph(imperial) - 64.37376f) < 0.001f,
          "fixed cruise imperial SET conversion");
  release_cruise_button(&imperial, 40.0f, true, 1.1);
  update_clu11(&imperial, 40.0f, 1, true, 1.2);
  require(std::fabs(cruise_set_speed_kph(imperial) - 67.592448f) < 0.001f,
          "fixed cruise imperial increment");

  std::array<uint8_t, 8> scc11{};
  scc11[1] = 88;
  update_vehicle_can_state(&imperial, kHyundaiScc11Address, scc11,
                           scc11.size(), kPowertrainBus, 1.3);
  require(std::fabs(cruise_set_speed_kph(imperial) - 141.622272f) < 0.001f,
          "valid SCC set speed must override fixed cruise estimate");
}

void verify_mdps_fault_filter() {
  VehicleCanState vehicle;
  std::array<uint8_t, 8> bytes{};
  bytes[1] = (1U << 6) | (1U << 7);
  update_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                           bytes.size(), kMdpsBus, 1.0);
  require(vehicle.mdps_hard_fault && !vehicle.steering_fault,
          "transient MDPS ToiFlt/FailStat must match openpilot filtering");

  bytes[1] = 1U << 4;
  for (int frame = 0; frame < 100; ++frame) {
    update_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                             bytes.size(), kMdpsBus, 1.0 + frame * 0.02);
  }
  require(!vehicle.steering_fault, "MDPS unavailable debounce threshold");
  update_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                           bytes.size(), kMdpsBus, 3.0);
  require(vehicle.steering_fault, "sustained MDPS unavailable fault");
}

void verify_braking_does_not_disengage() {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음
  vehicle.brake_light = true;

  const auto brake_light_result =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  require(brake_light_result.active,
          "brake light without DriverBraking must remain active");

  vehicle.brake_pressed = true;
  const auto brake_pressed_result =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  require(brake_pressed_result.active,
          "DriverBraking must not disengage lateral control");
}

/* K7 MDPS는 steer 요청이 켜진 채 85도 위에 1초 머물면 fault를 낸다(2026-09-18 실측).
 * 85도 위: 토크를 램프로 0까지 내리고 steer 요청은 유지, 89프레임마다 2프레임 컷.
 * 램프는 fault 실측 하한(98프레임)보다 먼저 끝나야 하고, 컷과 복귀가 토크 0에서
 * 일어나야 어시스트가 빠졌다 돌아오는 충격이 없다. 즉시 0으로 떨어뜨리지 않는 것은
 * 짧게 스치는 커브에서 어시스트를 유지하기 위해서다. */
void verify_large_angle_fault_avoidance() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 72.0f;
  int frame = 0;
  auto step = [&]() {
    const int f = frame++;
    stamp_can_times(&vehicle, 1.0 + f * 0.01);  // 4초 넘게 돌리므로 CAN 신선도를 유지한다
    return controller.update(replay_path(), replay_target(), vehicle, 1.0 + f * 0.01, f);
  };

  vehicle.steering_angle_deg = 20.0f;
  LateralControlResult result;
  for (int i = 0; i < 100; ++i) result = step();
  require(result.active && result.apply_torque != 0, "torque flows below the MDPS fault angle");

  vehicle.steering_angle_deg = 100.0f;
  const int crossed = frame;
  const int ramp = config.steering_params.avoid_lkas_fault_max_frames - 20;
  while (frame < crossed + 89) {
    result = step();
    const int since = frame - crossed;
    require(result.active && !result.cut_steer_temp,
            "large-angle control must remain requested before RK fault limit");
    if (since == 1)
      require(result.desired_torque != 0,
              "the first frame above the fault angle keeps assist");
    if (since >= ramp)
      require(result.desired_torque == 0,
              "the angle ramp reaches zero well before the measured fault time");
    if (frame > crossed + 80)
      require(result.apply_torque == 0, "torque must have ramped to zero before the first cut");
  }
  for (int i = 0; i < config.steering_params.avoid_lkas_fault_cut_frames; ++i) {
    result = step();
    require(result.active && result.cut_steer_temp && !result.frames.empty(),
            "large-angle fault avoidance must cut request without disengaging");
    const HyundaiLkas11Values lkas = decode_lkas11(result.frames.front().data);
    require(!lkas.steer_req && lkas.toi_fault,
            "fault-avoidance LKAS11 request and temporary-fault bits");
    require(lkas.steer_torque == 0 && result.apply_torque == 0,
            "the cut must happen at zero torque");
  }
  result = step();
  require(result.active && !result.cut_steer_temp && !result.frames.empty(),
          "large-angle steering request must resume after the configured cut");
  const HyundaiLkas11Values lkas = decode_lkas11(result.frames.front().data);
  require(lkas.steer_req && !lkas.toi_fault && result.apply_torque == 0,
          "resumed LKAS11 request keeps zero torque above the fault angle");

  vehicle.steering_angle_deg = 20.0f;
  for (int i = 0; i < 30; ++i) result = step();
  require(result.active && result.apply_torque != 0,
          "torque resumes with the normal ramp below the fault angle");
}

// 정지 부근 path 깜빡임: active 재진입은 0.5s 연속 유효 후에만.
void verify_path_flicker_debounce() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  LateralPath bad = replay_path();
  bad.usable_for_steering = false;
  double t = 1.0;
  auto step = [&](const LateralPath &path) {
    VehicleCanState vehicle = ready_vehicle(t);
    const auto r = controller.update(path, replay_target(), vehicle, t, 0);
    t += 0.01;
    return r;
  };
  require(step(replay_path()).active, "initial valid path must activate");
  require(!step(bad).active, "invalid path must deactivate immediately");
  int reactivated = 0;
  for (int i = 0; i < 20; ++i) reactivated += step(replay_path()).active ? 1 : 0;
  require(reactivated == 0, "path flicker must not reactivate before the hold");
  require(!step(bad).active, "still inactive on the next dropout");
  int active_after = 0;
  for (int i = 0; i < 60; ++i) active_after = step(replay_path()).active ? 1 : 0;
  require(active_after == 1, "sustained valid path must reactivate after the hold");
}

// 정차(path 무효)에서도 engage는 받아야 한다 — 조향만 쉰다.
void verify_engage_allowed_with_unavailable_path() {
  LateralControllerConfig config;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  LateralPath bad = replay_path();
  bad.usable_for_steering = false;
  VehicleCanState vehicle = ready_vehicle(1.0);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
  vehicle.clu_button = 2;
  controller.update(bad, replay_target(), vehicle, 1.0, 0, true, true);
  vehicle.clu_button = 0;
  const auto engaged =
      controller.update(bad, replay_target(), vehicle, 1.01, 1, true, true);
  require(engaged.engaged && !engaged.engage_rejected,
          "standstill engage must be accepted with an unavailable path");
  // 정지 + path 무효는 오류가 아니라 대기 상태로 보고한다
  require(!engaged.active && engaged.active_block == BlockReason::Stopped,
          "standstill without a path must report stopped, not an error");
  // 대기 중에도 steer_req/스푸프는 유지(토크 0) — 정차 천이 부저 방지
  require(!engaged.frames.empty() &&
              decode_lkas11(engaged.frames.front().data).steer_req &&
              decode_lkas11(engaged.frames.front().data).steer_torque == 0,
          "availability wait must hold steer_req with zero torque");
  // 주행 중 path 무효는 진짜 문제로 보고한다
  vehicle = ready_vehicle(1.02);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  const auto rolling =
      controller.update(bad, replay_target(), vehicle, 1.02, 2, true, true);
  require(!rolling.active && rolling.active_block == BlockReason::PathInvalid,
          "unusable path while moving must report path_invalid");
  // 결함은 가용성 대기보다 우선한다 (정차 중 문 열림 -> hard disengage)
  vehicle = ready_vehicle(1.03);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
  vehicle.door_open = true;
  const auto door =
      controller.update(bad, replay_target(), vehicle, 1.03, 3, true, true);
  require(door.active_block == BlockReason::DoorOpen && !door.engaged,
          "faults must outrank availability and hard-disengage at standstill");
}

void verify_fixed_max_curvature() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 3.6f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 3.6f;
  vehicle.cluster_speed_raw = 3.6f;

  LateralTarget target = replay_target();
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = 0.5f;
    target.psis[i] = 0.23f;
  }
  /* 틱당 변화율 제한이 걸리므로 한 번에 상한까지 뛰지 않는다. 1 m/s에서
   * 창은 5/(1*1)*0.01 = 0.05 1/m 다. */
  const auto first = controller.update(replay_path(), target, vehicle, 1.0, 0);
  require(first.active && std::fabs(first.desired_curvature - 0.05f) < 1e-6f,
          "the first tick must move by one lateral-jerk step, not jump to the limit");
  LateralControlResult result = first;
  for (int tick = 1; tick < 40; ++tick)
    result = controller.update(replay_path(), target, vehicle,
                               1.0 + 0.01 * tick, tick);
  require(std::fabs(result.desired_curvature - 0.2f) < 1e-6f,
          "maximum curvature must remain fixed at the openpilot 0.2 1/m");
}

/* v0.11식 지연 보정: 요청 스텝 직후 delay 동안은 P가 과거 요청(0)과 현재
 * 측정(0)을 비교해 오차가 없어야 하고, 토크는 FF만으로 나와야 한다. */
void verify_delay_compensated_error() {
  TorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  params.torque_use_angle = true;
  // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
  params.angle_offset_deg = 0.0f;
  const float v = 20.0f;

  // 요청 0으로 버퍼를 채운다
  for (int i = 0; i < 120; ++i)
    torque.update(true, v, 0.0f, 0.0f, false, false, params);
  require(std::fabs(torque.error()) < 1e-6f, "steady zero request has no error");

  // 곡률 스텝. 조향각은 아직 0(차가 반응 전).
  torque.update(true, v, 0.01f, 0.0f, false, false, params);
  require(std::fabs(torque.error()) < 1e-4f,
          "error must stay ~0 right after a step (delay compensation)");
  require(torque.feedforward() > 1.0f,
          "feedforward must carry the step immediately");

  // delay(31프레임)를 넘겨도 차가 반응하지 않으면 그때 오차가 나타난다
  for (int i = 0; i < 40; ++i)
    torque.update(true, v, 0.01f, 0.0f, false, false, params);
  require(torque.error() > 1.0f,
          "unmet request must surface as error after the delay");
}

// inactive 동안에도 요청 버퍼가 갱신되어야 재engage 때 낡은 요청과 비교되지 않는다.
void verify_reengage_has_no_stale_buffer_spike() {
  TorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  params.torque_use_angle = true;
  // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
  params.angle_offset_deg = 0.0f;
  const float v = 20.0f;

  // 커브 요청으로 버퍼를 채운 뒤 disengage
  for (int i = 0; i < 120; ++i)
    torque.update(true, v, 0.01f, 0.0f, false, false, params);
  // inactive 동안 요청은 0으로 돌아간다 (직선 수동 주행)
  for (int i = 0; i < 120; ++i)
    torque.update(false, v, 0.0f, 0.0f, false, false, params);
  // 직선에서 re-engage: 버퍼가 신선하면 오차 ~0, 얼었다면 큰 스파이크
  torque.update(true, v, 0.0f, 0.0f, false, false, params);
  require(std::fabs(torque.error()) < 1e-4f,
          "re-engage must not compare against stale pre-disengage requests");
}

/* 속도별 비례 이득(openpilot KP_INTERP). 오차는 횡가속도만으로 내고 저속 보강은
 * 이 곡선이 맡는다. 곡률 오차 항(LOW_SPEED_Y)은 더 이상 없다. */
void verify_kp_speed_schedule() {
  SteeringParams base;
  base.enabled = true;
  base.torque_use_angle = true;
  base.angle_offset_deg = 0.0f;
  base.torque_friction = 0.0f;   // P항만 남긴다
  base.torque_ki = 0.0f;
  /* 조향각 3도를 실제 곡률로 두고 요청 곡률 0을 준다. 오차 = -actual_lat_accel이라
   * 출력은 kp(v) x kf x v^2 x |actual_curvature|에 비례한다. */
  const auto run = [&](float v, float kp) {
    SteeringParams p = base;
    p.torque_kp = kp;
    TorqueController torque;
    for (int i = 0; i < 120; ++i) torque.update(true, v, 0.0f, 3.0f, false, false, p);
    return torque.normalized_output();
  };
  // 이득 곡선의 노드에서 출력비가 KP_INTERP 비율 x v^2 비율과 맞아야 한다.
  const auto gain_at = [&](float v) {
    const float out = run(v, 0.8f);
    TorqueController probe;
    SteeringParams p = base;
    for (int i = 0; i < 120; ++i) probe.update(true, v, 0.0f, 3.0f, false, false, p);
    return std::fabs(out / (probe.error() == 0.0f ? 1.0f : probe.error()));
  };
  // 5 m/s 노드는 11.5, 10 m/s 노드는 3.5 -> 이득비 3.2857
  const float g5 = gain_at(5.0f), g10 = gain_at(10.0f);
  require(g5 > 0.0f && g10 > 0.0f, "the schedule must produce gain at both nodes");
  const float ratio = g5 / g10;
  require(ratio > 3.2f && ratio < 3.4f,
          "the 5 m/s node must be 11.5/3.5 times the 10 m/s node");
  // 30 m/s 위는 torque_kp가 그대로 끝점이다.
  const float top8 = std::fabs(run(35.0f, 0.8f));
  const float top16 = std::fabs(run(35.0f, 1.6f));
  require(top8 > 1e-4f && top8 < 0.95f, "the top-of-curve case must be unsaturated");
  require(std::fabs(top16 / top8 - 2.0f) < 0.05f,
          "above 30 m/s the gain must scale with torque_kp");
  /* LOW_SPEED_Y가 남아 있으면 곡률 항이 저속에서 오차를 수십 배로 키운다.
   * 오차가 순수 횡가속도인지 확인한다. */
  TorqueController t;
  SteeringParams p = base;
  for (int i = 0; i < 120; ++i) t.update(true, 4.0f, 0.002f, 3.0f, false, false, p);
  const float curvature_error = 0.002f - t.actual_curvature();
  require(std::fabs(t.error() - curvature_error * 16.0f) < 1e-4f,
          "the error must be lateral acceleration only, with no low-speed curvature term");
}

// 라이브 뱅크: 편경사에 해당하는 만큼 FF가 이동해야 한다.
void verify_live_bank_compensation() {
  TorqueController with_bank, without_bank;
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  params.live_bank_compensation = true;
  SteeringParams off = params;
  off.live_bank_compensation = false;
  for (int i = 0; i < 120; ++i) {
    with_bank.update(true, 20.0f, 0.002f, 1.0f, false, false, params,
                     0.0f, false, -0.117f);
    without_bank.update(true, 20.0f, 0.002f, 1.0f, false, false, off,
                        0.0f, false, -0.117f);
  }
  // bank -0.117(우측 기움) -> 중력이 우로 끄니 FF는 좌로 0.117 이동해야 한다
  const float diff = with_bank.feedforward() - without_bank.feedforward();
  require(std::fabs(diff + 0.117f) < 1e-3f,
          "live bank must shift feedforward by +bank (gravity opposes)");
}

// 라이브 뱅크: 커브(|yaw*v| >= 0.4)에서는 갱신을 멈추고 직선 값을 유지해야 한다.
void verify_bank_holds_during_curves() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  double t = 1.0;
  auto step = [&](float yaw_rate_rad_s, float lat_accel_mps2) {
    VehicleCanState vehicle = ready_vehicle(t);
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
    vehicle.cluster_speed_raw = 63.0f;
    vehicle.yaw_rate_valid = true;
    vehicle.yaw_rate_rad_s = yaw_rate_rad_s;
    vehicle.lat_accel_valid = true;
    vehicle.lat_accel_mps2 = lat_accel_mps2;
    controller.update(replay_path(), replay_target(), vehicle, t, 0);
    t += 0.01;
  };
  for (int i = 0; i < 1500; ++i) step(0.0f, -0.117f);  // 직선 크라운
  const float straight_bank = controller.road_bank_lat_accel();
  require(std::fabs(straight_bank + 0.117f) < 5e-3f,
          "bank must converge to crown on straights");
  // 커브: yaw*v = +1.2, 롤 누설 +0.5 (기구학 성분 상쇄 후 잔여)
  const float v = 60.0f / 3.6f;
  for (int i = 0; i < 500; ++i) step(1.2f / v, -1.2f + 0.5f);
  require(std::fabs(controller.road_bank_lat_accel() - straight_bank) < 1e-4f,
          "bank must hold during curves, not track roll leak");
}

// latAccelOffset: 상수 편향이 FF에서 그대로 빠져야 한다.
void verify_lat_accel_offset_shifts_feedforward() {
  TorqueController a, b;
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  SteeringParams offset_params = params;
  offset_params.torque_lat_accel_offset = 0.25f;
  for (int i = 0; i < 120; ++i) {
    a.update(true, 20.0f, 0.002f, 1.0f, false, false, params);
    b.update(true, 20.0f, 0.002f, 1.0f, false, false, offset_params);
  }
  const float diff = a.feedforward() - b.feedforward();
  require(std::fabs(diff - 0.25f) < 1e-3f,
          "lat_accel_offset must subtract from feedforward exactly");
}

void verify_runtime_params_apply_immediately() {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 72.0f;

  const auto active =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  require(active.active, "runtime parameter test must start active");

  SteeringParams steering = config.steering_params;
  steering.enabled = false;
  controller.update_params(steering, config.driving_params);
  const auto disabled =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  require(!disabled.active && disabled.active_block == BlockReason::ControllerDisabled,
          "runtime steering parameters must apply on the next control tick");

  steering.enabled = true;
  controller.update_params(steering, config.driving_params);
  const auto resumed =
      controller.update(replay_path(), replay_target(), vehicle, 1.02, 2);
  require(resumed.active,
          "runtime parameter update must preserve controller operation");
}

void verify_lkas_hud_state_stability() {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

  LateralPath no_lane_path = replay_path();
  no_lane_path.left_valid = false;
  no_lane_path.right_valid = false;
  const auto active =
      controller.update(no_lane_path, replay_target(), vehicle, 1.0, 0);
  require(active.active && !active.frames.empty(), "active HUD test frame");
  require(decode_lkas11(active.frames.front().data).ldws_sys_state == 3,
          "active HUD state must remain active with fluctuating lane probability");

  /* 클러스터는 sys_state 천이마다 부저를 울리므로, sys_state는 active가
   * 아니라 engaged만 따른다. enable/disable에서만 천이가 생긴다. */
  LateralTarget invalid_target = replay_target();
  invalid_target.mpc_solution_valid = false;
  const auto inactive =
      controller.update(no_lane_path, invalid_target, vehicle, 2.5, 2);
  require(!inactive.active && !inactive.frames.empty(), "inactive HUD test frame");
  require(decode_lkas11(inactive.frames.front().data).ldws_sys_state == 3,
          "engaged but inactive must keep sys_state to avoid chimes");
}

void verify_panda_gate_and_handoff() {
  LateralControllerConfig config;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

  vehicle.clu_button = 2;
  const auto set_press =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0, true, true);
  require(!set_press.engaged, "SET press must not engage before release");

  vehicle.clu_button = 0;
  const auto panda_blocked =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1, true, false);
  require(panda_blocked.engaged && !panda_blocked.active &&
              panda_blocked.active_block == BlockReason::PandaControlsOff,
          "Panda controls gate");
  require(!panda_blocked.engage_rejected,
          "Panda controls handshake must not reject a valid SET request");
  require(panda_blocked.should_send && !panda_blocked.frames.empty(),
          "Panda mismatch must keep zero replacement stream");
  const HyundaiLkas11Values zero_lkas =
      decode_lkas11(panda_blocked.frames.front().data);
  require(zero_lkas.steer_torque == 0 && !zero_lkas.steer_req,
          "Panda controls off must generate zero LKAS");

  LateralController panda_timeout_controller(config);
  /* 이 블록은 t=10 에서 돈다. CAN 타임스탬프를 함께 옮기지 않으면 차량 상태가
   * 9초 낡아 vehicle_state_stale 이 되고, 검증하려던 Panda 유예가 아니라
   * freshness 게이트를 보게 된다. */
  VehicleCanState timeout_vehicle = ready_vehicle(10.0);
  timeout_vehicle.clu_button = 2;
  panda_timeout_controller.update(replay_path(), replay_target(), timeout_vehicle,
                                  10.0, 0, true, true);
  timeout_vehicle.clu_button = 0;
  const auto panda_waiting = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle, 10.01, 1, true, false);
  require(panda_waiting.engaged && !panda_waiting.engage_rejected,
          "Panda handshake grace must keep a valid request latched");
  // 유예(1초)를 넘기며 CAN 은 계속 신선하게 유지한다.
  VehicleCanState timeout_vehicle_later = ready_vehicle(11.0);
  const auto panda_timeout = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle_later, 11.02, 102, true, false);
  require(!panda_timeout.engaged && panda_timeout.engage_rejected &&
              panda_timeout.active_block == BlockReason::PandaControlsOff,
          "persistent Panda mismatch must eventually reject engage");

  LateralController deferred_static_controller(config);
  // t=12 시점 검사이므로 차량 데이터도 신선해야 한다(낡으면 stale이 우선).
  VehicleCanState deferred_vehicle = ready_vehicle(12.0);
  deferred_vehicle.wheel_speed_fl_kph = deferred_vehicle.wheel_speed_fr_kph = 60.0f;
  deferred_vehicle.wheel_speed_rl_kph = deferred_vehicle.wheel_speed_rr_kph = 60.0f;
  deferred_vehicle.clu_button = 2;
  deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle,
                                    12.0, 0, true, true);
  deferred_vehicle.clu_button = 0;
  deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle,
                                    12.01, 1, true, false);
  LateralTarget deferred_invalid_target = replay_target();
  deferred_invalid_target.mpc_solution_valid = false;
  const auto deferred_static = deferred_static_controller.update(
      replay_path(), deferred_invalid_target, deferred_vehicle, 12.02, 2, true, true);
  require(!deferred_static.engaged && deferred_static.engage_rejected &&
              deferred_static.active_block == BlockReason::LateralPlanInvalid,
          "Panda recovery must re-evaluate static engage gates");

  const auto active =
      controller.update(replay_path(), replay_target(), vehicle, 1.02, 2, true, true);
  require(active.engaged && active.active, "Panda controls on must activate");

  vehicle.clu_button = 4;
  const auto cancel =
      controller.update(replay_path(), replay_target(), vehicle, 1.03, 3, true, false);
  require(!cancel.engaged && cancel.should_send,
          "CANCEL press must start zero-frame handoff");
  const auto release_tail =
      controller.update(replay_path(), replay_target(), vehicle, 4.02, 302, true, false);
  require(release_tail.should_send, "handoff must last for 3000 ms");
  const auto stock_handoff =
      controller.update(replay_path(), replay_target(), vehicle, 4.04, 304, true, false);
  require(!stock_handoff.should_send, "handoff must stop after 3000 ms");

  vehicle.lkas11_seed[4] = 9U << 4;
  stamp_can_times(&vehicle, 4.05);
  vehicle.clu_button = 2;
  controller.update(replay_path(), replay_target(), vehicle, 4.05, 305, true, true);
  vehicle.clu_button = 0;
  const auto reengaged =
      controller.update(replay_path(), replay_target(), vehicle, 4.06, 306, true, true);
  require(reengaged.active && !reengaged.frames.empty(), "re-engage after handoff");
  require(decode_lkas11(reengaged.frames.front().data).msg_count == 10,
          "re-engage must seed LKAS counter from stock camera");

  LateralController rejected_controller(config);
  VehicleCanState rejected_vehicle = ready_vehicle();
  rejected_vehicle.clu_button = 2;
  rejected_controller.update(replay_path(), replay_target(), rejected_vehicle,
                              1.0, 0, true, true);
  rejected_vehicle.clu_button = 0;
  LateralTarget invalid_target = replay_target();
  invalid_target.mpc_solution_valid = false;
  const auto rejected = rejected_controller.update(
      replay_path(), invalid_target, rejected_vehicle, 1.01, 1, true, true);
  require(!rejected.engaged && rejected.engage_rejected &&
              rejected.active_block == BlockReason::LateralPlanInvalid,
          "static engage gate must reject without latching engaged state");
}

/* 시동 직후 첫 engage: Panda health 가 아직 없고 안전벨트/기어가 막고 있을 때.
 * 실차(2026-09-12)에서 engage 톤이 울린 뒤 해제되고, 두 번째 시도부터만
 * 거절음이 났다. 하드 결함이 panda_not_ready 뒤로 밀려 가려졌기 때문이다. */
void verify_cold_start_engage_reports_hard_block() {
  const auto cold_start_attempt = [](bool panda_ready, bool seatbelt_unlatched,
                                     int gear) {
    LateralControllerConfig config;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.seatbelt_unlatched = seatbelt_unlatched;
    vehicle.gear = gear;
    vehicle.clu_button = 2;
    controller.update(replay_path(), replay_target(), vehicle, 1.0, 0,
                      panda_ready, panda_ready);
    vehicle.clu_button = 0;
    return controller.update(replay_path(), replay_target(), vehicle, 1.01, 1,
                             panda_ready, panda_ready);
  };

  const auto belt = cold_start_attempt(false, true, 5);
  require(!belt.engaged && belt.engage_rejected &&
              belt.active_block == BlockReason::SeatbeltUnlatched,
          "cold-start SET with seatbelt off must report the seatbelt, not defer on panda");

  const auto gear = cold_start_attempt(false, false, 0);
  require(!gear.engaged && gear.engage_rejected &&
              gear.active_block == BlockReason::GearNotDrive,
          "cold-start SET out of D must report the gear, not defer on panda");

  // 차량이 정상이면 Panda 핸드셰이크 유예는 그대로 살아 있어야 한다.
  const auto handshake = cold_start_attempt(false, false, 5);
  require(handshake.engaged && !handshake.engage_rejected &&
              handshake.active_block == BlockReason::PandaNotReady,
          "cold-start SET with a healthy car must still wait for the panda handshake");
}

/* t=1.0 s에 나온, 60 m 이상 뻗은 조향 가능 plan. */
K230ModelState usable_model_state() {
  K230ModelState state;
  state.valid = 1;
  state.model_timestamp_ns = 1000000000ULL;
  state.plan_probability = 0.9f;
  state.lane_probabilities[1] = 0.8f;
  state.lane_probabilities[2] = 0.7f;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float x = 2.0f * static_cast<float>(i + 1);
    state.plan[i].x = x;
    state.plan[i].y = -0.0004f * x * x;
  }
  return state;
}

// 점 수는 충분하지만 몇 미터로 주저앉은 plan.
void collapse_plan(K230ModelState *state) {
  for (int i = 0; i < kTrajectorySize; ++i)
    state->plan[i].x = 1.0f + 0.1f * static_cast<float>(i);
}

void verify_model_path_adapter() {
  const K230ModelState state = usable_model_state();
  const LateralPath path =
      path_from_model_state(state, 1100000000ULL, 250000000ULL);
  require(path.usable_for_steering && path.left_valid && path.right_valid,
          "model path adapter validity");
  require(path.point_count == kTrajectorySize && path.reach_m >= 60.0f,
          "model path adapter counts every forward plan point");

  /* 정차에서 plan이 몇 미터로 주저앉으면 점 수는 충분해도 조향에 못 쓴다. */
  K230ModelState short_state = state;
  collapse_plan(&short_state);
  const LateralPath short_path =
      path_from_model_state(short_state, 1100000000ULL, 250000000ULL);
  require(!short_path.usable_for_steering &&
              short_path.invalid_reason == "path_invalid",
          "short plan reach must fail the steering gate");
}

/* 문서화된 안전 홀드 1: Panda 헬스 스냅샷 공백은 100 ms까지만, 신선한
 * controls_allowed=0은 절대 유지하지 않는다. */
void verify_panda_health_hold() {
  const uint64_t t0 = 1000000000ULL;
  K230PandaState ready;
  ready.timestamp_ns = t0;
  ready.connected = ready.comms_healthy = ready.tx_enabled = 1;
  ready.controls_allowed = 1;
  ready.safety_mode = kExpectedPandaSafetyModel;
  ready.safety_param = kExpectedPandaSafetyParam;
  K230PandaState unhealthy = ready;
  unhealthy.comms_healthy = 0;

  PandaHealthGate gate;
  PandaGateOutput out = gate.update(ready, t0, false);
  require(out.state_fresh && out.ready_raw && out.ready && out.controls_allowed &&
              !out.hold_applied,
          "a ready panda passes the gate without a hold");
  out = gate.update(unhealthy, t0 + 50000000ULL, false);
  require(!out.ready_raw && out.hold_applied && out.ready && out.controls_allowed,
          "a 50 ms health gap keeps the last verdict");
  out = gate.update(unhealthy, t0 + 100000000ULL, false);
  require(out.hold_applied, "the hold still covers exactly 100 ms");
  out = gate.update(unhealthy, t0 + 100000001ULL, false);
  require(!out.hold_applied && !out.ready && !out.controls_allowed,
          "the hold ends after 100 ms");
  out = gate.update(ready, t0 + 1200000000ULL, false);
  require(!out.state_fresh && !out.ready_raw && !out.ready,
          "a 1.2 s old snapshot is stale even if its fields look ready");

  PandaHealthGate explicit_off;
  explicit_off.update(ready, t0, false);
  K230PandaState off = ready;
  off.controls_allowed = 0;
  off.timestamp_ns = t0 + 10000000ULL;
  out = explicit_off.update(off, t0 + 10000000ULL, false);
  require(out.ready_raw && out.controls_off_explicit && !out.hold_applied &&
              out.ready && !out.controls_allowed,
          "a fresh, transport-ready controls_allowed=0 is never held");
  K230PandaState gap = off;
  gap.comms_healthy = 0;
  out = explicit_off.update(gap, t0 + 60000000ULL, false);
  require(out.hold_applied && out.ready && !out.controls_allowed,
          "a health gap after an explicit off keeps controls off");

  PandaHealthGate cold;
  out = cold.update(unhealthy, t0, false);
  require(!out.ready && !out.hold_applied, "no hold before the first ready snapshot");
  out = cold.update(unhealthy, t0, true);
  require(out.ready && out.controls_allowed && !out.ready_raw,
          "force_engaged bypasses the panda gate");
}

/* 문서화된 안전 홀드 2: 잘못된 plan 프레임은 150 ms까지 마지막 유효 경로로
 * 덮고, 모델 freshness 타임아웃은 그대로 하드 게이트다. */
void verify_path_invalid_hold() {
  const uint64_t timeout_ns = 250000000ULL;
  const K230ModelState good = usable_model_state();

  PathHoldGate gate;
  PathHoldOutput out = gate.update(good, 1100000000ULL, timeout_ns);
  require(out.path.usable_for_steering && !out.hold_applied,
          "a usable plan passes through the hold");
  K230ModelState collapsed = good;
  collapsed.model_timestamp_ns = 1050000000ULL;
  collapse_plan(&collapsed);
  out = gate.update(collapsed, 1100000000ULL, timeout_ns);
  require(out.raw.invalid_reason == "path_invalid" && out.hold_applied &&
              out.path.usable_for_steering && out.path.invalid_reason.empty(),
          "one collapsed frame is covered by the last usable path");
  collapsed.model_timestamp_ns = 1120000000ULL;
  out = gate.update(collapsed, 1150000000ULL, timeout_ns);
  require(out.hold_applied, "the hold still covers exactly 150 ms");
  out = gate.update(collapsed, 1150000001ULL, timeout_ns);
  require(!out.hold_applied && !out.path.usable_for_steering &&
              out.path.invalid_reason == "path_invalid",
          "the hold ends 150 ms after the last usable frame");

  PathHoldGate stale_gate;
  stale_gate.update(good, 1100000000ULL, timeout_ns);
  K230ModelState stale = collapsed;
  stale.model_timestamp_ns = 1000000000ULL;
  out = stale_gate.update(stale, 1400000000ULL, timeout_ns);
  require(!out.hold_applied && out.path.invalid_reason == "model_stale",
          "a stale model is a hard gate, never held");

  PathHoldGate invalid_gate;
  invalid_gate.update(good, 1100000000ULL, timeout_ns);
  K230ModelState invalid = good;
  invalid.valid = 0;
  invalid.model_timestamp_ns = 1120000000ULL;
  out = invalid_gate.update(invalid, 1130000000ULL, timeout_ns);
  require(!out.hold_applied && out.path.invalid_reason == "model_invalid",
          "an invalid model is not held");
}

// ---------------------------------------------------------------- paramsd·torqued 소비

LiveLateralParams odd_live_params() {
  LiveLateralParams live;
  live.use_vehicle = true;
  live.steer_ratio = 13.1f;
  live.stiffness_factor = 0.7f;
  live.angle_offset_deg = 2.5f;
  live.roll_rad = 0.04f;
  live.use_torque = true;
  live.lat_accel_factor = 3.0f;
  live.lat_accel_offset = 0.2f;
  live.friction = 0.05f;
  return live;
}

/* 스위치를 끄면 학습값을 넣어도 모든 틱이 비트 단위로 같아야 한다. 무효·캘리브 완료도
 * 끈 쪽에서는 차단하지 않는다. */
void verify_live_params_switch_off_is_identical() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController plain(config), fed(config);
  double t = 1.0;
  for (int tick = 0; tick < 600; ++tick, t += 0.01) {
    VehicleCanState vehicle = ready_vehicle(t);
    vehicle.steering_angle_deg = 12.0f * std::sin(0.02f * tick);
    vehicle.yaw_rate_valid = true;
    vehicle.yaw_rate_rad_s = 0.05f * std::sin(0.02f * tick);
    vehicle.lat_accel_valid = true;
    vehicle.lat_accel_mps2 = -0.3f;
    vehicle.driver_torque = tick % 97 == 0 ? 200 : 10;
    LateralTarget target = replay_target();
    for (int i = 0; i < kLateralControlN; ++i) target.curvatures[i] = 0.002f * std::cos(0.01f * tick);
    fed.set_live_params(odd_live_params(), false, true);
    const LateralControlResult a = plain.update(replay_path(), target, vehicle, t, tick);
    const LateralControlResult b = fed.update(replay_path(), target, vehicle, t, tick);
    require(a.active_block == b.active_block && a.active == b.active,
            "switched-off learners never block");
    require(std::memcmp(&a.desired_curvature, &b.desired_curvature, sizeof(float)) == 0 &&
                std::memcmp(&a.actual_curvature, &b.actual_curvature, sizeof(float)) == 0 &&
                std::memcmp(&a.normalized_output, &b.normalized_output, sizeof(float)) == 0 &&
                std::memcmp(&a.feedforward, &b.feedforward, sizeof(float)) == 0 &&
                a.desired_torque == b.desired_torque && a.apply_torque == b.apply_torque,
            "switched-off learners leave every tick bit-identical");
  }
}

/* 켜면 실제 곡률은 opendbc VehicleModel(update_params(x, sr) → calc_curvature(sa, u, roll))을
 * 부호 반전한 값이다. double 독립 전사본과 비교한다. */
double upstream_measured_curvature(const SteeringParams &p, const LiveLateralParams &live,
                                   double angle_deg, double u) {
  const double civic_m = 1326.0 + 136.0, civic_l = 2.70, civic_af = civic_l * 0.4;
  const double civic_ar = civic_l - civic_af;
  const double m = p.mass_kg, l = p.wheelbase_m, af = p.center_to_front_m(), ar = l - af;
  const double tsf = p.tire_stiffness_factor, x = std::max<double>(live.stiffness_factor, 0.1);
  const double cf = 192150.0 * tsf * m / civic_m * (ar / l) / (civic_ar / civic_l) * x;
  const double cr = 202500.0 * tsf * m / civic_m * (af / l) / (civic_af / civic_l) * x;
  const double sf = m * (cf * af - cr * ar) / (l * l * cf * cr);
  const double factor = (1.0 - p.steer_ratio_rear) / (1.0 - sf * u * u) / l;
  const double sa = (angle_deg - live.angle_offset_deg) * 3.14159265358979323846 / 180.0;
  const double roll = std::fabs(sf) < 1e-6 ? 0.0 : 9.81 * live.roll_rad / ((1.0 / sf) - u * u);
  return -(factor * sa / std::max<double>(live.steer_ratio, 0.1) + roll);
}

void verify_live_vehicle_params_follow_vehicle_model() {
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  LiveLateralParams live = odd_live_params();
  live.use_torque = false;
  for (float u : {3.0f, 12.0f, 27.0f}) {
    for (float angle : {-30.0f, 0.0f, 4.0f}) {
      TorqueController torque;
      const float got = torque.estimate_actual_curvature(u, angle, params, 0.0f, false, live);
      const double want = upstream_measured_curvature(params, live, angle, u);
      require(std::fabs(got - want) <= 2e-6 * std::fabs(want) + 1e-9,
              "live SR, stiffness, offset and roll follow opendbc calc_curvature");
    }
  }
  // 롤은 FF에서 roll·g를 빼고 편경사 추정은 쓰지 않는다(마찰은 포화 구간이라 같다)
  TorqueController with_roll, without_roll;
  LiveLateralParams flat = live;
  flat.roll_rad = 0.0f;
  params.live_bank_compensation = true;
  for (int i = 0; i < 150; ++i) {
    with_roll.update(true, 20.0f, 0.004f, 1.0f, false, false, params, 0.0f, false, -0.5f, live);
    without_roll.update(true, 20.0f, 0.004f, 1.0f, false, false, params, 0.0f, false, -0.5f, flat);
  }
  require(std::fabs((with_roll.feedforward() - without_roll.feedforward()) + live.roll_rad * 9.81f) < 1e-4f,
          "live roll subtracts roll*g from feedforward and replaces the bank estimate");
}

/* 상류는 PID를 횡가속 공간에서 돌리고 끝에서 latAccelFactor로 나눈다. 그러면 마찰이 없을 때
 * 출력 × latAccelFactor가 배율과 무관하다(사전값 경로 포함). 마찰은 토크 공간에 그대로,
 * 절편은 −offset/latAccelFactor로 더해진다. */
void verify_live_torque_params_match_upstream_structure() {
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  params.torque_friction = 0.0f;
  params.live_bank_compensation = false;
  const float prior = params.torque_lat_accel_factor;
  auto run = [&](bool use, float factor, float offset, float friction, std::vector<float> *out) {
    TorqueController torque;
    LiveLateralParams live;
    live.use_torque = use;
    live.lat_accel_factor = factor;
    live.lat_accel_offset = offset;
    live.friction = friction;
    for (int i = 0; i < 300; ++i) {
      const float desired = 0.0015f * std::sin(0.03f * i);
      const float angle = 1.5f * std::sin(0.03f * i - 0.4f);
      torque.update(true, 20.0f, desired, angle, false, false, params, 0.0f, false, 0.0f, live);
      out->push_back(torque.normalized_output());
    }
  };
  std::vector<float> base, low, high, offset, friction;
  run(false, 0.0f, 0.0f, 0.0f, &base);
  run(true, 3.0f, 0.0f, 0.0f, &low);
  run(true, 5.5f, 0.0f, 0.0f, &high);
  run(true, 3.0f, 0.1f, 0.0f, &offset);
  run(true, 3.0f, 0.0f, 0.05f, &friction);
  float worst_scale = 0.0f, worst_offset = 0.0f, worst_friction = 0.0f;
  for (size_t i = 0; i < base.size(); ++i) {
    require(std::fabs(base[i]) < 0.9f && std::fabs(low[i]) < 0.9f, "outputs stay unsaturated");
    const float ref = base[i] * prior;
    worst_scale = std::max({worst_scale, std::fabs(low[i] * 3.0f - ref), std::fabs(high[i] * 5.5f - ref)});
    const float sign = params.torque_output_sign >= 0 ? 1.0f : -1.0f;
    worst_offset = std::max(worst_offset, std::fabs((offset[i] - low[i]) * 3.0f + sign * 0.1f));
    // 마찰은 |오차| < 0.2에서 선형이라 차이가 토크 공간 0.05 이하, 부호는 오차를 따른다
    worst_friction = std::max(worst_friction, std::fabs(friction[i] - low[i]) - 0.05f);
  }
  require(worst_scale < 2e-5f, "output times latAccelFactor is independent of the factor");
  require(worst_offset < 2e-5f, "the learned offset enters as -offset/latAccelFactor");
  require(worst_friction < 1e-6f, "friction is a torque-space term bounded by the coefficient");
}

void verify_paramsd_invalid_blocks_only_when_used() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  config.steering_params.use_live_vehicle_params = true;
  LateralController controller(config);
  const VehicleCanState vehicle = ready_vehicle(1.0);
  controller.set_live_params(odd_live_params(), true, true);
  require(controller.update(replay_path(), replay_target(), vehicle, 1.0, 0).active,
          "valid learned params keep control active");
  controller.set_live_params(odd_live_params(), false, false);
  require(controller.update(replay_path(), replay_target(), vehicle, 1.01, 1).active,
          "invalid params before calibration do not block (upstream cal_status check)");
  controller.set_live_params(odd_live_params(), false, true);
  require(controller.update(replay_path(), replay_target(), vehicle, 1.02, 2).active_block ==
              BlockReason::ParamsdInvalid,
          "invalid learned params block once calibrated");
  LiveLateralParams unseen = odd_live_params();
  unseen.use_vehicle = false;
  controller.set_live_params(unseen, false, true);
  require(controller.update(replay_path(), replay_target(), vehicle, 1.03, 3).active_block !=
              BlockReason::ParamsdInvalid,
          "no block before paramsd has published (upstream sm.seen)");
}

void verify_curvature_limit_follows_roll() {
  LateralTarget target = replay_target();
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = 0.05f;
    target.psis[i] = 0.05f * 20.0f * model_t_idx(i);
  }
  const float v = 20.0f, roll = 0.03f;
  const float up = lag_adjusted_desired_curvature(target, v, 0.0f, 0.34f, 0.05f, roll);
  require(std::fabs(up - (kMaxLateralAccel + roll * 9.81f) / (v * v)) < 1e-7f,
          "roll moves the upper lateral accel bound by roll*g");
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = -0.05f;
    target.psis[i] = -target.psis[i];
  }
  const float down = lag_adjusted_desired_curvature(target, v, 0.0f, 0.34f, -0.05f, roll);
  require(std::fabs(down - (-kMaxLateralAccel + roll * 9.81f) / (v * v)) < 1e-7f,
          "and the lower bound by the same roll*g");
}

// 2026-09-24 실차: 663 ms 멈춤 뒤 NaN 속도가 좌측 최대 곡률을 심어 재활성 때 32° 조향했다.
void verify_stale_speed_keeps_curvature() {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  const LateralTarget target = replay_target();
  float before = 0.0f;
  for (int i = 0; i < 20; ++i) {
    const double t = 1.0 + 0.01 * i;
    before = controller.update(replay_path(), target, ready_vehicle(t), t, i).desired_curvature;
  }
  const LateralControlResult stale =
      controller.update(replay_path(), target, ready_vehicle(1.19), 1.9, 20);
  require(stale.active_block == BlockReason::VehicleStateStale, "wheel speed older than the timeout");
  require(stale.desired_curvature == before, "stale speed keeps the last desired curvature");
  const LateralControlResult back =
      controller.update(replay_path(), target, ready_vehicle(1.91), 1.91, 21);
  const float step = kMaxLateralJerk / (60.0f / 3.6f * 60.0f / 3.6f) * kCurvatureRateWindowS;
  require(std::fabs(back.desired_curvature - before) <= step * 1.001f,
          "recovery resumes from the pre-stall curvature");
}

// 보드는 JSON을 읽고 재생·테스트는 기본값을 쓴다. 둘이 갈리면 재생 대조가 보드를 대변하지 못한다.
void verify_steering_json_matches_defaults() {
  SteeringParams json, defaults;
  std::string error;
  require(load_steering_params_json("params/steering.json", &json, &error), "load steering.json");
  require(json.torque_lat_accel_factor == defaults.torque_lat_accel_factor &&
              json.torque_kp == defaults.torque_kp && json.torque_ki == defaults.torque_ki &&
              json.torque_friction == defaults.torque_friction,
          "steering.json torque gains equal the compiled defaults");
  const std::string path = "/tmp/check_control_replay_raw_keys.json";
  std::FILE *f = std::fopen(path.c_str(), "w");
  require(f != nullptr, "write raw-key fixture");
  std::fputs("{\"torque_kf_raw\": 20}\n", f);
  std::fclose(f);
  SteeringParams rejected;
  const bool loaded = load_steering_params_json(path, &rejected, &error);
  std::remove(path.c_str());
  require(!loaded && error.find("torque_kf_raw") != std::string::npos,
          "pre-2026-09-24 raw gain keys are rejected, not silently defaulted");
}

}  // namespace

int main(int argc, char **argv) {
  return run_checks(argc == 1 ? "CONTROL_SELF_TEST_OK" : nullptr, [&] {
    verify_mdps_speed_spoof();
    verify_lca11();
    verify_whl_spd11();
    verify_tpms11();
    verify_tcs15();
    verify_tcs13_driver_override();
    verify_scc11();
    verify_fixed_cruise_speed_estimate();
    verify_mdps_fault_filter();
    verify_braking_does_not_disengage();
    verify_large_angle_fault_avoidance();
    verify_fixed_max_curvature();
    verify_delay_compensated_error();
    verify_reengage_has_no_stale_buffer_spike();
    verify_lat_accel_offset_shifts_feedforward();
    verify_kp_speed_schedule();
    verify_live_bank_compensation();
    verify_engage_allowed_with_unavailable_path();
    verify_path_flicker_debounce();
    verify_bank_holds_during_curves();
    verify_runtime_params_apply_immediately();
    verify_lkas_hud_state_stability();
    verify_panda_gate_and_handoff();
    verify_cold_start_engage_reports_hard_block();
    verify_model_path_adapter();
    verify_panda_health_hold();
    verify_path_invalid_hold();
    verify_live_params_switch_off_is_identical();
    verify_live_vehicle_params_follow_vehicle_model();
    verify_live_torque_params_match_upstream_structure();
    verify_paramsd_invalid_blocks_only_when_used();
    verify_curvature_limit_follows_roll();
    verify_stale_speed_keeps_curvature();
    verify_steering_json_matches_defaults();
    if (argc == 1) return;
    /* 픽스처는 60초 연속 주행 구간이어야 한다(active > 5900틱, 토크 > 0). 정차
     * 구간은 이 전제에 걸려 실패한다. tools/control/export_can_fixture.py가 녹화
     * events/NNN.bin 하나를 이 형식으로 내보낸다. */
    if (argc != 2) throw std::runtime_error("usage: check_control_replay [fixture.k230can]");

    CanReplaySource replay;
    replay.open(argv[1]);
    LateralControllerConfig config;
    config.force_engaged = true;
    std::string error;
    require(load_steering_params_json("params/steering.json",
                                         &config.steering_params, &error),
            "load steering params");
    require(load_driving_params_json("params/driving.json",
                                        &config.driving_params, &error),
            "load driving params");
    require(std::fabs(config.driving_params.mdps_speed_spoof_kph - 60.0f) < 1e-6f,
            "driving params MDPS speed");
    LateralController controller(config);
    VehicleCanState vehicle;
    const LateralPath path = replay_path();
    const LateralTarget target = replay_target();
    size_t rx_frames = 0;
    size_t generated_frames = 0;
    size_t active_ticks = 0;
    size_t invalid_frames = 0;
    size_t lkas0 = 0;
    size_t lkas1 = 0;
    size_t clu1 = 0;
    size_t mdps2 = 0;
    int max_torque = 0;
    float max_curvature_error = 0.0f;
    float ref_prev_curvature = 0.0f;

    const auto begin = std::chrono::steady_clock::now();
    const int ticks = static_cast<int>(std::ceil(replay.duration_s() * 100.0)) + 2;
    for (int tick = 0; tick < ticks; ++tick) {
      const double now_s = static_cast<double>(tick) * 0.01;
      std::vector<CanFrame> frames;
      replay.poll(now_s, &frames);
      rx_frames += frames.size();
      for (const CanFrame &frame : frames) {
        update_vehicle_can_state(&vehicle, frame.address, frame.data,
                                 frame.length, frame.bus, now_s);
      }
      const auto result = controller.update(path, target, vehicle, now_s, tick);
      /* 컨트롤러는 휠 속도 평균으로 곡률을 낸다. 클러스터 속도를 먹이면
       * 참조식이 다른 입력을 보게 되어 비교가 성립하지 않는다. 첫 WHL_SPD11
       * 전에는 속도가 NaN이고 컨트롤러가 어차피 비활성이라 비교하지 않는다. */
      if (std::isfinite(result.control_speed_kph)) {
        const float expected_curvature = reference_lag_adjusted_curvature(
            target, std::max(0.0f, result.control_speed_kph / 3.6f),
            config.steering_params.steer_actuator_delay, ref_prev_curvature);
        if (target.valid) ref_prev_curvature = expected_curvature;
        max_curvature_error = std::max(
            max_curvature_error, std::fabs(result.desired_curvature - expected_curvature));
      }
      if (result.active) ++active_ticks;
      max_torque = std::max(max_torque, std::abs(result.apply_torque));
      generated_frames += result.frames.size();
      for (const CanFrame &frame : result.frames) {
        const bool valid_length =
            (frame.address == kHyundaiClu11Address && frame.length == 4) ||
            (frame.address != kHyundaiClu11Address && frame.length == 8);
        if (!valid_length) ++invalid_frames;
        if (frame.address == kHyundaiLkas11Address && frame.bus == 0) ++lkas0;
        if (frame.address == kHyundaiLkas11Address && frame.bus == 1) ++lkas1;
        if (frame.address == kHyundaiClu11Address && frame.bus == 1) ++clu1;
        if (frame.address == kHyundaiMdps12Address && frame.bus == 2) ++mdps2;
      }
    }
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();

    require(replay.finished(), "replay did not consume all frames");
    require(rx_frames == replay.total_frames(), "replay frame count mismatch");
    require(active_ticks > 5900, "controller was not active for the driving segment");
    require(invalid_frames == 0, "invalid generated CAN frame");
    require(lkas0 > 5900 && lkas0 == lkas1 && lkas0 == mdps2,
            "100 Hz LKAS/MDPS schedule mismatch");
    require(clu1 > 2900 && clu1 * 2 >= lkas0 - 1 && clu1 * 2 <= lkas0 + 1,
            "50 Hz CLU11 schedule mismatch");
    require(max_torque > 0 && max_torque <= 384, "steering torque range");
    require(max_curvature_error < 1e-6f,
            "lag-adjusted curvature differs from openpilot reference");

    std::printf(
        "REPLAY_OK records=%zu duration_s=%.3f ticks=%d active=%zu "
        "generated=%zu lkas0=%zu lkas1=%zu clu1=%zu mdps2=%zu "
        "max_torque=%d curvature_err=%.8f compute_ms=%.3f\n",
        rx_frames, replay.duration_s(), ticks, active_ticks, generated_frames,
        lkas0, lkas1, clu1, mdps2, max_torque, max_curvature_error, elapsed_ms);
  });
}
