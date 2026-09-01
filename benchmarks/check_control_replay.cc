#include "can_replay.h"
#include "hyundai_can.h"
#include "k230_ipc.h"
#include "lateral_controller.h"
#include "lateral_path.h"
#include "openpilot_torque_controller.h"
#include "vehicle_can.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void set_signal_le(std::array<uint8_t, 8> *data, int start_bit, int length,
                   uint32_t value) {
  for (int i = 0; i < length; ++i) {
    const int bit = start_bit + i;
    const uint8_t mask = static_cast<uint8_t>(1U << (bit % 8));
    if ((value & (1U << i)) != 0U)
      (*data)[bit / 8] |= mask;
    else
      (*data)[bit / 8] &= static_cast<uint8_t>(~mask);
  }
}

LateralPath replay_path() {
  LateralPath path;
  path.left_valid = true;
  path.right_valid = true;
  path.usable_for_steering = true;
  path.confidence = 1.0f;
  for (int x = 2; x <= 60; x += 2) {
    const float xf = static_cast<float>(x);
    path.points.push_back({xf, 0.0004f * xf * xf, 1.0f});
  }
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

VehicleCanState ready_vehicle(double timestamp_s = 1.0) {
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  vehicle.lkas11_time_s = timestamp_s;
  vehicle.clu11_time_s = timestamp_s;
  vehicle.sas11_time_s = timestamp_s;
  vehicle.esp12_time_s = timestamp_s;
  vehicle.mdps12_time_s = timestamp_s;
  vehicle.tcs13_time_s = timestamp_s;
  vehicle.tcs15_time_s = timestamp_s;
  vehicle.e_ems11_time_s = timestamp_s;
  vehicle.elect_gear_time_s = timestamp_s;
  vehicle.whl_spd11_time_s = timestamp_s;
  vehicle.cgw1_time_s = timestamp_s;
  vehicle.cgw2_time_s = timestamp_s;
  vehicle.whl_spd11_time_s = timestamp_s;
  // 저속 조향 게이트를 넘는 주행 상태가 헬퍼의 기본값이다.
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.cluster_speed_raw = 63.0f;
  vehicle.gear = 5;
  return vehicle;
}

float original_lag_adjusted_curvature(const LateralTarget &target, float speed_mps,
                                      float delay) {
  constexpr float desired_curvature_limit = 0.05f;
  constexpr float max_lateral_jerk = 5.0f;
  constexpr float max_lateral_accel = 3.0f;
  constexpr float max_curvature = 0.2f;
  const auto model_t = [](int i) {
    const float ratio = static_cast<float>(i) / 32.0f;
    return 10.0f * ratio * ratio;
  };
  float psi = target.psis[kLateralControlN - 1];
  for (int i = 1; i < kLateralControlN; ++i) {
    if (delay <= model_t(i)) {
      const float p = (delay - model_t(i - 1)) / (model_t(i) - model_t(i - 1));
      psi = target.psis[i - 1] + p * (target.psis[i] - target.psis[i - 1]);
      break;
    }
  }
  const float speed = std::max(speed_mps, 0.1f);
  const float current = target.curvatures[0];
  float desired = current + 2.0f * (psi / (speed * delay) - current);
  const float rate_limit = max_lateral_jerk / (speed * speed);
  desired = std::clamp(desired,
                       current - rate_limit * desired_curvature_limit,
                       current + rate_limit * desired_curvature_limit);
  const float accel_speed = std::max(speed, 1.0f);
  desired = std::clamp(desired,
                       -max_lateral_accel / (accel_speed * accel_speed),
                       max_lateral_accel / (accel_speed * accel_speed));
  return std::clamp(desired, -max_curvature, max_curvature);
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
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  vehicle.lkas11_time_s = 1.0;
  vehicle.clu11_time_s = 1.0;
  vehicle.sas11_time_s = 1.0;
  vehicle.esp12_time_s = 1.0;
  vehicle.mdps12_time_s = 1.0;
  vehicle.tcs13_time_s = 1.0;
  vehicle.tcs15_time_s = 1.0;
  vehicle.e_ems11_time_s = 1.0;
  vehicle.elect_gear_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.gear = 5;
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

void verify_large_angle_fault_avoidance() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  vehicle.lkas11_time_s = 1.0;
  vehicle.clu11_time_s = 1.0;
  vehicle.sas11_time_s = 1.0;
  vehicle.esp12_time_s = 1.0;
  vehicle.mdps12_time_s = 1.0;
  vehicle.tcs13_time_s = 1.0;
  vehicle.tcs15_time_s = 1.0;
  vehicle.e_ems11_time_s = 1.0;
  vehicle.elect_gear_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.gear = 5;
  vehicle.cluster_speed_raw = 72.0f;
  vehicle.steering_angle_deg = 85.0f;

  for (int frame = 0; frame < 89; ++frame) {
    const auto result =
        controller.update(replay_path(), replay_target(), vehicle,
                          1.0 + frame * 0.01, frame);
    require(result.active && !result.cut_steer_temp,
            "large-angle control must remain requested before RK fault limit");
  }

  for (int frame = 89; frame < 91; ++frame) {
    const auto result =
        controller.update(replay_path(), replay_target(), vehicle,
                          1.0 + frame * 0.01, frame);
    require(result.active && result.cut_steer_temp && !result.frames.empty(),
            "large-angle fault avoidance must cut request without disengaging");
    const HyundaiLkas11Values lkas = decode_lkas11(result.frames.front().data);
    require(!lkas.steer_req && lkas.toi_fault,
            "fault-avoidance LKAS11 request and temporary-fault bits");
    require(lkas.steer_torque == result.apply_torque &&
                std::abs(result.apply_torque) > 0,
            "fault avoidance must preserve steering torque");
  }

  const auto resumed =
      controller.update(replay_path(), replay_target(), vehicle, 1.91, 91);
  require(resumed.active && !resumed.cut_steer_temp && !resumed.frames.empty(),
          "large-angle steering request must resume after two frames");
  const HyundaiLkas11Values lkas = decode_lkas11(resumed.frames.front().data);
  require(lkas.steer_req && !lkas.toi_fault,
          "resumed LKAS11 request and temporary-fault bits");
}

void verify_configured_steering_angle_limit() {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  config.steering_params.avoid_lkas_fault_enabled = false;
  config.steering_params.max_steering_angle_deg = 80.0f;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 72.0f;

  vehicle.steering_angle_deg = 79.9f;
  const auto below_limit =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  require(below_limit.active, "steering below configured angle limit must remain active");

  vehicle.steering_angle_deg = 80.0f;
  const auto at_limit =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  require(!at_limit.active && at_limit.active_block == "steering_angle_limit",
          "configured steering angle limit must apply below 90 degrees");
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
  require(!engaged.active && engaged.active_block == "stopped",
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
  require(!rolling.active && rolling.active_block == "path_invalid",
          "unusable path while moving must report path_invalid");
  // 결함은 가용성 대기보다 우선한다 (정차 중 문 열림 -> hard disengage)
  vehicle = ready_vehicle(1.03);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
  vehicle.door_open = true;
  const auto door =
      controller.update(bad, replay_target(), vehicle, 1.03, 3, true, true);
  require(door.active_block == "door_open" && !door.engaged,
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
  const auto result = controller.update(replay_path(), target, vehicle, 1.0, 0);
  require(result.active && std::fabs(result.desired_curvature - 0.3f) < 1e-6f,
          "K7 maximum curvature must remain fixed at 0.3 1/m");
}

/* v0.11식 지연 보정: 요청 스텝 직후 delay 동안은 P가 과거 요청(0)과 현재
 * 측정(0)을 비교해 오차가 없어야 하고, 토크는 FF만으로 나와야 한다. */
void verify_delay_compensated_error() {
  OpenpilotTorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  params.torque_use_angle = true;
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
  OpenpilotTorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  params.torque_use_angle = true;
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

// 라이브 뱅크: 편경사에 해당하는 만큼 FF가 이동해야 한다.
void verify_live_bank_compensation() {
  OpenpilotTorqueController with_bank, without_bank;
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
  OpenpilotTorqueController a, b;
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
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  vehicle.lkas11_time_s = 1.0;
  vehicle.clu11_time_s = 1.0;
  vehicle.sas11_time_s = 1.0;
  vehicle.esp12_time_s = 1.0;
  vehicle.mdps12_time_s = 1.0;
  vehicle.tcs13_time_s = 1.0;
  vehicle.tcs15_time_s = 1.0;
  vehicle.e_ems11_time_s = 1.0;
  vehicle.elect_gear_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.gear = 5;
  vehicle.cluster_speed_raw = 72.0f;

  const auto active =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  require(active.active, "runtime parameter test must start active");

  SteeringParams steering = config.steering_params;
  steering.enabled = false;
  controller.update_params(steering, config.driving_params);
  const auto disabled =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  require(!disabled.active && disabled.active_block == "controller_disabled",
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
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  vehicle.lkas11_time_s = 1.0;
  vehicle.clu11_time_s = 1.0;
  vehicle.sas11_time_s = 1.0;
  vehicle.esp12_time_s = 1.0;
  vehicle.mdps12_time_s = 1.0;
  vehicle.tcs13_time_s = 1.0;
  vehicle.tcs15_time_s = 1.0;
  vehicle.e_ems11_time_s = 1.0;
  vehicle.elect_gear_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.gear = 5;

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
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  vehicle.lkas11_time_s = 1.0;
  vehicle.clu11_time_s = 1.0;
  vehicle.sas11_time_s = 1.0;
  vehicle.esp12_time_s = 1.0;
  vehicle.mdps12_time_s = 1.0;
  vehicle.tcs13_time_s = 1.0;
  vehicle.tcs15_time_s = 1.0;
  vehicle.e_ems11_time_s = 1.0;
  vehicle.elect_gear_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
  vehicle.whl_spd11_time_s = 1.0;
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.gear = 5;

  vehicle.clu_button = 2;
  const auto set_press =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0, true, true);
  require(!set_press.engaged, "SET press must not engage before release");

  vehicle.clu_button = 0;
  const auto panda_blocked =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1, true, false);
  require(panda_blocked.engaged && !panda_blocked.active &&
              panda_blocked.active_block == "panda_controls_off",
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
  VehicleCanState timeout_vehicle = vehicle;
  timeout_vehicle.clu_button = 2;
  panda_timeout_controller.update(replay_path(), replay_target(), timeout_vehicle,
                                  10.0, 0, true, true);
  timeout_vehicle.clu_button = 0;
  const auto panda_waiting = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle, 10.01, 1, true, false);
  require(panda_waiting.engaged && !panda_waiting.engage_rejected,
          "Panda handshake grace must keep a valid request latched");
  const auto panda_timeout = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle, 11.02, 102, true, false);
  require(!panda_timeout.engaged && panda_timeout.engage_rejected &&
              panda_timeout.active_block == "panda_controls_off",
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
              deferred_static.active_block == "lateral_plan_invalid",
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
  vehicle.lkas11_time_s = 4.05;
  vehicle.clu11_time_s = 4.05;
  vehicle.sas11_time_s = 4.05;
  vehicle.esp12_time_s = 4.05;
  vehicle.mdps12_time_s = 4.05;
  vehicle.tcs13_time_s = 4.05;
  vehicle.tcs15_time_s = 4.05;
  vehicle.e_ems11_time_s = 4.05;
  vehicle.elect_gear_time_s = 4.05;
  vehicle.whl_spd11_time_s = 4.05;
  vehicle.cgw1_time_s = 4.05;
  vehicle.cgw2_time_s = 4.05;
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
              rejected.active_block == "lateral_plan_invalid",
          "static engage gate must reject without latching engaged state");
}

void verify_model_path_adapter() {
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
  const LateralPath path =
      path_from_model_state(state, 1100000000ULL, 250000000ULL);
  float lateral_20m = 0.0f;
  require(path.usable_for_steering && path.left_valid && path.right_valid,
          "model path adapter validity");
  require(path_lateral_at(path, 20.0f, &lateral_20m) && lateral_20m > 0.0f,
          "openpilot-left to K7-right coordinate conversion");
  require(steering_curvature(path, 20.0f) > 0.0f,
          "model path curvature sign");
}

}  // namespace

int main(int argc, char **argv) {
  try {
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
    verify_configured_steering_angle_limit();
    verify_fixed_max_curvature();
    verify_delay_compensated_error();
    verify_reengage_has_no_stale_buffer_spike();
    verify_lat_accel_offset_shifts_feedforward();
    verify_live_bank_compensation();
    verify_engage_allowed_with_unavailable_path();
    verify_path_flicker_debounce();
    verify_bank_holds_during_curves();
    verify_runtime_params_apply_immediately();
    verify_lkas_hud_state_stability();
    verify_panda_gate_and_handoff();
    verify_model_path_adapter();
    if (argc == 1) {
      std::puts("CONTROL_SELF_TEST_OK");
      return 0;
    }
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
      const float speed_kph = vehicle.cluster_speed_raw *
          (vehicle.speed_unit_mph ? 1.609344f : 1.0f);
      const float expected_curvature = original_lag_adjusted_curvature(
          target, std::max(0.0f, speed_kph / 3.6f),
          config.steering_params.steer_actuator_delay);
      max_curvature_error = std::max(
          max_curvature_error, std::fabs(result.desired_curvature - expected_curvature));
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
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "check_control_replay: %s\n", error.what());
    return 1;
  }
}
