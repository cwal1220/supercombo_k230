#include "can_replay.h"
#include "hyundai_can.h"
#include "k230_ipc.h"
#include "k7_lateral_controller.h"
#include "k7_path.h"
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

float original_lag_adjusted_curvature(const LateralTarget &target, float speed_mps,
                                      float delay) {
  constexpr float desired_curvature_limit = 0.1f;
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
  const auto frames = build_k7_hev_lateral_can_frames(
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
  const auto custom_frames = build_k7_hev_lateral_can_frames(
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

  K7VehicleCanState vehicle;
  update_k7_vehicle_can_state(&vehicle, kHyundaiLca11Address, bytes,
                              bytes.size(), kK7PowertrainBus, 1.0);
  require(vehicle.left_blindspot && vehicle.right_blindspot,
          "LCA11 vehicle-state update");
}

void verify_mdps_fault_filter() {
  K7VehicleCanState vehicle;
  std::array<uint8_t, 8> bytes{};
  bytes[1] = (1U << 6) | (1U << 7);
  update_k7_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                              bytes.size(), kK7MdpsBus, 1.0);
  require(vehicle.mdps_hard_fault && !vehicle.steering_fault,
          "transient MDPS ToiFlt/FailStat must match openpilot filtering");

  bytes[1] = 1U << 4;
  for (int frame = 0; frame < 100; ++frame) {
    update_k7_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                                bytes.size(), kK7MdpsBus, 1.0 + frame * 0.02);
  }
  require(!vehicle.steering_fault, "MDPS unavailable debounce threshold");
  update_k7_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                              bytes.size(), kK7MdpsBus, 3.0);
  require(vehicle.steering_fault, "sustained MDPS unavailable fault");
}

void verify_braking_does_not_disengage() {
  K7LateralControllerConfig config;
  config.force_engaged = true;
  K7LateralController controller(config);
  K7VehicleCanState vehicle;
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
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
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

void verify_lkas_hud_state_stability() {
  K7LateralControllerConfig config;
  config.force_engaged = true;
  K7LateralController controller(config);
  K7VehicleCanState vehicle;
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
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
  vehicle.gear = 5;

  LateralPath no_lane_path = replay_path();
  no_lane_path.left_valid = false;
  no_lane_path.right_valid = false;
  const auto active =
      controller.update(no_lane_path, replay_target(), vehicle, 1.0, 0);
  require(active.active && !active.frames.empty(), "active HUD test frame");
  require(decode_lkas11(active.frames.front().data).ldws_sys_state == 3,
          "active HUD state must remain active with fluctuating lane probability");

  LateralTarget invalid_target = replay_target();
  invalid_target.mpc_solution_valid = false;
  const auto inactive =
      controller.update(no_lane_path, invalid_target, vehicle, 1.01, 1);
  require(!inactive.active && !inactive.frames.empty(), "inactive HUD test frame");
  require(decode_lkas11(inactive.frames.front().data).ldws_sys_state == 4,
          "inactive HUD state must remain standby with fluctuating lane probability");
}

void verify_panda_gate_and_handoff() {
  K7LateralControllerConfig config;
  K7LateralController controller(config);
  K7VehicleCanState vehicle;
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
  vehicle.cgw1_time_s = 1.0;
  vehicle.cgw2_time_s = 1.0;
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
  require(panda_blocked.should_send && !panda_blocked.frames.empty(),
          "Panda mismatch must keep zero replacement stream");
  const HyundaiLkas11Values zero_lkas =
      decode_lkas11(panda_blocked.frames.front().data);
  require(zero_lkas.steer_torque == 0 && !zero_lkas.steer_req,
          "Panda controls off must generate zero LKAS");

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
}

void verify_model_path_adapter() {
  K230ModelState state;
  state.valid = 1;
  state.lateral_target.valid = 1;
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
      k7_path_from_model_state(state, 1100000000ULL, 250000000ULL);
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
    verify_mdps_fault_filter();
    verify_braking_does_not_disengage();
    verify_lkas_hud_state_stability();
    verify_panda_gate_and_handoff();
    verify_model_path_adapter();
    if (argc == 1) {
      std::puts("K7_CONTROL_SELF_TEST_OK");
      return 0;
    }
    if (argc != 2) throw std::runtime_error("usage: check_k7_control_replay [fixture.k230can]");

    CanReplaySource replay;
    replay.open(argv[1]);
    K7LateralControllerConfig config;
    config.force_engaged = true;
    std::string error;
    require(load_k7_steering_params_json("params/k7_yg_steering.json",
                                         &config.steering_params, &error),
            "load steering params");
    require(load_k7_driving_params_json("params/k7_yg_driving.json",
                                        &config.driving_params, &error),
            "load driving params");
    require(std::fabs(config.driving_params.mdps_speed_spoof_kph - 60.0f) < 1e-6f,
            "driving params MDPS speed");
    K7LateralController controller(config);
    K7VehicleCanState vehicle;
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
        update_k7_vehicle_can_state(&vehicle, frame.address, frame.data,
                                    frame.length, frame.bus, now_s);
      }
      const auto result = controller.update(path, target, vehicle, now_s, tick);
      const float speed_kph = vehicle.cluster_speed *
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
        "K7_REPLAY_OK records=%zu duration_s=%.3f ticks=%d active=%zu "
        "generated=%zu lkas0=%zu lkas1=%zu clu1=%zu mdps2=%zu "
        "max_torque=%d curvature_err=%.8f compute_ms=%.3f\n",
        rx_frames, replay.duration_s(), ticks, active_ticks, generated_frames,
        lkas0, lkas1, clu1, mdps2, max_torque, max_curvature_error, elapsed_ms);
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "check_k7_control_replay: %s\n", error.what());
    return 1;
  }
}
