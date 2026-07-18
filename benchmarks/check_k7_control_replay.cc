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
    if (argc != 2) throw std::runtime_error("usage: check_k7_control_replay fixture.k230can");
    verify_mdps_speed_spoof();
    verify_model_path_adapter();

    CanReplaySource replay;
    replay.open(argv[1]);
    K7LateralControllerConfig config;
    config.force_engaged = true;
    K7LateralController controller(config);
    K7VehicleCanState vehicle;
    const LateralPath path = replay_path();
    size_t rx_frames = 0;
    size_t generated_frames = 0;
    size_t active_ticks = 0;
    size_t invalid_frames = 0;
    size_t lkas0 = 0;
    size_t lkas1 = 0;
    size_t clu1 = 0;
    size_t mdps2 = 0;
    int max_torque = 0;

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
      const auto result = controller.update(path, vehicle, now_s, tick);
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

    std::printf(
        "K7_REPLAY_OK records=%zu duration_s=%.3f ticks=%d active=%zu "
        "generated=%zu lkas0=%zu lkas1=%zu clu1=%zu mdps2=%zu "
        "max_torque=%d compute_ms=%.3f\n",
        rx_frames, replay.duration_s(), ticks, active_ticks, generated_frames,
        lkas0, lkas1, clu1, mdps2, max_torque, elapsed_ms);
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "check_k7_control_replay: %s\n", error.what());
    return 1;
  }
}
