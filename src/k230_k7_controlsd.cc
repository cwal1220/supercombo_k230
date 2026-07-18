#include "k230_ipc.h"
#include "k7_lateral_controller.h"
#include "k7_path.h"
#include "steering_params.h"
#include "vehicle_can.h"

#include <signal.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace {

volatile sig_atomic_t g_stop = 0;

void signal_handler(int) {
  g_stop = 1;
}

bool env_enabled(const char *name, bool default_value = false) {
  const char *value = std::getenv(name);
  if (!value) return default_value;
  return std::strcmp(value, "0") != 0 && std::strcmp(value, "false") != 0 &&
         std::strcmp(value, "FALSE") != 0;
}

bool open_when_ready(K230LatestChannel *channel, const char *topic,
                     size_t size, bool create) {
  while (!g_stop) {
    if (channel->open(topic, size, create)) return true;
    std::fprintf(stderr, "k230_k7_controlsd: waiting for %s\n", topic);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

void apply_can_batch(const K230CanBatch &batch, double now_s,
                     K7VehicleCanState *vehicle) {
  if (!batch.valid) return;
  const uint32_t count = std::min<uint32_t>(batch.count, kK230CanBatchMaxFrames);
  for (uint32_t i = 0; i < count; ++i) {
    const K230CanFrame &frame = batch.frames[i];
    if (frame.flags != 0 || frame.data_len > 8 || frame.src > 7) continue;
    std::array<uint8_t, 8> data = {};
    std::copy_n(frame.data, frame.data_len, data.begin());
    update_k7_vehicle_can_state(vehicle, frame.address, data,
                                static_cast<uint8_t>(frame.data_len),
                                static_cast<uint8_t>(frame.src), now_s);
  }
}

K230CanBatch make_send_batch(const std::vector<CanFrame> &frames) {
  K230CanBatch batch;
  batch.timestamp_ns = k230_now_ns();
  batch.valid = 1;
  batch.count = static_cast<uint32_t>(
      std::min<size_t>(frames.size(), kK230CanBatchMaxFrames));
  batch.dropped = static_cast<uint32_t>(frames.size() - batch.count);
  for (uint32_t i = 0; i < batch.count; ++i) {
    const CanFrame &src = frames[i];
    K230CanFrame &dst = batch.frames[i];
    dst.address = src.address;
    dst.src = src.bus;
    dst.data_len = src.length;
    std::copy_n(src.data.begin(), src.length, dst.data);
  }
  return batch;
}

}  // namespace

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    K230LatestChannel can_sub;
    K230LatestChannel model_sub;
    K230LatestChannel sendcan_pub;
    K230LatestChannel control_state_pub;
    if (!open_when_ready(&can_sub, kK230CanTopic, sizeof(K230CanBatch), false) ||
        !open_when_ready(&model_sub, kK230ModelStateTopic, sizeof(K230ModelState), false) ||
        !open_when_ready(&sendcan_pub, kK230SendCanTopic, sizeof(K230CanBatch), true) ||
        !open_when_ready(&control_state_pub, kK230ControlStateTopic,
                         sizeof(K230ControlState), true)) {
      return 0;
    }

    K7LateralControllerConfig config;
    config.enabled = env_enabled("K230_K7_CONTROL", true);
    config.force_engaged = env_enabled("K230_K7_FORCE_ENGAGED", false);
    if (const char *params = std::getenv("K230_K7_STEERING_PARAMS")) {
      std::string error;
      if (!load_k7_steering_params_json(params, &config.steering_params, &error)) {
        throw std::runtime_error("steering params: " + error);
      }
    }
    K7LateralController controller(config);
    K7VehicleCanState vehicle;
    K230ModelState model;
    uint64_t can_seq = 0;
    uint64_t model_seq = 0;
    unsigned can_frames = 0;
    unsigned generated_frames = 0;
    unsigned publish_errors = 0;
    int control_frame = 0;

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto next_tick = start;
    auto log_start = start;
    double work_sum_us = 0.0;
    double work_max_us = 0.0;
    unsigned ticks = 0;
    unsigned misses = 0;
    K7LateralControlResult last_result;

    while (!g_stop) {
      next_tick += std::chrono::milliseconds(10);
      const auto work_start = Clock::now();
      const double now_s = std::chrono::duration<double>(work_start - start).count();

      K230CanBatch can_batch;
      uint64_t next_can_seq = can_seq;
      if (can_sub.read(&can_batch, sizeof(can_batch), &next_can_seq) &&
          next_can_seq != can_seq) {
        can_seq = next_can_seq;
        apply_can_batch(can_batch, now_s, &vehicle);
        can_frames += std::min<uint32_t>(can_batch.count, kK230CanBatchMaxFrames);
      }
      uint64_t next_model_seq = model_seq;
      if (model_sub.read(&model, sizeof(model), &next_model_seq) &&
          next_model_seq != model_seq) {
        model_seq = next_model_seq;
      }

      const LateralPath path = k7_path_from_model_state(model, k230_now_ns());
      last_result = controller.update(path, vehicle, now_s, control_frame++);

      K230ControlState control_state;
      control_state.timestamp_ns = k230_now_ns();
      control_state.enabled = config.enabled ? 1U : 0U;
      control_state.engaged = last_result.engaged ? 1U : 0U;
      control_state.active = last_result.active ? 1U : 0U;
      control_state.should_send = last_result.should_send ? 1U : 0U;
      control_state.path_usable = last_result.path_usable ? 1U : 0U;
      control_state.seeds_ready = last_result.seeds_ready ? 1U : 0U;
      control_state.vehicle_fresh = last_result.vehicle_fresh ? 1U : 0U;
      control_state.steering_fault = vehicle.steering_fault ? 1U : 0U;
      control_state.speed_kph = last_result.speed_kph;
      control_state.steering_angle_deg = vehicle.steering_angle_deg;
      control_state.desired_curvature = last_result.desired_curvature;
      control_state.actual_curvature = last_result.actual_curvature;
      control_state.normalized_output = last_result.normalized_output;
      control_state.desired_torque = last_result.desired_torque;
      control_state.apply_torque = last_result.apply_torque;
      control_state.driver_torque = vehicle.driver_torque;
      std::snprintf(control_state.active_block, sizeof(control_state.active_block), "%s",
                    last_result.active_block.c_str());
      if (!control_state_pub.publish(&control_state, sizeof(control_state))) {
        ++publish_errors;
      }

      if (last_result.should_send && !last_result.frames.empty()) {
        const K230CanBatch send_batch = make_send_batch(last_result.frames);
        if (!sendcan_pub.publish(&send_batch, sizeof(send_batch))) {
          ++publish_errors;
        } else {
          generated_frames += static_cast<unsigned>(last_result.frames.size());
        }
      }

      const auto work_end = Clock::now();
      const double work_us =
          std::chrono::duration<double, std::micro>(work_end - work_start).count();
      work_sum_us += work_us;
      work_max_us = std::max(work_max_us, work_us);
      ++ticks;
      if (work_end > next_tick) ++misses;
      if (work_end - log_start >= std::chrono::seconds(1)) {
        const double window_s = std::chrono::duration<double>(work_end - log_start).count();
        std::fprintf(stderr,
                     "k230_k7_controlsd: hz=%.3f work_avg_us=%.1f work_max_us=%.1f "
                     "misses=%u can=%u generated=%u errors=%u engaged=%u active=%u "
                     "torque=%d speed=%.1f block=%s\n",
                     ticks / window_s, work_sum_us / std::max(1U, ticks), work_max_us,
                     misses, can_frames, generated_frames, publish_errors,
                     last_result.engaged ? 1 : 0, last_result.active ? 1 : 0,
                     last_result.apply_torque, last_result.speed_kph,
                     last_result.active_block.c_str());
        log_start = work_end;
        work_sum_us = work_max_us = 0.0;
        ticks = misses = can_frames = generated_frames = publish_errors = 0;
      }

      if (next_tick > Clock::now()) std::this_thread::sleep_until(next_tick);
    }
    std::fprintf(stderr, "k230_k7_controlsd: stopping\n");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "k230_k7_controlsd error: %s\n", error.what());
    return 1;
  }
}
