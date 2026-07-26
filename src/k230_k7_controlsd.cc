#include "k230_ipc.h"
#include "k7_lateral_controller.h"
#include "k7_path.h"
#include "openpilot_lateral_planner.h"
#include "param_paths.h"
#include "steering_params.h"
#include "vehicle_can.h"

#include <signal.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <mutex>

namespace {

volatile sig_atomic_t g_stop = 0;
constexpr uint32_t kExpectedPandaSafetyModel = 24;
constexpr uint32_t kExpectedPandaSafetyParam = 0;
constexpr uint64_t kPandaStateTimeoutNs = 1100000000ULL;
constexpr uint64_t kMaxCanRxAgeNs = 100000000ULL;

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

bool open_when_ready(K230CanQueue *queue, const char *topic, bool create) {
  while (!g_stop) {
    if (queue->open(topic, kK230CanQueueSlots, create)) return true;
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

float vehicle_speed_mps(const K7VehicleCanState &vehicle) {
  const float unit_scale = vehicle.speed_unit_mph ? 1.609344f : 1.0f;
  return std::max(0.0f, vehicle.cluster_speed * unit_scale / 3.6f);
}

class LateralPlannerWorker {
public:
  explicit LateralPlannerWorker(const K7SteeringParams &params)
      : planner_(params), thread_(&LateralPlannerWorker::run, this) {}

  ~LateralPlannerWorker() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    condition_.notify_one();
    thread_.join();
  }

  void submit(const K230ModelState &model, const K7VehicleCanState &vehicle,
              float v_ego, float measured_curvature, bool active,
              float output_scale) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      request_.model = model;
      request_.vehicle = vehicle;
      request_.v_ego = v_ego;
      request_.measured_curvature = measured_curvature;
      request_.active = active;
      request_.output_scale = output_scale;
      pending_ = true;
    }
    condition_.notify_one();
  }

  LateralTarget latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }

private:
  struct Request {
    K230ModelState model;
    K7VehicleCanState vehicle;
    float v_ego = 0.0f;
    float measured_curvature = 0.0f;
    bool active = false;
    float output_scale = 0.0f;
  };

  void run() {
    while (true) {
      Request request;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stop_ || pending_; });
        if (stop_) return;
        request = request_;
        pending_ = false;
      }
      const LateralTarget result = planner_.update(
          request.model, request.vehicle, request.v_ego,
          request.measured_curvature, request.active, request.output_scale);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = result;
      }
    }
  }

  OpenpilotLateralPlanner planner_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  Request request_;
  LateralTarget latest_;
  bool pending_ = false;
  bool stop_ = false;
  std::thread thread_;
};

}  // namespace

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    K230CanQueue can_sub;
    K230LatestChannel model_sub;
    K230LatestChannel panda_state_sub;
    K230CanQueue sendcan_pub;
    K230LatestChannel control_state_pub;
    if (!open_when_ready(&can_sub, kK230CanTopic, true) ||
        !open_when_ready(&model_sub, kK230ModelStateTopic, sizeof(K230ModelState), false) ||
        !open_when_ready(&panda_state_sub, kK230PandaStateTopic,
                         sizeof(K230PandaState), true) ||
        !open_when_ready(&sendcan_pub, kK230SendCanTopic, true) ||
        !open_when_ready(&control_state_pub, kK230ControlStateTopic,
                         sizeof(K230ControlState), true)) {
      return 0;
    }
    sendcan_pub.reset();

    K7LateralControllerConfig config;
    config.enabled = env_enabled("K230_K7_CONTROL", true);
    config.force_engaged = env_enabled("K230_K7_FORCE_ENGAGED", false);
    const char *steering_override = std::getenv("K230_K7_STEERING_PARAMS");
    const char *driving_override = std::getenv("K230_K7_DRIVING_PARAMS");
    const std::string steering_path = steering_override && steering_override[0] != '\0'
        ? steering_override : k230_param_path("k7_yg_steering.json");
    const std::string driving_path = driving_override && driving_override[0] != '\0'
        ? driving_override : k230_param_path("k7_yg_driving.json");
    std::string error;
    if (!load_k7_steering_params_json(steering_path, &config.steering_params, &error)) {
      throw std::runtime_error("steering params " + steering_path + ": " + error);
    }
    if (!load_k7_driving_params_json(driving_path, &config.driving_params, &error)) {
      throw std::runtime_error("driving params " + driving_path + ": " + error);
    }
    std::fprintf(stderr,
                 "k230_k7_controlsd: params steering=%s driving=%s mdpsSpoof=%.1fkph\n",
                 steering_path.c_str(), driving_path.c_str(),
                 config.driving_params.mdps_speed_spoof_kph);
    K7LateralController controller(config);
    LateralPlannerWorker lateral_planner(config.steering_params);
    K7VehicleCanState vehicle;
    K230ModelState model;
    K230PandaState panda_state;
    LateralTarget lateral_target;
    uint64_t model_seq = 0;
    uint64_t panda_state_seq = 0;
    unsigned can_frames = 0;
    unsigned generated_frames = 0;
    unsigned publish_errors = 0;
    unsigned send_queue_full = 0;
    unsigned stale_can_batches = 0;
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
      const uint64_t now_ns = k230_now_ns();

      K230CanBatch can_batch;
      while (can_sub.pop(&can_batch)) {
        if (!k230_can_batch_is_fresh(can_batch, now_ns, kMaxCanRxAgeNs)) {
          ++stale_can_batches;
          continue;
        }
        apply_can_batch(can_batch, now_s, &vehicle);
        can_frames += std::min<uint32_t>(can_batch.count, kK230CanBatchMaxFrames);
      }
      uint64_t next_model_seq = model_seq;
      if (model_sub.read(&model, sizeof(model), &next_model_seq) &&
          next_model_seq != model_seq) {
        model_seq = next_model_seq;
        lateral_planner.submit(
            model, vehicle, vehicle_speed_mps(vehicle),
            last_result.actual_curvature, last_result.active,
            last_result.normalized_output);
      }
      uint64_t next_panda_state_seq = panda_state_seq;
      if (panda_state_sub.read(&panda_state, sizeof(panda_state),
                               &next_panda_state_seq) &&
          next_panda_state_seq != panda_state_seq) {
        panda_state_seq = next_panda_state_seq;
      }
      lateral_target = lateral_planner.latest();

      const bool panda_state_fresh =
          panda_state.timestamp_ns != 0 && now_ns >= panda_state.timestamp_ns &&
          now_ns - panda_state.timestamp_ns <= kPandaStateTimeoutNs;
      const bool panda_ready =
          config.force_engaged ||
          (panda_state_fresh && panda_state.connected != 0 &&
           panda_state.comms_healthy != 0 && panda_state.tx_enabled != 0 &&
           panda_state.heartbeat_lost == 0 &&
           panda_state.safety_mode == kExpectedPandaSafetyModel &&
           panda_state.safety_param == kExpectedPandaSafetyParam);
      const bool panda_controls_allowed =
          config.force_engaged || (panda_ready && panda_state.controls_allowed != 0);
      const LateralPath path = k7_path_from_model_state(
          model, now_ns,
          static_cast<unsigned long long>(config.driving_params.model_timeout_ms) * 1000000ULL);
      last_result = controller.update(path, lateral_target, vehicle, now_s,
                                      control_frame++, panda_ready,
                                      panda_controls_allowed);

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
      control_state.left_blinker = vehicle.left_blinker ? 1U : 0U;
      control_state.right_blinker = vehicle.right_blinker ? 1U : 0U;
      control_state.speed_kph = last_result.speed_kph;
      control_state.steering_angle_deg = vehicle.steering_angle_deg;
      control_state.desired_curvature = last_result.desired_curvature;
      control_state.actual_curvature = last_result.actual_curvature;
      control_state.normalized_output = last_result.normalized_output;
      control_state.desired_torque = last_result.desired_torque;
      control_state.apply_torque = last_result.apply_torque;
      control_state.driver_torque = vehicle.driver_torque;
      control_state.desire = static_cast<uint32_t>(lateral_target.desire);
      std::snprintf(control_state.active_block, sizeof(control_state.active_block), "%s",
                    last_result.active_block.c_str());
      if (!control_state_pub.publish(&control_state, sizeof(control_state))) {
        ++publish_errors;
      }

      if (last_result.should_send && !last_result.frames.empty()) {
        const K230CanBatch send_batch = make_send_batch(last_result.frames);
        if (!sendcan_pub.push(send_batch)) {
          ++publish_errors;
          ++send_queue_full;
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
                     "misses=%u can=%u generated=%u errors=%u txFull=%u rxStale=%u "
                     "queue=%llu/%llu "
                     "engaged=%u active=%u "
                     "panda=%u/%u plan=%u mpc=%u desire=%d "
                     "torque=%d/%d driver=%d angle=%.2f "
                     "curve=%.6f/%.6f error=%.6f pathY=%.3f "
                     "speed=%.1f block=%s\n",
                     ticks / window_s, work_sum_us / std::max(1U, ticks), work_max_us,
                     misses, can_frames, generated_frames, publish_errors,
                     send_queue_full, stale_can_batches,
                     static_cast<unsigned long long>(sendcan_pub.depth()),
                     static_cast<unsigned long long>(can_sub.depth()),
                     last_result.engaged ? 1 : 0, last_result.active ? 1 : 0,
                     panda_ready ? 1 : 0, panda_controls_allowed ? 1 : 0,
                     lateral_target.valid ? 1 : 0,
                     lateral_target.mpc_solution_valid ? 1 : 0,
                     lateral_target.desire,
                     last_result.desired_torque, last_result.apply_torque,
                     vehicle.driver_torque, vehicle.steering_angle_deg,
                     last_result.desired_curvature, last_result.actual_curvature,
                     last_result.curvature_error, lateral_target.target_y,
                     last_result.speed_kph,
                     last_result.active_block.c_str());
        log_start = work_end;
        work_sum_us = work_max_us = 0.0;
        ticks = misses = can_frames = generated_frames = publish_errors =
            send_queue_full = stale_can_batches = 0;
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
