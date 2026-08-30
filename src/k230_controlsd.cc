#include "k230_ipc.h"
#include "departure_alert.h"
#include "adaptive_cruise.h"
#include "lateral_controller.h"
#include "lateral_path.h"
#include "openpilot_lateral_planner.h"
#include "common_utils.h"
#include "steering_params.h"
#include "vehicle_can.h"

#include <signal.h>
#include <sys/stat.h>

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
volatile sig_atomic_t g_reload_params = 0;
constexpr uint32_t kExpectedPandaSafetyModel = 24;
constexpr uint32_t kExpectedPandaSafetyParam = 0;
constexpr uint64_t kPandaStateTimeoutNs = 1100000000ULL;
constexpr uint64_t kPandaHealthHoldNs = 100000000ULL;
constexpr uint64_t kAlertModelTimeoutNs = 500000000ULL;
constexpr float kLeadProbabilityThreshold = 0.5f;
constexpr float kRadarToCameraDistanceM = 1.52f;
constexpr uint64_t kMaxCanRxAgeNs = 100000000ULL;
/* 잘못된 모델 프레임 하나 때문에 LKAS active가 깜박이지 않게 한다. 모델
 * 생산자가 정상 타임아웃 안에 있을 때만 마지막 유효 경로를 유지하며, 모델이
 * 멈추거나 계속 잘못되면 즉시 제어를 해제한다. */
constexpr uint64_t kPathInvalidHoldNs = 150000000ULL;
constexpr int kParamPollIntervalMs = 100;

void reload_signal_handler(int) {
  g_reload_params = 1;
}

struct FileStamp {
  bool valid = false;
  unsigned long long device = 0;
  unsigned long long inode = 0;
  unsigned long long size = 0;
  long long modified_sec = 0;
  long long modified_nsec = 0;
};

bool operator!=(const FileStamp &left, const FileStamp &right) {
  return left.valid != right.valid ||
         left.device != right.device ||
         left.inode != right.inode ||
         left.size != right.size ||
         left.modified_sec != right.modified_sec ||
         left.modified_nsec != right.modified_nsec;
}

FileStamp file_stamp(const std::string &path) {
  struct stat info = {};
  FileStamp stamp;
  if (stat(path.c_str(), &info) != 0) return stamp;
  stamp.valid = true;
  stamp.device = static_cast<unsigned long long>(info.st_dev);
  stamp.inode = static_cast<unsigned long long>(info.st_ino);
  stamp.size = static_cast<unsigned long long>(info.st_size);
#if defined(__APPLE__)
  stamp.modified_sec = static_cast<long long>(info.st_mtimespec.tv_sec);
  stamp.modified_nsec = static_cast<long long>(info.st_mtimespec.tv_nsec);
#else
  stamp.modified_sec = static_cast<long long>(info.st_mtim.tv_sec);
  stamp.modified_nsec = static_cast<long long>(info.st_mtim.tv_nsec);
#endif
  return stamp;
}

bool load_runtime_params(const std::string &steering_path,
                         const std::string &driving_path,
                         const std::string &adaptive_cruise_path,
                         LateralControllerConfig *config,
                         AdaptiveCruiseConfig *adaptive_cruise_config,
                         std::string *error) {
  SteeringParams steering = config->steering_params;
  DrivingParams driving = config->driving_params;
  AdaptiveCruiseConfig adaptive_cruise = *adaptive_cruise_config;
  std::string load_error;
  if (!load_steering_params_json(steering_path, &steering, &load_error)) {
    if (error) *error = "steering " + steering_path + ": " + load_error;
    return false;
  }
  if (!load_driving_params_json(driving_path, &driving, &load_error)) {
    if (error) *error = "driving " + driving_path + ": " + load_error;
    return false;
  }
  if (!load_adaptive_cruise_params_json(
          adaptive_cruise_path, &adaptive_cruise, &load_error)) {
    if (error) {
      *error = "adaptive cruise " + adaptive_cruise_path + ": " + load_error;
    }
    return false;
  }
  config->steering_params = steering;
  config->driving_params = driving;
  *adaptive_cruise_config = adaptive_cruise;
  return true;
}

bool open_when_ready(K230LatestChannel *channel, const char *topic,
                     size_t size, bool create) {
  while (!g_stop) {
    if (channel->open(topic, size, create)) return true;
    std::fprintf(stderr, "k230_controlsd: waiting for %s\n", topic);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

bool open_when_ready(K230CanQueue *queue, const char *topic, bool create) {
  while (!g_stop) {
    if (queue->open(topic, kK230CanQueueSlots, create)) return true;
    std::fprintf(stderr, "k230_controlsd: waiting for %s\n", topic);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

void apply_can_batch(const K230CanBatch &batch, double now_s,
                     VehicleCanState *vehicle) {
  if (!batch.valid) return;
  const uint32_t count = std::min<uint32_t>(batch.count, kK230CanBatchMaxFrames);
  for (uint32_t i = 0; i < count; ++i) {
    const K230CanFrame &frame = batch.frames[i];
    if (frame.flags != 0 || frame.data_len > 8 || frame.src > 7) continue;
    std::array<uint8_t, 8> data = {};
    std::copy_n(frame.data, frame.data_len, data.begin());
    update_vehicle_can_state(vehicle, frame.address, data,
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

float vehicle_speed_mps(const VehicleCanState &vehicle, double now_s,
                        double timeout_s) {
  return std::max(0.0f, vehicle_speed_kph(vehicle, now_s, timeout_s) / 3.6f);
}

class LateralPlannerWorker {
public:
  LateralPlannerWorker(const SteeringParams &params,
                       const DrivingParams &driving)
      : planner_(params, driving), pending_steering_(params),
        pending_driving_(driving),
        thread_(&LateralPlannerWorker::run, this) {}

  ~LateralPlannerWorker() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    condition_.notify_one();
    thread_.join();
  }

  void submit(const K230ModelState &model, const VehicleCanState &vehicle,
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

  void update_params(const SteeringParams &params,
                     const DrivingParams &driving) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_steering_ = params;
      pending_driving_ = driving;
      params_pending_ = true;
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
    VehicleCanState vehicle;
    float v_ego = 0.0f;
    float measured_curvature = 0.0f;
    bool active = false;
    float output_scale = 0.0f;
  };

  void run() {
    while (true) {
      Request request;
      SteeringParams steering;
      DrivingParams driving;
      bool has_request = false;
      bool apply_params = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
          return stop_ || pending_ || params_pending_;
        });
        if (stop_) return;
        if (params_pending_) {
          steering = pending_steering_;
          driving = pending_driving_;
          params_pending_ = false;
          apply_params = true;
        }
        if (pending_) {
          request = request_;
          pending_ = false;
          has_request = true;
        }
      }
      if (apply_params) planner_.update_params(steering, driving);
      if (!has_request) continue;
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
  SteeringParams pending_steering_;
  DrivingParams pending_driving_;
  LateralTarget latest_;
  bool pending_ = false;
  bool params_pending_ = false;
  bool stop_ = false;
  std::thread thread_;
};

}  // namespace

int main() {
  install_stop_signal_handlers(&g_stop);
  signal(SIGHUP, reload_signal_handler);

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

    LateralControllerConfig config;
    config.enabled = env_flag("K230_CONTROL", true);
    config.force_engaged = env_flag("K230_FORCE_ENGAGED", false);
    const bool adaptive_cruise_env_enabled =
        env_flag("K230_ADAPTIVE_CRUISE", true);
    AdaptiveCruiseConfig adaptive_cruise_config;
    const char *steering_override = std::getenv("K230_STEERING_PARAMS");
    const char *driving_override = std::getenv("K230_DRIVING_PARAMS");
    const char *adaptive_cruise_override =
        std::getenv("K230_ADAPTIVE_CRUISE_PARAMS");
    const std::string steering_path = steering_override && steering_override[0] != '\0'
        ? steering_override : k230_param_path("steering.json");
    const std::string driving_path = driving_override && driving_override[0] != '\0'
        ? driving_override : k230_param_path("driving.json");
    const std::string adaptive_cruise_path =
        adaptive_cruise_override && adaptive_cruise_override[0] != '\0'
            ? adaptive_cruise_override
            : k230_param_path("adaptive_cruise.json");
    std::string error;
    if (!load_runtime_params(steering_path, driving_path, adaptive_cruise_path,
                             &config, &adaptive_cruise_config, &error)) {
      throw std::runtime_error(error);
    }
    std::fprintf(stderr,
                 "k230_controlsd: params steering=%s driving=%s adaptive=%s "
                 "mdpsSpoof=%.1fkph adaptiveCruise=%u gap=%.1fm/%.1fs "
                 "decel=%.1fkph/s\n",
                 steering_path.c_str(), driving_path.c_str(),
                 adaptive_cruise_path.c_str(),
                 config.driving_params.mdps_speed_spoof_kph,
                 adaptive_cruise_env_enabled && adaptive_cruise_config.enabled
                     ? 1U : 0U,
                 adaptive_cruise_config.standstill_gap_m,
                 adaptive_cruise_config.following_time_s,
                 adaptive_cruise_config.deceleration_rate_kph_per_s);
    LateralController controller(config);
    AdaptiveCruiseController adaptive_cruise_controller(
        adaptive_cruise_config);
    DepartureAlertDetector departure_alert_detector;
    LateralPlannerWorker lateral_planner(config.steering_params,
                                         config.driving_params);
    VehicleCanState vehicle;
    K230ModelState model;
    K230PandaState panda_state;
    LateralTarget lateral_target;
    AdaptiveCruiseOutput adaptive_cruise;
    uint64_t model_seq = 0;
    uint64_t panda_state_seq = 0;
    unsigned can_frames = 0;
    unsigned generated_frames = 0;
    unsigned publish_errors = 0;
    unsigned send_queue_full = 0;
    unsigned stale_can_batches = 0;
    int control_frame = 0;
    uint32_t last_logged_alert_event_id = 0;

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto next_tick = start;
    auto log_start = start;
    auto next_param_check =
        start + std::chrono::milliseconds(kParamPollIntervalMs);
    FileStamp steering_stamp = file_stamp(steering_path);
    FileStamp driving_stamp = file_stamp(driving_path);
    FileStamp adaptive_cruise_stamp = file_stamp(adaptive_cruise_path);
    unsigned param_generation = 1;
    double work_sum_us = 0.0;
    double work_max_us = 0.0;
    unsigned ticks = 0;
    unsigned misses = 0;
    LateralControlResult last_result;
    uint32_t engage_event_id = 0;
    uint32_t disengage_event_id = 0;
    uint32_t engage_reject_event_id = 0;
    char engage_reject_block[32] = {};
    bool have_previous_engaged = false;
    bool previous_engaged = false;
    bool have_previous_active = false;
    bool previous_active = false;
    LateralPath last_usable_path;
    uint64_t last_usable_path_model_timestamp_ns = 0;
    bool have_last_panda_ready = false;
    bool last_panda_controls_allowed = false;
    uint64_t last_panda_ready_ns = 0;

    while (!g_stop) {
      next_tick += std::chrono::milliseconds(10);
      const auto work_start = Clock::now();
      const double now_s = std::chrono::duration<double>(work_start - start).count();
      const uint64_t can_now_ns = k230_now_ns();

      const bool reload_requested = g_reload_params != 0;
      if (reload_requested) g_reload_params = 0;
      if (reload_requested || work_start >= next_param_check) {
        next_param_check =
            work_start + std::chrono::milliseconds(kParamPollIntervalMs);
        const FileStamp current_steering_stamp = file_stamp(steering_path);
        const FileStamp current_driving_stamp = file_stamp(driving_path);
        const FileStamp current_adaptive_cruise_stamp =
            file_stamp(adaptive_cruise_path);
        if (reload_requested || current_steering_stamp != steering_stamp ||
            current_driving_stamp != driving_stamp ||
            current_adaptive_cruise_stamp != adaptive_cruise_stamp) {
          LateralControllerConfig candidate = config;
          AdaptiveCruiseConfig adaptive_cruise_candidate =
              adaptive_cruise_config;
          if (load_runtime_params(steering_path, driving_path,
                                  adaptive_cruise_path, &candidate,
                                  &adaptive_cruise_candidate, &error)) {
            config.steering_params = candidate.steering_params;
            config.driving_params = candidate.driving_params;
            adaptive_cruise_config = adaptive_cruise_candidate;
            controller.update_params(config.steering_params, config.driving_params);
            lateral_planner.update_params(config.steering_params,
                                          config.driving_params);
            adaptive_cruise_controller.update_config(adaptive_cruise_config);
            ++param_generation;
            std::fprintf(stderr,
                         "k230_controlsd: params reloaded generation=%u "
                         "mdpsSpoof=%.1fkph adaptiveCruise=%u gap=%.1fm/%.1fs "
                         "decel=%.1fkph/s\n",
                         param_generation,
                         config.driving_params.mdps_speed_spoof_kph,
                         adaptive_cruise_env_enabled &&
                                 adaptive_cruise_config.enabled
                             ? 1U : 0U,
                         adaptive_cruise_config.standstill_gap_m,
                         adaptive_cruise_config.following_time_s,
                         adaptive_cruise_config.deceleration_rate_kph_per_s);
          } else {
            std::fprintf(stderr,
                         "k230_controlsd: params reload rejected: %s\n",
                         error.c_str());
          }
          steering_stamp = current_steering_stamp;
          driving_stamp = current_driving_stamp;
          adaptive_cruise_stamp = current_adaptive_cruise_stamp;
        }
      }

      K230CanBatch can_batch;
      while (can_sub.pop(&can_batch)) {
        if (!k230_can_batch_is_fresh(can_batch, can_now_ns, kMaxCanRxAgeNs)) {
          ++stale_can_batches;
          continue;
        }
        apply_can_batch(can_batch, now_s, &vehicle);
        can_frames += std::min<uint32_t>(can_batch.count, kK230CanBatchMaxFrames);
      }
      bool model_updated = false;
      uint64_t next_model_seq = model_seq;
      if (model_sub.read(&model, sizeof(model), &next_model_seq) &&
          next_model_seq != model_seq) {
        model_seq = next_model_seq;
        model_updated = true;
        lateral_planner.submit(
            model, vehicle, vehicle_speed_mps(
                vehicle, now_s,
                static_cast<double>(config.driving_params.vehicle_state_timeout_ms) / 1000.0),
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

      /* IPC를 읽는 동안 새 모델/Panda 상태가 발행될 수 있으므로 freshness
       * 판정에는 공유 상태를 읽은 직후의 시간을 사용한다. */
      const uint64_t now_ns = k230_now_ns();

      const bool panda_state_fresh =
          panda_state.timestamp_ns != 0 && now_ns >= panda_state.timestamp_ns &&
          now_ns - panda_state.timestamp_ns <= kPandaStateTimeoutNs;
      const bool panda_transport_ready =
          panda_state_fresh && panda_state.connected != 0 &&
          panda_state.comms_healthy != 0 && panda_state.tx_enabled != 0;
      const bool panda_safety_ready =
          panda_state.heartbeat_lost == 0 &&
          panda_state.safety_mode == kExpectedPandaSafetyModel &&
          panda_state.safety_param == kExpectedPandaSafetyParam;
      const bool panda_ready_raw = panda_transport_ready && panda_safety_ready;
      const bool panda_controls_allowed_raw =
          panda_ready_raw && panda_state.controls_allowed != 0;
      const bool panda_controls_off_explicit =
          panda_ready_raw && panda_state.controls_allowed == 0;
      if (panda_ready_raw) {
        have_last_panda_ready = true;
        last_panda_controls_allowed = panda_controls_allowed_raw;
        last_panda_ready_ns = now_ns;
      }
      const bool panda_health_hold =
          !panda_ready_raw && !panda_controls_off_explicit &&
          have_last_panda_ready && now_ns >= last_panda_ready_ns &&
          now_ns - last_panda_ready_ns <= kPandaHealthHoldNs;
      const bool panda_ready =
          config.force_engaged || panda_ready_raw || panda_health_hold;
      const bool panda_controls_allowed =
          config.force_engaged || panda_controls_allowed_raw ||
          (panda_health_hold && last_panda_controls_allowed);
      const uint64_t model_timeout_ns =
          static_cast<unsigned long long>(config.driving_params.model_timeout_ms) *
          1000000ULL;
      const LateralPath raw_path = path_from_model_state(
          model, now_ns,
          model_timeout_ns);
      LateralPath path = raw_path;
      bool path_hold_applied = false;
      if (raw_path.usable_for_steering) {
        last_usable_path = raw_path;
        last_usable_path_model_timestamp_ns = model.model_timestamp_ns;
      } else if (raw_path.invalid_reason == "path_invalid" &&
                 last_usable_path.usable_for_steering &&
                 model.model_timestamp_ns != 0 &&
                 now_ns >= model.model_timestamp_ns &&
                 now_ns - model.model_timestamp_ns <= model_timeout_ns &&
                 last_usable_path_model_timestamp_ns != 0 &&
                 now_ns >= last_usable_path_model_timestamp_ns &&
                 now_ns - last_usable_path_model_timestamp_ns <=
                     kPathInvalidHoldNs) {
        path = last_usable_path;
        path.invalid_reason.clear();
        path_hold_applied = true;
      }
      const int frame = control_frame++;
      last_result = controller.update(path, lateral_target, vehicle, now_s,
                                      frame, panda_ready,
                                      panda_controls_allowed);
      if (last_result.engage_rejected) {
        if (++engage_reject_event_id == 0) engage_reject_event_id = 1;
        std::snprintf(engage_reject_block, sizeof(engage_reject_block), "%s",
                      last_result.active_block.c_str());
        std::fprintf(stderr,
                     "k230_controlsd: engage rejected block=%s event=%u\n",
                     engage_reject_block, engage_reject_event_id);
      } else if (have_previous_engaged &&
                 last_result.engaged != previous_engaged) {
        if (last_result.engaged) {
          if (++engage_event_id == 0) engage_event_id = 1;
        } else {
          if (++disengage_event_id == 0) disengage_event_id = 1;
        }
        std::fprintf(stderr,
                     "k230_controlsd: engaged transition %u->%u "
                     "active=%u block=%s button=%d gear=%d "
                     "panda=%u/%u\n",
                     previous_engaged ? 1U : 0U, last_result.engaged ? 1U : 0U,
                     last_result.active ? 1U : 0U,
                     last_result.active_block.c_str(), vehicle.clu_button,
                     vehicle.gear, panda_ready ? 1U : 0U,
                     panda_controls_allowed ? 1U : 0U);
      }
      if (have_previous_active && last_result.active != previous_active) {
        std::fprintf(stderr,
                     "k230_controlsd: active transition %u->%u "
                     "engaged=%u block=%s raw=%s rawPoints=%zu pathPoints=%zu "
                     "hold=%u modelAgeMs=%llu panda=%u/%u "
                     "state=%u/%u/%u/%u safety=%u:%u hb=%u fresh=%u\n",
                     previous_active ? 1U : 0U, last_result.active ? 1U : 0U,
                     last_result.engaged ? 1U : 0U,
                     last_result.active_block.c_str(),
                     raw_path.invalid_reason.empty() ? "none" :
                         raw_path.invalid_reason.c_str(),
                     raw_path.points.size(), path.points.size(),
                     path_hold_applied ? 1U : 0U,
                     model.model_timestamp_ns != 0 && now_ns >= model.model_timestamp_ns
                         ? static_cast<unsigned long long>(
                               (now_ns - model.model_timestamp_ns) / 1000000ULL)
                         : 0ULL,
                     panda_ready ? 1U : 0U,
                     panda_controls_allowed ? 1U : 0U,
                     panda_state.connected, panda_state.comms_healthy,
                     panda_state.tx_enabled, panda_state.controls_allowed,
                     panda_state.safety_mode, panda_state.safety_param,
                     panda_state.heartbeat_lost, panda_state_fresh ? 1U : 0U);
      }
      previous_engaged = last_result.engaged;
      have_previous_engaged = true;
      previous_active = last_result.active;
      have_previous_active = true;

      const bool radar_lead_fresh =
          vehicle.scc11_time_s >= 0.0 && now_s >= vehicle.scc11_time_s &&
          now_s - vehicle.scc11_time_s <= 0.5;
      const bool model_fresh =
          model.valid != 0 && model.model_timestamp_ns != 0 &&
          now_ns >= model.model_timestamp_ns &&
          now_ns - model.model_timestamp_ns <= kAlertModelTimeoutNs;
      const bool vision_lead_signal_valid =
          model_fresh && model.lead.valid != 0 &&
          std::isfinite(model.lead.x) && std::isfinite(model.lead.velocity);
      const bool vision_lead_valid =
          vision_lead_signal_valid &&
          model.lead.probability >= kLeadProbabilityThreshold;
      const float ego_speed_kph = vehicle_speed_kph(vehicle, now_s);
      const float ego_speed_mps = ego_speed_kph / 3.6f;
      const float vision_lead_distance_m = vision_lead_signal_valid
          ? model.lead.x - kRadarToCameraDistanceM
          : 0.0f;
      const float vision_lead_relative_speed_mps = vision_lead_signal_valid
          ? model.lead.velocity - ego_speed_mps
          : 0.0f;
      DepartureAlertInput alert_input;
      alert_input.now_s = now_s;
      alert_input.vehicle_valid = last_result.vehicle_fresh;
      alert_input.gear = vehicle.gear;
      alert_input.speed_mps = ego_speed_mps;
      alert_input.gas_pressed = vehicle.gas_pressed;
      alert_input.lead_updated = model_updated;
      alert_input.lead_valid = vision_lead_valid;
      alert_input.lead_distance_m =
          vision_lead_valid ? vision_lead_distance_m : 0.0f;
      alert_input.lead_relative_speed_mps =
          vision_lead_valid ? vision_lead_relative_speed_mps : 0.0f;

      AdaptiveCruiseInput adaptive_input;
      adaptive_input.now_s = now_s;
      adaptive_input.enabled =
          adaptive_cruise_env_enabled && adaptive_cruise_config.enabled;
      adaptive_input.controls_ready =
          last_result.active && panda_ready && panda_controls_allowed &&
          vehicle.has_clu11_seed;
      adaptive_input.cruise_active = vehicle.cruise_active;
      adaptive_input.brake_pressed = vehicle.brake_pressed;
      adaptive_input.gas_pressed = vehicle.gas_pressed;
      adaptive_input.driver_accelerator_override = vehicle.driver_override != 0;
      adaptive_input.speed_unit_mph = vehicle.speed_unit_mph;
      adaptive_input.driver_button = vehicle.clu_button;
      adaptive_input.driver_main_button = vehicle.clu_main_button;
      adaptive_input.ego_speed_kph = ego_speed_kph;
      adaptive_input.driver_set_speed_kph = cruise_set_speed_kph(vehicle);
      adaptive_input.vision_lead_updated = model_updated;
      adaptive_input.vision_lead_valid = vision_lead_signal_valid;
      adaptive_input.vision_lead_probability = model.lead.probability;
      adaptive_input.vision_lead_distance_m = vision_lead_distance_m;
      adaptive_input.vision_lead_relative_speed_mps =
          vision_lead_relative_speed_mps;
      adaptive_cruise = adaptive_cruise_controller.update(adaptive_input);

      if (adaptive_cruise.command_button != 0) {
        const HyundaiClu11Values clu_seed = decode_clu11(vehicle.clu11_seed);
        HyundaiCluCommand command;
        command.button = adaptive_cruise.command_button;
        command.speed = clu_seed.speed;
        command.frame = frame;
        last_result.frames.push_back(
            create_clu11_frame(clu_seed, command, kPowertrainBus));
        last_result.should_send = true;
      }

      alert_input.model_updated = model_updated;
      alert_input.model_valid = model_fresh;
      alert_input.plan_distance_m =
          model_fresh ? model.plan[kTrajectorySize - 1].x : 0.0f;
      const DepartureAlertOutput departure_alert =
          departure_alert_detector.update(alert_input);
      if (departure_alert.event_id != 0 &&
          departure_alert.event_id != last_logged_alert_event_id) {
        last_logged_alert_event_id = departure_alert.event_id;
        std::fprintf(
            stderr,
            "k230_controlsd: departure alert=%s event=%u "
            "visionLead=%.1fm rel=%.1fm/s p=%.2f plan=%.1fm\n",
            departure_alert_name(departure_alert.type),
            departure_alert.event_id,
            alert_input.lead_distance_m,
            alert_input.lead_relative_speed_mps,
            model.lead.probability,
            alert_input.plan_distance_m);
      }

      K230ControlState control_state;
      control_state.timestamp_ns = k230_now_ns();
      control_state.enabled = config.enabled ? 1U : 0U;
      control_state.engaged = last_result.engaged ? 1U : 0U;
      control_state.active = last_result.active ? 1U : 0U;
      control_state.should_send = last_result.should_send ? 1U : 0U;
      control_state.path_usable = last_result.path_usable ? 1U : 0U;
      control_state.hud_flags =
          (lateral_target.laneless_mode ? kK230HudFlagLaneless : 0U) |
          (last_result.vehicle_fresh && vehicle.brake_hold
               ? kK230HudFlagBrakeHold
               : 0U);
      control_state.seeds_ready = last_result.seeds_ready ? 1U : 0U;
      control_state.vehicle_fresh = last_result.vehicle_fresh ? 1U : 0U;
      control_state.steering_fault = vehicle.steering_fault ? 1U : 0U;
      control_state.left_blinker = vehicle.left_blinker ? 1U : 0U;
      control_state.right_blinker = vehicle.right_blinker ? 1U : 0U;
      control_state.cruise_active = vehicle.cruise_active ? 1U : 0U;
      control_state.gear = vehicle.gear;
      control_state.cluster_speed_kph = last_result.cluster_speed_kph;
      const float driver_set_speed_kph = cruise_set_speed_kph(vehicle);
      control_state.cruise_max_speed_kph = adaptive_cruise.session_valid
          ? adaptive_cruise.maximum_speed_kph
          : driver_set_speed_kph;
      control_state.cruise_command_speed_kph = adaptive_cruise.session_valid
          ? adaptive_cruise.commanded_speed_kph
          : driver_set_speed_kph;
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
      control_state.radar_lead_valid =
          radar_lead_fresh && vehicle.radar_lead_valid ? 1U : 0U;
      control_state.radar_lead_distance_m = vehicle.radar_lead_distance_m;
      control_state.radar_lead_relative_speed_mps =
          vehicle.radar_lead_relative_speed_mps;
      control_state.departure_alert_type =
          static_cast<uint32_t>(departure_alert.type);
      control_state.departure_alert_event_id = departure_alert.event_id;
      control_state.green_light_alert_armed =
          departure_alert.green_light_armed ? 1U : 0U;
      control_state.tpms_valid =
          tpms_state_fresh(vehicle, now_s) ? 1U : 0U;
      control_state.tpms_unit = static_cast<uint32_t>(vehicle.tpms_unit);
      control_state.tpms_pressure_fl = vehicle.tpms_pressure_fl;
      control_state.tpms_pressure_fr = vehicle.tpms_pressure_fr;
      control_state.tpms_pressure_rl = vehicle.tpms_pressure_rl;
      control_state.tpms_pressure_rr = vehicle.tpms_pressure_rr;
      control_state.tpms_warning = vehicle.tpms_warning ? 1U : 0U;
      control_state.engage_event_id = engage_event_id;
      control_state.disengage_event_id = disengage_event_id;
      control_state.engage_reject_event_id = engage_reject_event_id;
      std::memcpy(control_state.engage_reject_block, engage_reject_block,
                  sizeof(control_state.engage_reject_block));
      control_state.ego_speed_kph = ego_speed_kph;
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
                     "k230_controlsd: hz=%.3f work_avg_us=%.1f work_max_us=%.1f "
                     "misses=%u can=%u generated=%u errors=%u txFull=%u rxStale=%u "
                     "queue=%llu/%llu params=%u "
                     "engaged=%u active=%u "
                     "panda=%u/%u plan=%u mpc=%u desire=%d "
                     "torque=%d/%d driver=%d angle=%.2f "
                     "curve=%.6f/%.6f curveVm=%.6f curveYaw=%.6f lat=%.2f bank=%.2f long=%.2f "
                     "error=%.6f pathY=%.3f "
                     "laneC=%.3f laneW=%.2f lane=%.2f/%.2f "
                     "lprob=%.2f/%.2f/%.2f lstd=%.2f/%.2f "
                     "cluster=%.1f wheel=%.1f cruise=%u max=%.1f cmd=%.1f target=%.1f "
                     "lead=%u/%.1f/%.1f button=%d pedal=%d/%d block=%s\n",
                     ticks / window_s, work_sum_us / std::max(1U, ticks), work_max_us,
                     misses, can_frames, generated_frames, publish_errors,
                     send_queue_full, stale_can_batches,
                     static_cast<unsigned long long>(sendcan_pub.depth()),
                     static_cast<unsigned long long>(can_sub.depth()),
                     param_generation,
                     last_result.engaged ? 1 : 0, last_result.active ? 1 : 0,
                     panda_ready ? 1 : 0, panda_controls_allowed ? 1 : 0,
                     lateral_target.valid ? 1 : 0,
                     lateral_target.mpc_solution_valid ? 1 : 0,
                     lateral_target.desire,
                     last_result.desired_torque, last_result.apply_torque,
                     vehicle.driver_torque, vehicle.steering_angle_deg,
                     last_result.desired_curvature, last_result.actual_curvature,
                     last_result.actual_curvature_vm,
                     last_result.actual_curvature_yaw,
                     vehicle.lat_accel_mps2,
                     controller.road_bank_lat_accel(),
                     vehicle.long_accel_mps2,
                     last_result.curvature_error, lateral_target.target_y_m,
                     0.5 * (lateral_target.lane_left_y_m +
                            lateral_target.lane_right_y_m),
                     lateral_target.lane_width_m,
                     lateral_target.lane_left_y_m,
                     lateral_target.lane_right_y_m,
                     lateral_target.lane_left_prob,
                     lateral_target.lane_right_prob,
                     lateral_target.lane_d_prob,
                     lateral_target.lane_left_std,
                     lateral_target.lane_right_std,
                     last_result.cluster_speed_kph,
                     last_result.control_speed_kph,
                     adaptive_cruise.active ? 1U : 0U,
                     adaptive_cruise.maximum_speed_kph,
                     adaptive_cruise.commanded_speed_kph,
                     adaptive_cruise.target_speed_kph,
                     adaptive_cruise.lead_valid ? 1U : 0U,
                     alert_input.lead_distance_m,
                     alert_input.lead_relative_speed_mps,
                     adaptive_cruise.command_button,
                     vehicle.gas, vehicle.driver_override,
                     last_result.active_block.c_str());
        log_start = work_end;
        work_sum_us = work_max_us = 0.0;
        ticks = misses = can_frames = generated_frames = publish_errors =
            send_queue_full = stale_can_batches = 0;
      }

      /* tick을 넘겼으면 밀린 만큼 따라잡지 않고 현재 시각으로 재동기화한다.
       * next_tick을 과거에 둔 채로 두면 다음 몇 번의 반복이 sleep 없이 연속
       * 실행되어 LKAS frame과 counter가 한꺼번에 몰려 나간다. */
      const auto tick_end = Clock::now();
      if (next_tick > tick_end) {
        std::this_thread::sleep_until(next_tick);
      } else {
        next_tick = tick_end;
      }
    }
    std::fprintf(stderr, "k230_controlsd: stopping\n");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "k230_controlsd error: %s\n", error.what());
    return 1;
  }
}
