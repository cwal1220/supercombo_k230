#include "ipc_channels.h"
#include "departure_alert.h"
#include "adaptive_cruise.h"
#include "control_holds.h"
#include "lateral_controller.h"
#include "lateral_path.h"
#include "lateral_planner.h"
#include "utils_process.h"
#include "utils_time.h"
#include "control_params.h"
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
constexpr uint64_t kAlertModelTimeoutNs = 500000000ULL;
constexpr float kLeadProbabilityThreshold = 0.5f;
constexpr float kRadarToCameraDistanceM = 1.52f;
constexpr uint64_t kMaxCanRxAgeNs = 100000000ULL;
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
  return k230_make_can_batch(frames, [](K230CanFrame *dst, const CanFrame &src) {
    dst->address = src.address;
    dst->src = src.bus;
    dst->data_len = src.length;
    std::copy_n(src.data.begin(), src.length, dst->data);
  });
}

float vehicle_speed_mps(const VehicleCanState &vehicle, double now_s,
                        double timeout_s) {
  return std::max(0.0f, vehicle_speed_kph(vehicle, now_s, timeout_s) / 3.6f);
}

/* steering/driving/adaptive_cruise JSON을 stat으로 감시하고, 바뀌었거나 SIGHUP이
 * 오면 셋을 다시 읽는다. 하나라도 거부되면 셋 다 이전 값을 유지한다. */
class RuntimeParams {
public:
  RuntimeParams(std::string steering_path, std::string driving_path,
                std::string adaptive_cruise_path,
                std::chrono::steady_clock::time_point now)
      : steering_path_(std::move(steering_path)),
        driving_path_(std::move(driving_path)),
        adaptive_cruise_path_(std::move(adaptive_cruise_path)),
        next_check_(now + std::chrono::milliseconds(kParamPollIntervalMs)) {
    stamp();
  }

  // 적용됐으면 true. 호출자가 컨트롤러/플래너에 새 값을 넘긴다.
  bool poll(std::chrono::steady_clock::time_point now, bool reload_requested,
            LateralControllerConfig *config,
            AdaptiveCruiseConfig *adaptive_cruise_config) {
    if (!reload_requested && now < next_check_) return false;
    next_check_ = now + std::chrono::milliseconds(kParamPollIntervalMs);
    const FileStamp steering = file_stamp(steering_path_);
    const FileStamp driving = file_stamp(driving_path_);
    const FileStamp adaptive = file_stamp(adaptive_cruise_path_);
    const bool changed = steering != steering_stamp_ || driving != driving_stamp_ ||
                         adaptive != adaptive_cruise_stamp_;
    if (!reload_requested && !changed) return false;
    steering_stamp_ = steering;
    driving_stamp_ = driving;
    adaptive_cruise_stamp_ = adaptive;
    LateralControllerConfig candidate = *config;
    AdaptiveCruiseConfig adaptive_candidate = *adaptive_cruise_config;
    std::string error;
    if (!load_runtime_params(steering_path_, driving_path_, adaptive_cruise_path_,
                             &candidate, &adaptive_candidate, &error)) {
      std::fprintf(stderr, "k230_controlsd: params reload rejected: %s\n",
                   error.c_str());
      return false;
    }
    config->steering_params = candidate.steering_params;
    config->driving_params = candidate.driving_params;
    *adaptive_cruise_config = adaptive_candidate;
    ++generation_;
    std::fprintf(stderr,
                 "k230_controlsd: params reloaded generation=%u "
                 "mdpsSpoof=%.1fkph adaptiveCruise=%u gap=%.1fm/%.1fs "
                 "decel=%.1fkph/s\n",
                 generation_, config->driving_params.mdps_speed_spoof_kph,
                 adaptive_cruise_config->enabled ? 1U : 0U,
                 adaptive_cruise_config->standstill_gap_m,
                 adaptive_cruise_config->following_time_s,
                 adaptive_cruise_config->deceleration_rate_kph_per_s);
    return true;
  }

  unsigned generation() const { return generation_; }

private:
  void stamp() {
    steering_stamp_ = file_stamp(steering_path_);
    driving_stamp_ = file_stamp(driving_path_);
    adaptive_cruise_stamp_ = file_stamp(adaptive_cruise_path_);
  }

  std::string steering_path_;
  std::string driving_path_;
  std::string adaptive_cruise_path_;
  FileStamp steering_stamp_;
  FileStamp driving_stamp_;
  FileStamp adaptive_cruise_stamp_;
  std::chrono::steady_clock::time_point next_check_;
  unsigned generation_ = 1;
};

/* engage/disengage/거부 이벤트 id와 전이 로그. id는 0을 건너뛰어 HUD가 새
 * 이벤트를 구분한다. */
struct EngageEvents {
  uint32_t engage_id = 0;
  uint32_t disengage_id = 0;
  uint32_t reject_id = 0;
  char reject_block[32] = {};
  bool have_previous = false;
  bool previous_engaged = false;
  bool previous_active = false;

  void update(const LateralControlResult &result, const VehicleCanState &vehicle,
              const PandaGateOutput &panda, const K230PandaState &panda_state,
              const PathHoldOutput &held, const K230ModelState &model,
              uint64_t now_ns) {
    if (result.engage_rejected) {
      if (++reject_id == 0) reject_id = 1;
      std::snprintf(reject_block, sizeof(reject_block), "%s",
                    result.active_block.c_str());
      std::fprintf(stderr,
                   "k230_controlsd: engage rejected block=%s event=%u\n",
                   reject_block, reject_id);
    } else if (have_previous && result.engaged != previous_engaged) {
      if (result.engaged) {
        if (++engage_id == 0) engage_id = 1;
      } else {
        if (++disengage_id == 0) disengage_id = 1;
      }
      std::fprintf(stderr,
                   "k230_controlsd: engaged transition %u->%u "
                   "active=%u block=%s button=%d gear=%d "
                   "panda=%u/%u\n",
                   previous_engaged ? 1U : 0U, result.engaged ? 1U : 0U,
                   result.active ? 1U : 0U,
                   result.active_block.c_str(), vehicle.clu_button,
                   vehicle.gear, panda.ready ? 1U : 0U,
                   panda.controls_allowed ? 1U : 0U);
    }
    if (have_previous && result.active != previous_active) {
      std::fprintf(stderr,
                   "k230_controlsd: active transition %u->%u "
                   "engaged=%u block=%s raw=%s rawPoints=%d rawReachM=%.1f "
                   "pathPoints=%d hold=%u modelAgeMs=%llu panda=%u/%u "
                   "state=%u/%u/%u/%u safety=%u:%u hb=%u fresh=%u\n",
                   previous_active ? 1U : 0U, result.active ? 1U : 0U,
                   result.engaged ? 1U : 0U,
                   result.active_block.c_str(),
                   held.raw.invalid_reason.empty() ? "none" :
                       held.raw.invalid_reason.c_str(),
                   held.raw.point_count, static_cast<double>(held.raw.reach_m),
                   held.path.point_count,
                   held.hold_applied ? 1U : 0U,
                   model.model_timestamp_ns != 0 && now_ns >= model.model_timestamp_ns
                       ? static_cast<unsigned long long>(
                             (now_ns - model.model_timestamp_ns) / 1000000ULL)
                       : 0ULL,
                   panda.ready ? 1U : 0U,
                   panda.controls_allowed ? 1U : 0U,
                   panda_state.connected, panda_state.comms_healthy,
                   panda_state.tx_enabled, panda_state.controls_allowed,
                   panda_state.safety_mode, panda_state.safety_param,
                   panda_state.heartbeat_lost, panda.state_fresh ? 1U : 0U);
    }
    previous_engaged = result.engaged;
    previous_active = result.active;
    have_previous = true;
  }
};

/* 모델 lead 출력을 알림/크루즈 입력으로 환산한다. signal_valid는 값이 유효한지,
 * valid는 확률 문턱까지 넘었는지. 거리는 레이더 기준점으로 옮긴다. */
struct VisionLead {
  bool model_fresh = false;
  bool signal_valid = false;
  bool valid = false;
  float distance_m = 0.0f;
  float relative_speed_mps = 0.0f;
};

VisionLead observe_vision_lead(const K230ModelState &model, uint64_t now_ns,
                               float ego_speed_mps) {
  VisionLead lead;
  lead.model_fresh = model.valid != 0 &&
                     timestamp_fresh_ns(model.model_timestamp_ns, now_ns, kAlertModelTimeoutNs);
  lead.signal_valid =
      lead.model_fresh && model.lead.valid != 0 &&
      std::isfinite(model.lead.x) && std::isfinite(model.lead.velocity);
  lead.valid = lead.signal_valid && model.lead.probability >= kLeadProbabilityThreshold;
  lead.distance_m = lead.signal_valid ? model.lead.x - kRadarToCameraDistanceM : 0.0f;
  lead.relative_speed_mps = lead.signal_valid ? model.lead.velocity - ego_speed_mps : 0.0f;
  return lead;
}

DepartureAlertInput make_alert_input(double now_s, const VehicleCanState &vehicle,
                                     const LateralControlResult &result,
                                     const K230ModelState &model, bool model_updated,
                                     const VisionLead &lead, float ego_speed_mps) {
  DepartureAlertInput input;
  input.now_s = now_s;
  input.vehicle_valid = result.vehicle_fresh;
  input.gear = vehicle.gear;
  input.speed_mps = ego_speed_mps;
  input.gas_pressed = vehicle.gas_pressed;
  input.lead_updated = model_updated;
  input.lead_valid = lead.valid;
  input.lead_distance_m = lead.valid ? lead.distance_m : 0.0f;
  input.lead_relative_speed_mps = lead.valid ? lead.relative_speed_mps : 0.0f;
  input.model_updated = model_updated;
  input.model_valid = lead.model_fresh;
  input.plan_distance_m = lead.model_fresh ? model.plan[kTrajectorySize - 1].x : 0.0f;
  return input;
}

AdaptiveCruiseInput make_adaptive_input(double now_s, bool enabled,
                                        const VehicleCanState &vehicle,
                                        const LateralControlResult &result,
                                        const PandaGateOutput &panda,
                                        const K230ModelState &model, bool model_updated,
                                        const VisionLead &lead, float ego_speed_kph) {
  AdaptiveCruiseInput input;
  input.now_s = now_s;
  input.enabled = enabled;
  input.controls_ready = result.active && panda.ready && panda.controls_allowed &&
                         vehicle.has_clu11_seed;
  input.cruise_active = vehicle.cruise_active;
  input.brake_pressed = vehicle.brake_pressed;
  input.gas_pressed = vehicle.gas_pressed;
  input.driver_accelerator_override = vehicle.driver_override != 0;
  input.speed_unit_mph = vehicle.speed_unit_mph;
  input.driver_button = vehicle.clu_button;
  input.driver_main_button = vehicle.clu_main_button;
  input.ego_speed_kph = ego_speed_kph;
  input.cluster_speed_kph = result.cluster_speed_kph;
  input.driver_set_speed_kph = cruise_set_speed_kph(vehicle);
  input.vision_lead_updated = model_updated;
  input.vision_lead_valid = lead.signal_valid;
  input.vision_lead_probability = model.lead.probability;
  input.vision_lead_distance_m = lead.distance_m;
  input.vision_lead_relative_speed_mps = lead.relative_speed_mps;
  return input;
}

// overlayd/recordd가 읽는 100 Hz 스냅샷. 필드 순서는 ipc_messages.h가 고정한다.
K230ControlState make_control_state(const LateralControllerConfig &config,
                                    const LateralControlResult &result,
                                    const LateralTarget &target,
                                    const VehicleCanState &vehicle,
                                    const AdaptiveCruiseOutput &adaptive_cruise,
                                    const DepartureAlertOutput &departure_alert,
                                    const EngageEvents &events, bool radar_lead_fresh,
                                    float ego_speed_kph, double now_s) {
  K230ControlState state;
  state.timestamp_ns = k230_now_ns();
  state.enabled = config.steering_params.enabled ? 1U : 0U;
  state.engaged = result.engaged ? 1U : 0U;
  state.active = result.active ? 1U : 0U;
  state.should_send = result.should_send ? 1U : 0U;
  state.path_usable = result.path_usable ? 1U : 0U;
  state.hud_flags =
      (target.laneless_mode ? kK230HudFlagLaneless : 0U) |
      (result.vehicle_fresh && vehicle.brake_hold ? kK230HudFlagBrakeHold : 0U);
  state.seeds_ready = result.seeds_ready ? 1U : 0U;
  state.vehicle_fresh = result.vehicle_fresh ? 1U : 0U;
  state.steering_fault = vehicle.steering_fault ? 1U : 0U;
  state.left_blinker = vehicle.left_blinker ? 1U : 0U;
  state.right_blinker = vehicle.right_blinker ? 1U : 0U;
  state.cruise_active = vehicle.cruise_active ? 1U : 0U;
  state.gear = vehicle.gear;
  state.cluster_speed_kph = result.cluster_speed_kph;
  const float driver_set_speed_kph = cruise_set_speed_kph(vehicle);
  state.cruise_max_speed_kph = adaptive_cruise.session_valid
      ? adaptive_cruise.maximum_speed_kph : driver_set_speed_kph;
  state.cruise_command_speed_kph = adaptive_cruise.session_valid
      ? adaptive_cruise.commanded_speed_kph : driver_set_speed_kph;
  state.steering_angle_deg = vehicle.steering_angle_deg;
  state.desired_curvature = result.desired_curvature;
  state.actual_curvature = result.actual_curvature;
  state.normalized_output = result.normalized_output;
  state.desired_torque = result.desired_torque;
  state.apply_torque = result.apply_torque;
  state.driver_torque = vehicle.driver_torque;
  state.desire = static_cast<uint32_t>(target.desire);
  std::snprintf(state.active_block, sizeof(state.active_block), "%s",
                result.active_block.c_str());
  state.radar_lead_valid = radar_lead_fresh && vehicle.radar_lead_valid ? 1U : 0U;
  state.radar_lead_distance_m = vehicle.radar_lead_distance_m;
  state.radar_lead_relative_speed_mps = vehicle.radar_lead_relative_speed_mps;
  state.departure_alert_type = static_cast<uint32_t>(departure_alert.type);
  state.departure_alert_event_id = departure_alert.event_id;
  state.green_light_alert_armed = departure_alert.green_light_armed ? 1U : 0U;
  state.tpms_valid = tpms_state_fresh(vehicle, now_s) ? 1U : 0U;
  state.tpms_unit = static_cast<uint32_t>(vehicle.tpms_unit);
  state.tpms_pressure_fl = vehicle.tpms_pressure_fl;
  state.tpms_pressure_fr = vehicle.tpms_pressure_fr;
  state.tpms_pressure_rl = vehicle.tpms_pressure_rl;
  state.tpms_pressure_rr = vehicle.tpms_pressure_rr;
  state.tpms_warning = vehicle.tpms_warning ? 1U : 0U;
  state.engage_event_id = events.engage_id;
  state.disengage_event_id = events.disengage_id;
  state.engage_reject_event_id = events.reject_id;
  std::memcpy(state.engage_reject_block, events.reject_block,
              sizeof(state.engage_reject_block));
  state.ego_speed_kph = ego_speed_kph;
  return state;
}

/* 1초 창의 루프 통계. 창이 끝나면 한 줄로 찍고 비운다. */
struct TickStats {
  unsigned can_frames = 0;
  unsigned generated_frames = 0;
  unsigned publish_errors = 0;
  unsigned send_queue_full = 0;
  unsigned stale_can_batches = 0;
  unsigned ticks = 0;
  unsigned misses = 0;
  double work_sum_us = 0.0;
  double work_max_us = 0.0;

  void account(double work_us, bool missed) {
    work_sum_us += work_us;
    work_max_us = std::max(work_max_us, work_us);
    ++ticks;
    if (missed) ++misses;
  }

  void log(double window_s, unsigned long long tx_depth, unsigned long long rx_depth,
           unsigned param_generation, const LateralControlResult &result,
           const PandaGateOutput &panda, const LateralTarget &target,
           const VehicleCanState &vehicle, float road_bank_lat_accel,
           const AdaptiveCruiseOutput &adaptive_cruise,
           const DepartureAlertInput &alert_input) {
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
                 send_queue_full, stale_can_batches, tx_depth, rx_depth,
                 param_generation,
                 result.engaged ? 1 : 0, result.active ? 1 : 0,
                 panda.ready ? 1 : 0, panda.controls_allowed ? 1 : 0,
                 target.valid ? 1 : 0, target.mpc_solution_valid ? 1 : 0, target.desire,
                 result.desired_torque, result.apply_torque,
                 vehicle.driver_torque, vehicle.steering_angle_deg,
                 result.desired_curvature, result.actual_curvature,
                 result.actual_curvature_vm, result.actual_curvature_yaw,
                 vehicle.lat_accel_mps2, road_bank_lat_accel, vehicle.long_accel_mps2,
                 result.curvature_error, target.target_y_m,
                 0.5 * (target.lane_left_y_m + target.lane_right_y_m),
                 target.lane_width_m, target.lane_left_y_m, target.lane_right_y_m,
                 target.lane_left_prob, target.lane_right_prob, target.lane_d_prob,
                 target.lane_left_std, target.lane_right_std,
                 result.cluster_speed_kph, result.control_speed_kph,
                 adaptive_cruise.active ? 1U : 0U,
                 adaptive_cruise.maximum_speed_kph, adaptive_cruise.commanded_speed_kph,
                 adaptive_cruise.target_speed_kph,
                 adaptive_cruise.lead_valid ? 1U : 0U,
                 alert_input.lead_distance_m, alert_input.lead_relative_speed_mps,
                 adaptive_cruise.command_button,
                 vehicle.gas, vehicle.driver_override,
                 result.active_block.c_str());
    *this = TickStats{};
  }
};

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

  LateralPlanner planner_;
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
    config.force_engaged = env_flag("K230_FORCE_ENGAGED", false);
    AdaptiveCruiseConfig adaptive_cruise_config;
    const std::string steering_path = k230_param_path("steering.json");
    const std::string driving_path = k230_param_path("driving.json");
    const std::string adaptive_cruise_path = k230_param_path("adaptive_cruise.json");
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
                 adaptive_cruise_config.enabled
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
    PandaHealthGate panda_gate;
    PathHoldGate path_gate;
    EngageEvents events;
    TickStats stats;
    VehicleCanState vehicle;
    K230ModelState model;
    K230PandaState panda_state;
    LateralTarget lateral_target;
    AdaptiveCruiseOutput adaptive_cruise;
    LateralControlResult last_result;
    uint64_t model_seq = 0;
    uint64_t panda_state_seq = 0;
    int control_frame = 0;
    uint32_t last_logged_alert_event_id = 0;

    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    auto next_tick = start;
    auto log_start = start;
    RuntimeParams runtime_params(steering_path, driving_path, adaptive_cruise_path, start);

    while (!g_stop) {
      next_tick += std::chrono::milliseconds(10);
      const auto work_start = Clock::now();
      const double now_s = std::chrono::duration<double>(work_start - start).count();
      const uint64_t can_now_ns = k230_now_ns();

      const bool reload_requested = g_reload_params != 0;
      if (reload_requested) g_reload_params = 0;
      if (runtime_params.poll(work_start, reload_requested, &config,
                              &adaptive_cruise_config)) {
        controller.update_params(config.steering_params, config.driving_params);
        lateral_planner.update_params(config.steering_params, config.driving_params);
        adaptive_cruise_controller.update_config(adaptive_cruise_config);
      }

      K230CanBatch can_batch;
      while (can_sub.pop(&can_batch)) {
        if (!k230_can_batch_is_fresh(can_batch, can_now_ns, kMaxCanRxAgeNs)) {
          ++stats.stale_can_batches;
          continue;
        }
        apply_can_batch(can_batch, now_s, &vehicle);
        stats.can_frames += std::min<uint32_t>(can_batch.count, kK230CanBatchMaxFrames);
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
      const PandaGateOutput panda =
          panda_gate.update(panda_state, now_ns, config.force_engaged);
      const uint64_t model_timeout_ns =
          static_cast<unsigned long long>(config.driving_params.model_timeout_ms) *
          1000000ULL;
      const PathHoldOutput held = path_gate.update(model, now_ns, model_timeout_ns);
      const int frame = control_frame++;
      last_result = controller.update(held.path, lateral_target, vehicle, now_s,
                                      frame, panda.ready, panda.controls_allowed);
      events.update(last_result, vehicle, panda, panda_state, held, model, now_ns);

      const bool radar_lead_fresh = signal_time_fresh(vehicle.scc11_time_s, now_s, 0.5);
      const float ego_speed_kph = vehicle_speed_kph(vehicle, now_s);
      const float ego_speed_mps = ego_speed_kph / 3.6f;
      const VisionLead lead = observe_vision_lead(model, now_ns, ego_speed_mps);
      const DepartureAlertInput alert_input = make_alert_input(
          now_s, vehicle, last_result, model, model_updated, lead, ego_speed_mps);
      adaptive_cruise = adaptive_cruise_controller.update(make_adaptive_input(
          now_s, adaptive_cruise_config.enabled, vehicle, last_result, panda, model,
          model_updated, lead, ego_speed_kph));

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

      const K230ControlState control_state = make_control_state(
          config, last_result, lateral_target, vehicle, adaptive_cruise, departure_alert,
          events, radar_lead_fresh, ego_speed_kph, now_s);
      if (!control_state_pub.publish(&control_state, sizeof(control_state))) {
        ++stats.publish_errors;
      }

      if (last_result.should_send && !last_result.frames.empty()) {
        const K230CanBatch send_batch = make_send_batch(last_result.frames);
        if (!sendcan_pub.push(send_batch)) {
          ++stats.publish_errors;
          ++stats.send_queue_full;
        } else {
          stats.generated_frames += static_cast<unsigned>(last_result.frames.size());
        }
      }

      const auto work_end = Clock::now();
      stats.account(
          std::chrono::duration<double, std::micro>(work_end - work_start).count(),
          work_end > next_tick);
      if (work_end - log_start >= std::chrono::seconds(1)) {
        stats.log(std::chrono::duration<double>(work_end - log_start).count(),
                  static_cast<unsigned long long>(sendcan_pub.depth()),
                  static_cast<unsigned long long>(can_sub.depth()),
                  runtime_params.generation(), last_result, panda, lateral_target,
                  vehicle, controller.road_bank_lat_accel(), adaptive_cruise,
                  alert_input);
        log_start = work_end;
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
