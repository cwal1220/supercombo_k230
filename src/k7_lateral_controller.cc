#include "k7_lateral_controller.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kButtonSetDecel = 2;
constexpr int kButtonCancel = 4;
constexpr int kGearDrive = 5;
constexpr float kLaneChangeSpeedMinMps = 30.0f * 0.2777777778f;
constexpr int kManualSteerDisableFrames = 50;
constexpr int kDriverSteeringTorqueAbove = 170;
constexpr float kSmoothSteerRecoverStep = 0.005f;

float clamp_float(float value, float lo, float hi) {
  if (!std::isfinite(value)) return lo;
  return std::min(std::max(value, lo), hi);
}

bool is_hard_disengage_block(const std::string &block) {
  return block == "brake_pressed" ||
         block == "gear_not_drive" ||
         block == "mdps_fault" ||
         block == "controller_disabled" ||
         block == "door_open" ||
         block == "seatbelt_unlatched" ||
         block == "esp_disabled" ||
         block == "park_brake" ||
         block == "brake_error";
}

float cluster_speed_kph(const K7VehicleCanState &vehicle_state) {
  if (!std::isfinite(vehicle_state.cluster_speed) || vehicle_state.cluster_speed < 0.0f) {
    return 0.0f;
  }
  return vehicle_state.cluster_speed * (vehicle_state.speed_unit_mph ? 1.609344f : 1.0f);
}

bool signal_fresh(double timestamp_s, double now_s, double timeout_s = 0.5) {
  return timestamp_s >= 0.0 && now_s >= timestamp_s && now_s - timestamp_s <= timeout_s;
}

}  // namespace

K7LateralController::K7LateralController(K7LateralControllerConfig config)
    : config_(config) {
  config_.can_config.main_bus = kK7PowertrainBus;
  config_.can_config.mdps_bus = kK7MdpsBus;
  config_.can_config.scc_bus = kK7PowertrainBus;
  config_.can_config.send_lkas_on_scc_bus = false;
  config_.can_config.send_lkas_on_mdps_bus = true;
  config_.can_config.send_clu11_speed_to_mdps = true;
}

// 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
K7LateralControlResult K7LateralController::update(const LateralPath &path,
                                                   const K7VehicleCanState &vehicle_state,
                                                   double now_s,
                                                   int frame) {
  update_button_state(vehicle_state.clu_button, now_s);
  const bool logical_engaged = config_.force_engaged || engaged_;

  K7LateralControlResult result;
  result.engaged = logical_engaged;
  result.path_usable = path.usable_for_steering;
  result.left_lane = path.left_valid;
  result.right_lane = path.right_valid;
  result.seeds_ready = k7_seed_frames_ready(vehicle_state);
  result.vehicle_fresh = k7_vehicle_state_fresh(vehicle_state, now_s);
  result.speed_kph = cluster_speed_kph(vehicle_state);
  const float speed_mps = result.speed_kph / 3.6f;
  if (logical_engaged) {
    update_manual_blinker_timers(vehicle_state, speed_mps);
  }

  std::string curvature_block;
  result.desired_curvature = sanitized_desired_curvature(path, speed_mps, &curvature_block);
  result.active_block = active_block_reason(path, vehicle_state, now_s, result.seeds_ready,
                                            result.vehicle_fresh, result.speed_kph);
  if (result.active_block.empty() && !curvature_block.empty()) {
    result.active_block = curvature_block;
  }
  if (result.active_block.empty()) {
    const float actual_curvature = torque_controller_.estimate_actual_curvature(
        speed_mps, vehicle_state.steering_angle_deg, config_.steering_params,
        vehicle_state.yaw_rate_rad_s, vehicle_state.yaw_rate_valid);
    result.active_block = curvature_consistency_block(
        result.desired_curvature, actual_curvature, result.speed_kph);
  }
  if (logical_engaged && is_hard_disengage_block(result.active_block)) {
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
    result.engaged = config_.force_engaged;
  }
  result.active = result.active_block.empty();
  result.cut_steer_temp = update_cut_steer_state(result.active, vehicle_state);

  const bool steering_pressed =
      std::abs(vehicle_state.driver_torque) > config_.steering_params.steer_driver_allowance;
  const bool driver_guard_active =
      driver_steering_torque_above_timer_ >= 0 && driver_steering_torque_above_timer_ < 100;
  const EffectiveSteerLimits effective_limits = config_.steering_params.effective_steer_limits(
      result.speed_kph, vehicle_state.steering_angle_deg, speed_mps, driver_guard_active);
  K7SteeringParams control_params = config_.steering_params;
  control_params.steer_max = effective_limits.steer_max;
  control_params.steer_delta_up = effective_limits.steer_delta_up;
  control_params.steer_delta_down = effective_limits.steer_delta_down;
  const bool yaw_rate_valid = signal_fresh(vehicle_state.esp12_time_s, now_s) &&
                              vehicle_state.yaw_rate_valid;
  if (result.active) {
    const int raw_torque = torque_controller_.update(
        true, speed_mps, result.desired_curvature, vehicle_state.steering_angle_deg,
        steering_pressed, steer_rate_limited_, control_params,
        vehicle_state.yaw_rate_rad_s, yaw_rate_valid);
    if (config_.steering_params.smooth_steer_method == 1) {
      result.desired_torque = smooth_steer_torque(raw_torque, vehicle_state, steering_pressed);
    } else {
      result.desired_torque = static_cast<int>(
          std::lround(static_cast<float>(raw_torque) * driver_torque_scale()));
    }
    result.actual_curvature = torque_controller_.actual_curvature();
    result.curvature_error = result.desired_curvature - result.actual_curvature;
    result.normalized_output = torque_controller_.normalized_output();
    result.feedforward = torque_controller_.feedforward();
    result.saturated = torque_controller_.saturated();
    result.apply_torque = apply_hyundai_steer_torque_limits(
        result.desired_torque, last_torque_, vehicle_state.driver_torque,
        control_params.hyundai_limits(effective_limits));
  } else {
    torque_controller_.update(false, speed_mps, 0.0f, vehicle_state.steering_angle_deg,
                              false, steer_rate_limited_, control_params,
                              vehicle_state.yaw_rate_rad_s, yaw_rate_valid);
    result.actual_curvature = torque_controller_.actual_curvature();
    result.curvature_error = result.desired_curvature - result.actual_curvature;
    result.normalized_output = torque_controller_.normalized_output();
    result.feedforward = torque_controller_.feedforward();
    result.saturated = torque_controller_.saturated();
    result.desired_torque = 0;
    result.apply_torque = 0;
  }

  result.should_send =
      result.seeds_ready &&
      (result.active ||
       (config_.zero_release_when_inactive &&
        (result.engaged || now_s - last_disengage_s_ < 0.25)));
  if (result.should_send) {
    result.frames = build_frames(vehicle_state, result, frame);
  } else {
    lkas11_counter_valid_ = false;
  }

  last_torque_ = result.apply_torque;
  steer_rate_limited_ = result.desired_torque != result.apply_torque;
  if (!result.active) {
    last_torque_ = 0;
    steer_rate_limited_ = false;
  }
  update_driver_steering_guard(vehicle_state, speed_mps);
  decay_manual_blinker_timers();
  return result;
}

// 현재 engage 상태를 반환한다.
bool K7LateralController::engaged() const {
  return engaged_;
}

// 제어 상태를 초기값으로 되돌린다.
void K7LateralController::reset() {
  engaged_ = false;
  last_button_ = 0;
  last_disengage_s_ = -1000.0;
  reset_control_state();
  lkas11_counter_valid_ = false;
  lkas11_counter_ = 0;
}

// CLU 버튼 edge로 engage/disengage 상태를 갱신한다.
void K7LateralController::update_button_state(int button, double now_s) {
  if (button == last_button_) return;
  if (button == kButtonCancel) {
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
  } else if (button == kButtonSetDecel) {
    engaged_ = true;
  }
  last_button_ = button;
}

// 방향지시등 기반 수동 조향 차단 타이머를 갱신한다.
void K7LateralController::update_manual_blinker_timers(
    const K7VehicleCanState &vehicle_state, float speed_mps) {
  const bool one_side_blinker = vehicle_state.left_blinker != vehicle_state.right_blinker;
  if (one_side_blinker &&
      speed_mps < kLaneChangeSpeedMinMps &&
      config_.steering_params.turn_steering_disable) {
    lanechange_manual_timer_ = kManualSteerDisableFrames;
  }
  if (vehicle_state.hazard ||
      (vehicle_state.left_blinker && vehicle_state.right_blinker)) {
    emergency_manual_timer_ = kManualSteerDisableFrames;
  }
}

// 수동 조향 차단 타이머를 한 프레임 감소시킨다.
void K7LateralController::decay_manual_blinker_timers() {
  if (lanechange_manual_timer_ > 0) --lanechange_manual_timer_;
  if (emergency_manual_timer_ > 0) --emergency_manual_timer_;
}

// 현재 수동 조향 차단 사유를 반환한다.
std::string K7LateralController::manual_blinker_block_reason() const {
  if (lanechange_manual_timer_ > 0) return "lanechange_manual";
  return "";
}

// openpilot K7 조향각 제한값을 현재 속도에 맞게 계산한다.
float K7LateralController::steering_angle_limit_deg(float speed_kph) const {
  const float limit = config_.steering_params.max_steering_angle_deg;
  if (limit < 90.0f) return 0.0f;
  if (std::fabs(limit - 90.0f) < 1e-6f) return 90.0f;
  const float speed = clamp_float(speed_kph, 0.0f, 20.0f);
  return (limit + 60.0f) + (speed / 20.0f) * (limit - (limit + 60.0f));
}

// 조향각 제한으로 LKAS active를 막아야 하는지 확인한다.
std::string K7LateralController::steering_angle_block(
    const K7VehicleCanState &vehicle_state, float speed_kph) const {
  if (config_.steering_params.avoid_lkas_fault_enabled) return "";
  const float limit = steering_angle_limit_deg(speed_kph);
  if (limit > 0.0f && std::fabs(vehicle_state.steering_angle_deg) >= limit) {
    return "steering_angle_limit";
  }
  return "";
}

// LKAS fault 회피를 위한 임시 cut-steer 상태를 갱신한다.
bool K7LateralController::update_cut_steer_state(
    bool active, const K7VehicleCanState &vehicle_state) {
  const K7SteeringParams &params = config_.steering_params;
  if (params.avoid_lkas_fault_enabled) {
    if (active && std::fabs(vehicle_state.steering_angle_deg) >
                      params.avoid_lkas_fault_max_angle_deg) {
      ++angle_limit_counter_;
    } else {
      angle_limit_counter_ = 0;
    }

    if (angle_limit_counter_ > params.avoid_lkas_fault_max_frames) {
      cut_steer_ = true;
    } else if (cut_steer_frames_ > 1) {
      cut_steer_frames_ = 0;
      cut_steer_ = false;
    }
  } else {
    angle_limit_counter_ = 0;
    if (vehicle_state.mdps_error_count > params.avoid_lkas_fault_max_frames) {
      cut_steer_ = true;
    } else if (cut_steer_frames_ > 1) {
      cut_steer_frames_ = 0;
      cut_steer_ = false;
    }
  }

  if (!cut_steer_) return false;
  angle_limit_counter_ = 0;
  ++cut_steer_frames_;
  return true;
}

// 운전자 조향 토크 감지 타이머를 openpilot K7 방식으로 갱신한다.
void K7LateralController::update_driver_steering_guard(
    const K7VehicleCanState &vehicle_state, float speed_mps) {
  const bool driver_steering_torque_above =
      std::abs(vehicle_state.driver_torque) > kDriverSteeringTorqueAbove &&
      speed_mps < kLaneChangeSpeedMinMps;
  if (driver_steering_torque_above) {
    driver_steering_torque_above_timer_ =
        std::max(0, driver_steering_torque_above_timer_ - 1);
  } else {
    driver_steering_torque_above_timer_ =
        std::min(100, driver_steering_torque_above_timer_ + 5);
  }
}

// 운전자 조향 중 요청 토크 fade 비율을 반환한다.
float K7LateralController::driver_torque_scale() const {
  if (driver_steering_torque_above_timer_ >= 0 &&
      driver_steering_torque_above_timer_ < 100) {
    return clamp_float(static_cast<float>(driver_steering_torque_above_timer_) / 100.0f,
                       0.0f, 1.0f);
  }
  return 1.0f;
}

// smooth steer 모드에서 요청 토크를 서서히 줄이거나 회복한다.
int K7LateralController::smooth_steer_torque(
    int raw_torque, const K7VehicleCanState &vehicle_state, bool steering_pressed) {
  const K7SteeringParams &params = config_.steering_params;
  if (params.smooth_max_steering_angle_deg > 0.0f &&
      std::fabs(vehicle_state.steering_angle_deg) > params.smooth_max_steering_angle_deg) {
    if (params.smooth_max_driver_angle_wait > 0.0f && steering_pressed) {
      steer_timer_apply_torque_ -= params.smooth_max_driver_angle_wait;
    } else if (params.smooth_max_steer_angle_wait > 0.0f) {
      steer_timer_apply_torque_ -= params.smooth_max_steer_angle_wait;
    }
  } else if (params.smooth_driver_angle_wait > 0.0f && steering_pressed) {
    steer_timer_apply_torque_ -= params.smooth_driver_angle_wait;
  } else {
    if (steer_timer_apply_torque_ >= 1.0f) return raw_torque;
    steer_timer_apply_torque_ += kSmoothSteerRecoverStep;
  }

  steer_timer_apply_torque_ = clamp_float(steer_timer_apply_torque_, 0.0f, 1.0f);
  return static_cast<int>(
      std::lround(static_cast<float>(raw_torque) * steer_timer_apply_torque_));
}

// 제어 내부 상태를 초기값으로 되돌린다.
void K7LateralController::reset_control_state() {
  last_torque_ = 0;
  steer_rate_limited_ = false;
  angle_limit_counter_ = 0;
  cut_steer_frames_ = 0;
  cut_steer_ = false;
  lanechange_manual_timer_ = 0;
  emergency_manual_timer_ = 0;
  driver_steering_torque_above_timer_ = 100;
  steer_timer_apply_torque_ = 1.0f;
  torque_controller_.reset();
}

// active를 막는 현재 gate reason을 계산한다.
std::string K7LateralController::active_block_reason(
    const LateralPath &path,
    const K7VehicleCanState &vehicle_state,
    double now_s,
    bool seeds_ready,
    bool vehicle_fresh,
    float speed_kph) const {
  if (!config_.force_engaged && !engaged_) return "not_engaged";
  if (!config_.enabled || !config_.steering_params.enabled) return "controller_disabled";
  if (!path.usable_for_steering) return "path_invalid";
  if (!seeds_ready) return "seeds_missing";
  if (!vehicle_fresh) return "vehicle_state_stale";
  if (vehicle_state.door_open) return "door_open";
  if (vehicle_state.seatbelt_unlatched) return "seatbelt_unlatched";
  if (vehicle_state.esp_disabled) return "esp_disabled";
  if (vehicle_state.park_brake) return "park_brake";
  if (vehicle_state.brake_error) return "brake_error";
  if (!config_.steering_params.torque_use_angle) {
    if (!signal_fresh(vehicle_state.esp12_time_s, now_s)) return "esp_stale";
    if (!vehicle_state.yaw_rate_valid) return "yaw_rate_invalid";
  }
  if (vehicle_state.gear != kGearDrive) return "gear_not_drive";
  if (config_.steering_params.no_smart_mdps &&
      speed_kph / 3.6f < config_.steering_params.min_steer_speed_mps) {
    return "no_smart_mdps_low_speed";
  }
  const std::string angle_block = steering_angle_block(vehicle_state, speed_kph);
  if (!angle_block.empty()) return angle_block;
  if (vehicle_state.brake_pressed || vehicle_state.brake_light) return "brake_pressed";
  if (vehicle_state.steering_fault) return "mdps_fault";
  const std::string manual_block = manual_blinker_block_reason();
  if (!manual_block.empty()) return manual_block;
  if (!std::isfinite(speed_kph)) return "speed_invalid";
  return "";
}

// path curvature를 안전한 desired curvature로 제한한다.
float K7LateralController::sanitized_desired_curvature(const LateralPath &path,
                                                       float speed_mps,
                                                       std::string *block) const {
  if (block) block->clear();
  float lateral_20m = 0.0f;
  if (!path_lateral_at(path, 20.0f, &lateral_20m) ||
      !std::isfinite(lateral_20m) ||
      std::fabs(lateral_20m) > config_.max_abs_steering_lateral_m) {
    if (block) *block = "path_lateral_outlier";
    return 0.0f;
  }
  const float curvature = steering_curvature(path, 20.0f);
  if (!std::isfinite(curvature) ||
      std::fabs(curvature) > config_.max_command_curvature) {
    if (block) *block = "curvature_invalid";
    return 0.0f;
  }
  const float speed_for_limit = std::max(std::fabs(speed_mps), 1.0f);
  const float max_lat_accel =
      std::max(1.0f, config_.steering_params.torque_max_lat_accel() * 1.2f);
  const float max_curvature = max_lat_accel / (speed_for_limit * speed_for_limit);
  const float signed_curvature =
      config_.steering_params.invert_steer ? -curvature : curvature;
  return clamp_float(signed_curvature, -max_curvature, max_curvature);
}

// 현재 차량 curvature와 path curvature가 크게 어긋나는지 검사한다.
std::string K7LateralController::curvature_consistency_block(float desired_curvature,
                                                             float actual_curvature,
                                                             float speed_kph) const {
  if (!std::isfinite(speed_kph) ||
      speed_kph < config_.min_curvature_consistency_speed_kph) {
    return "";
  }
  if (!std::isfinite(desired_curvature) || !std::isfinite(actual_curvature)) {
    return "curvature_invalid";
  }
  const float max_error = std::max(0.0f, config_.max_curvature_consistency_error);
  if (max_error > 0.0f && std::fabs(desired_curvature - actual_curvature) > max_error) {
    return "path_curvature_mismatch";
  }
  return "";
}

// LKAS HUD state 값을 lane availability와 active 상태에서 만든다.
int K7LateralController::lkas_sys_state(bool active, bool left_lane, bool right_lane) const {
  if (left_lane && right_lane) return active ? 3 : 4;
  if (left_lane) return 5;
  if (right_lane) return 6;
  return 1;
}

// 최종 송신 frame 묶음을 만든다.
std::vector<CanFrame> K7LateralController::build_frames(
    const K7VehicleCanState &vehicle_state,
    const K7LateralControlResult &result,
    int frame) {
  HyundaiLkasCommand command;
  command.apply_steer = result.apply_torque;
  command.steer_req = result.active;
  command.cut_steer_temp = result.cut_steer_temp;
  command.sys_state = lkas_sys_state(result.active, result.left_lane, result.right_lane);
  command.sys_warning = false;
  command.left_lane = result.left_lane;
  command.right_lane = result.right_lane;
  command.lkas_msg_count = next_lkas11_counter(vehicle_state);
  command.ldws_fix = config_.steering_params.ldws_car_fix;

  const HyundaiLkas11Values lkas_seed = decode_lkas11(vehicle_state.lkas11_seed);
  const HyundaiClu11Values clu_seed = decode_clu11(vehicle_state.clu11_seed);
  std::vector<CanFrame> frames = build_k7_hev_lateral_can_frames(
      lkas_seed, clu_seed, command, config_.can_config, result.active,
      clu_seed.speed, vehicle_state.speed_unit_mph, frame);
  if (vehicle_state.has_mdps12_seed &&
      config_.can_config.mdps_bus != config_.can_config.main_bus) {
    frames.push_back(create_mdps12_frame(vehicle_state.mdps12_seed, frame));
  }
  return frames;
}

int K7LateralController::next_lkas11_counter(const K7VehicleCanState &vehicle_state) {
  if (!lkas11_counter_valid_) {
    const HyundaiLkas11Values seed = decode_lkas11(vehicle_state.lkas11_seed);
    lkas11_counter_ = (seed.msg_count + 1) & 0xf;
    lkas11_counter_valid_ = true;
  }
  const int counter = lkas11_counter_ & 0xf;
  lkas11_counter_ = (lkas11_counter_ + 1) & 0xf;
  return counter;
}
