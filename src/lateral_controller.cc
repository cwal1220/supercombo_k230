#include "lateral_controller.h"

#include "common_utils.h"
#include "model_output.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kButtonSetDecel = 2;
constexpr int kButtonCancel = 4;
constexpr int kGearDrive = 5;
constexpr int kSteeringPressedMinCount = 5;
constexpr float kSmoothSteerRecoverStep = 0.005f;
/* plan 곡률에서 벗어날 수 있는 lateral jerk 허용 창(초). openpilot DT_MDL과
 * 같은 값이다. */
constexpr float kCurvatureDeviationWindowS = 0.05f;
/* lag 보상에 더하는 plan 나이의 상한. 이 이상 낡은 plan은 staleness gate가
 * 별도로 차단한다. */
constexpr float kMaxPlanAgeCompS = 0.25f;
constexpr float kMaxCurvature = 0.3f;
// EU 안전 한계(openpilot MAX_LATERAL_JERK/ACCEL). accel 3.3은 K7 실측 기준.
constexpr float kMaxLateralJerk = 5.0f;
constexpr float kMaxLateralAccel = 3.3f;
constexpr float kGravity = 9.8f;
constexpr double kPandaEngageGraceS = 1.0;

bool is_hard_disengage_block(const std::string &block) {
  return block == "gear_not_drive" ||
         block == "mdps_fault" ||
         block == "controller_disabled" ||
         block == "door_open" ||
         block == "seatbelt_unlatched" ||
         block == "esp_disabled" ||
         block == "park_brake" ||
         block == "brake_error";
}

// Panda의 controls_allowed는 비동기적으로 보고된다(브리지가 100 Hz
// 컨트롤러보다 낮은 주기로 health를 폴링한다). 따라서 대응하는 Panda 허가보다
// SET 해제가 한두 틱 먼저 도착할 수 있다. 이 handshake가 완료될 때까지 앱의
// engage를 유지하며, 이는 가용성 gate이지 요청 실패가 아니다. 차량/컨트롤러의
// 정적 gate는 여전히 SET을 거부한다.
bool is_transient_engage_block(const std::string &block) {
  return block == "panda_not_ready" || block == "panda_controls_off";
}

float cluster_speed_kph(const VehicleCanState &vehicle_state) {
  if (!std::isfinite(vehicle_state.cluster_speed_raw) || vehicle_state.cluster_speed_raw < 0.0f) {
    return 0.0f;
  }
  return vehicle_state.cluster_speed_raw * (vehicle_state.speed_unit_mph ? 1.609344f : 1.0f);
}

float interp_lateral(float x, const float *values) {
  if (x <= 0.0f) return values[0];
  for (int i = 1; i < kLateralControlN; ++i) {
    const float high_x = model_t_idx(i);
    if (x <= high_x) {
      const float low_x = model_t_idx(i - 1);
      const float p = (x - low_x) / (high_x - low_x);
      return values[i - 1] + p * (values[i] - values[i - 1]);
    }
  }
  return values[kLateralControlN - 1];
}

}  // namespace

LateralController::LateralController(LateralControllerConfig config)
    : config_(config) {
  config_.can_config.main_bus = kPowertrainBus;
  config_.can_config.mdps_bus = kMdpsBus;
  config_.can_config.scc_bus = kPowertrainBus;
  config_.can_config.send_lkas_on_scc_bus = false;
  config_.can_config.send_lkas_on_mdps_bus = true;
  config_.can_config.send_clu11_speed_to_mdps = true;
  config_.can_config.mdps_speed_spoof_kph = config_.driving_params.mdps_speed_spoof_kph;
}

// 제어 상태를 유지한 채 런타임 파라미터를 즉시 교체한다.
void LateralController::update_params(
    const SteeringParams &steering_params,
    const DrivingParams &driving_params) {
  config_.steering_params = steering_params;
  config_.driving_params = driving_params;
  config_.can_config.mdps_speed_spoof_kph = driving_params.mdps_speed_spoof_kph;
}

// 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
LateralControlResult LateralController::update(const LateralPath &path,
                                                   const LateralTarget &target,
                                                   const VehicleCanState &vehicle_state,
                                                   double now_s,
                                                   int frame,
                                                   bool panda_ready,
                                                   bool panda_controls_allowed) {
  const bool engage_requested =
      !config_.force_engaged && !engaged_ &&
      vehicle_state.clu_button == 0 && last_button_ == kButtonSetDecel;
  update_button_state(vehicle_state.clu_button, now_s);
  const bool logical_engaged = config_.force_engaged || engaged_;

  LateralControlResult result;
  result.engaged = logical_engaged;
  result.path_usable = path.usable_for_steering;
  result.left_lane = path.left_valid;
  result.right_lane = path.right_valid;
  result.seeds_ready = seed_frames_ready(vehicle_state);
  result.vehicle_fresh = vehicle_state_fresh(
      vehicle_state, now_s,
      static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) / 1000.0);
  result.cluster_speed_kph = cluster_speed_kph(vehicle_state);
  result.control_speed_kph = vehicle_speed_kph(
      vehicle_state, now_s,
      static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) / 1000.0);
  const float speed_mps = result.control_speed_kph / 3.6f;
  if (logical_engaged) {
    update_manual_blinker_timers(vehicle_state, speed_mps);
  }

  /* plan 나이: 근거 프레임 캡처 시각부터 지금까지. lag 보상과 staleness
   * gate가 함께 쓴다. 타임스탬프가 없으면(테스트, 초기값) 0으로 둔다. */
  float plan_age_s = 0.0f;
  if (target.valid && target.capture_timestamp_ns != 0) {
    const uint64_t control_now_ns = k230_now_ns();
    if (control_now_ns > target.capture_timestamp_ns) {
      plan_age_s = static_cast<float>(
          static_cast<double>(control_now_ns - target.capture_timestamp_ns) * 1e-9);
    }
  }
  result.desired_curvature =
      lag_adjusted_desired_curvature(target, speed_mps, plan_age_s);
  result.active_block = active_block_reason(path, target, vehicle_state, now_s,
                                            result.seeds_ready, result.vehicle_fresh,
                                            panda_ready, panda_controls_allowed,
                                            result.control_speed_kph, plan_age_s);
  if (logical_engaged && is_hard_disengage_block(result.active_block)) {
    panda_engage_pending_ = false;
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
    result.engaged = config_.force_engaged;
  }

  if (engage_requested) {
    if (is_transient_engage_block(result.active_block)) {
      panda_engage_pending_ = true;
      panda_engage_pending_s_ = now_s;
    } else {
      panda_engage_pending_ = false;
    }
  }

  if (engage_requested && !result.active_block.empty() &&
      !is_transient_engage_block(result.active_block)) {
    // 차량/컨트롤러의 정적 gate는 실제 engage 요청 실패로 처리한다.
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
    result.engaged = config_.force_engaged;
    result.engage_rejected = true;
  }

  if (panda_engage_pending_) {
    const bool panda_waiting = is_transient_engage_block(result.active_block);
    const bool grace_elapsed = now_s - panda_engage_pending_s_ >= kPandaEngageGraceS;
    if (result.active_block.empty()) {
      // Panda 허가가 도착했고 다른 engage gate도 모두 해소되었다.
      panda_engage_pending_ = false;
    } else if (!panda_waiting || grace_elapsed) {
      /* 정적 실패를 저장했다가 gate가 해소되면 조용히 engage하지 않는다. Panda에는
       * 짧은 비동기 health handshake 유예만 허용하며, 완료되지 않으면 실제 차단
       * 사유를 한 번 보고한다. */
      panda_engage_pending_ = false;
      engaged_ = false;
      reset_control_state();
      last_disengage_s_ = now_s;
      result.engaged = config_.force_engaged;
      result.engage_rejected = true;
    }
  }
  result.active = result.active_block.empty();
  result.cut_steer_temp = update_cut_steer_state(result.active, vehicle_state);

  const bool steering_pressed = update_steering_pressed(vehicle_state.driver_torque);
  const EffectiveSteerLimits effective_limits =
      config_.steering_params.effective_steer_limits();
  SteeringParams control_params = config_.steering_params;
  control_params.steer_max = effective_limits.steer_max;
  control_params.steer_delta_up = effective_limits.steer_delta_up;
  control_params.steer_delta_down = effective_limits.steer_delta_down;
  const bool yaw_rate_valid = signal_time_fresh(
                                  vehicle_state.esp12_time_s, now_s,
                                  static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) /
                                      1000.0) &&
                              vehicle_state.yaw_rate_valid;
  /* 편경사: 8 m/s 이상 + 신호 유효할 때만 갱신, rc 2초. 실측 검증식
   * (2026-08-27, 직선 -0.117 재현): bank = lat + yaw_rate*v. */
  constexpr float kBankAlpha = 0.01f / (2.0f + 0.01f);
  if (yaw_rate_valid && vehicle_state.lat_accel_valid &&
      std::isfinite(vehicle_state.lat_accel_mps2) && speed_mps > 8.0f) {
    const float bank = clamp_float(
        vehicle_state.lat_accel_mps2 + vehicle_state.yaw_rate_rad_s * speed_mps,
        -2.0f, 2.0f);
    if (!road_bank_init_) { road_bank_lat_accel_ = bank; road_bank_init_ = true; }
    road_bank_lat_accel_ += kBankAlpha * (bank - road_bank_lat_accel_);
    road_bank_stale_frames_ = 0;
  } else if (++road_bank_stale_frames_ > 3000) {
    // 30초 넘게 갱신이 없으면 낡은 편경사를 0으로 감쇠하고 재초기화를 허용
    road_bank_lat_accel_ += kBankAlpha * (0.0f - road_bank_lat_accel_);
    road_bank_init_ = false;
  }

  if (result.active) {
    const int raw_torque = torque_controller_.update(
        true, speed_mps, result.desired_curvature, vehicle_state.steering_angle_deg,
        steering_pressed, steer_rate_limited_, control_params,
        vehicle_state.yaw_rate_rad_s, yaw_rate_valid, road_bank_lat_accel_);
    if (config_.steering_params.smooth_steer_method == 1) {
      result.desired_torque = smooth_steer_torque(raw_torque, vehicle_state, steering_pressed);
    } else {
      result.desired_torque = static_cast<int>(
          std::lround(static_cast<float>(raw_torque) * driver_torque_scale()));
    }
    result.actual_curvature = torque_controller_.actual_curvature();
    result.actual_curvature_vm = torque_controller_.actual_curvature_vm();
    result.actual_curvature_yaw = torque_controller_.actual_curvature_yaw();
    result.curvature_error = result.desired_curvature - result.actual_curvature;
    result.normalized_output = torque_controller_.normalized_output();
    result.feedforward = torque_controller_.feedforward();
    result.apply_torque = apply_hyundai_steer_torque_limits(
        result.desired_torque, last_torque_, vehicle_state.driver_torque,
        control_params.hyundai_limits(effective_limits));
  } else {
    // 0을 넘기면 커브 중 engage 시 지연 버퍼가 0-setpoint로 P를 튀게 한다
    torque_controller_.update(false, speed_mps, result.desired_curvature,
                              vehicle_state.steering_angle_deg,
                              false, steer_rate_limited_, control_params,
                              vehicle_state.yaw_rate_rad_s, yaw_rate_valid,
                              road_bank_lat_accel_);
    result.actual_curvature = torque_controller_.actual_curvature();
    result.actual_curvature_vm = torque_controller_.actual_curvature_vm();
    result.actual_curvature_yaw = torque_controller_.actual_curvature_yaw();
    result.curvature_error = result.desired_curvature - result.actual_curvature;
    result.normalized_output = torque_controller_.normalized_output();
    result.feedforward = torque_controller_.feedforward();
    result.desired_torque = 0;
    result.apply_torque = 0;
  }

  result.should_send =
      result.seeds_ready &&
      (result.active ||
       (config_.zero_release_when_inactive &&
        (result.engaged || now_s - last_disengage_s_ <
            static_cast<double>(config_.driving_params.inactive_release_ms) / 1000.0)));
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

// CLU 버튼 edge로 engage/disengage 상태를 갱신한다.
void LateralController::update_button_state(int button, double now_s) {
  if (button == last_button_) return;
  if (button == kButtonCancel) {
    panda_engage_pending_ = false;
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
  } else if (button == 0 && last_button_ == kButtonSetDecel) {
    engaged_ = true;
  }
  last_button_ = button;
}

// 방향지시등 기반 수동 조향 차단 타이머를 갱신한다.
void LateralController::update_manual_blinker_timers(
    const VehicleCanState &vehicle_state, float speed_mps) {
  const bool one_side_blinker = vehicle_state.left_blinker != vehicle_state.right_blinker;
  const float lane_change_min_speed_mps =
      config_.driving_params.lane_change_min_speed_kph / 3.6f;
  if (one_side_blinker &&
      speed_mps < lane_change_min_speed_mps &&
      config_.steering_params.turn_steering_disable) {
    lanechange_manual_timer_ = config_.driving_params.manual_steer_disable_frames;
  }
}

// 수동 조향 차단 타이머를 한 프레임 감소시킨다.
void LateralController::decay_manual_blinker_timers() {
  if (lanechange_manual_timer_ > 0) --lanechange_manual_timer_;
}

// 현재 수동 조향 차단 사유를 반환한다.
std::string LateralController::manual_blinker_block_reason() const {
  if (lanechange_manual_timer_ > 0) return "lanechange_manual";
  return "";
}

// openpilot K7 조향각 제한값을 현재 속도에 맞게 계산한다.
float LateralController::steering_angle_limit_deg(float speed_kph) const {
  const float limit = config_.steering_params.max_steering_angle_deg;
  if (limit <= 0.0f) return 0.0f;
  if (limit <= 90.0f) return limit;
  const float speed = clamp_float(speed_kph, 0.0f, 20.0f);
  return (limit + 60.0f) + (speed / 20.0f) * (limit - (limit + 60.0f));
}

// 조향각 제한으로 LKAS active를 막아야 하는지 확인한다.
std::string LateralController::steering_angle_block(
    const VehicleCanState &vehicle_state, float speed_kph) const {
  if (config_.steering_params.avoid_lkas_fault_enabled) return "";
  const float limit = steering_angle_limit_deg(speed_kph);
  if (limit > 0.0f && std::fabs(vehicle_state.steering_angle_deg) >= limit) {
    return "steering_angle_limit";
  }
  return "";
}

// LKAS fault 회피를 위한 임시 cut-steer 상태를 갱신한다.
bool LateralController::update_cut_steer_state(
    bool active, const VehicleCanState &vehicle_state) {
  const SteeringParams &params = config_.steering_params;
  if (params.avoid_lkas_fault_enabled) {
    if (active && std::fabs(vehicle_state.steering_angle_deg) >=
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

// 노이즈가 있는 운전자 조향 토크를 openpilot 방식으로 필터링한다.
bool LateralController::update_steering_pressed(int driver_torque) {
  const bool pressed =
      std::abs(driver_torque) > config_.steering_params.steering_pressed_threshold;
  steering_pressed_counter_ += pressed ? 1 : -1;
  steering_pressed_counter_ =
      std::clamp(steering_pressed_counter_, 0, kSteeringPressedMinCount * 2 + 1);
  return steering_pressed_counter_ > kSteeringPressedMinCount;
}

// 운전자 조향 토크 감지 타이머를 openpilot K7 방식으로 갱신한다.
void LateralController::update_driver_steering_guard(
    const VehicleCanState &vehicle_state, float speed_mps) {
  const bool driver_steering_torque_above =
      std::abs(vehicle_state.driver_torque) > config_.driving_params.driver_torque_threshold &&
      speed_mps < config_.driving_params.lane_change_min_speed_kph / 3.6f;
  if (driver_steering_torque_above) {
    driver_steering_torque_above_timer_ =
        std::max(0, driver_steering_torque_above_timer_ - 1);
  } else {
    driver_steering_torque_above_timer_ =
        std::min(100, driver_steering_torque_above_timer_ + 5);
  }
}

// 운전자 조향 중 요청 토크 fade 비율을 반환한다.
float LateralController::driver_torque_scale() const {
  if (driver_steering_torque_above_timer_ >= 0 &&
      driver_steering_torque_above_timer_ < 100) {
    return clamp_float(static_cast<float>(driver_steering_torque_above_timer_) / 100.0f,
                       0.0f, 1.0f);
  }
  return 1.0f;
}

// smooth steer 모드에서 요청 토크를 서서히 줄이거나 회복한다.
int LateralController::smooth_steer_torque(
    int raw_torque, const VehicleCanState &vehicle_state, bool steering_pressed) {
  const SteeringParams &params = config_.steering_params;
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
void LateralController::reset_control_state() {
  last_torque_ = 0;
  steer_rate_limited_ = false;
  angle_limit_counter_ = 0;
  cut_steer_frames_ = 0;
  cut_steer_ = false;
  lanechange_manual_timer_ = 0;
  driver_steering_torque_above_timer_ = 100;
  steer_timer_apply_torque_ = 1.0f;
  torque_controller_.reset();
}

// active를 막는 현재 gate reason을 계산한다.
std::string LateralController::active_block_reason(
    const LateralPath &path,
    const LateralTarget &target,
    const VehicleCanState &vehicle_state,
    double now_s,
    bool seeds_ready,
    bool vehicle_fresh,
    bool panda_ready,
    bool panda_controls_allowed,
    float speed_kph,
    float plan_age_s) const {
  if (!config_.force_engaged && !engaged_) return "not_engaged";
  if (!config_.enabled || !config_.steering_params.enabled) return "controller_disabled";
  if (!panda_ready) return "panda_not_ready";
  if (!panda_controls_allowed) return "panda_controls_off";
  if (!path.usable_for_steering) return "path_invalid";
  if (!target.valid || !target.mpc_solution_valid) return "lateral_plan_invalid";
  /* 모델 경로 gate는 모델 발행 시각만 본다. 플래너 스레드가 멈춰 target이
   * 갱신되지 않는 경우까지 근거 프레임 캡처 시각으로 함께 막는다. */
  if (target.capture_timestamp_ns != 0 &&
      plan_age_s > static_cast<float>(config_.driving_params.model_timeout_ms) /
                       1000.0f) {
    return "lateral_plan_stale";
  }
  if (!seeds_ready) return "seeds_missing";
  if (!vehicle_fresh) return "vehicle_state_stale";
  if (vehicle_state.door_open) return "door_open";
  if (vehicle_state.seatbelt_unlatched) return "seatbelt_unlatched";
  if (vehicle_state.esp_disabled) return "esp_disabled";
  if (vehicle_state.park_brake) return "park_brake";
  if (vehicle_state.brake_error) return "brake_error";
  if (!config_.steering_params.torque_use_angle) {
    if (!signal_time_fresh(vehicle_state.esp12_time_s, now_s,
                      static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) /
                          1000.0)) {
      return "esp_stale";
    }
    if (!vehicle_state.yaw_rate_valid) return "yaw_rate_invalid";
  }
  if (vehicle_state.gear != kGearDrive) return "gear_not_drive";
  if (config_.steering_params.no_smart_mdps &&
      speed_kph / 3.6f < config_.steering_params.min_steer_speed_mps) {
    return "no_smart_mdps_low_speed";
  }
  const std::string angle_block = steering_angle_block(vehicle_state, speed_kph);
  if (!angle_block.empty()) return angle_block;
  if (vehicle_state.steering_fault) return "mdps_fault";
  const std::string manual_block = manual_blinker_block_reason();
  if (!manual_block.empty()) return manual_block;
  if (!std::isfinite(speed_kph)) return "speed_invalid";
  return "";
}

float LateralController::lag_adjusted_desired_curvature(
    const LateralTarget &target, float speed_mps, float plan_age_s) const {
  if (!target.valid) return 0.0f;
  /* plan은 카메라 캡처 시점 기준이므로 소비 시점까지의 실측 나이를 actuator
   * delay에 더해 보간한다. 부수 효과로 desired curvature가 20Hz 계단 대신
   * 매 tick plan 위를 따라 전진한다. */
  const float delay = std::max(0.01f, config_.steering_params.steer_actuator_delay) +
      clamp_float(plan_age_s, 0.0f, kMaxPlanAgeCompS);
  const float current_curvature = target.curvatures[0];
  const float psi = interp_lateral(delay, target.psis);
  const float speed = std::max(speed_mps, 0.1f);
  const float curvature_from_psi = psi / (speed * delay);
  float desired_curvature = current_curvature +
      2.0f * (curvature_from_psi - current_curvature);

  const float max_curvature_rate = kMaxLateralJerk /
      (speed * speed);
  desired_curvature = clamp_float(
      desired_curvature,
      current_curvature - max_curvature_rate * kCurvatureDeviationWindowS,
      current_curvature + max_curvature_rate * kCurvatureDeviationWindowS);

  const float limit_speed = std::max(speed, 1.0f);
  const float roll_compensation = config_.steering_params.roll_rad * kGravity;
  desired_curvature = clamp_float(
      desired_curvature,
      (-kMaxLateralAccel + roll_compensation) /
          (limit_speed * limit_speed),
      (kMaxLateralAccel + roll_compensation) /
          (limit_speed * limit_speed));
  desired_curvature = clamp_float(desired_curvature,
                                  -kMaxCurvature, kMaxCurvature);
  return desired_curvature;
}

// LKAS HUD state 값을 lane availability와 active 상태에서 만든다.
int LateralController::lkas_sys_state(bool active, bool left_lane, bool right_lane) const {
  if (left_lane && right_lane) return active ? 3 : 4;
  if (left_lane) return 5;
  if (right_lane) return 6;
  return 1;
}

// 최종 송신 frame 묶음을 만든다.
std::vector<CanFrame> LateralController::build_frames(
    const VehicleCanState &vehicle_state,
    const LateralControlResult &result,
    int frame) {
  HyundaiLkasCommand command;
  command.apply_steer = result.apply_torque;
  command.steer_req = result.active;
  command.cut_steer_temp = result.cut_steer_temp;
  command.sys_state = lkas_sys_state(result.active, true, true);
  command.sys_warning = false;
  command.left_lane = result.left_lane;
  command.right_lane = result.right_lane;
  command.lkas_msg_count = next_lkas11_counter(vehicle_state);
  command.ldws_fix = false;  // K7 YG는 LDWS 전용차가 아니다

  const HyundaiLkas11Values lkas_seed = decode_lkas11(vehicle_state.lkas11_seed);
  const HyundaiClu11Values clu_seed = decode_clu11(vehicle_state.clu11_seed);
  std::vector<CanFrame> frames = build_lateral_can_frames(
      lkas_seed, clu_seed, command, config_.can_config, result.active,
      clu_seed.speed, vehicle_state.speed_unit_mph, frame);
  if (vehicle_state.has_mdps12_seed &&
      config_.can_config.mdps_bus != config_.can_config.main_bus) {
    frames.push_back(create_mdps12_frame(vehicle_state.mdps12_seed, frame));
  }
  return frames;
}

int LateralController::next_lkas11_counter(const VehicleCanState &vehicle_state) {
  if (!lkas11_counter_valid_) {
    const HyundaiLkas11Values seed = decode_lkas11(vehicle_state.lkas11_seed);
    lkas11_counter_ = (seed.msg_count + 1) & 0xf;
    lkas11_counter_valid_ = true;
  }
  const int counter = lkas11_counter_ & 0xf;
  lkas11_counter_ = (lkas11_counter_ + 1) & 0xf;
  return counter;
}
