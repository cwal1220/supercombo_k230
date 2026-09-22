#include "lateral_controller.h"

#include "utils_math.h"
#include "utils_time.h"
#include "model_output.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kButtonSetDecel = 2;
constexpr int kButtonCancel = 4;
constexpr int kGearDrive = 5;
constexpr int kSteeringPressedMinCount = 5;
constexpr double kPandaEngageGraceS = 1.0;

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
  // path 복귀 디바운스: 무효는 즉시 반영, 무효를 겪은 뒤의 복귀는 0.5s
  // 연속 유효를 요구한다(경계 깜빡임 차단). 최초 유효는 바로 통과.
  if (!path.usable_for_steering) {
    path_valid_since_s_ = -1.0;
    path_usable_debounced_ = false;
    path_seen_invalid_ = true;
  } else if (!path_usable_debounced_) {
    if (path_valid_since_s_ < 0.0) path_valid_since_s_ = now_s;
    if (!path_seen_invalid_ || now_s - path_valid_since_s_ >= 0.5)
      path_usable_debounced_ = true;
  }
  LateralPath gated_path = path;
  gated_path.usable_for_steering = path_usable_debounced_;
  if (!path_usable_debounced_) gated_path.invalid_reason = "path_invalid";
  result.path_usable = gated_path.usable_for_steering;
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
  result.desired_curvature = lag_adjusted_desired_curvature(
      target, speed_mps, plan_age_s, config_.steering_params.steer_actuator_delay,
      prev_desired_curvature_);
  /* plan이 무효인 프레임은 0을 돌려주므로 직전 값을 보존한다. 짧은 공백 뒤에
   * 0에서 다시 램프업하면 복귀가 느려진다. */
  if (target.valid) prev_desired_curvature_ = result.desired_curvature;
  result.active_block = active_block_reason(gated_path, target, vehicle_state, now_s,
                                            result.seeds_ready, result.vehicle_fresh,
                                            panda_ready, panda_controls_allowed,
                                            result.control_speed_kph, plan_age_s);
  const BlockKind kind = block_kind(result.active_block);
  if (logical_engaged && kind == BlockKind::Hard) {
    panda_engage_pending_ = false;
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
    result.engaged = config_.force_engaged;
  }

  if (engage_requested) {
    if (kind == BlockKind::Transient) {
      panda_engage_pending_ = true;
      panda_engage_pending_s_ = now_s;
    } else {
      panda_engage_pending_ = false;
    }
  }

  if (engage_requested && (kind == BlockKind::Hard || kind == BlockKind::Reject)) {
    // 차량/컨트롤러의 정적 gate는 실제 engage 요청 실패로 처리한다.
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
    result.engaged = config_.force_engaged;
    result.engage_rejected = true;
  }

  if (panda_engage_pending_) {
    const bool panda_waiting = kind == BlockKind::Transient;
    const bool grace_elapsed = now_s - panda_engage_pending_s_ >= kPandaEngageGraceS;
    if (kind == BlockKind::None || kind == BlockKind::Availability) {
      // Panda 허가가 도착했고 나머지는 가용성 상태뿐이면 engage를 유지한다.
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
  result.active = kind == BlockKind::None;
  /* 가용성 대기 중에는 토크만 0으로 하고 steer_req/MDPS 속도 스푸프는
   * 유지한다 — 매 정차마다 끊기면 MDPS/클러스터가 천이 경보를 낸다.
   * 결함/해제는 즉시 끊는다. plan 무효는 engage는 거부하되 steer_req는 잡아둔다. */
  steer_availability_hold_ = logical_engaged && !result.active &&
      (kind == BlockKind::Availability ||
       result.active_block == BlockReason::LateralPlanInvalid);
  result.cut_steer_temp = update_cut_steer_state(result.active, vehicle_state);

  const bool steering_pressed = update_steering_pressed(vehicle_state.driver_torque);
  const SteeringParams &control_params = config_.steering_params;
  const bool yaw_rate_valid = signal_time_fresh(
                                  vehicle_state.esp12_time_s, now_s,
                                  static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) /
                                      1000.0) &&
                              vehicle_state.yaw_rate_valid;
  /* 편경사: 직선(|yaw*v| < 0.4)에서만 갱신, 커브는 홀드 — 커브에서는 차체 롤
   * 중력 누설이 섞인다(2026-08-30 drive10: 커브 방향 반상관 ±0.2~0.5 + 탈출 꼬리).
   * rc 2초. 실측 검증식 (2026-08-27, 직선 -0.117 재현): bank = lat + yaw_rate*v. */
  constexpr float kBankAlpha = 0.01f / (2.0f + 0.01f);
  if (yaw_rate_valid && vehicle_state.lat_accel_valid &&
      std::isfinite(vehicle_state.lat_accel_mps2) && speed_mps > 8.0f &&
      std::fabs(vehicle_state.yaw_rate_rad_s * speed_mps) < 0.4f) {
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
    /* MDPS는 steer 요청이 켜진 채 |조향각|이 85도 위에 1초 머물면 fault를 낸다
     * (2026-09-18 K7 실측 0.98~1.12 s, 토크 크기 무관). fault가 날 때 이미 토크가
     * 0이면 어시스트가 빠졌다 돌아오는 "탁"이 없다. 즉시 0으로 떨어뜨릴 필요는
     * 없고 fault 전에 닿기만 하면 되므로, 램프로 내려 짧게 스치는 커브에서는
     * 어시스트를 유지한다(2026-09-21: 85도 진입 46회 중 46%가 0.5 s 미만).
     * 적분기는 램프 내내 얼린다. */
    const bool above_fault_angle =
        control_params.avoid_lkas_fault_enabled &&
        std::fabs(vehicle_state.steering_angle_deg) >=
            control_params.avoid_lkas_fault_max_angle_deg;
    /* cut_steer의 angle_limit_counter_는 컷이 나가며 0으로 돌아가 램프를
     * 되살리므로 따로 센다. */
    if (above_fault_angle) ++fault_angle_frames_; else fault_angle_frames_ = 0;
    // fault 실측 하한 98프레임보다 먼저 0에 닿도록 컷 프레임 수에서 여유를 뺀다.
    const int ramp_frames = std::max(1, control_params.avoid_lkas_fault_max_frames - 20);
    const float angle_scale = above_fault_angle
        ? clamp_float(1.0f - static_cast<float>(fault_angle_frames_) /
                                 static_cast<float>(ramp_frames), 0.0f, 1.0f)
        : 1.0f;
    const int raw_torque = torque_controller_.update(
        true, speed_mps, result.desired_curvature, vehicle_state.steering_angle_deg,
        steering_pressed, steer_rate_limited_ || above_fault_angle, control_params,
        vehicle_state.yaw_rate_rad_s, yaw_rate_valid, road_bank_lat_accel_);
    result.desired_torque = static_cast<int>(std::lround(
        static_cast<float>(raw_torque) * driver_torque_scale() * angle_scale));
    result.actual_curvature = torque_controller_.actual_curvature();
    result.actual_curvature_vm = torque_controller_.actual_curvature_vm();
    result.actual_curvature_yaw = torque_controller_.actual_curvature_yaw();
    result.curvature_error = result.desired_curvature - result.actual_curvature;
    result.normalized_output = torque_controller_.normalized_output();
    result.feedforward = torque_controller_.feedforward();
    result.apply_torque = apply_hyundai_steer_torque_limits(
        result.desired_torque, last_torque_, vehicle_state.driver_torque,
        hyundai_limits(control_params));
  } else {
    // 0을 넘기면 커브 중 engage 시 지연 버퍼가 0-setpoint로 P를 튀게 한다
    fault_angle_frames_ = 0;
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
  update_driver_steering_guard(vehicle_state);
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
    } else if (cut_steer_frames_ >= std::max(1, params.avoid_lkas_fault_cut_frames)) {
      cut_steer_frames_ = 0;
      cut_steer_ = false;
    }
  } else {
    angle_limit_counter_ = 0;
    if (vehicle_state.mdps_error_count > params.avoid_lkas_fault_max_frames) {
      cut_steer_ = true;
    } else if (cut_steer_frames_ >= std::max(1, params.avoid_lkas_fault_cut_frames)) {
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
    const VehicleCanState &vehicle_state) {
  /* 속도 제한 없이 건다. 30 km/h 위에서 fade가 꺼져 있으면 운전자와 부호가
   * 반대인 요청을 panda 운전자 클램프가 통째로 자른다(2026-09-21 실측:
   * 급락 프레임의 61%가 클램프, 그 절반이 30 km/h 위). */
  const bool driver_steering_torque_above =
      std::abs(vehicle_state.driver_torque) > config_.driving_params.driver_torque_threshold;
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

// 제어 내부 상태를 초기값으로 되돌린다.
void LateralController::reset_control_state() {
  last_torque_ = 0;
  steer_rate_limited_ = false;
  angle_limit_counter_ = 0;
  fault_angle_frames_ = 0;
  cut_steer_frames_ = 0;
  cut_steer_ = false;
  driver_steering_torque_above_timer_ = 100;
  torque_controller_.reset();
}

// active를 막는 현재 gate reason을 계산한다.
BlockReason LateralController::active_block_reason(
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
  /* 순서 규칙: 데이터 유효성 -> 차량 결함(hard disengage) -> 핸드셰이크 ->
   * 가용성 대기. 결함이 뒤로 밀리면 앞선 일시적 사유가 결함을 가리고, 그
   * 사이 engage가 유예되어 톤만 울렸다가 해제된다. */
  if (!config_.force_engaged && !engaged_) return BlockReason::NotEngaged;
  if (!config_.steering_params.enabled) return BlockReason::ControllerDisabled;
  if (!seeds_ready) return BlockReason::SeedsMissing;
  if (!vehicle_fresh) return BlockReason::VehicleStateStale;
  if (!std::isfinite(speed_kph)) return BlockReason::SpeedInvalid;
  if (vehicle_state.door_open) return BlockReason::DoorOpen;
  if (vehicle_state.seatbelt_unlatched) return BlockReason::SeatbeltUnlatched;
  if (vehicle_state.esp_disabled) return BlockReason::EspDisabled;
  if (vehicle_state.park_brake) return BlockReason::ParkBrake;
  if (vehicle_state.brake_error) return BlockReason::BrakeError;
  if (vehicle_state.gear != kGearDrive) return BlockReason::GearNotDrive;
  if (vehicle_state.steering_fault) return BlockReason::MdpsFault;
  /* Panda 핸드셰이크는 차량 결함 뒤에 온다. 앞에 두면 시동 직후 health가
   * 도착하기 전의 engage 요청이 panda_not_ready(일시적)로 분류되어 유예되고,
   * 안전벨트/기어 같은 하드 결함이 가려진 채 engage 톤이 울린 뒤 해제된다. */
  if (!panda_ready) return BlockReason::PandaNotReady;
  if (!panda_controls_allowed) return BlockReason::PandaControlsOff;
  if (!config_.steering_params.torque_use_angle) {
    if (!signal_time_fresh(vehicle_state.esp12_time_s, now_s,
                      static_cast<double>(config_.driving_params.vehicle_state_timeout_ms) /
                          1000.0)) {
      return BlockReason::EspStale;
    }
    if (!vehicle_state.yaw_rate_valid) return BlockReason::YawRateInvalid;
  }
  if (!path.usable_for_steering) {
    /* 정지에서는 plan이 원래 짧아 path 무효가 정상이다. 오류가 아니라
     * 대기로 보고한다. 이 속도 밑은 min_steer_speed로 토크도 0이다. */
    if (speed_kph / 3.6f < config_.steering_params.min_steer_speed_mps)
      return BlockReason::Stopped;
    return BlockReason::PathInvalid;
  }
  if (!target.valid || !target.mpc_solution_valid) return BlockReason::LateralPlanInvalid;
  /* 모델 경로 gate는 모델 발행 시각만 본다. 플래너 스레드가 멈춰 target이
   * 갱신되지 않는 경우까지 근거 프레임 캡처 시각으로 함께 막는다. */
  if (target.capture_timestamp_ns != 0 &&
      plan_age_s > static_cast<float>(config_.driving_params.model_timeout_ms) /
                       1000.0f) {
    return BlockReason::LateralPlanStale;
  }
  return BlockReason::None;
}

float lag_adjusted_desired_curvature(const LateralTarget &target, float speed_mps,
                                     float plan_age_s, float steer_actuator_delay_s,
                                     float prev_curvature) {
  if (!target.valid) return 0.0f;
  /* plan은 카메라 캡처 시점 기준이므로 소비 시점까지의 실측 나이를 actuator
   * delay에 더해 보간한다. 부수 효과로 desired curvature가 20Hz 계단 대신
   * 매 tick plan 위를 따라 전진한다. */
  const float delay = std::max(0.01f, steer_actuator_delay_s) +
      clamp_float(plan_age_s, 0.0f, kMaxPlanAgeCompS);
  const float current_curvature = target.curvatures[0];
  const float psi = interp_lateral(delay, target.psis);
  // openpilot drive_helpers.MIN_SPEED. 하한이 낮으면 psi/(v*delay)가 정지
  // 부근에서 발산해 작은 plan 오차가 곡률 상한까지 증폭된다.
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  const float curvature_from_psi = psi / (speed * delay);
  float desired_curvature = current_curvature +
      2.0f * (curvature_from_psi - current_curvature);

  /* ISO 횡저크 한계를 직전 출력 기준 틱당 변화율로 건다. 플랜 노드 기준 창이던
   * v0.9.4와 달리 틱간 계단을 실제로 막아, 변화율 제한된 와이어가 예산을
   * 노이즈에 쓰지 않는다. */
  const float max_curvature_rate = kMaxLateralJerk /
      (speed * speed);
  desired_curvature = clamp_float(
      desired_curvature,
      prev_curvature - max_curvature_rate * kCurvatureRateWindowS,
      prev_curvature + max_curvature_rate * kCurvatureRateWindowS);

  const float limit_speed = std::max(speed, 1.0f);
  desired_curvature = clamp_float(
      desired_curvature,
      -kMaxLateralAccel / (limit_speed * limit_speed),
      kMaxLateralAccel / (limit_speed * limit_speed));
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
  command.steer_req = result.active || steer_availability_hold_;
  command.cut_steer_temp = result.cut_steer_temp;
  /* 클러스터는 sys_state 천이마다 부저를 울린다. active(정차 대기 등
   * 가용성)가 아니라 engaged를 따르게 해 enable/disable에서만 울린다.
   * steer_req는 별도 비트로 매 프레임 정확히 나간다. */
  command.sys_state = lkas_sys_state(result.engaged, true, true);
  command.sys_warning = false;
  command.left_lane = result.left_lane;
  command.right_lane = result.right_lane;
  command.lkas_msg_count = next_lkas11_counter(vehicle_state);
  command.ldws_fix = false;  // K7 YG는 LDWS 전용차가 아니다

  const HyundaiLkas11Values lkas_seed = decode_lkas11(vehicle_state.lkas11_seed);
  const HyundaiClu11Values clu_seed = decode_clu11(vehicle_state.clu11_seed);
  std::vector<CanFrame> frames = build_lateral_can_frames(
      lkas_seed, clu_seed, command, config_.can_config,
      result.active || steer_availability_hold_,
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
