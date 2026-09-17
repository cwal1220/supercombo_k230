#include "adaptive_cruise.h"

#include "utils_math.h"
#include "utils_json.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kAcceleratorReleaseDelayS = 0.5;
/* cruise_active 가 이만큼 꺼져 있어야 세션을 버린다. SCC12와 CLU11 추정이
 * 같은 필드를 쓰므로 한 틱 깜빡임으로 천장을 잃지 않게 유예를 둔다. */
constexpr double kCruiseInactiveTeardownS = 0.3;
/* 운전자가 버튼을 뗀 뒤 차량이 정착할 때까지. 이 구간에는 자동 명령을 쉬고,
 * 끝나면 실측 속도로 천장과 추정을 다시 잡는다. 길게 누르기(추정치는 엣지
 * 하나만 세므로 실제 감속량을 모른다)와 스텝 크기 불일치를 한꺼번에 흡수한다. */
constexpr double kDriverSettleS = 2.0;
/* 클러스터 속도가 추정과 이만큼 이 시간 이상 어긋나면 추정을 버리고 실측에
 * 다시 앵커한다. 펄스 유실이나 차량 스텝이 2 km/h가 아닌 경우를 잡는다. */
constexpr float kMismatchKph = 5.0f;
constexpr double kMismatchHoldS = 5.0;
/* 명령이 듣지 않는다고 본 뒤 물러나 있는 시간. */
constexpr double kIneffectiveBackoffS = 30.0;
/* 클러스터 속도가 이 폭 안에서 이 시간 유지되면 정착한 것으로 본다. */
constexpr float kClusterSteadyKph = 0.5f;
constexpr double kClusterSteadyHoldS = 0.5;
/* 클러스터/휠 비의 학습 시정수와 허용 범위. 클램프 끝에 붙으면 신호가 깨진
 * 것으로 보고 학습되지 않은 것으로 취급한다. */
constexpr double kDisplayScaleTauS = 5.0;
constexpr float kDisplayScaleMin = 0.97f;
constexpr float kDisplayScaleMax = 1.25f;
constexpr float kDisplayScaleLearnMinKph = 30.0f;

constexpr int kCruiseButtonResume = 1;
constexpr int kCruiseButtonSet = 2;
constexpr float kMphToKph = 1.609344f;
constexpr float kDisplayStep = 2.0f;
constexpr float kMinimumSpeedKph = 30.0f;
constexpr float kMinimumSpeedMph = 20.0f;

bool valid_set_speed(float speed_kph) {
  return std::isfinite(speed_kph) && speed_kph > 0.0f && speed_kph < 300.0f;
}

bool valid_vision_lead(const AdaptiveCruiseInput &input,
                       const AdaptiveCruiseConfig &config) {
  return input.vision_lead_valid &&
         std::isfinite(input.vision_lead_probability) &&
         input.vision_lead_probability >= config.lead_probability_threshold &&
         std::isfinite(input.vision_lead_distance_m) &&
         std::isfinite(input.vision_lead_relative_speed_mps) &&
         input.vision_lead_distance_m >= 1.0f &&
         input.vision_lead_distance_m <= 150.0f &&
         std::fabs(input.vision_lead_relative_speed_mps) <= 40.0f;
}

}  // namespace

bool load_adaptive_cruise_params_json(
    const std::string &path, AdaptiveCruiseConfig *config,
    std::string *error) {
  if (!config) return false;
  return load_json_param_file(path, [config](const std::string &text) {
    parse_json_optional_bool(text, "enabled", &config->enabled);
    parse_json_optional_float(text, "lead_probability_threshold", 0.2f, 0.99f,
                         &config->lead_probability_threshold);
    parse_json_optional_float(text, "standstill_gap_m", 2.0f, 20.0f,
                         &config->standstill_gap_m);
    parse_json_optional_float(text, "following_time_s", 0.8f, 4.0f,
                         &config->following_time_s);
    parse_json_optional_float(text, "gap_correction_gain", 0.05f, 1.0f,
                         &config->gap_correction_gain);
    parse_json_optional_float(text, "max_slowdown_correction_mps", 0.5f, 10.0f,
                         &config->max_slowdown_correction_mps);
    parse_json_optional_float(text, "max_speedup_correction_mps", 0.0f, 5.0f,
                         &config->max_speedup_correction_mps);
    parse_json_optional_float(text, "deceleration_rate_kph_per_s", 0.5f, 5.0f,
                         &config->deceleration_rate_kph_per_s);
    parse_json_optional_float(text, "lead_hold_s", 0.1f, 2.0f,
                         &config->lead_hold_s);
    parse_json_optional_float(text, "lead_restore_delay_s", 1.0f, 10.0f,
                         &config->lead_restore_delay_s);
    parse_json_optional_float(text, "command_interval_s", 0.5f, 5.0f,
                         &config->command_interval_s);
    parse_json_optional_int(text, "button_pulse_frames", 1, 10,
                       &config->button_pulse_frames);
  }, error);
}

AdaptiveCruiseController::AdaptiveCruiseController(
    AdaptiveCruiseConfig config)
    : config_(config) {}

void AdaptiveCruiseController::update_config(
    const AdaptiveCruiseConfig &config) {
  config_ = config;
}

float AdaptiveCruiseController::minimum_speed_kph(bool speed_unit_mph) const {
  return speed_unit_mph ? kMinimumSpeedMph * kMphToKph : kMinimumSpeedKph;
}

float AdaptiveCruiseController::display_step_kph(bool speed_unit_mph) const {
  return kDisplayStep * (speed_unit_mph ? kMphToKph : 1.0f);
}

void AdaptiveCruiseController::begin_session(float speed_kph, double now_s) {
  if (!valid_set_speed(speed_kph)) return;
  session_valid_ = true;
  maximum_speed_kph_ = speed_kph;
  commanded_speed_kph_ = speed_kph;
  last_command_s_ = now_s;
  last_auto_command_was_set_ = false;
  command_button_ = 0;
  command_frames_remaining_ = 0;
  reanchor_pending_ = false;
  mismatch_since_s_ = -1.0;
  ineffective_until_s_ = -1.0;
}

void AdaptiveCruiseController::update_display_scale(
    const AdaptiveCruiseInput &input, double dt_s) {
  if (dt_s <= 0.0) return;
  if (!std::isfinite(input.cluster_speed_kph) ||
      !std::isfinite(input.ego_speed_kph) ||
      input.cluster_speed_kph < kDisplayScaleLearnMinKph ||
      input.ego_speed_kph < kDisplayScaleLearnMinKph) {
    return;
  }
  const float raw = input.cluster_speed_kph / input.ego_speed_kph;
  /* 범위를 벗어나면 둘 중 하나가 깨진 것이다. 클램프해서 쓰면 "학습됐다"와
   * 구분이 안 되므로 학습을 무효로 돌리고 명령을 막는다. */
  if (raw < kDisplayScaleMin || raw > kDisplayScaleMax) {
    display_scale_valid_ = false;
    return;
  }
  if (!display_scale_valid_) {
    display_scale_ = raw;
    display_scale_valid_ = true;
    return;
  }
  const float alpha = static_cast<float>(dt_s / (kDisplayScaleTauS + dt_s));
  display_scale_ += alpha * (raw - display_scale_);
}

void AdaptiveCruiseController::update_vision_lead(
    const AdaptiveCruiseInput &input) {
  if (!input.vision_lead_updated || !valid_vision_lead(input, config_)) return;

  const bool reacquired = last_valid_lead_s_ < 0.0 ||
                          input.now_s - last_valid_lead_s_ > config_.lead_hold_s;
  if (reacquired) {
    filtered_lead_distance_m_ = input.vision_lead_distance_m;
    filtered_lead_relative_speed_mps_ =
        input.vision_lead_relative_speed_mps;
  } else {
    const float distance_alpha =
        input.vision_lead_distance_m < filtered_lead_distance_m_ ? 0.45f : 0.2f;
    filtered_lead_distance_m_ +=
        distance_alpha * (input.vision_lead_distance_m - filtered_lead_distance_m_);
    filtered_lead_relative_speed_mps_ +=
        0.3f * (input.vision_lead_relative_speed_mps -
                filtered_lead_relative_speed_mps_);
  }
  last_valid_lead_s_ = input.now_s;
}

AdaptiveCruiseOutput AdaptiveCruiseController::update(
    const AdaptiveCruiseInput &input) {
  const double dt_s = last_update_s_ < 0.0 || input.now_s < last_update_s_
      ? 0.0 : std::min(0.1, input.now_s - last_update_s_);
  last_update_s_ = input.now_s;
  update_display_scale(input, dt_s);

  const bool accelerator_override =
      input.gas_pressed || input.driver_accelerator_override;
  if (accelerator_override) last_accelerator_override_s_ = input.now_s;

  const bool driver_main_pressed =
      input.driver_main_button != 0 && previous_driver_main_button_ == 0;
  const float minimum_kph = minimum_speed_kph(input.speed_unit_mph);
  const float step_kph = display_step_kph(input.speed_unit_mph);
  const bool speed_valid = std::isfinite(input.ego_speed_kph);
  const bool cluster_valid = std::isfinite(input.cluster_speed_kph) &&
                             input.cluster_speed_kph > 0.0f;

  /* 세션 종료. cruise_active 는 SCC12와 CLU11 추정이 같은 필드를 쓰므로 한 틱
   * 깜빡임이 가능하다. 유예를 두되, 실제로 꺼지면 반드시 버린다 — 예전에는
   * 버리지 않아 다음 engage 때 이전 주행의 천장이 남았다. */
  if (input.cruise_active) {
    cruise_inactive_since_s_ = -1.0;
  } else if (cruise_inactive_since_s_ < 0.0) {
    cruise_inactive_since_s_ = input.now_s;
  }
  const bool cruise_off_settled =
      !input.cruise_active && cruise_inactive_since_s_ >= 0.0 &&
      input.now_s - cruise_inactive_since_s_ >= kCruiseInactiveTeardownS;
  if (!input.enabled || driver_main_pressed || cruise_off_settled) {
    session_valid_ = false;
    maximum_speed_kph_ = 0.0f;
    commanded_speed_kph_ = 0.0f;
    last_valid_lead_s_ = -1.0;
    reanchor_pending_ = false;
    mismatch_since_s_ = -1.0;
    ineffective_until_s_ = -1.0;
    driver_adjust_until_s_ = -1.0;
    last_auto_command_was_set_ = false;
  }

  /* 운전자가 버튼을 만지는 동안과 뗀 뒤 정착까지는 자동 명령을 쉰다. */
  /* 세션이 이미 있을 때만 유예한다. 세션을 여는 첫 SET 자체를 막으면 안 된다. */
  if (input.driver_button != 0 && session_valid_) {
    driver_adjust_until_s_ = input.now_s + kDriverSettleS;
    reanchor_pending_ = true;
  }
  const bool driver_adjusting =
      driver_adjust_until_s_ >= 0.0 && input.now_s < driver_adjust_until_s_;

  /* 세션 시작은 차량이 유효한 설정 속도를 보고한 뒤로 미룬다. */
  if (input.enabled && input.cruise_active && !driver_main_pressed &&
      !session_valid_ &&
      valid_set_speed(input.driver_set_speed_kph)) {
    begin_session(std::max(minimum_kph, input.driver_set_speed_kph), input.now_s);
  }

  if (cluster_valid) {
    if (cluster_ref_s_ < 0.0 ||
        std::fabs(input.cluster_speed_kph - cluster_ref_kph_) > kClusterSteadyKph) {
      cluster_ref_kph_ = input.cluster_speed_kph;
      cluster_ref_s_ = input.now_s;
    }
  } else {
    cluster_ref_s_ = -1.0;
  }
  const bool cluster_steady = cluster_valid && cluster_ref_s_ >= 0.0 &&
                              input.now_s - cluster_ref_s_ >= kClusterSteadyHoldS;

  /* 운전자 조작이 끝나면 추측을 버리고 실측 속도에 다시 앵커한다. 고정형
   * 크루즈는 정착하면 클러스터 속도가 곧 설정 속도다. 길게 누르기(추정치는
   * 버튼 엣지 하나만 세므로 실제 감속량을 모른다), 2 km/h가 아닌 스텝,
   * 펄스 유실이 모두 여기서 흡수된다. */
  if (session_valid_ && input.cruise_active && reanchor_pending_ &&
      !driver_adjusting && input.driver_button == 0) {
    if (cluster_steady) {
      const float anchor = std::max(minimum_kph, input.cluster_speed_kph);
      commanded_speed_kph_ = anchor;
      maximum_speed_kph_ = anchor;
      reanchor_pending_ = false;
      mismatch_since_s_ = -1.0;
      ineffective_until_s_ = -1.0;
      last_command_s_ = input.now_s;
    }
  }

  /* 추측 적산이 실제와 오래 어긋나면(펄스 유실, 스텝 크기 불일치) 실측으로
   * 되돌린다. 천장은 건드리지 않는다 — 운전자 의도가 아니기 때문이다. */
  if (session_valid_ && input.cruise_active && cluster_valid &&
      !driver_adjusting && command_frames_remaining_ == 0 &&
      last_command_s_ >= 0.0 &&
      input.now_s - last_command_s_ >= kMismatchHoldS) {
    if (std::fabs(input.cluster_speed_kph - commanded_speed_kph_) > kMismatchKph) {
      if (mismatch_since_s_ < 0.0) mismatch_since_s_ = input.now_s;
      if (input.now_s - mismatch_since_s_ >= kMismatchHoldS) {
        /* 우리 추정이 실제보다 낮다는 건 SET-이 먹지 않았다는 뜻이다.
         * 실측으로 되돌리기만 하면 여유가 되살아나 같은 명령을 무한 반복한다. */
        if (input.cluster_speed_kph > commanded_speed_kph_)
          ineffective_until_s_ = input.now_s + kIneffectiveBackoffS;
        commanded_speed_kph_ = clamp_float(input.cluster_speed_kph,
                                           minimum_kph, maximum_speed_kph_);
        mismatch_since_s_ = -1.0;
      }
    } else {
      mismatch_since_s_ = -1.0;
    }
  }

  update_vision_lead(input);

  const bool lead_valid = session_valid_ && speed_valid &&
                          last_valid_lead_s_ >= 0.0 &&
                          input.now_s >= last_valid_lead_s_ &&
                          input.now_s - last_valid_lead_s_ <= config_.lead_hold_s;
  float target_speed_kph = commanded_speed_kph_;
  if (session_valid_ && lead_valid && display_scale_valid_) {
    const float ego_speed_mps = std::max(0.0f, input.ego_speed_kph / 3.6f);
    const float lead_speed_mps =
        std::max(0.0f, ego_speed_mps + filtered_lead_relative_speed_mps_);
    const float desired_gap_m =
        config_.standstill_gap_m + config_.following_time_s * ego_speed_mps;
    const float slowdown_response_s =
        step_kph / std::max(0.1f, config_.deceleration_rate_kph_per_s);
    const float prediction_horizon_s =
        std::min(2.0f, config_.command_interval_s + 0.5f * slowdown_response_s);
    const float predicted_lead_distance_m = std::max(
        1.0f, filtered_lead_distance_m_ +
                  std::min(0.0f, filtered_lead_relative_speed_mps_) *
                      prediction_horizon_s);
    const float gap_correction_mps = clamp_float(
        (predicted_lead_distance_m - desired_gap_m) *
            config_.gap_correction_gain,
        -config_.max_slowdown_correction_mps,
        config_.max_speedup_correction_mps);
    /* 차간·상대속도는 휠 속도 기준, 설정 속도는 클러스터 표시 기준이다. K7
     * 실측으로 클러스터가 6.6% 높아, 환산 없이 비교하면 차간이 맞아도 SET-이
     * 계속 나가 설정 속도가 바닥까지 내려갔다. */
    target_speed_kph = clamp_float(
        (lead_speed_mps + gap_correction_mps) * 3.6f * display_scale_,
        minimum_kph, maximum_speed_kph_);
  } else if (session_valid_ &&
             (last_valid_lead_s_ < 0.0 ||
              input.now_s - last_valid_lead_s_ >=
                  config_.lead_restore_delay_s)) {
    target_speed_kph = maximum_speed_kph_;
  }

  const bool active = input.enabled && session_valid_ && input.cruise_active;
  const bool accelerator_released =
      !accelerator_override &&
      (last_accelerator_override_s_ < 0.0 ||
       input.now_s - last_accelerator_override_s_ >=
           kAcceleratorReleaseDelayS);
  /* 척도를 모르면 두 척도를 비교할 수 없으므로 아예 명령하지 않는다. */
  const bool command_allowed = active && speed_valid && display_scale_valid_ &&
                               input.controls_ready && !input.brake_pressed &&
                               accelerator_released && !driver_adjusting &&
                               (ineffective_until_s_ < 0.0 ||
                                input.now_s >= ineffective_until_s_) &&
                               input.driver_button == 0 && !driver_main_pressed;
  if (!command_allowed) {
    command_button_ = 0;
    command_frames_remaining_ = 0;
  }

  int output_button = 0;
  if (command_allowed && command_frames_remaining_ > 0) {
    output_button = command_button_;
    --command_frames_remaining_;
  } else if (command_allowed) {
    /* 데드밴드는 한 스텝. 0.75스텝이면 한 번 누른 결과가 다시 밴드 밖으로
     * 나가 SET-/RES+가 번갈아 나온다(65 km/h 추종 180초: 87회 -> 37회). */
    const float command_deadband_kph = step_kph;
    const bool wants_set =
        target_speed_kph <= commanded_speed_kph_ - command_deadband_kph &&
        commanded_speed_kph_ > minimum_kph + 0.1f;
    const bool wants_resume =
        target_speed_kph >= commanded_speed_kph_ + command_deadband_kph &&
        commanded_speed_kph_ < maximum_speed_kph_ - 0.1f;
    /* SET-과 RES+에 같은 간격. 감속 응답을 기다리는 긴 간격은 연속 SET- 사이
     * 에만 쓴다. */
    const double slowdown_interval_s = std::max(
        static_cast<double>(config_.command_interval_s),
        static_cast<double>(step_kph) /
            std::max(0.1, static_cast<double>(config_.deceleration_rate_kph_per_s)));
    const double button_interval_s = last_auto_command_was_set_
        ? slowdown_interval_s
        : static_cast<double>(config_.command_interval_s);
    const bool interval_ready =
        last_command_s_ < 0.0 ||
        input.now_s - last_command_s_ >= button_interval_s;

    if (wants_set && interval_ready) {
      command_button_ = kCruiseButtonSet;
      last_auto_command_was_set_ = true;
      commanded_speed_kph_ =
          std::max(minimum_kph, commanded_speed_kph_ - step_kph);
    } else if (wants_resume && interval_ready) {
      command_button_ = kCruiseButtonResume;
      last_auto_command_was_set_ = false;
      commanded_speed_kph_ =
          std::min(maximum_speed_kph_, commanded_speed_kph_ + step_kph);
    } else {
      command_button_ = 0;
    }
    if (command_button_ != 0) {
      last_command_s_ = input.now_s;
      command_frames_remaining_ = config_.button_pulse_frames - 1;
      output_button = command_button_;
      mismatch_since_s_ = -1.0;
    }
  }

  previous_cruise_active_ = input.cruise_active;
  previous_driver_button_ = input.driver_button;
  previous_driver_main_button_ = input.driver_main_button;

  AdaptiveCruiseOutput output;
  output.session_valid = session_valid_;
  output.active = active;
  output.lead_valid = lead_valid;
  output.maximum_speed_kph = maximum_speed_kph_;
  output.commanded_speed_kph = commanded_speed_kph_;
  output.target_speed_kph = target_speed_kph;
  output.display_scale = display_scale_;
  output.command_button = output_button;
  return output;
}
