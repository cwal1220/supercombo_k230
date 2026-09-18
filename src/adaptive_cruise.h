#pragma once

#include <string>

struct AdaptiveCruiseConfig {
  bool enabled = true;
  float lead_probability_threshold = 0.5f;
  float standstill_gap_m = 5.0f;
  float following_time_s = 1.8f;
  float gap_correction_gain = 0.25f;
  float max_slowdown_correction_mps = 4.0f;
  float max_speedup_correction_mps = 2.5f;
  float deceleration_rate_kph_per_s = 1.5f;
  float lead_hold_s = 0.6f;
  float lead_restore_delay_s = 2.0f;
  float command_interval_s = 1.0f;
  int button_pulse_frames = 5;
};

bool load_adaptive_cruise_params_json(
    const std::string &path, AdaptiveCruiseConfig *config,
    std::string *error);

struct AdaptiveCruiseInput {
  double now_s = 0.0;
  bool enabled = true;
  bool controls_ready = false;
  bool cruise_active = false;
  bool brake_pressed = false;
  bool gas_pressed = false;
  bool driver_accelerator_override = false;
  bool speed_unit_mph = false;
  int driver_button = 0;
  int driver_main_button = 0;
  float ego_speed_kph = 0.0f;
  /* 클러스터 표시 속도. 설정 속도와 같은 척도이고 휠 속도와는 다르다
   * (K7 실측 6.6% 높음). 두 척도를 잇는 비율을 여기서 학습한다. */
  float cluster_speed_kph = 0.0f;
  float driver_set_speed_kph = 0.0f;
  bool vision_lead_updated = false;
  bool vision_lead_valid = false;
  float vision_lead_probability = 0.0f;
  float vision_lead_distance_m = 0.0f;
  float vision_lead_relative_speed_mps = 0.0f;
};

struct AdaptiveCruiseOutput {
  bool session_valid = false;
  bool active = false;
  bool lead_valid = false;
  float maximum_speed_kph = 0.0f;
  float commanded_speed_kph = 0.0f;
  float target_speed_kph = 0.0f;
  /* 학습된 클러스터/휠 속도 비. 1에서 멀어지면 척도 보정이 동작 중이다. */
  float display_scale = 1.0f;
  int command_button = 0;
};

class AdaptiveCruiseController {
public:
  explicit AdaptiveCruiseController(
      AdaptiveCruiseConfig config = AdaptiveCruiseConfig());

  void update_config(const AdaptiveCruiseConfig &config);
  AdaptiveCruiseOutput update(const AdaptiveCruiseInput &input);

private:
  /* 한 update()가 입력에서 파생한 값. 아래 단계 함수들이 이 순서로 공유한다. */
  struct Tick;
  void teardown_session(const AdaptiveCruiseInput &input, const Tick &tick);
  void track_driver_adjustment(const AdaptiveCruiseInput &input, Tick *tick);
  void begin_session_if_ready(const AdaptiveCruiseInput &input, const Tick &tick);
  void reanchor_after_driver(const AdaptiveCruiseInput &input, Tick *tick);
  void resolve_command_mismatch(const AdaptiveCruiseInput &input, const Tick &tick);
  float target_speed(const AdaptiveCruiseInput &input, const Tick &tick, bool *lead_valid);
  int pace_buttons(const AdaptiveCruiseInput &input, const Tick &tick,
                   float target_speed_kph, bool active);

  void begin_session(float speed_kph, double now_s);
  void update_display_scale(const AdaptiveCruiseInput &input, double dt_s);
  void update_vision_lead(const AdaptiveCruiseInput &input);
  float minimum_speed_kph(bool speed_unit_mph) const;
  float display_step_kph(bool speed_unit_mph) const;

  AdaptiveCruiseConfig config_;
  bool session_valid_ = false;
  int previous_driver_main_button_ = 0;
  /* 운전자가 의도한 상한. 세션 시작과 운전자 조작이 끝난 뒤에만 정한다. */
  float maximum_speed_kph_ = 0.0f;
  /* 차량의 실제 설정 속도에 대한 우리 추정. 이 차는 설정 속도를 CAN으로
   * 보고하지 않으므로(SCC 없음) 자기 명령을 적산하는 수밖에 없고, 그래서
   * 틀릴 수 있다는 전제로 아래 두 장치를 둔다: 천장 대비 하강 총량 상한과
   * 클러스터 속도와 어긋날 때의 재앵커. */
  float commanded_speed_kph_ = 0.0f;
  float filtered_lead_distance_m_ = 0.0f;
  float filtered_lead_relative_speed_mps_ = 0.0f;
  double last_valid_lead_s_ = -1.0;
  double last_command_s_ = -1.0;
  double last_accelerator_override_s_ = -1.0;
  double last_update_s_ = -1.0;
  /* cruise_active 가 꺼진 시각. 한 틱 깜빡임으로 세션을 버리지 않도록 유예. */
  double cruise_inactive_since_s_ = -1.0;
  /* 운전자가 버튼을 만지는 동안과 그 뒤 정착 시간. 이 구간에는 자동 명령을
   * 쉬고, 끝나면 실측 속도로 다시 앵커한다. */
  double driver_adjust_until_s_ = -1.0;
  bool reanchor_pending_ = false;
  /* 클러스터 속도가 우리 추정과 오래 어긋난 시각(펄스 유실·스텝 크기 불일치). */
  double mismatch_since_s_ = -1.0;
  /* 명령이 차량에 듣지 않는다고 판단한 뒤의 휴지 시각. 계속 눌러봐야 CAN만
   * 채우므로 물러났다가 가끔 다시 시도한다. */
  double ineffective_until_s_ = -1.0;
  /* 재앵커 전에 클러스터 속도가 실제로 정착했는지 본다. 고정 시간만 기다리면
   * 아직 수렴 중인 값을 천장으로 굳힌다. */
  float cluster_ref_kph_ = 0.0f;
  double cluster_ref_s_ = -1.0;
  bool last_auto_command_was_set_ = false;
  float display_scale_ = 1.0f;
  bool display_scale_valid_ = false;
  int command_button_ = 0;
  int command_frames_remaining_ = 0;
};
