#pragma once

#include <string>
#include <vector>

#include "driving_params.h"
#include "hyundai_can.h"
#include "lateral_path.h"
#include "lateral_target.h"
#include "openpilot_torque_controller.h"
#include "vehicle_can.h"

struct LateralControllerConfig {
  bool enabled = true;
  bool zero_release_when_inactive = true;
  bool force_engaged = false;
  SteeringParams steering_params{};
  DrivingParams driving_params{};
  HyundaiCanConfig can_config{};
};

struct LateralControlResult {
  bool engaged = false;
  bool active = false;
  bool engage_rejected = false;
  bool should_send = false;
  bool path_usable = false;
  bool seeds_ready = false;
  bool vehicle_fresh = false;
  bool left_lane = false;
  bool right_lane = false;
  float cluster_speed_kph = 0.0f;
  float control_speed_kph = 0.0f;
  float desired_curvature = 0.0f;
  float actual_curvature = 0.0f;
  // 조향각 모델/ESP yaw 기반 실측 곡률. torque_use_angle 전환 검증용 로그 값.
  float actual_curvature_vm = 0.0f;
  float actual_curvature_yaw = 0.0f;
  float curvature_error = 0.0f;
  float normalized_output = 0.0f;
  float feedforward = 0.0f;
  int desired_torque = 0;
  int apply_torque = 0;
  bool cut_steer_temp = false;
  std::string active_block;
  std::vector<CanFrame> frames;
};

class LateralController {
public:
  float road_bank_lat_accel() const { return road_bank_lat_accel_; }
  explicit LateralController(LateralControllerConfig config = LateralControllerConfig{});

  // 제어 상태를 유지한 채 런타임 파라미터를 즉시 교체한다.
  void update_params(const SteeringParams &steering_params,
                     const DrivingParams &driving_params);

  // 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
  LateralControlResult update(const LateralPath &path,
                              const LateralTarget &target,
                              const VehicleCanState &vehicle_state,
                              double now_s,
                              int frame,
                              bool panda_ready = true,
                              bool panda_controls_allowed = true);

private:
  // CLU 버튼 edge로 engage/disengage 상태를 갱신한다.
  void update_button_state(int button, double now_s);

  // active를 막는 현재 gate reason을 계산한다.
  std::string active_block_reason(const LateralPath &path,
                                  const LateralTarget &target,
                                  const VehicleCanState &vehicle_state,
                                  double now_s,
                                  bool seeds_ready,
                                  bool vehicle_fresh,
                                  bool panda_ready,
                                  bool panda_controls_allowed,
                                  float speed_kph,
                                  float plan_age_s) const;

  // 방향지시등 기반 수동 조향 차단 타이머를 갱신한다.
  void update_manual_blinker_timers(const VehicleCanState &vehicle_state,
                                    float speed_mps);

  // 수동 조향 차단 타이머를 한 프레임 감소시킨다.
  void decay_manual_blinker_timers();

  // 현재 수동 조향 차단 사유를 반환한다.
  std::string manual_blinker_block_reason() const;

  // openpilot K7 조향각 제한값을 현재 속도에 맞게 계산한다.
  float steering_angle_limit_deg(float speed_kph) const;

  // 조향각 제한으로 LKAS active를 막아야 하는지 확인한다.
  std::string steering_angle_block(const VehicleCanState &vehicle_state,
                                   float speed_kph) const;

  // LKAS fault 회피를 위한 임시 cut-steer 상태를 갱신한다.
  bool update_cut_steer_state(bool active, const VehicleCanState &vehicle_state);

  // 노이즈가 있는 운전자 조향 토크를 openpilot 방식으로 필터링한다.
  bool update_steering_pressed(int driver_torque);

  // 운전자 조향 토크 감지 타이머를 openpilot K7 방식으로 갱신한다.
  void update_driver_steering_guard(const VehicleCanState &vehicle_state,
                                    float speed_mps);

  // 운전자 조향 중 요청 토크 fade 비율을 반환한다.
  float driver_torque_scale() const;

  // smooth steer 모드에서 요청 토크를 서서히 줄이거나 회복한다.
  int smooth_steer_torque(int raw_torque,
                          const VehicleCanState &vehicle_state,
                          bool steering_pressed);

  // 제어 내부 상태를 초기값으로 되돌린다.
  void reset_control_state();

  // lateral MPC 출력을 actuator delay + plan 나이와 횡가속도 한계에 맞춰 보정한다.
  float lag_adjusted_desired_curvature(const LateralTarget &target,
                                       float speed_mps,
                                       float plan_age_s) const;

  // LKAS HUD state 값을 lane availability와 active 상태에서 만든다.
  int lkas_sys_state(bool active, bool left_lane, bool right_lane) const;

  // 최종 송신 frame 묶음을 만든다.
  std::vector<CanFrame> build_frames(const VehicleCanState &vehicle_state,
                                     const LateralControlResult &result,
                                     int frame);

  // LKAS11 counter를 seed frame 기준으로 openpilot 방식에 맞춰 증가시킨다.
  int next_lkas11_counter(const VehicleCanState &vehicle_state);

  LateralControllerConfig config_{};
  OpenpilotTorqueController torque_controller_;
  bool engaged_ = false;
  /* path 유효성 디바운스: 차단은 즉시, 복귀는 연속 유효 0.5s 후.
   * 정지 부근에서 plan 도달거리가 경계를 넘나들며 active가 깜빡이고
   * 클러스터가 천이마다 부저를 울리는 것을 막는다. */
  double path_valid_since_s_ = -1.0;
  bool path_usable_debounced_ = false;
  bool path_seen_invalid_ = false;
  // 가용성 대기 중 steer_req/스푸프 유지(토크는 0) — 정차 부저 방지
  bool steer_availability_hold_ = false;
  int last_button_ = 0;
  int last_torque_ = 0;
  bool steer_rate_limited_ = false;
  double last_disengage_s_ = -1000.0;
  int angle_limit_counter_ = 0;
  int cut_steer_frames_ = 0;
  bool cut_steer_ = false;
  int lanechange_manual_timer_ = 0;
  int steering_pressed_counter_ = 0;
  int driver_steering_torque_above_timer_ = 100;
  float steer_timer_apply_torque_ = 1.0f;
  // 라이브 편경사 추정: bank = lat실측 + yaw_rate*v, 2초 저역통과, 직선에서만 갱신
  float road_bank_lat_accel_ = 0.0f;
  bool road_bank_init_ = false;
  int road_bank_stale_frames_ = 0;
  bool lkas11_counter_valid_ = false;
  int lkas11_counter_ = 0;
  // Panda health는 100 Hz 컨트롤러보다 낮은 주기로 발행된다.
  // 비동기 허가가 도착할 때까지 SET 요청을 잠시 보류하고, 이후에도 Panda나
  // 다른 gate가 차단 중이면 요청을 거부한다.
  bool panda_engage_pending_ = false;
  double panda_engage_pending_s_ = -1000.0;
};
