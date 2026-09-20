#pragma once

#include <string>
#include <vector>

#include "control_block.h"
#include "control_params.h"
#include "hyundai_can.h"
#include "lateral_path.h"
#include "lateral_target.h"
#include "lateral_torque.h"
#include "vehicle_can.h"

/* lag 보상 곡률의 고정 한계. 런타임 튜닝 항목이 아니다. 진단용 참조 구현이
 * 같은 값을 쓰도록 헤더에 둔다(값이 갈리면 리플레이 검증이 조용히 썩는다). */
/* 직전 출력 대비 틱당 변화율 창. openpilot clip_curvature는 DT_CTRL을 쓴다.
 * v0.9.4는 플랜 노드 기준 DT_MDL(0.05) 편차 창이라 틱간 제한이 없었다. */
constexpr float kCurvatureRateWindowS = 0.01f;
// openpilot MAX_CURVATURE. 12 km/h 아래에서만 횡가속 한계보다 먼저 물고,
// 실주행 1~12 km/h 곡률은 p99 0.017 / 최대 0.064라 닿지 않는다.
constexpr float kMaxCurvature = 0.2f;
// openpilot drive_helpers.MIN_SPEED
constexpr float kMinCurvatureSpeedMps = 1.0f;
// EU 안전 한계(openpilot MAX_LATERAL_JERK/ACCEL). accel 3.3은 K7 실측 기준.
constexpr float kMaxLateralJerk = 5.0f;
constexpr float kMaxLateralAccel = 3.3f;
/* lag 보상에 더하는 plan 나이의 상한. 이 이상 낡은 plan은 staleness gate가
 * 별도로 차단한다. */
constexpr float kMaxPlanAgeCompS = 0.25f;

/* lateral MPC 출력을 actuator delay + plan 나이와 횡가속도 한계에 맞춰 보정한다.
 * 컨트롤러와 replay_planner가 같은 구현을 호출한다. */
float lag_adjusted_desired_curvature(const LateralTarget &target, float speed_mps,
                                     float plan_age_s, float steer_actuator_delay_s,
                                     float prev_curvature);

struct LateralControllerConfig {
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
  BlockReason active_block = BlockReason::None;
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
  BlockReason active_block_reason(const LateralPath &path,
                                  const LateralTarget &target,
                                  const VehicleCanState &vehicle_state,
                                  double now_s,
                                  bool seeds_ready,
                                  bool vehicle_fresh,
                                  bool panda_ready,
                                  bool panda_controls_allowed,
                                  float speed_kph,
                                  float plan_age_s) const;

  // LKAS fault 회피를 위한 임시 cut-steer 상태를 갱신한다.
  bool update_cut_steer_state(bool active, const VehicleCanState &vehicle_state);

  // 노이즈가 있는 운전자 조향 토크를 openpilot 방식으로 필터링한다.
  bool update_steering_pressed(int driver_torque);

  // 운전자 조향 토크 감지 타이머를 openpilot K7 방식으로 갱신한다.
  void update_driver_steering_guard(const VehicleCanState &vehicle_state,
                                    float speed_mps);

  // 운전자 조향 중 요청 토크 fade 비율을 반환한다.
  float driver_torque_scale() const;

  // 제어 내부 상태를 초기값으로 되돌린다.
  void reset_control_state();

  // LKAS HUD state 값을 lane availability와 active 상태에서 만든다.
  int lkas_sys_state(bool active, bool left_lane, bool right_lane) const;

  // 최종 송신 frame 묶음을 만든다.
  std::vector<CanFrame> build_frames(const VehicleCanState &vehicle_state,
                                     const LateralControlResult &result,
                                     int frame);

  // LKAS11 counter를 seed frame 기준으로 openpilot 방식에 맞춰 증가시킨다.
  int next_lkas11_counter(const VehicleCanState &vehicle_state);

  LateralControllerConfig config_{};
  TorqueController torque_controller_;
  bool engaged_ = false;
  /* clip_curvature의 직전 출력. active와 무관하게 이어가야 재engage 때 0에서
   * 램프업하지 않는다(openpilot controlsd도 매 틱 갱신한다). */
  float prev_desired_curvature_ = 0.0f;
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
  int steering_pressed_counter_ = 0;
  int driver_steering_torque_above_timer_ = 100;
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
