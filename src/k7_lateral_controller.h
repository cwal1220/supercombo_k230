#pragma once

#include <string>
#include <vector>

#include "driving_params.h"
#include "hyundai_can.h"
#include "k7_path.h"
#include "lateral_control.h"
#include "steering_params.h"
#include "vehicle_can.h"

struct K7LateralControllerConfig {
  bool enabled = true;
  bool zero_release_when_inactive = true;
  bool force_engaged = false;
  K7SteeringParams steering_params{};
  K7DrivingParams driving_params{};
  HyundaiCanConfig can_config{};
};

struct K7LateralControlResult {
  bool engaged = false;
  bool active = false;
  bool should_send = false;
  bool path_usable = false;
  bool seeds_ready = false;
  bool vehicle_fresh = false;
  bool left_lane = false;
  bool right_lane = false;
  float speed_kph = 0.0f;
  float desired_curvature = 0.0f;
  float actual_curvature = 0.0f;
  float curvature_error = 0.0f;
  float normalized_output = 0.0f;
  float feedforward = 0.0f;
  int desired_torque = 0;
  int apply_torque = 0;
  bool cut_steer_temp = false;
  std::string active_block;
  std::vector<CanFrame> frames;
};

class K7LateralController {
public:
  explicit K7LateralController(K7LateralControllerConfig config = K7LateralControllerConfig{});

  // 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
  K7LateralControlResult update(const LateralPath &path,
                                const LateralTarget &target,
                                const K7VehicleCanState &vehicle_state,
                                double now_s,
                                int frame);

  // 현재 engage 상태를 반환한다.
  bool engaged() const;

  // 제어 상태를 초기값으로 되돌린다.
  void reset();

private:
  // CLU 버튼 edge로 engage/disengage 상태를 갱신한다.
  void update_button_state(int button, double now_s);

  // active를 막는 현재 gate reason을 계산한다.
  std::string active_block_reason(const LateralPath &path,
                                  const LateralTarget &target,
                                  const K7VehicleCanState &vehicle_state,
                                  double now_s,
                                  bool seeds_ready,
                                  bool vehicle_fresh,
                                  float speed_kph) const;

  // 방향지시등 기반 수동 조향 차단 타이머를 갱신한다.
  void update_manual_blinker_timers(const K7VehicleCanState &vehicle_state,
                                    float speed_mps);

  // 수동 조향 차단 타이머를 한 프레임 감소시킨다.
  void decay_manual_blinker_timers();

  // 현재 수동 조향 차단 사유를 반환한다.
  std::string manual_blinker_block_reason() const;

  // openpilot K7 조향각 제한값을 현재 속도에 맞게 계산한다.
  float steering_angle_limit_deg(float speed_kph) const;

  // 조향각 제한으로 LKAS active를 막아야 하는지 확인한다.
  std::string steering_angle_block(const K7VehicleCanState &vehicle_state,
                                   float speed_kph) const;

  // LKAS fault 회피를 위한 임시 cut-steer 상태를 갱신한다.
  bool update_cut_steer_state(bool active, const K7VehicleCanState &vehicle_state);

  // 운전자 조향 토크 감지 타이머를 openpilot K7 방식으로 갱신한다.
  void update_driver_steering_guard(const K7VehicleCanState &vehicle_state,
                                    float speed_mps);

  // 운전자 조향 중 요청 토크 fade 비율을 반환한다.
  float driver_torque_scale() const;

  // smooth steer 모드에서 요청 토크를 서서히 줄이거나 회복한다.
  int smooth_steer_torque(int raw_torque,
                          const K7VehicleCanState &vehicle_state,
                          bool steering_pressed);

  // 제어 내부 상태를 초기값으로 되돌린다.
  void reset_control_state();

  // lateral MPC 출력을 actuator delay와 횡가속도 한계에 맞춰 보정한다.
  float lag_adjusted_desired_curvature(const LateralTarget &target,
                                       float speed_mps) const;

  // LKAS HUD state 값을 lane availability와 active 상태에서 만든다.
  int lkas_sys_state(bool active, bool left_lane, bool right_lane) const;

  // 최종 송신 frame 묶음을 만든다.
  std::vector<CanFrame> build_frames(const K7VehicleCanState &vehicle_state,
                                     const K7LateralControlResult &result,
                                     int frame);

  // LKAS11 counter를 seed frame 기준으로 openpilot 방식에 맞춰 증가시킨다.
  int next_lkas11_counter(const K7VehicleCanState &vehicle_state);

  K7LateralControllerConfig config_{};
  OpenpilotTorqueController torque_controller_;
  bool engaged_ = false;
  int last_button_ = 0;
  int last_torque_ = 0;
  bool steer_rate_limited_ = false;
  double last_disengage_s_ = -1000.0;
  int angle_limit_counter_ = 0;
  int cut_steer_frames_ = 0;
  bool cut_steer_ = false;
  int lanechange_manual_timer_ = 0;
  int driver_steering_torque_above_timer_ = 100;
  float steer_timer_apply_torque_ = 1.0f;
  bool lkas11_counter_valid_ = false;
  int lkas11_counter_ = 0;
};
