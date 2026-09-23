#ifndef LATERAL_TORQUE_H
#define LATERAL_TORQUE_H

// openpilot latcontrol_torque(v0.11)의 C++ 이식.

#include "control_params.h"

/* 학습값. 상류 controlsd가 vehicleParameters·lateralTorqueParameters를 쓰는 자리다.
 * 끈 쪽은 SteeringParams를 그대로 쓴다. */
struct LiveLateralParams {
  bool use_vehicle = false;  // paramsd: SR·강성·영점 합계·롤
  float steer_ratio = 0.0f;
  float stiffness_factor = 1.0f;
  float angle_offset_deg = 0.0f;
  float roll_rad = 0.0f;  // 양수 = 오른쪽이 낮다
  bool use_torque = false;  // torqued 필터값
  float lat_accel_factor = 0.0f;
  float lat_accel_offset = 0.0f;
  float friction = 0.0f;
};

class TorqueController {
public:
  // PID와 saturation 상태를 초기화한다.
  void reset();

  // openpilot LatControlTorque와 같은 형태로 조향 토크를 계산한다.
  int update(bool active,
             float speed_mps,
             float desired_curvature,
             float steering_angle_deg,
             bool steering_pressed,
             bool steering_rate_limited,
             const SteeringParams &params,
             float yaw_rate_rad_s = 0.0f,
             bool yaw_rate_valid = false,
             float road_bank_lat_accel = 0.0f,
             const LiveLateralParams &live = LiveLateralParams{});

  // 현재 조향각/속도에서 차량 모델 기반 실제 curvature를 추정한다.
  float estimate_actual_curvature(float speed_mps,
                                  float steering_angle_deg,
                                  const SteeringParams &params,
                                  float yaw_rate_rad_s = 0.0f,
                                  bool yaw_rate_valid = false,
                                  const LiveLateralParams &live = LiveLateralParams{});

  float normalized_output() const { return normalized_output_; }
  float error() const { return error_; }
  float feedforward() const { return feedforward_; }
  float actual_curvature() const { return actual_curvature_; }
  // 조향각 차량 모델 기반 곡률과 ESP yaw rate 기반 곡률. 주행 로그에서 두
  // 값의 부호/크기 일치를 확인해 torque_use_angle 설정을 검증하는 용도.
  float actual_curvature_vm() const { return actual_curvature_vm_; }
  float actual_curvature_yaw() const { return actual_curvature_yaw_; }

private:
  // 토크 공간 이득. torqued를 쓰면 kf = 1/latAccelFactor이고 ki도 같은 비로 옮긴다.
  struct Gains {
    float kf = 0.0f;
    float ki = 0.0f;
    float friction = 0.0f;
    float lat_accel_offset = 0.0f;
  };
  static Gains gains(const SteeringParams &params, const LiveLateralParams &live);

  // 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
  void update_vehicle_model(const SteeringParams &params);

  // 조향각과 속도에서 실제 curvature를 계산한다.
  float vehicle_model_curvature(float steering_angle_rad,
                                float speed_mps,
                                const SteeringParams &params);

  // opendbc VehicleModel.roll_compensation. vehicle_model_curvature 뒤에 부른다.
  float roll_compensation(float roll_rad, float speed_mps) const;

  // PID 한 스텝을 계산한다. 비례 이득은 속도별 곡선을 따른다.
  float pid_update(float error,
                   float feedforward,
                   bool freeze_integrator,
                   const SteeringParams &params,
                   const Gains &gains,
                   float speed_mps);

  float p_ = 0.0f;
  float i_ = 0.0f;
  float f_ = 0.0f;
  float slip_factor_ = 0.0f;
  float last_mass_kg_ = -1.0f;
  float last_wheelbase_m_ = -1.0f;
  float last_center_to_front_m_ = -1.0f;
  float last_tire_stiffness_factor_ = -1.0f;
  float last_steer_ratio_ = -1.0f;
  float last_steer_ratio_rear_ = -1.0f;
  float normalized_output_ = 0.0f;
  float error_ = 0.0f;
  float feedforward_ = 0.0f;
  float actual_curvature_ = 0.0f;
  float actual_curvature_vm_ = 0.0f;
  float actual_curvature_yaw_ = 0.0f;
  SteeringParams live_vehicle_params_{};  // 학습 SR·강성을 넣은 사본

  // 지연 보정 링버퍼(100Hz 1초): 오차 = delay 전 요청 - 지금 측정.
  static constexpr int kRequestBufferLen = 100;
  float lat_accel_request_[kRequestBufferLen] = {};
  int request_head_ = 0;
  // 저크 선행 마찰용 1.2Hz 저역통과 상태
  float jerk_filtered_ = 0.0f;
};

#endif  // LATERAL_TORQUE_H
