#ifndef OPENPILOT_TORQUE_CONTROLLER_H
#define OPENPILOT_TORQUE_CONTROLLER_H

// openpilot latcontrol_torque(v0.11)의 C++ 이식.

#include "steering_params.h"

class OpenpilotTorqueController {
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
             bool yaw_rate_valid = false);

  // 현재 조향각/속도에서 차량 모델 기반 실제 curvature를 추정한다.
  float estimate_actual_curvature(float speed_mps,
                                  float steering_angle_deg,
                                  const SteeringParams &params,
                                  float yaw_rate_rad_s = 0.0f,
                                  bool yaw_rate_valid = false);

  float normalized_output() const { return normalized_output_; }
  float error() const { return error_; }
  float feedforward() const { return feedforward_; }
  float actual_curvature() const { return actual_curvature_; }
  // 조향각 차량 모델 기반 곡률과 ESP yaw rate 기반 곡률. 주행 로그에서 두
  // 값의 부호/크기 일치를 확인해 torque_use_angle 설정을 검증하는 용도.
  float actual_curvature_vm() const { return actual_curvature_vm_; }
  float actual_curvature_yaw() const { return actual_curvature_yaw_; }

private:
  // 차량 모델 slip factor를 파라미터에 맞춰 갱신한다.
  void update_vehicle_model(const SteeringParams &params);

  // 조향각과 속도에서 실제 curvature를 계산한다.
  float vehicle_model_curvature(float steering_angle_rad,
                                float speed_mps,
                                float roll_rad,
                                const SteeringParams &params);

  // PID 한 스텝을 계산한다.
  float pid_update(float error,
                   float feedforward,
                   bool freeze_integrator,
                   const SteeringParams &params);

  float p_ = 0.0f;
  float i_ = 0.0f;
  float f_ = 0.0f;
  float slip_factor_ = 0.0f;
  float inv_slip_factor_ = 0.0f;
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

  // 지연 보정 링버퍼(100Hz 1초): 오차 = delay 전 요청 - 지금 측정.
  static constexpr int kRequestBufferLen = 100;
  float lat_accel_request_[kRequestBufferLen] = {};
  float curvature_request_[kRequestBufferLen] = {};
  int request_head_ = 0;
  // 저크 선행 마찰용 1.2Hz 저역통과 상태
  float jerk_filtered_ = 0.0f;
};

#endif  // OPENPILOT_TORQUE_CONTROLLER_H
