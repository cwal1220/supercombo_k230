#pragma once

#include "vehicle_can.h"

/* 컨트롤러가 보는 수신 메시지 시각을 한 번에 t로 맞춘다. */
inline void stamp_can_times(VehicleCanState *vehicle, double t) {
  vehicle->lkas11_time_s = t;
  vehicle->clu11_time_s = t;
  vehicle->sas11_time_s = t;
  vehicle->esp12_time_s = t;
  vehicle->mdps12_time_s = t;
  vehicle->tcs13_time_s = t;
  vehicle->tcs15_time_s = t;
  vehicle->e_ems11_time_s = t;
  vehicle->elect_gear_time_s = t;
  vehicle->whl_spd11_time_s = t;
  vehicle->cgw1_time_s = t;
  vehicle->cgw2_time_s = t;
}

inline VehicleCanState ready_vehicle(double timestamp_s = 1.0) {
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  stamp_can_times(&vehicle, timestamp_s);
  // 저속 조향 게이트를 넘는 주행 상태가 헬퍼의 기본값이다.
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.cluster_speed_raw = 63.0f;
  vehicle.gear = 5;
  return vehicle;
}
