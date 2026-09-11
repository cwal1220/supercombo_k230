#pragma once

#include <string>

struct K230ModelState;

/* 모델 plan을 조향 가용성 gate로만 환산한다. 목표 곡률은 MPC(LateralTarget)가
 * 내므로 여기서 경로 기하를 계산하지 않는다. */
struct LateralPath {
  int point_count = 0;
  float reach_m = 0.0f;
  bool left_valid = false;
  bool right_valid = false;
  bool usable_for_steering = false;
  std::string invalid_reason;
};

LateralPath path_from_model_state(const K230ModelState &state,
                                  unsigned long long now_ns,
                                  unsigned long long timeout_ns = 250000000ULL);
