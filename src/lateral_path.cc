#include "lateral_path.h"

#include "k230_ipc.h"

#include <cmath>

LateralPath path_from_model_state(const K230ModelState &state,
                                  unsigned long long now_ns,
                                  unsigned long long timeout_ns) {
  LateralPath path;
  if (!state.valid) {
    path.invalid_reason = "model_invalid";
    return path;
  }
  if (state.model_timestamp_ns == 0 || now_ns < state.model_timestamp_ns ||
      now_ns - state.model_timestamp_ns > timeout_ns) {
    path.invalid_reason = "model_stale";
    return path;
  }

  path.left_valid = state.lane_probabilities[1] >= 0.3f;
  path.right_valid = state.lane_probabilities[2] >= 0.3f;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float x = state.plan[i].x;
    const float y = state.plan[i].y;
    if (!std::isfinite(x) || !std::isfinite(y) || x < 1.0f || x > 80.0f) continue;
    if (path.point_count > 0 && x <= path.reach_m) continue;
    path.reach_m = x;
    ++path.point_count;
  }

  /* 점 개수만 보면 몇 미터짜리 경로도 통과한다. 그런 경로의 psi는 무의미하고
   * 저속 토크 보정이 그걸 상한까지 키운다. */
  constexpr float kMinPathReachM = 5.0f;
  path.usable_for_steering =
      path.point_count >= 4 && path.reach_m >= kMinPathReachM;
  if (!path.usable_for_steering) path.invalid_reason = "path_invalid";
  return path;
}
