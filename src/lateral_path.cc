#include "lateral_path.h"

#include "k230_ipc.h"

#include <algorithm>
#include <cmath>

namespace {

bool finite(float value) {
  return std::isfinite(value);
}

float path_curvature_at(const LateralPath &path, float forward_m, float span_m) {
  if (span_m <= 0.1f) return 0.0f;
  float y1 = 0.0f;
  float y2 = 0.0f;
  float y3 = 0.0f;
  if (!path_lateral_at(path, forward_m - span_m, &y1) ||
      !path_lateral_at(path, forward_m, &y2) ||
      !path_lateral_at(path, forward_m + span_m, &y3)) {
    return 0.0f;
  }
  const float inv_span = 1.0f / span_m;
  const float slope = (y3 - y1) * 0.5f * inv_span;
  const float second_derivative = (y3 - 2.0f * y2 + y1) * inv_span * inv_span;
  const float slope_term = 1.0f + slope * slope;
  const float denom = slope_term * std::sqrt(slope_term);
  return finite(denom) && denom >= 1e-6f ? second_derivative / denom : 0.0f;
}

}  // namespace

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

  path.confidence = std::clamp(state.plan_probability, 0.0f, 1.0f);
  path.left_valid = state.lane_probabilities[1] >= 0.3f;
  path.right_valid = state.lane_probabilities[2] >= 0.3f;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float x = state.plan[i].x;
    const float y = state.plan[i].y;
    if (!finite(x) || !finite(y) || x < 1.0f || x > 80.0f) continue;
    if (!path.points.empty() && x <= path.points.back().forward_m) continue;
    path.points.push_back({x, -y, path.confidence});
  }

  /* 점 개수만 보면 몇 미터짜리 경로도 통과한다. 그런 경로의 psi는 무의미하고
   * 저속 토크 보정이 그걸 상한까지 키운다. */
  constexpr float kMinPathReachM = 5.0f;
  path.usable_for_steering = path.points.size() >= 4 &&
                             path.points.back().forward_m >= kMinPathReachM;
  if (!path.usable_for_steering) path.invalid_reason = "path_invalid";
  return path;
}

bool path_lateral_at(const LateralPath &path, float forward_m, float *lateral_m) {
  if (!lateral_m || path.points.empty() ||
      forward_m < path.points.front().forward_m ||
      forward_m > path.points.back().forward_m) {
    return false;
  }
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto &a = path.points[i - 1];
    const auto &b = path.points[i];
    if (forward_m < a.forward_m || forward_m > b.forward_m) continue;
    const float span = b.forward_m - a.forward_m;
    if (std::fabs(span) < 1e-4f) {
      *lateral_m = a.lateral_m;
      return true;
    }
    const float t = (forward_m - a.forward_m) / span;
    *lateral_m = a.lateral_m * (1.0f - t) + b.lateral_m * t;
    return finite(*lateral_m);
  }
  return false;
}

float steering_curvature(const LateralPath &path, float lookahead_m) {
  return path_curvature_at(path, lookahead_m, 6.0f);
}
