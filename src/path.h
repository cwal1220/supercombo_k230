#pragma once

#include <string>
#include <vector>

struct K230ModelState;

struct LateralPathPoint {
  float forward_m = 0.0f;
  float lateral_m = 0.0f;
  float confidence = 0.0f;
};

struct LateralPath {
  std::vector<LateralPathPoint> points;
  float confidence = 0.0f;
  bool left_valid = false;
  bool right_valid = false;
  bool usable_for_steering = false;
  std::string invalid_reason;
};

LateralPath k7_path_from_model_state(const K230ModelState &state,
                                     unsigned long long now_ns,
                                     unsigned long long timeout_ns = 250000000ULL);
bool path_lateral_at(const LateralPath &path, float forward_m, float *lateral_m);
float steering_curvature(const LateralPath &path, float lookahead_m);
