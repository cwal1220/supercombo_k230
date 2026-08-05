#ifndef LANE_PLAN_FUSION_H
#define LANE_PLAN_FUSION_H

#include <vector>

// Replaces the quantization-sensitive lateral plan and yaw with the center of
// the high-confidence inner lane lines while preserving the policy time/x and
// longitudinal/vertical outputs. Returns true when fusion was applied.
bool fuse_lane_center_plan(std::vector<float> &raw_output);

#endif
