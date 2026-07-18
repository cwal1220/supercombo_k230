#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

#include "model_output.h"
#include "projection.h"

constexpr int kLateralControlN = 17;

struct LateralTarget {
    bool valid = false;
    bool mpc_solution_valid = false;
    float lookahead_x = 0.0f;
    float target_y = 0.0f;
    float heading = 0.0f;
    float curvature = 0.0f;
    float output_scale = 0.0f;
    int desire = 0;
    float psis[kLateralControlN] = {};
    float curvatures[kLateralControlN] = {};
    float curvature_rates[kLateralControlN] = {};
    float d_path_points[kLateralControlN] = {};
};

class LateralControlDraft {
public:
    LateralControlDraft() = default;

    LateralTarget update(const ParsedPlan &plan, const ProjectionState &projection);
};

#endif
