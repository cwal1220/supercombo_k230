#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

#include "model_output.h"
#include "projection.h"

struct LateralTarget {
    bool valid = false;
    float lookahead_x = 0.0f;
    float target_y = 0.0f;
    float heading = 0.0f;
    float curvature = 0.0f;
};

class LateralControlDraft {
public:
    LateralControlDraft() = default;

    LateralTarget update(const ParsedPlan &plan, const ProjectionState &projection);
};

#endif
