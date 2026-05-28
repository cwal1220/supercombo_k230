#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

#include "app_config.h"
#include "model_output.h"
#include "projection.h"

#include <chrono>

struct LateralTarget {
    bool valid = false;
    float lookahead_x = 0.0f;
    float target_y = 0.0f;
    float heading = 0.0f;
    float curvature = 0.0f;
};

class LateralControlDraft {
public:
    explicit LateralControlDraft(const AppConfig &config);

    LateralTarget update(const ParsedPlan &plan, const ProjectionState &projection);

private:
    bool log_enabled_ = false;
    std::chrono::steady_clock::time_point last_log_{};
};

#endif
