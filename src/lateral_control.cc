#include "lateral_control.h"

#include <cmath>

namespace {

bool finite_point(const ModelPoint &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

} // namespace

LateralTarget LateralControlDraft::update(const ParsedPlan &plan, const ProjectionState &projection)
{
    (void)projection;

    LateralTarget target;
    if (!plan.valid) return target;

    constexpr float kLookahead = 20.0f;
    int idx = 0;
    for (int i = 0; i < kTrajectorySize; ++i) {
        if (!finite_point(plan.points[i])) continue;
        idx = i;
        if (plan.points[i].x >= kLookahead) break;
    }

    const ModelPoint &point = plan.points[idx];
    if (!finite_point(point) || point.x < 1.0f) return target;

    int prev_idx = std::max(0, idx - 1);
    while (prev_idx > 0 && !finite_point(plan.points[prev_idx]))
        --prev_idx;
    const ModelPoint &prev = plan.points[prev_idx];
    if (!finite_point(prev)) return target;

    const float dx = point.x - prev.x;
    const float dy = point.y - prev.y;
    target.valid = true;
    target.lookahead_x = point.x;
    target.target_y = point.y;
    target.heading = std::atan2(dy, std::fabs(dx) > 0.001f ? dx : 0.001f);
    target.curvature = 2.0f * point.y / (point.x * point.x + point.y * point.y);

    return target;
}
