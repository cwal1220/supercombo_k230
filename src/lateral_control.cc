#include "lateral_control.h"

#include <algorithm>
#include <cmath>

namespace {

bool finite_point(const ModelPoint &point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

float model_t_idx(int i)
{
    const float t = static_cast<float>(i) / static_cast<float>(kTrajectorySize - 1);
    return 10.0f * t * t;
}

float safe_dx(float dx)
{
    if (std::fabs(dx) >= 0.001f) return dx;
    return dx < 0.0f ? -0.001f : 0.001f;
}

float path_heading(const ParsedPlan &plan, int idx)
{
    if (std::isfinite(plan.orientations[idx].z))
        return plan.orientations[idx].z;

    const int prev = std::max(0, idx - 1);
    const int next = std::min(kTrajectorySize - 1, idx + 1);
    if (prev == next || !finite_point(plan.points[prev]) || !finite_point(plan.points[next]))
        return 0.0f;
    return std::atan2(plan.points[next].y - plan.points[prev].y,
                      safe_dx(plan.points[next].x - plan.points[prev].x));
}

float path_curvature(const ParsedPlan &plan, int idx)
{
    const int prev = std::max(0, idx - 1);
    const int next = std::min(kTrajectorySize - 1, idx + 1);
    if (prev == idx || next == idx ||
        !finite_point(plan.points[prev]) ||
        !finite_point(plan.points[idx]) ||
        !finite_point(plan.points[next])) {
        return 0.0f;
    }

    const float x0 = plan.points[prev].x;
    const float x1 = plan.points[idx].x;
    const float x2 = plan.points[next].x;
    const float y0 = plan.points[prev].y;
    const float y1 = plan.points[idx].y;
    const float y2 = plan.points[next].y;
    const float dx0 = safe_dx(x1 - x0);
    const float dx1 = safe_dx(x2 - x1);
    const float dy0 = (y1 - y0) / dx0;
    const float dy1 = (y2 - y1) / dx1;
    const float span = safe_dx(x2 - x0);
    const float dy_dx = (y2 - y0) / span;
    const float d2y_dx2 = 2.0f * (dy1 - dy0) / span;
    const float denom = std::pow(1.0f + dy_dx * dy_dx, 1.5f);
    if (!std::isfinite(denom) || denom < 1e-6f) return 0.0f;
    return d2y_dx2 / denom;
}

} // namespace

LateralTarget LateralControlDraft::update(const ParsedPlan &plan)
{
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
    target.mpc_solution_valid = true;
    target.lookahead_x = point.x;
    target.target_y = point.y;
    target.heading = std::atan2(dy, std::fabs(dx) > 0.001f ? dx : 0.001f);
    target.curvature = 2.0f * point.y / (point.x * point.x + point.y * point.y);

    for (int i = 0; i < kLateralControlN; ++i) {
        target.d_path_points[i] = finite_point(plan.points[i]) ? plan.points[i].y : 0.0f;
        target.psis[i] = path_heading(plan, i);
        target.curvatures[i] = path_curvature(plan, i);
        if (!std::isfinite(target.psis[i])) target.psis[i] = 0.0f;
        if (!std::isfinite(target.curvatures[i])) target.curvatures[i] = 0.0f;
    }
    target.curvatures[0] = target.curvatures[1];

    for (int i = 0; i < kLateralControlN - 1; ++i) {
        const float dt = model_t_idx(i + 1) - model_t_idx(i);
        target.curvature_rates[i] = (target.curvatures[i + 1] - target.curvatures[i]) /
            (std::fabs(dt) > 1e-3f ? dt : 1e-3f);
        if (!std::isfinite(target.curvature_rates[i])) target.curvature_rates[i] = 0.0f;
    }
    target.curvature_rates[kLateralControlN - 1] = 0.0f;
    target.output_scale = std::min(1.0f, std::fabs(target.curvature) * 1000.0f);

    return target;
}
