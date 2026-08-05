#include "lane_plan_fusion.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

constexpr size_t kLaneOffset = 117;
constexpr size_t kLaneStdOffset = 381;
constexpr size_t kLaneProbOffset = 645;
constexpr size_t kPlanOffset = 1576;
constexpr int kTrajectorySize = 33;
constexpr int kLaneStride = kTrajectorySize * 2;
constexpr int kPlanStride = 15;

float x_index(int index)
{
    const float t = static_cast<float>(index) / (kTrajectorySize - 1);
    return 192.0f * t * t;
}

std::vector<float> make_output(float probability_logit, float width)
{
    std::vector<float> raw(2576, 0.0f);
    for (int lane : {1, 2}) {
        raw[kLaneProbOffset + lane * 2 + 1] = probability_logit;
        raw[kLaneStdOffset + lane * kLaneStride] = std::log(0.1f);
    }
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float x = x_index(i);
        raw[kLaneOffset + 1 * kLaneStride + i * 2] = 0.1f * x - width * 0.5f;
        raw[kLaneOffset + 2 * kLaneStride + i * 2] = 0.1f * x + width * 0.5f;
        raw[kPlanOffset + i * kPlanStride] = x;
    }
    return raw;
}

} // namespace

int main()
{
    auto raw = make_output(8.0f, 3.2f);
    if (!fuse_lane_center_plan(raw)) {
        std::cerr << "high-confidence lane fusion was not applied\n";
        return 1;
    }
    for (int i = 1; i < kTrajectorySize; ++i) {
        const size_t plan = kPlanOffset + i * kPlanStride;
        const float expected_y = 0.1f * raw[plan];
        if (std::fabs(raw[plan + 1] - expected_y) > 1e-4f ||
            std::fabs(raw[plan + 11] - std::atan(0.1f)) > 1e-4f) {
            std::cerr << "lane fusion geometry mismatch at " << i << "\n";
            return 1;
        }
    }

    auto low_probability = make_output(-8.0f, 3.2f);
    if (fuse_lane_center_plan(low_probability)) {
        std::cerr << "low-confidence lanes must not modify the policy plan\n";
        return 1;
    }
    auto bad_width = make_output(8.0f, 8.0f);
    if (fuse_lane_center_plan(bad_width)) {
        std::cerr << "implausible lane width must not modify the policy plan\n";
        return 1;
    }
    std::cout << "LANE_PLAN_FUSION_OK\n";
    return 0;
}
