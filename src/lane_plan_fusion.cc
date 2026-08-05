#include "lane_plan_fusion.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr size_t kOutputFloats = 2576;
constexpr size_t kLaneOffset = 117;
constexpr size_t kLaneStdOffset = 381;
constexpr size_t kLaneProbOffset = 645;
constexpr size_t kPlanOffset = 1576;
constexpr int kTrajectorySize = 33;
constexpr int kLaneStride = kTrajectorySize * 2;
constexpr int kPlanStride = 15;

float sigmoid(float value)
{
    if (value >= 0.0f) {
        const float z = std::exp(-std::min(value, 50.0f));
        return 1.0f / (1.0f + z);
    }
    const float z = std::exp(std::max(value, -50.0f));
    return z / (1.0f + z);
}

float x_index(int index)
{
    const float t = static_cast<float>(index) / (kTrajectorySize - 1);
    return 192.0f * t * t;
}

float interpolate(const std::array<float, kTrajectorySize> &values, float x)
{
    if (x <= 0.0f) return values.front();
    if (x >= x_index(kTrajectorySize - 1)) return values.back();
    int upper = 1;
    while (upper < kTrajectorySize && x_index(upper) < x) ++upper;
    const int lower = std::max(0, upper - 1);
    const float x0 = x_index(lower);
    const float x1 = x_index(upper);
    const float fraction = x1 > x0 ? (x - x0) / (x1 - x0) : 0.0f;
    return values[lower] * (1.0f - fraction) + values[upper] * fraction;
}

} // namespace

bool fuse_lane_center_plan(std::vector<float> &raw_output)
{
    if (raw_output.size() != kOutputFloats ||
        !std::all_of(raw_output.begin(), raw_output.end(),
                     [](float value) { return std::isfinite(value); })) {
        return false;
    }

    constexpr int left_lane = 1;
    constexpr int right_lane = 2;
    const float left_probability = sigmoid(raw_output[kLaneProbOffset + left_lane * 2 + 1]);
    const float right_probability = sigmoid(raw_output[kLaneProbOffset + right_lane * 2 + 1]);
    const float left_std = std::exp(std::min(raw_output[
        kLaneStdOffset + left_lane * kLaneStride], 11.0f));
    const float right_std = std::exp(std::min(raw_output[
        kLaneStdOffset + right_lane * kLaneStride], 11.0f));
    const float probability_weight = std::clamp(
        (std::min(left_probability, right_probability) - 0.3f) / 0.4f, 0.0f, 1.0f);
    const float std_weight = std::clamp(
        (0.3f - std::max(left_std, right_std)) / 0.15f, 0.0f, 1.0f);
    const float fusion_weight = probability_weight * std_weight;
    if (fusion_weight <= 0.0f) return false;

    std::array<float, kTrajectorySize> center_y{};
    int plausible_widths = 0;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const size_t left = kLaneOffset + left_lane * kLaneStride + i * 2;
        const size_t right = kLaneOffset + right_lane * kLaneStride + i * 2;
        const float width = raw_output[right] - raw_output[left];
        if (width >= 2.0f && width <= 5.0f) ++plausible_widths;
        center_y[i] = 0.5f * (raw_output[left] + raw_output[right]);
    }
    if (plausible_widths < kTrajectorySize * 3 / 4) return false;

    for (int i = 0; i < kTrajectorySize; ++i) {
        const size_t plan = kPlanOffset + i * kPlanStride;
        const float x = raw_output[plan];
        if (!std::isfinite(x) || x < 0.0f || x > x_index(kTrajectorySize - 1))
            continue;
        const float lane_y = interpolate(center_y, x);
        raw_output[plan + 1] = raw_output[plan + 1] * (1.0f - fusion_weight) +
            lane_y * fusion_weight;

        const float x0 = std::max(0.0f, x - 1.0f);
        const float x1 = std::min(x_index(kTrajectorySize - 1), x + 1.0f);
        if (x1 > x0) {
            const float lane_yaw = std::atan2(
                interpolate(center_y, x1) - interpolate(center_y, x0), x1 - x0);
            raw_output[plan + 11] = raw_output[plan + 11] * (1.0f - fusion_weight) +
                lane_yaw * fusion_weight;
        }
    }
    return true;
}
