#include "model_output.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr size_t kOutputFloats = 2576;
constexpr size_t kPoseOffset = 87;
constexpr size_t kLaneOffset = 117;
constexpr size_t kLaneMeanFloats = 4 * kTrajectorySize * 2;
constexpr size_t kLaneStdOffset = kLaneOffset + kLaneMeanFloats;
constexpr size_t kLaneProbOffset = 645;
constexpr size_t kRoadEdgeOffset = 653;
constexpr size_t kRoadEdgeMeanFloats = 2 * kTrajectorySize * 2;
constexpr size_t kRoadEdgeStdOffset = kRoadEdgeOffset + kRoadEdgeMeanFloats;
constexpr size_t kLeadOffset = 917;
constexpr size_t kLeadProbOffset = 1061;
constexpr size_t kPlanOffset = 1576;
constexpr size_t kPlanMeanFloats = kTrajectorySize * 15;
constexpr size_t kPlanStdOffset = kPlanOffset + kPlanMeanFloats;
constexpr size_t kDesireStateOffset = 2566;

float safe_exp(float value)
{
    return std::exp(std::min(value, 11.0f));
}

bool finite_range(const std::vector<float> &raw)
{
    return std::all_of(raw.begin(), raw.end(),
                       [](float value) { return std::isfinite(value); });
}

bool plan_is_sane(const ParsedPlan &plan)
{
    if (!plan.valid) return false;
    int large_regressions = 0;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const ModelPoint &point = plan.points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            point.x < -20.0f || point.x > 350.0f ||
            std::fabs(point.y) > 100.0f || std::fabs(point.z) > 50.0f) {
            return false;
        }
        if (i > 0 && point.x < plan.points[i - 1].x - 2.0f) ++large_regressions;
    }
    return large_regressions <= 1 &&
        plan.points.back().x - plan.points.front().x >= 20.0f;
}

void softmax(const float *input, float *output, int size)
{
    const float max_value = *std::max_element(input, input + size);
    float denominator = 0.0f;
    for (int i = 0; i < size; ++i) {
        output[i] = safe_exp(input[i] - max_value);
        denominator += output[i];
    }
    if (!std::isfinite(denominator) || denominator <= 0.0f) {
        std::fill(output, output + size, 0.0f);
        return;
    }
    const float inv_denominator = 1.0f / denominator;
    for (int i = 0; i < size; ++i) output[i] *= inv_denominator;
}

} // namespace

float ModelOutputParser::sigmoid(float x)
{
    if (x >= 0.0f) {
        const float z = safe_exp(-x);
        return 1.0f / (1.0f + z);
    }
    const float z = safe_exp(x);
    return z / (1.0f + z);
}

float ModelOutputParser::x_idx(int i)
{
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return static_cast<float>(192.0 * t * t);
}

bool ParsedLeads::primary(int time_idx, float min_probability, ParsedLeadPoint *lead,
                          float *probability) const
{
    if (!valid || !lead) return false;
    const int idx = std::max(0, std::min(kLeadMhpSelection - 1, time_idx));
    const float global_probability = global_probabilities[idx];
    if (!std::isfinite(global_probability) || global_probability < min_probability)
        return false;
    *lead = predictions[idx].points[0];
    if (probability) *probability = global_probability;
    return true;
}

ParsedModelOutput ModelOutputParser::parse(const std::vector<float> &raw)
{
    ParsedModelOutput output;
    if (raw.size() != kOutputFloats || !finite_range(raw)) return output;
    output.valid = true;

    output.plan.valid = true;
    output.plan.best_index = 0;
    output.plan.probability = 1.0f;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const size_t mean = kPlanOffset + static_cast<size_t>(i) * 15;
        const size_t std = kPlanStdOffset + static_cast<size_t>(i) * 15;
        output.plan.points[i] = {raw[mean], raw[mean + 1], raw[mean + 2]};
        output.plan.position_stds[i] = {
            safe_exp(raw[std]), safe_exp(raw[std + 1]), safe_exp(raw[std + 2])};
        output.plan.orientations[i] = {
            raw[mean + 9], raw[mean + 10], raw[mean + 11]};
        output.plan.orientation_rates[i] = {
            raw[mean + 12], raw[mean + 13], raw[mean + 14]};
    }
    output.plan.valid = plan_is_sane(output.plan);
    if (!output.plan.valid) output.valid = false;

    for (int lane = 0; lane < 4; ++lane) {
        ParsedLaneLine &line = output.lanes[lane];
        const size_t base = kLaneOffset + static_cast<size_t>(lane) * kTrajectorySize * 2;
        const size_t std_base = kLaneStdOffset + static_cast<size_t>(lane) * kTrajectorySize * 2;
        line.valid = true;
        line.probability = sigmoid(raw[kLaneProbOffset + lane * 2 + 1]);
        line.std = safe_exp(raw[std_base]);
        for (int i = 0; i < kTrajectorySize; ++i) {
            line.points[i] = {x_idx(i), raw[base + i * 2], raw[base + i * 2 + 1]};
        }
    }

    for (int edge = 0; edge < 2; ++edge) {
        ParsedRoadEdge &road_edge = output.road_edges[edge];
        const size_t base = kRoadEdgeOffset + static_cast<size_t>(edge) * kTrajectorySize * 2;
        const size_t std_base = kRoadEdgeStdOffset + static_cast<size_t>(edge) * kTrajectorySize * 2;
        road_edge.valid = true;
        road_edge.std = safe_exp(raw[std_base]);
        for (int i = 0; i < kTrajectorySize; ++i) {
            road_edge.points[i] = {
                x_idx(i), raw[base + i * 2], raw[base + i * 2 + 1]};
        }
    }

    output.leads.valid = true;
    for (int selection = 0; selection < kLeadMhpSelection; ++selection) {
        ParsedLeadPrediction &prediction = output.leads.predictions[selection];
        const size_t base = kLeadOffset + static_cast<size_t>(selection) * kLeadTrajLen * 4;
        for (int i = 0; i < kLeadTrajLen; ++i) {
            const size_t point = base + static_cast<size_t>(i) * 4;
            prediction.points[i] = {
                raw[point], raw[point + 1], raw[point + 2], raw[point + 3]};
        }
        const float probability = sigmoid(raw[kLeadProbOffset + selection]);
        output.leads.global_probabilities[selection] = probability;
        prediction.probabilities[selection] = probability;
    }

    softmax(raw.data() + kDesireStateOffset, output.meta.desire_state.data(), kDesireLen);

    output.has_pose = true;
    for (int i = 0; i < 3; ++i) {
        output.pose.trans[i] = raw[kPoseOffset + i];
        output.pose.rot[i] = raw[kPoseOffset + 3 + i];
        output.pose.trans_std[i] = safe_exp(raw[kPoseOffset + 6 + i]);
        output.pose.rot_std[i] = safe_exp(raw[kPoseOffset + 9 + i]);
    }

    // This model has no stop-line head. Leaving valid=false prevents stale or
    // unrelated logits from reaching the alert/control path.
    output.stop_line.valid = false;
    return output;
}
