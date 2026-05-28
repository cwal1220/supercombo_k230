#include "model_output.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kPlanMhpN = 5;
constexpr int kPlanStride = kTrajectorySize * 15 * 2 + 1;
constexpr int kLaneOffset = kPlanMhpN * kPlanStride;
constexpr int kLaneLineSize = 4 * kTrajectorySize * 2;
constexpr int kLaneProbOffset = kLaneOffset + kLaneLineSize * 2;
constexpr int kRoadEdgeOffset = kLaneProbOffset + 8;
constexpr int kRoadEdgeMeanSize = 2 * kTrajectorySize * 2;
constexpr int kRoadEdgeSize = kRoadEdgeMeanSize * 2;
constexpr int kLeadElementSize = 4;
constexpr int kLeadPredictionStride = kLeadTrajLen * kLeadElementSize * 2 + kLeadMhpSelection;
constexpr int kLeadOffset = kRoadEdgeOffset + kRoadEdgeSize;
constexpr int kLeadProbOffset = kLeadOffset + kLeadMhpN * kLeadPredictionStride;
constexpr int kMinOverlayOutputFloats = kRoadEdgeOffset + kRoadEdgeMeanSize;
constexpr int kMinLeadOutputFloats = kLeadProbOffset + kLeadMhpSelection;
constexpr int kPoseOffset = 6000;
constexpr int kMinPoseOutputFloats = kPoseOffset + 12;

} // namespace

float ModelOutputParser::sigmoid(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

float ModelOutputParser::x_idx(int i)
{
    const float t = static_cast<float>(i) / static_cast<float>(kTrajectorySize - 1);
    return 192.0f * t * t;
}

bool ParsedLeads::primary(int time_idx, float min_probability, ParsedLeadPoint *lead, float *probability) const
{
    if (!valid || !lead) return false;
    const int idx = std::max(0, std::min(kLeadMhpSelection - 1, time_idx));
    const float global_prob = global_probabilities[idx];
    if (global_prob < min_probability) return false;

    int best = 0;
    float best_prob = predictions[0].probabilities[idx];
    for (int i = 1; i < kLeadMhpN; ++i) {
        if (predictions[i].probabilities[idx] > best_prob) {
            best_prob = predictions[i].probabilities[idx];
            best = i;
        }
    }

    *lead = predictions[best].points[0];
    if (probability) *probability = global_prob;
    return true;
}

ParsedModelOutput ModelOutputParser::parse(const std::vector<float> &raw)
{
    ParsedModelOutput output;
    output.valid = raw.size() >= static_cast<size_t>(kMinOverlayOutputFloats);
    if (!output.valid) return output;

    int best_plan = 0;
    float best_prob = raw[kPlanStride - 1];
    for (int i = 1; i < kPlanMhpN; ++i) {
        const float prob = raw[i * kPlanStride + kPlanStride - 1];
        if (prob > best_prob) {
            best_prob = prob;
            best_plan = i;
        }
    }

    output.plan.valid = true;
    output.plan.best_index = best_plan;
    output.plan.probability = sigmoid(best_prob);
    const int plan_base = best_plan * kPlanStride;
    for (int i = 0; i < kTrajectorySize; ++i) {
        output.plan.points[i] = {
            raw[plan_base + i * 15 + 0],
            raw[plan_base + i * 15 + 1],
            raw[plan_base + i * 15 + 2],
        };
    }

    for (int lane = 0; lane < 4; ++lane) {
        ParsedLaneLine &line = output.lanes[lane];
        const int prob_idx = kLaneProbOffset + lane * 2 + 1;
        line.valid = true;
        line.probability = sigmoid(raw[prob_idx]);
        const int base = kLaneOffset + lane * kTrajectorySize * 2;
        for (int i = 0; i < kTrajectorySize; ++i) {
            line.points[i] = {
                x_idx(i),
                raw[base + i * 2 + 0],
                raw[base + i * 2 + 1],
            };
        }
    }

    for (int edge = 0; edge < 2; ++edge) {
        ParsedRoadEdge &road_edge = output.road_edges[edge];
        road_edge.valid = true;
        const int base = kRoadEdgeOffset + edge * kTrajectorySize * 2;
        for (int i = 0; i < kTrajectorySize; ++i) {
            road_edge.points[i] = {
                x_idx(i),
                raw[base + i * 2 + 0],
                raw[base + i * 2 + 1],
            };
        }
    }

    if (raw.size() >= static_cast<size_t>(kMinLeadOutputFloats)) {
        output.leads.valid = true;
        for (int lead = 0; lead < kLeadMhpN; ++lead) {
            const int base = kLeadOffset + lead * kLeadPredictionStride;
            for (int i = 0; i < kLeadTrajLen; ++i) {
                output.leads.predictions[lead].points[i] = {
                    raw[base + i * kLeadElementSize + 0],
                    raw[base + i * kLeadElementSize + 1],
                    raw[base + i * kLeadElementSize + 2],
                    raw[base + i * kLeadElementSize + 3],
                };
            }
            const int prob_base = base + kLeadPredictionStride - kLeadMhpSelection;
            for (int i = 0; i < kLeadMhpSelection; ++i)
                output.leads.predictions[lead].probabilities[i] = sigmoid(raw[prob_base + i]);
        }
        for (int i = 0; i < kLeadMhpSelection; ++i)
            output.leads.global_probabilities[i] = sigmoid(raw[kLeadProbOffset + i]);
    }

    if (raw.size() >= static_cast<size_t>(kMinPoseOutputFloats)) {
        output.has_pose = true;
        const float *src = raw.data() + kPoseOffset;
        for (int i = 0; i < 3; ++i) {
            output.pose.trans[i] = src[i];
            output.pose.rot[i] = src[3 + i];
            output.pose.trans_std[i] = std::exp(src[6 + i]);
            output.pose.rot_std[i] = std::exp(src[9 + i]);
        }
    }

    return output;
}
