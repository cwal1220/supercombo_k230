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
constexpr int kDesireStateOffset = kLeadProbOffset + kLeadMhpSelection;  // 5860

/* meta 블록만 크기를 유도할 수 없어서(desire_state 뒤에 openpilot의 disengage
 * 확률과 desire_pred가 붙는다) pose 이후는 꼬리에서 역산한다. 꼬리 순서는
 * pose(12) / wide_from_device_euler(6) / sim_pose(12) / road_transform(12) /
 * feature(128) / pad(2)로 고정이다. */
constexpr int kPadFloats = 2;
constexpr int kRoadTransformFloats = 12;
constexpr int kSimPoseFloats = 12;
constexpr int kWideFromDeviceEulerFloats = 6;
constexpr int kPoseFloats = 12;
constexpr int kPoseOffset = kModelOutputFloats - kPadFloats - kModelFeatureLen -
                            kRoadTransformFloats - kSimPoseFloats -
                            kWideFromDeviceEulerFloats - kPoseFloats;
static_assert(kPoseOffset > kDesireStateOffset + kDesireLen,
              "pose block must follow the meta block");

float sigmoid_impl(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

void softmax(const float *input, float *output, int size)
{
    const float max_value = *std::max_element(input, input + size);
    float denominator = 0.0f;
    for (int i = 0; i < size; ++i) {
        output[i] = std::exp(input[i] - max_value);
        denominator += output[i];
    }
    const float inv_denominator = 1.0f / denominator;
    for (int i = 0; i < size; ++i)
        output[i] *= inv_denominator;
}

} // namespace

float ModelOutputParser::sigmoid(float x)
{
    return sigmoid_impl(x);
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
    // kmodel 계약이 6120 float로 고정이라 부분 길이를 받아줄 이유가 없다.
    output.valid = raw.size() >= static_cast<size_t>(kModelOutputFloats);
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
        const int mean_base = plan_base + i * 15;
        output.plan.points[i] = {
            raw[mean_base + 0],
            raw[mean_base + 1],
            raw[mean_base + 2],
        };
    }

    for (int lane = 0; lane < 4; ++lane) {
        ParsedLaneLine &line = output.lanes[lane];
        const int prob_idx = kLaneProbOffset + lane * 2 + 1;
        const int std_base = kLaneOffset + kLaneLineSize + lane * kTrajectorySize * 2;
        line.valid = true;
        line.probability = sigmoid(raw[prob_idx]);
        line.std = std::exp(raw[std_base]);
        const int base = kLaneOffset + lane * kTrajectorySize * 2;
        for (int i = 0; i < kTrajectorySize; ++i) {
            line.points[i] = {
                model_x_idx(i),
                raw[base + i * 2 + 0],
                raw[base + i * 2 + 1],
            };
        }
    }

    for (int edge = 0; edge < 2; ++edge) {
        ParsedRoadEdge &road_edge = output.road_edges[edge];
        road_edge.valid = true;
        const int std_base = kRoadEdgeOffset + kRoadEdgeMeanSize + edge * kTrajectorySize * 2;
        road_edge.std = std::exp(raw[std_base]);
        const int base = kRoadEdgeOffset + edge * kTrajectorySize * 2;
        for (int i = 0; i < kTrajectorySize; ++i) {
            road_edge.points[i] = {
                model_x_idx(i),
                raw[base + i * 2 + 0],
                raw[base + i * 2 + 1],
            };
        }
    }

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

    softmax(raw.data() + kDesireStateOffset,
            output.meta.desire_state.data(), kDesireLen);

    output.has_pose = true;
    const float *pose_src = raw.data() + kPoseOffset;
    for (int i = 0; i < 3; ++i) {
        output.pose.trans[i] = pose_src[i];
        output.pose.rot[i] = pose_src[3 + i];
        output.pose.trans_std[i] = std::exp(pose_src[6 + i]);
        output.pose.rot_std[i] = std::exp(pose_src[9 + i]);
    }

    return output;
}
