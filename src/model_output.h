#ifndef MODEL_OUTPUT_H
#define MODEL_OUTPUT_H

#include "calibration_online.h"

#include <array>
#include <vector>

constexpr int kTrajectorySize = 33;
constexpr int kLeadMhpSelection = 3;
constexpr int kLeadMhpN = 2;
constexpr int kLeadTrajLen = 6;
constexpr int kDesireLen = 8;
constexpr float kModelHeight = 1.22f;

/* openpilot v0.9.4 supercombo 출력 계약. SupercomboModel이 로드 시 이 값으로
 * kmodel을 검증하고, 파서가 꼬리 블록 오프셋을 여기서 역산한다. */
constexpr int kModelOutputFloats = 6120;
constexpr int kModelFeatureLen = 128;

/* raw 출력 6120 float의 블록 오프셋. 앞(plan/lane/edge/lead/meta)은 앞에서, 꼬리
 * (pose/feature)는 뒤에서 유도한다. meta 블록만 크기를 유도할 수 없어서(desire_state
 * 뒤에 openpilot의 disengage 확률과 desire_pred가 붙는다) 둘이 겹치지 않는지만 본다.
 * 꼬리 순서는 pose(12) / wide_from_device_euler(6) / sim_pose(12) / road_transform(12)
 * / feature(128) / pad(2). 모델을 바꾸면 이 표와 아래 static_assert만 손본다. */
namespace model_output_layout {
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
constexpr int kDesireStateOffset = kLeadProbOffset + kLeadMhpSelection;
constexpr int kPadFloats = 2;
constexpr int kRoadTransformFloats = 12;
constexpr int kSimPoseFloats = 12;
constexpr int kWideFromDeviceEulerFloats = 6;
constexpr int kPoseFloats = 12;
constexpr int kFeatureOffset = kModelOutputFloats - kPadFloats - kModelFeatureLen;
constexpr int kPoseOffset = kFeatureOffset - kRoadTransformFloats - kSimPoseFloats -
                            kWideFromDeviceEulerFloats - kPoseFloats;
static_assert(kPlanStride == 991 && kLaneOffset == 4955 && kLaneProbOffset == 5483 &&
                  kRoadEdgeOffset == 5491 && kLeadOffset == 5755 && kLeadProbOffset == 5857 &&
                  kDesireStateOffset == 5860 && kPoseOffset == 5948 && kFeatureOffset == 5990,
              "openpilot v0.9.4 supercombo output layout");
static_assert(kPoseOffset > kDesireStateOffset + kDesireLen, "pose block must follow the meta block");
}  // namespace model_output_layout

/* openpilot T_IDXS / X_IDXS 격자. 같은 식이 여러 파일에 재정의되지 않도록
 * 여기 한 벌만 둔다. */
inline double model_t_idx_double(int i)
{
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return 10.0 * t * t;
}

inline double model_x_idx_double(int i)
{
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return 192.0 * t * t;
}

inline float model_t_idx(int i) { return static_cast<float>(model_t_idx_double(i)); }
inline float model_x_idx(int i) { return static_cast<float>(model_x_idx_double(i)); }

struct ModelPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct ParsedPlan {
    bool valid = false;
    int best_index = 0;
    float probability = 0.0f;
    std::array<ModelPoint, kTrajectorySize> points{};
};

struct ParsedLaneLine {
    bool valid = false;
    float probability = 0.0f;
    float std = 0.0f;
    std::array<ModelPoint, kTrajectorySize> points{};
};

struct ParsedRoadEdge {
    bool valid = false;
    float std = 0.0f;
    std::array<ModelPoint, kTrajectorySize> points{};
};

struct ParsedMeta {
    std::array<float, kDesireLen> desire_state{};
};

struct ParsedLeadPoint {
    float x = 0.0f;
    float y = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
};

struct ParsedLeadPrediction {
    std::array<ParsedLeadPoint, kLeadTrajLen> points{};
    std::array<float, kLeadMhpSelection> probabilities{};
};

struct ParsedLeads {
    bool valid = false;
    std::array<ParsedLeadPrediction, kLeadMhpN> predictions{};
    std::array<float, kLeadMhpSelection> global_probabilities{};

    bool primary(int time_idx, float min_probability, ParsedLeadPoint *lead, float *probability = nullptr) const;
};

struct ParsedModelOutput {
    bool valid = false;
    ParsedPlan plan;
    std::array<ParsedLaneLine, 4> lanes{};
    std::array<ParsedRoadEdge, 2> road_edges{};
    ParsedLeads leads;
    ParsedMeta meta;
    bool has_pose = false;
    PoseObservation pose{};
};

class ModelOutputParser {
public:
    static ParsedModelOutput parse(const std::vector<float> &raw);

    static float sigmoid(float x);
};

#endif
