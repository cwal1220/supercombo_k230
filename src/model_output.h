#ifndef MODEL_OUTPUT_H
#define MODEL_OUTPUT_H

#include "online_calibrator.h"

#include <array>
#include <vector>

constexpr int kTrajectorySize = 33;
constexpr int kLeadMhpSelection = 3;
constexpr int kLeadMhpN = 2;
constexpr int kLeadTrajLen = 6;
constexpr int kDesireLen = 8;
constexpr float kModelHeight = 1.22f;

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
    std::array<ModelPoint, kTrajectorySize> position_stds{};
    std::array<ModelPoint, kTrajectorySize> orientations{};
    std::array<ModelPoint, kTrajectorySize> orientation_rates{};
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

struct ParsedStopLine {
    bool valid = false;
    int best_index = 0;
    float probability = 0.0f;
    ModelPoint position;
    ModelPoint rotation;
    float speed = 0.0f;
    float time = 0.0f;
};

struct ParsedModelOutput {
    bool valid = false;
    ParsedPlan plan;
    std::array<ParsedLaneLine, 4> lanes{};
    std::array<ParsedRoadEdge, 2> road_edges{};
    ParsedLeads leads;
    ParsedStopLine stop_line;
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
