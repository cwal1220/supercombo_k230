#ifndef MODEL_OUTPUT_H
#define MODEL_OUTPUT_H

#include "online_calibrator.h"

#include <array>
#include <vector>

constexpr int kTrajectorySize = 33;
constexpr int kLeadMhpSelection = 3;
constexpr int kLeadMhpN = 2;
constexpr int kLeadTrajLen = 6;
constexpr float kModelHeight = 1.22f;

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
    std::array<ModelPoint, kTrajectorySize> points{};
};

struct ParsedRoadEdge {
    bool valid = false;
    std::array<ModelPoint, kTrajectorySize> points{};
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
    bool has_pose = false;
    PoseObservation pose{};
};

class ModelOutputParser {
public:
    static ParsedModelOutput parse(const std::vector<float> &raw);

    static float x_idx(int i);
    static float sigmoid(float x);
};

#endif
