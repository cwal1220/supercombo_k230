#include "model_output.h"

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace {

constexpr int kPlanStride = kTrajectorySize * 15 * 2 + 1;
constexpr int kLaneOffset = 5 * kPlanStride;
constexpr int kLaneLineSize = 4 * kTrajectorySize * 2;
constexpr int kLaneProbOffset = kLaneOffset + kLaneLineSize * 2;
constexpr int kRoadEdgeOffset = kLaneProbOffset + 8;
constexpr int kRoadEdgeMeanSize = 2 * kTrajectorySize * 2;
constexpr int kLeadOffset = kRoadEdgeOffset + kRoadEdgeMeanSize * 2;
constexpr int kLeadStride = kLeadTrajLen * 4 * 2 + kLeadMhpSelection;
constexpr int kLeadProbOffset = kLeadOffset + kLeadMhpN * kLeadStride;
constexpr int kStopLineOffset = kLeadProbOffset + kLeadMhpSelection;
constexpr int kStopLineStride = 17;
constexpr int kStopLineSize = 3 * kStopLineStride + 1;
constexpr int kMetaOffset = kStopLineOffset + kStopLineSize;
constexpr int kPoseOffset = 6000;

bool near(float actual, float expected, float tolerance = 1e-6f)
{
    return std::fabs(actual - expected) <= tolerance;
}

int self_test()
{
    std::vector<float> raw(6012 + 512, 0.0f);
    for (int plan = 0; plan < 5; ++plan)
        raw[plan * kPlanStride + kPlanStride - 1] = static_cast<float>(plan);
    const int plan_base = 4 * kPlanStride;
    raw[plan_base + 7 * 15 + 0] = 17.0f;
    raw[plan_base + 7 * 15 + 1] = -1.25f;
    raw[plan_base + 7 * 15 + 9 + 2] = 0.12f;
    raw[plan_base + kTrajectorySize * 15 + 7 * 15 + 1] = std::log(0.4f);

    const int lane = 2;
    raw[kLaneOffset + lane * kTrajectorySize * 2 + 5 * 2] = 2.5f;
    raw[kLaneOffset + kLaneLineSize + lane * kTrajectorySize * 2] = std::log(0.2f);
    raw[kLaneProbOffset + lane * 2 + 1] = 1.5f;

    raw[kRoadEdgeOffset + kTrajectorySize * 2 + 3 * 2] = -3.0f;
    raw[kRoadEdgeOffset + kRoadEdgeMeanSize + kTrajectorySize * 2] = std::log(0.3f);

    raw[kLeadOffset + 1 * kLeadStride] = 31.0f;
    raw[kLeadOffset + 1 * kLeadStride + kLeadStride - kLeadMhpSelection] = 2.0f;
    raw[kLeadProbOffset] = 3.0f;
    const int stop_line_base = kStopLineOffset + 2 * kStopLineStride;
    raw[stop_line_base] = 6.5f;
    raw[stop_line_base + 1] = -0.2f;
    raw[stop_line_base + 6] = 0.1f;
    raw[stop_line_base + 7] = 1.4f;
    raw[stop_line_base + kStopLineStride - 1] = 3.0f;
    raw[kStopLineOffset + kStopLineSize - 1] = 2.0f;
    raw[kMetaOffset + 4] = 4.0f;
    for (int i = 0; i < 3; ++i) {
        raw[kPoseOffset + i] = 10.0f + i;
        raw[kPoseOffset + 3 + i] = 0.1f * i;
        raw[kPoseOffset + 6 + i] = std::log(0.01f * (i + 1));
        raw[kPoseOffset + 9 + i] = std::log(0.02f * (i + 1));
    }

    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    const ParsedModelOutput truncated =
        ModelOutputParser::parse(std::vector<float>(5600, 0.0f));
    std::vector<float> malformed_raw = raw;
    malformed_raw[0] = std::numeric_limits<float>::quiet_NaN();
    const ParsedModelOutput malformed = ModelOutputParser::parse(malformed_raw);
    ParsedLeadPoint lead;
    const bool ok = parsed.valid && parsed.plan.best_index == 4 &&
        !truncated.valid && !malformed.valid &&
        near(parsed.plan.points[7].x, 17.0f) &&
        near(parsed.plan.points[7].y, -1.25f) &&
        near(parsed.plan.orientations[7].z, 0.12f) &&
        near(parsed.plan.position_stds[7].y, 0.4f) &&
        near(parsed.lanes[2].points[5].y, 2.5f) &&
        near(parsed.lanes[2].std, 0.2f) &&
        near(parsed.road_edges[1].points[3].y, -3.0f) &&
        near(parsed.road_edges[1].std, 0.3f) &&
        parsed.leads.primary(0, 0.0f, &lead) && near(lead.x, 31.0f) &&
        parsed.stop_line.valid && parsed.stop_line.best_index == 2 &&
        near(parsed.stop_line.position.x, 6.5f) &&
        near(parsed.stop_line.position.y, -0.2f) &&
        near(parsed.stop_line.speed, 0.1f) &&
        near(parsed.stop_line.time, 1.4f) &&
        parsed.stop_line.probability > 0.8f &&
        parsed.meta.desire_state[4] > parsed.meta.desire_state[0] &&
        parsed.has_pose && near(parsed.pose.trans[2], 12.0f) &&
        near(parsed.pose.trans_std[2], 0.03f);
    if (!ok) {
        std::cerr << "model output layout self-test failed\n";
        return 1;
    }
    std::cout << "MODEL_OUTPUT_EQUIVALENCE_OK output=6012 recurrent=512 pose_offset=6000\n";
    return 0;
}

template <typename T>
bool read_exact(std::ifstream &file, T *value)
{
    file.read(reinterpret_cast<char *>(value), sizeof(T));
    return file.gcount() == static_cast<std::streamsize>(sizeof(T));
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc == 1) return self_test();
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <SCODMP1 raw dump>\n";
        return 1;
    }

    std::ifstream file(argv[1], std::ios::binary);
    if (!file) {
        std::perror("open raw dump");
        return 1;
    }

    char magic[8]{};
    file.read(magic, sizeof(magic));
    if (file.gcount() != static_cast<std::streamsize>(sizeof(magic)) ||
        std::memcmp(magic, "SCODMP1", 7) != 0) {
        std::cerr << "bad raw dump magic\n";
        return 1;
    }

    uint32_t raw_size = 0;
    uint32_t frame_count = 0;
    if (!read_exact(file, &raw_size) || !read_exact(file, &frame_count) || raw_size == 0) {
        std::cerr << "bad raw dump header\n";
        return 1;
    }

    std::vector<float> raw(raw_size);
    file.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(raw.size() * sizeof(float)));
    if (file.gcount() != static_cast<std::streamsize>(raw.size() * sizeof(float))) {
        std::cerr << "short first frame\n";
        return 1;
    }

    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    if (!parsed.valid || !parsed.plan.valid || !parsed.has_pose) {
        std::cerr << "parser did not produce required plan/pose outputs\n";
        return 1;
    }

    ParsedLeadPoint lead;
    const bool have_lead = parsed.leads.primary(0, 0.0f, &lead);
    std::cout << "raw_size=" << raw_size
              << " frames=" << frame_count
              << " plan_best=" << parsed.plan.best_index
              << " plan_prob=" << parsed.plan.probability
              << " lane0_prob=" << parsed.lanes[0].probability
              << " pose_vx=" << parsed.pose.trans[0]
              << " lead_valid=" << (have_lead ? 1 : 0);
    if (have_lead)
        std::cout << " lead_x=" << lead.x << " lead_y=" << lead.y;
    if (parsed.stop_line.valid)
        std::cout << " stopline_x=" << parsed.stop_line.position.x
                  << " stopline_prob=" << parsed.stop_line.probability;
    std::cout << "\n";
    return 0;
}
