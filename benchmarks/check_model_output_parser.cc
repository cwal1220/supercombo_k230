#include "model_output.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace {

constexpr size_t kOutputFloats = 2576;
constexpr size_t kPoseOffset = 87;
constexpr size_t kLaneOffset = 117;
constexpr size_t kLaneStdOffset = kLaneOffset + 4 * kTrajectorySize * 2;
constexpr size_t kLaneProbOffset = 645;
constexpr size_t kRoadEdgeOffset = 653;
constexpr size_t kRoadEdgeStdOffset = kRoadEdgeOffset + 2 * kTrajectorySize * 2;
constexpr size_t kLeadOffset = 917;
constexpr size_t kLeadProbOffset = 1061;
constexpr size_t kPlanOffset = 1576;
constexpr size_t kPlanStdOffset = kPlanOffset + kTrajectorySize * 15;
constexpr size_t kDesireStateOffset = 2566;

bool near(float actual, float expected, float tolerance = 1e-5f)
{
    return std::fabs(actual - expected) <= tolerance;
}

int self_test()
{
    std::vector<float> raw(kOutputFloats, 0.0f);
    for (int i = 0; i < kTrajectorySize; ++i)
        raw[kPlanOffset + i * 15] = static_cast<float>(i * 3);
    const int plan_point = 7;
    raw[kPlanOffset + plan_point * 15] = 17.0f;
    raw[kPlanOffset + plan_point * 15 + 1] = -1.25f;
    raw[kPlanOffset + plan_point * 15 + 11] = 0.12f;
    raw[kPlanStdOffset + plan_point * 15 + 1] = std::log(0.4f);

    const int lane = 2;
    raw[kLaneOffset + lane * kTrajectorySize * 2 + 5 * 2] = 2.5f;
    raw[kLaneStdOffset + lane * kTrajectorySize * 2] = std::log(0.2f);
    raw[kLaneProbOffset + lane * 2 + 1] = 1.5f;

    raw[kRoadEdgeOffset + kTrajectorySize * 2 + 3 * 2] = -3.0f;
    raw[kRoadEdgeStdOffset + kTrajectorySize * 2] = std::log(0.3f);

    const int lead_selection = 1;
    raw[kLeadOffset + lead_selection * kLeadTrajLen * 4] = 31.0f;
    raw[kLeadOffset + lead_selection * kLeadTrajLen * 4 + 1] = -0.5f;
    raw[kLeadProbOffset + lead_selection] = 3.0f;
    raw[kDesireStateOffset + 4] = 4.0f;

    for (int i = 0; i < 3; ++i) {
        raw[kPoseOffset + i] = 10.0f + i;
        raw[kPoseOffset + 3 + i] = 0.1f * i;
        raw[kPoseOffset + 6 + i] = std::log(0.01f * (i + 1));
        raw[kPoseOffset + 9 + i] = std::log(0.02f * (i + 1));
    }

    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    ParsedLeadPoint lead;
    const bool ok = parsed.valid && parsed.plan.valid && parsed.plan.best_index == 0 &&
        near(parsed.plan.probability, 1.0f) &&
        near(parsed.plan.points[plan_point].x, 17.0f) &&
        near(parsed.plan.points[plan_point].y, -1.25f) &&
        near(parsed.plan.orientations[plan_point].z, 0.12f) &&
        near(parsed.plan.position_stds[plan_point].y, 0.4f) &&
        near(parsed.lanes[lane].points[5].y, 2.5f) &&
        near(parsed.lanes[lane].std, 0.2f) &&
        parsed.lanes[lane].probability > 0.8f &&
        near(parsed.road_edges[1].points[3].y, -3.0f) &&
        near(parsed.road_edges[1].std, 0.3f) &&
        parsed.leads.primary(lead_selection, 0.0f, &lead) &&
        near(lead.x, 31.0f) && near(lead.y, -0.5f) &&
        !parsed.stop_line.valid &&
        parsed.meta.desire_state[4] > parsed.meta.desire_state[0] &&
        parsed.has_pose && near(parsed.pose.trans[2], 12.0f) &&
        near(parsed.pose.trans_std[2], 0.03f);
    if (!ok) {
        std::cerr << "modern model output layout self-test failed\n";
        return 1;
    }

    std::vector<float> short_raw(kOutputFloats - 1, 0.0f);
    if (ModelOutputParser::parse(short_raw).valid) {
        std::cerr << "short output was accepted\n";
        return 1;
    }
    raw[0] = std::numeric_limits<float>::quiet_NaN();
    if (ModelOutputParser::parse(raw).valid) {
        std::cerr << "non-finite output was accepted\n";
        return 1;
    }

    std::cout << "MODEL_OUTPUT_EQUIVALENCE_OK output=2576 hidden_offset=1064 plan_offset=1576\n";
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
    if (!read_exact(file, &raw_size) || !read_exact(file, &frame_count) ||
        raw_size != kOutputFloats || frame_count == 0) {
        std::cerr << "bad modern raw dump header\n";
        return 1;
    }

    std::vector<float> raw(raw_size);
    ParsedModelOutput parsed;
    for (uint32_t frame = 0; frame < frame_count; ++frame) {
        file.read(reinterpret_cast<char *>(raw.data()),
                  static_cast<std::streamsize>(raw.size() * sizeof(float)));
        if (file.gcount() != static_cast<std::streamsize>(raw.size() * sizeof(float))) {
            std::cerr << "short frame " << frame << "\n";
            return 1;
        }
        parsed = ModelOutputParser::parse(raw);
        if (!parsed.valid || !parsed.plan.valid || !parsed.has_pose) {
            std::cerr << "invalid parsed frame " << frame << "\n";
            return 1;
        }
    }

    ParsedLeadPoint lead;
    const bool have_lead = parsed.leads.primary(0, 0.0f, &lead);
    std::cout << "raw_size=" << raw_size << " frames=" << frame_count
              << " plan_prob=" << parsed.plan.probability
              << " lane0_prob=" << parsed.lanes[0].probability
              << " pose_vx=" << parsed.pose.trans[0]
              << " lead_valid=" << (have_lead ? 1 : 0);
    if (have_lead) std::cout << " lead_x=" << lead.x << " lead_y=" << lead.y;
    std::cout << "\n";
    return 0;
}
