#include "model_output.h"

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
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
constexpr int kDesireStateOffset = kLeadProbOffset + kLeadMhpSelection;

bool near(float actual, float expected, float tolerance = 1e-6f)
{
    return std::fabs(actual - expected) <= tolerance;
}

/* openpilot v0.9.4 출력 레이아웃 검사. */
int self_test_094()
{
    // 파서와 같은 방식으로 꼬리에서 역산한다: pose(12) / wide_from_device_euler(6)
    // / sim_pose(12) / road_transform(12) / feature(128) / pad(2).
    constexpr int kPoseOffset094 = kModelOutputFloats - 2 - kModelFeatureLen - 12 - 12 - 6 - 12;
    static_assert(kPoseOffset094 == 5948, "v0.9.4 pose offset moved");
    std::vector<float> raw(kModelOutputFloats, 0.0f);

    for (int plan = 0; plan < 5; ++plan)
        raw[plan * kPlanStride + kPlanStride - 1] = static_cast<float>(4 - plan);
    const int plan_base = 0;
    raw[plan_base + 7 * 15 + 0] = 23.0f;
    raw[plan_base + 7 * 15 + 1] = 0.75f;

    const int lane = 1;
    raw[kLaneOffset + lane * kTrajectorySize * 2 + 5 * 2] = -1.75f;
    raw[kLaneProbOffset + lane * 2 + 1] = 2.0f;
    raw[kLeadOffset] = 42.0f;
    raw[kLeadProbOffset] = 3.0f;

    raw[kDesireStateOffset + 3] = 5.0f;
    for (int i = 0; i < 3; ++i) {
        raw[kPoseOffset094 + i] = 20.0f + i;
        raw[kPoseOffset094 + 6 + i] = std::log(0.05f);
    }
    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    ParsedLeadPoint lead;
    const bool ok = parsed.valid && parsed.plan.best_index == 0 &&
        near(parsed.plan.points[7].x, 23.0f) &&
        near(parsed.plan.points[7].y, 0.75f) &&
        near(parsed.lanes[1].points[5].y, -1.75f) &&
        parsed.leads.primary(0, 0.0f, &lead) && near(lead.x, 42.0f) &&
        parsed.meta.desire_state[3] > parsed.meta.desire_state[0] &&
        parsed.has_pose && near(parsed.pose.trans[2], 22.0f) &&
        near(parsed.pose.trans_std[0], 0.05f);
    if (!ok) {
        std::cerr << "v0.9.4 model output layout self-test failed\n";
        return 1;
    }

    std::cout << "MODEL_OUTPUT_094_OK output=" << kModelOutputFloats
              << " feature=" << kModelFeatureLen
              << " pose_offset=" << kPoseOffset094 << "\n";
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
    if (argc == 1) return self_test_094();
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
    std::cout << "\n";
    return 0;
}
