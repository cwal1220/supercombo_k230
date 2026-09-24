/* supercombo(openpilot v0.9.4) raw 출력 파서와 시간축 입력 규약(desire 펄스, 특징 이력).
 * SCODMP1 덤프(SUPERCOMBO_RAW_DUMP)를 인자로 주면 테스트 대신 첫 프레임을 파싱해 출력한다. */
#include "model_output.h"
#include "model_temporal.h"

#include <gtest/gtest.h>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

namespace {

using namespace model_output_layout;

/* openpilot v0.9.4 출력 레이아웃 검사. */
TEST(ModelOutputParser, Model094)
{
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
        raw[kPoseOffset + i] = 20.0f + i;
        raw[kPoseOffset + 6 + i] = std::log(0.05f);
    }
    const ParsedModelOutput parsed = ModelOutputParser::parse(raw);
    ParsedLeadPoint lead;
    ASSERT_TRUE(parsed.valid);
    ASSERT_EQ(parsed.plan.best_index, 0);
    ASSERT_NEAR(parsed.plan.points[7].x, 23.0f, 1e-6f);
    ASSERT_NEAR(parsed.plan.points[7].y, 0.75f, 1e-6f);
    ASSERT_NEAR(parsed.lanes[1].points[5].y, -1.75f, 1e-6f);
    ASSERT_TRUE(parsed.leads.primary(0, 0.0f, &lead));
    ASSERT_NEAR(lead.x, 42.0f, 1e-6f);
    ASSERT_GT(parsed.meta.desire_state[3], parsed.meta.desire_state[0]);
    ASSERT_TRUE(parsed.has_pose);
    ASSERT_NEAR(parsed.pose.trans[2], 22.0f, 1e-6f);
    ASSERT_NEAR(parsed.pose.trans_std[0], 0.05f, 1e-6f);
}

/* 시간축 입력 규약: rising-edge desire 펄스, 100틱 이력 밀기, hidden_state
 * 128개가 특징 버퍼 마지막 슬롯에 들어가고 다음 틱에 한 칸 물러난다. */
TEST(ModelOutputParser, Temporal)
{
    static_assert(SupercomboTemporalState::kHiddenOffset == 5990, "hidden state offset moved");
    SupercomboTemporalState state;
    const auto &desire = state.desire_history();
    const size_t last = (SupercomboTemporalState::kDesireHistoryTicks - 1) * kDesireLen;
    state.set_desire(3);
    state.push_desire_pulse();
    // 새 desire는 제 칸에 펄스가 뜨고 0번 칸에는 뜨지 않는다
    ASSERT_EQ(desire[last + 3], 1.0f);
    ASSERT_EQ(desire[last + 0], 0.0f);
    state.set_desire(3);
    state.push_desire_pulse();
    // 유지된 desire는 펄스가 한 번만 뜨고 그 펄스는 한 틱 뒤로 밀린다
    ASSERT_EQ(desire[last + 3], 0.0f);
    ASSERT_EQ(desire[last - kDesireLen + 3], 1.0f);
    state.set_desire(0);
    state.set_desire(3);
    state.push_desire_pulse();
    ASSERT_EQ(desire[last + 3], 1.0f) << "desire를 놓았다 다시 요청하면 펄스가 다시 뜬다";

    const auto &features = state.feature_history();
    const size_t newest = (SupercomboTemporalState::kFeatureHistoryTicks - 1) * kModelFeatureLen;
    std::vector<float> raw(kModelOutputFloats, 0.0f);
    for (int i = 0; i < kModelFeatureLen; ++i)
        raw[SupercomboTemporalState::kHiddenOffset + i] = static_cast<float>(i + 1);
    ASSERT_TRUE(state.push_feature_history(raw.data(), raw.size()))
        << "raw 출력 전체가 특징 버퍼로 들어간다";
    // hidden_state는 가장 새 특징 칸에 들어간다
    ASSERT_EQ(features[newest], 1.0f);
    ASSERT_EQ(features[newest + kModelFeatureLen - 1], 128.0f);
    std::vector<float> zeros(kModelOutputFloats, 0.0f);
    ASSERT_TRUE(state.push_feature_history(zeros.data(), zeros.size())) << "두 번째 프레임";
    // 이전 프레임의 특징은 한 칸 뒤로 밀린다
    ASSERT_EQ(features[newest], 0.0f);
    ASSERT_EQ(features[newest - kModelFeatureLen], 1.0f);
    ASSERT_FALSE(state.push_feature_history(raw.data(), 100)) << "짧은 raw 출력은 거부한다";
    // 상수 입력은 v0.9.4 값 그대로다
    ASSERT_EQ(state.traffic_convention(), (std::vector<float>{1.0f, 0.0f}));
    ASSERT_EQ(state.nav_features().size(), SupercomboTemporalState::kNavFeatureLen);
}

template <typename T>
bool read_exact(std::ifstream &file, T *value)
{
    file.read(reinterpret_cast<char *>(value), sizeof(T));
    return file.gcount() == static_cast<std::streamsize>(sizeof(T));
}

} // namespace

// 인자가 없으면 테스트, SCODMP1 덤프 경로를 주면 첫 프레임을 파싱해 출력한다.
int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    if (argc == 1) return RUN_ALL_TESTS();
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " [SCODMP1 raw dump]\n";
        return 2;
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
