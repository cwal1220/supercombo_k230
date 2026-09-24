/* panda USB CAN 버퍼 코덱: panda python의 pack_can_buffer와 같은 바이트로 싸는지,
 * 소프트 청크(256 B)를 넘는 스트림까지 왕복하는지, 범위 밖 TX를 거부하는지 본다. */
#include "panda_can_codec.h"

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string>
#include <vector>

namespace {

PandaCanFrame make_frame(uint32_t address, uint8_t bus, const std::vector<uint8_t> &data)
{
    PandaCanFrame frame;
    frame.address = address;
    frame.bus = bus;
    frame.data_len = static_cast<uint8_t>(data.size());
    if (!data.empty()) std::memcpy(frame.data, data.data(), data.size());
    return frame;
}

bool equal_frame(const PandaCanFrame &a, const PandaCanFrame &b)
{
    return a.address == b.address &&
           a.bus == b.bus &&
           a.data_len == b.data_len &&
           a.returned == b.returned &&
           a.rejected == b.rejected &&
           std::memcmp(a.data, b.data, a.data_len) == 0;
}

TEST(PandaCanCodec, PackUnpackRoundTrip) {
    std::vector<PandaCanFrame> frames = {
        make_frame(0x123, 2, {0, 1, 2, 3, 4, 5, 6, 7}),
        make_frame(0x18da10f1U, 1, {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x10, 0x20}),
    };

    std::vector<uint8_t> packed;
    std::string error;
    EXPECT_TRUE(panda_can_pack_buffer(frames, &packed, &error)) << error;
    EXPECT_GE(packed.size(), 27u) << "CAN 패킷 두 개와 USB 카운터";

    const std::array<uint8_t, 14> expected_first = {
        0x00,  // USB 카운터
        0x84,  // dlc=8, bus=2
        0x18, 0x09, 0x00, 0x00,  // 0x123 << 3
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    };
    EXPECT_TRUE(std::equal(expected_first.begin(), expected_first.end(), packed.begin()))
        << "첫 헤더가 panda python의 pack_can_buffer와 같다";

    std::vector<uint8_t> recv_buf;
    std::vector<PandaCanFrame> unpacked;
    EXPECT_TRUE(panda_can_unpack_buffer(packed.data(), static_cast<int>(packed.size()),
                                        &recv_buf, &unpacked, &error)) << error;
    EXPECT_EQ(unpacked.size(), frames.size());
    for (size_t i = 0; i < unpacked.size() && i < frames.size(); ++i) {
        EXPECT_TRUE(equal_frame(unpacked[i], frames[i])) << "왕복한 프레임 " << i;
    }

    std::vector<PandaCanFrame> many;
    for (int i = 0; i < 40; ++i) {
        many.push_back(make_frame(0x340 + static_cast<uint32_t>(i), i % 4,
                                  {0, 1, 2, 3, 4, 5, 6, 7}));
    }
    packed.clear();
    EXPECT_TRUE(panda_can_pack_buffer(many, &packed, &error)) << error;
    EXPECT_GT(packed.size(), 256u) << "긴 스트림은 panda 소프트 청크 크기를 넘는다";
    EXPECT_TRUE(panda_can_unpack_buffer(packed.data(), static_cast<int>(packed.size()),
                                        &recv_buf, &unpacked, &error)) << error;
    EXPECT_EQ(unpacked.size(), many.size());

    std::vector<uint8_t> flagged = {
        0x00,
        0x80,                    // dlc=8, bus=0
        0x1b, 0x09, 0x00, 0x00,  // (0x123 << 3) | returned | rejected
        1, 2, 3, 4, 5, 6, 7, 8,
    };
    EXPECT_TRUE(panda_can_unpack_buffer(flagged.data(), static_cast<int>(flagged.size()),
                                        &recv_buf, &unpacked, &error)) << error;
    ASSERT_EQ(unpacked.size(), 1u);
    EXPECT_TRUE(unpacked[0].returned);
    EXPECT_TRUE(unpacked[0].rejected);

    PandaCanFrame bad_bus = make_frame(0x123, 4, {0, 1, 2, 3, 4, 5, 6, 7});
    EXPECT_FALSE(panda_can_pack_buffer({bad_bus}, &packed, &error)) << "TX 버스 범위 밖";
    PandaCanFrame bad_addr = make_frame(0x20000000U, 0, {0, 1, 2, 3, 4, 5, 6, 7});
    EXPECT_FALSE(panda_can_pack_buffer({bad_addr}, &packed, &error)) << "주소 범위 밖";
    PandaCanFrame bad_len = make_frame(0x123, 0, {0, 1, 2, 3, 4, 5, 6, 7, 8});
    EXPECT_FALSE(panda_can_pack_buffer({bad_len}, &packed, &error)) << "DLC에 없는 길이";

    packed.clear();
    EXPECT_TRUE(panda_can_pack_buffer({}, &packed, &error)) << "빈 TX 묶음은 아무것도 하지 않는다";
    EXPECT_TRUE(packed.empty()) << "빈 TX 묶음은 바이트를 만들지 않는다";
}

} // namespace
