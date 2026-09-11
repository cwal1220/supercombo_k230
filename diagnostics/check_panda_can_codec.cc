#include "panda_can_codec.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const char *message)
{
    if (!condition) std::fprintf(stderr, "FAIL: %s\n", message);
    return condition;
}

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

} // namespace

int main()
{
    bool ok = true;

    std::vector<PandaCanFrame> frames = {
        make_frame(0x123, 2, {0, 1, 2, 3, 4, 5, 6, 7}),
        make_frame(0x18da10f1U, 1, {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x10, 0x20}),
    };

    std::vector<uint8_t> packed;
    std::string error;
    ok &= expect(panda_can_pack_buffer(frames, &packed, &error), "pack valid frames");
    ok &= expect(packed.size() >= 27, "packed size includes two CAN packets plus USB counters");

    const std::array<uint8_t, 14> expected_first = {
        0x00,  // USB counter
        0x84,  // dlc=8, bus=2
        0x18, 0x09, 0x00, 0x00,  // 0x123 << 3
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    };
    ok &= expect(std::equal(expected_first.begin(), expected_first.end(), packed.begin()),
                 "first header matches panda/python pack_can_buffer");

    std::vector<uint8_t> recv_buf;
    std::vector<PandaCanFrame> unpacked;
    ok &= expect(panda_can_unpack_buffer(packed.data(), static_cast<int>(packed.size()),
                                         &recv_buf, &unpacked, &error),
                 "unpack valid frames");
    ok &= expect(unpacked.size() == frames.size(), "round-trip frame count");
    for (size_t i = 0; i < unpacked.size() && i < frames.size(); ++i) {
        ok &= expect(equal_frame(unpacked[i], frames[i]), "round-trip frame content");
    }

    std::vector<PandaCanFrame> many;
    for (int i = 0; i < 40; ++i) {
        many.push_back(make_frame(0x340 + static_cast<uint32_t>(i), i % 4,
                                  {0, 1, 2, 3, 4, 5, 6, 7}));
    }
    packed.clear();
    ok &= expect(panda_can_pack_buffer(many, &packed, &error), "pack long stream");
    ok &= expect(packed.size() > 256, "long stream crosses panda soft chunk size");
    ok &= expect(panda_can_unpack_buffer(packed.data(), static_cast<int>(packed.size()),
                                         &recv_buf, &unpacked, &error),
                 "unpack long stream");
    ok &= expect(unpacked.size() == many.size(), "long stream frame count");

    std::vector<uint8_t> flagged = {
        0x00,
        0x80,                    // dlc=8, bus=0
        0x1b, 0x09, 0x00, 0x00,  // (0x123 << 3) | returned | rejected
        1, 2, 3, 4, 5, 6, 7, 8,
    };
    ok &= expect(panda_can_unpack_buffer(flagged.data(), static_cast<int>(flagged.size()),
                                         &recv_buf, &unpacked, &error),
                 "unpack returned/rejected flags");
    ok &= expect(unpacked.size() == 1 && unpacked[0].returned && unpacked[0].rejected,
                 "returned/rejected flags preserved");

    PandaCanFrame bad_bus = make_frame(0x123, 4, {0, 1, 2, 3, 4, 5, 6, 7});
    ok &= expect(!panda_can_pack_buffer({bad_bus}, &packed, &error), "reject invalid TX bus");
    PandaCanFrame bad_addr = make_frame(0x20000000U, 0, {0, 1, 2, 3, 4, 5, 6, 7});
    ok &= expect(!panda_can_pack_buffer({bad_addr}, &packed, &error), "reject invalid address");
    PandaCanFrame bad_len = make_frame(0x123, 0, {0, 1, 2, 3, 4, 5, 6, 7, 8});
    ok &= expect(!panda_can_pack_buffer({bad_len}, &packed, &error), "reject non-DLC length");

    packed.clear();
    ok &= expect(panda_can_pack_buffer({}, &packed, &error), "empty TX batch is a no-op");
    ok &= expect(packed.empty(), "empty TX batch produces no bytes");

    if (!ok) return 1;
    std::puts("check_panda_can_codec: ok");
    return 0;
}
