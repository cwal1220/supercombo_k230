#ifndef PANDA_CAN_CODEC_H
#define PANDA_CAN_CODEC_H

#include <cstddef>
#include <cstdint>
#include <vector>


constexpr uint32_t kPandaCanMaxAddress = 0x1fffffffU;
constexpr uint8_t kPandaCanMaxTxBus = 3U;
constexpr uint8_t kPandaCanMaxDataLen = 64U;

struct PandaCanFrame {
    uint32_t address = 0;
    uint8_t bus = 0;
    uint8_t data_len = 0;
    uint8_t data[kPandaCanMaxDataLen] = {};
    bool returned = false;
    bool rejected = false;
};


#include <cstdint>
#include <string>
#include <vector>

bool panda_can_is_valid_data_len(uint8_t len);
bool panda_can_pack_buffer(const std::vector<PandaCanFrame> &frames,
                           std::vector<uint8_t> *out,
                           std::string *error = nullptr);
bool panda_can_unpack_buffer(const uint8_t *data, int size,
                             std::vector<uint8_t> *recv_buf,
                             std::vector<PandaCanFrame> *frames,
                             std::string *error = nullptr);

#endif
