#ifndef PANDA_CAN_FRAME_H
#define PANDA_CAN_FRAME_H

#include <cstdint>

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

#endif
