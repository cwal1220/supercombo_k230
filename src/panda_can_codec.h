#ifndef PANDA_CAN_CODEC_H
#define PANDA_CAN_CODEC_H

#include "panda_can_frame.h"

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
