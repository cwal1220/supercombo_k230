#include "panda_can_codec.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>

namespace {

constexpr int kUsbPacketMaxSize = 0x40;
constexpr int kCanPacketHeadSize = 5;

constexpr uint8_t kDlcToLen[16] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 12, 16, 20, 24, 32, 48, 64,
};

void set_error(std::string *error, const char *message)
{
    if (error) *error = message;
}

uint8_t len_to_dlc(uint8_t len)
{
    for (uint8_t dlc = 0; dlc < 16; ++dlc) {
        if (kDlcToLen[dlc] == len) return dlc;
    }
    return 0xff;
}

uint32_t read_le32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

void write_le32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xffU);
    data[1] = static_cast<uint8_t>((value >> 8) & 0xffU);
    data[2] = static_cast<uint8_t>((value >> 16) & 0xffU);
    data[3] = static_cast<uint8_t>((value >> 24) & 0xffU);
}

void append_packet_bytes(std::vector<uint8_t> *dest, const uint8_t *src, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        if ((dest->size() % kUsbPacketMaxSize) == 0) {
            dest->push_back(static_cast<uint8_t>(dest->size() / kUsbPacketMaxSize));
        }
        dest->push_back(src[i]);
    }
}

} // namespace

bool panda_can_is_valid_data_len(uint8_t len)
{
    return len_to_dlc(len) != 0xff;
}

bool panda_can_pack_buffer(const std::vector<PandaCanFrame> &frames,
                           std::vector<uint8_t> *out,
                           std::string *error)
{
    if (!out) {
        set_error(error, "output buffer is null");
        return false;
    }
    out->clear();

    for (const PandaCanFrame &frame : frames) {
        if (frame.address > kPandaCanMaxAddress) {
            set_error(error, "CAN address exceeds panda 29-bit address range");
            return false;
        }
        if (frame.bus > kPandaCanMaxTxBus) {
            set_error(error, "CAN bus is outside panda TX bus range");
            return false;
        }
        if (frame.returned || frame.rejected) {
            set_error(error, "returned/rejected CAN flags are RX-only");
            return false;
        }

        const uint8_t dlc = len_to_dlc(frame.data_len);
        if (dlc == 0xff) {
            set_error(error, "CAN data length is not representable by panda DLC");
            return false;
        }

        uint8_t header[kCanPacketHeadSize] = {};
        const uint8_t extended = frame.address >= 0x800U ? 1U : 0U;
        const uint32_t word_4b = (frame.address << 3) | (extended << 2);
        header[0] = static_cast<uint8_t>((dlc << 4) | (frame.bus << 1));
        write_le32(&header[1], word_4b);

        append_packet_bytes(out, header, sizeof(header));
        append_packet_bytes(out, frame.data, frame.data_len);
    }
    return true;
}

bool panda_can_unpack_buffer(const uint8_t *data, int size,
                             std::vector<uint8_t> *recv_buf,
                             std::vector<PandaCanFrame> *frames,
                             std::string *error)
{
    if (!data || !recv_buf || !frames) {
        set_error(error, "null unpack argument");
        return false;
    }
    if (size < 0) {
        set_error(error, "negative USB buffer size");
        return false;
    }

    recv_buf->clear();
    frames->clear();
    for (int i = 0; i < size; i += kUsbPacketMaxSize) {
        const uint8_t expected_counter = static_cast<uint8_t>(i / kUsbPacketMaxSize);
        if (data[i] != expected_counter) {
            set_error(error, "malformed CAN USB packet counter");
            return false;
        }
        const int chunk_len = std::min(kUsbPacketMaxSize, size - i);
        recv_buf->insert(recv_buf->end(), &data[i + 1], &data[i + chunk_len]);
    }

    size_t pos = 0;
    while (pos < recv_buf->size()) {
        if (pos + kCanPacketHeadSize > recv_buf->size()) {
            set_error(error, "truncated CAN packet header");
            return false;
        }

        const uint8_t header0 = (*recv_buf)[pos];
        const uint8_t dlc = (header0 >> 4) & 0xfU;
        const uint8_t data_len = kDlcToLen[dlc];
        if (pos + kCanPacketHeadSize + data_len > recv_buf->size()) {
            set_error(error, "truncated CAN packet payload");
            return false;
        }

        const uint32_t word_4b = read_le32(recv_buf->data() + pos + 1);
        PandaCanFrame frame;
        frame.bus = (header0 >> 1) & 0x7U;
        frame.address = word_4b >> 3;
        frame.rejected = (word_4b & 0x1U) != 0;
        frame.returned = (word_4b & 0x2U) != 0;
        frame.data_len = data_len;
        std::copy_n(recv_buf->data() + pos + kCanPacketHeadSize, data_len, frame.data);
        frames->push_back(frame);

        pos += kCanPacketHeadSize + data_len;
    }
    return true;
}
