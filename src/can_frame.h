#pragma once

/* CAN 전송 단위와 K7 YG HEV 주소·버스 표. 수신 디코드(vehicle_can)와 송신 인코드
 * (hyundai_can)가 같은 표를 본다. */

#include <array>
#include <cstdint>

constexpr uint32_t kHyundaiSas11Address = 688;       // 0x2b0
constexpr uint32_t kHyundaiEsp12Address = 544;       // 0x220
constexpr uint32_t kHyundaiWhlSpd11Address = 902;    // 0x386
constexpr uint32_t kHyundaiScc11Address = 1056;      // 0x420
constexpr uint32_t kHyundaiScc12Address = 1057;      // 0x421
constexpr uint32_t kHyundaiTcs13Address = 916;       // 0x394
constexpr uint32_t kHyundaiTcs15Address = 1287;      // 0x507
constexpr uint32_t kHyundaiEEms11Address = 881;      // 0x371
constexpr uint32_t kHyundaiElectGearAddress = 882;   // 0x372
constexpr uint32_t kHyundaiCgw1Address = 1345;       // 0x541
constexpr uint32_t kHyundaiCgw2Address = 1363;       // 0x553
constexpr uint32_t kHyundaiLca11Address = 1419;      // 0x58b
constexpr uint32_t kHyundaiTpms11Address = 1427;     // 0x593

constexpr uint32_t kHyundaiLkas11Address = 832;   // 0x340
constexpr uint32_t kHyundaiClu11Address = 1265;   // 0x4f1
constexpr uint32_t kHyundaiMdps12Address = 593;   // 0x251
constexpr uint8_t kHyundaiMdps12TxBus = 2;

constexpr uint8_t kPowertrainBus = 0;
constexpr uint8_t kMdpsBus = 1;
constexpr uint8_t kCameraBus = 2;

struct CanFrame {
  uint32_t address = 0;
  uint8_t bus = 0;
  uint8_t length = 0;
  std::array<uint8_t, 8> data = {};
};

// 리틀엔디언 비트 필드 읽기/쓰기. 디코더와 인코더가 같은 비트 규약을 쓴다.
inline uint32_t get_signal_le(const uint8_t *data, int start_bit, int length)
{
    uint32_t raw = 0;
    for (int i = 0; i < length; ++i) {
        const int bit = start_bit + i;
        if (data[bit / 8] & (1U << (bit % 8)))
            raw |= 1U << i;
    }
    return raw;
}

inline void set_signal_le(std::array<uint8_t, 8> *data, int start_bit, int length, uint32_t raw) {
  for (int i = 0; i < length; ++i) {
    const int bit = start_bit + i;
    const int byte_index = bit / 8;
    const int bit_index = bit % 8;
    const uint8_t mask = static_cast<uint8_t>(1U << bit_index);
    if (raw & (1U << i)) {
      (*data)[byte_index] |= mask;
    } else {
      (*data)[byte_index] &= static_cast<uint8_t>(~mask);
    }
  }
}
