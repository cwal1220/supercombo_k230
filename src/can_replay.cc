#include "can_replay.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace {

constexpr std::array<uint8_t, 8> kFixtureMagic = {
    'K', '2', '3', '0', 'C', 'A', 'N', '1'};
constexpr uint32_t kFixtureVersion = 1;
constexpr uint32_t kFixtureRecordSize = 24;
constexpr size_t kFixtureHeaderSize = 24;

uint32_t read_u32_le(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8u) |
         (static_cast<uint32_t>(data[2]) << 16u) |
         (static_cast<uint32_t>(data[3]) << 24u);
}

uint64_t read_u64_le(const uint8_t *data) {
  return static_cast<uint64_t>(read_u32_le(data)) |
         (static_cast<uint64_t>(read_u32_le(data + 4)) << 32u);
}

}  // namespace

// K230CAN1 binary fixture를 읽고 구조와 시간 순서를 검증한다.
void CanReplaySource::open(const std::string &path) {
  records_.clear();
  next_record_ = 0;

  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("failed to open CAN replay fixture: " + path);
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                                   std::istreambuf_iterator<char>());
  if (bytes.size() < kFixtureHeaderSize ||
      !std::equal(kFixtureMagic.begin(), kFixtureMagic.end(), bytes.begin())) {
    throw std::runtime_error("invalid CAN replay fixture magic");
  }
  const uint32_t version = read_u32_le(bytes.data() + 8);
  const uint32_t record_size = read_u32_le(bytes.data() + 12);
  const uint64_t record_count = read_u64_le(bytes.data() + 16);
  if (version != kFixtureVersion || record_size != kFixtureRecordSize) {
    throw std::runtime_error("unsupported CAN replay fixture version");
  }
  if (record_count > (std::numeric_limits<size_t>::max() - kFixtureHeaderSize) /
                         kFixtureRecordSize ||
      bytes.size() != kFixtureHeaderSize + static_cast<size_t>(record_count) *
                                           kFixtureRecordSize) {
    throw std::runtime_error("invalid CAN replay fixture size");
  }

  records_.reserve(static_cast<size_t>(record_count));
  uint64_t previous_timestamp = 0;
  for (size_t index = 0; index < static_cast<size_t>(record_count); ++index) {
    const uint8_t *record = bytes.data() + kFixtureHeaderSize + index * kFixtureRecordSize;
    TimedCanFrame timed;
    timed.timestamp_us = read_u64_le(record);
    timed.frame.address = read_u32_le(record + 8);
    timed.frame.bus = record[12];
    timed.frame.length = record[13];
    std::copy_n(record + 14, timed.frame.data.size(), timed.frame.data.begin());
    if ((index > 0 && timed.timestamp_us < previous_timestamp) ||
        timed.frame.address > 0x1fffffffU || timed.frame.bus > 7u ||
        timed.frame.length > timed.frame.data.size()) {
      throw std::runtime_error("invalid CAN replay record at index " +
                               std::to_string(index));
    }
    previous_timestamp = timed.timestamp_us;
    records_.push_back(timed);
  }
  if (records_.empty()) throw std::runtime_error("CAN replay fixture is empty");
}

// elapsed time까지 도착한 CAN frame을 반환한다.
size_t CanReplaySource::poll(double elapsed_s, std::vector<CanFrame> *frames) {
  if (!frames) return 0;
  frames->clear();
  if (!std::isfinite(elapsed_s) || elapsed_s < 0.0) return 0;
  const uint64_t elapsed_us = static_cast<uint64_t>(elapsed_s * 1000000.0);
  while (next_record_ < records_.size() &&
         records_[next_record_].timestamp_us <= elapsed_us) {
    frames->push_back(records_[next_record_].frame);
    ++next_record_;
  }
  return frames->size();
}

double CanReplaySource::duration_s() const {
  return records_.empty() ? 0.0 : records_.back().timestamp_us / 1000000.0;
}
