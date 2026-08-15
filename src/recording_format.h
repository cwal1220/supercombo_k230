#pragma once

#include <cstdint>

/* v2: K230ModelState에서 소비자가 없던 lateral_target/lateral_plan(308 B)을
 * 제거해 페이로드가 4384 -> 4076 B로 줄었다. v1 녹화와 호환되지 않는다. */
constexpr uint32_t kK230RecordingVersion = 2;

enum class K230RecordType : uint16_t {
  CanRx = 1,
  CanTx = 2,
  ModelState = 3,
  ControlState = 4,
  PandaState = 5,
};

#pragma pack(push, 1)
struct K230EventFileHeader {
  char magic[8] = {'K', '2', '3', '0', 'L', 'O', 'G', '1'};
  uint32_t version = kK230RecordingVersion;
  uint32_t header_size = sizeof(K230EventFileHeader);
  uint64_t route_start_ns = 0;
  uint64_t reserved = 0;
};

struct K230EventRecordHeader {
  uint64_t timestamp_ns = 0;
  uint16_t type = 0;
  uint16_t flags = 0;
  uint32_t payload_size = 0;
};

struct K230RecordedCanBatchHeader {
  uint32_t count = 0;
  uint32_t dropped = 0;
};

struct K230RecordedCanFrame {
  uint32_t address = 0;
  uint32_t src = 0;
  uint32_t bus_time = 0;
  uint32_t data_len = 0;
  uint32_t flags = 0;
  uint8_t data[64] = {};
};

struct K230FrameIndexHeader {
  char magic[8] = {'K', '2', '3', '0', 'I', 'D', 'X', '1'};
  uint32_t version = kK230RecordingVersion;
  uint32_t header_size = sizeof(K230FrameIndexHeader);
  uint32_t record_size = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t fps = 0;
  uint64_t segment_start_ns = 0;
  uint64_t reserved = 0;
};

struct K230FrameIndexRecord {
  uint64_t frame_id = 0;
  uint64_t capture_timestamp_ns = 0;
  uint64_t encode_index = 0;
  uint64_t file_offset = 0;
  uint32_t packet_size = 0;
  uint32_t flags = 0;
};
#pragma pack(pop)

static_assert(sizeof(K230EventRecordHeader) == 16);
static_assert(sizeof(K230RecordedCanFrame) == 84);
static_assert(sizeof(K230FrameIndexRecord) == 40);
