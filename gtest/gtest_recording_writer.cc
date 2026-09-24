/* RecordingWriter가 디스크에 남기는 것: 60초 청크 이벤트 로그(K230LOG1), 세그먼트
 * 프레임 인덱스(K230IDX1), 매니페스트, params 스냅샷, 그리고 tmpfs 스테이징 →
 * 최종 경로 이동. 보드·인코더 없이 합성 레코드로 검사한다. */
#include "recording_format.h"
#include "recording_writer.h"

#include <gtest/gtest.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

std::vector<uint8_t> read_file(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    ADD_FAILURE() << "열기 실패 " << path;
    return {};
  }
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
}

std::vector<std::string> list_dir(const std::string &path) {
  std::vector<std::string> names;
  if (DIR *dir = opendir(path.c_str())) {
    while (dirent *entry = readdir(dir)) {
      const std::string name = entry->d_name;
      if (name != "." && name != "..") names.push_back(name);
    }
    closedir(dir);
  }
  return names;
}

template <class T>
T read_at(const std::vector<uint8_t> &bytes, size_t offset) {
  if (offset + sizeof(T) > bytes.size()) {
    ADD_FAILURE() << "레코드가 파일 끝을 넘는다";
    return T{};
  }
  T value;
  std::memcpy(&value, bytes.data() + offset, sizeof(T));
  return value;
}

K230CanBatch synthetic_batch(uint64_t timestamp_ns, uint32_t count, uint32_t dropped) {
  K230CanBatch batch;
  batch.timestamp_ns = timestamp_ns;
  batch.valid = 1;
  batch.count = count;
  batch.dropped = dropped;
  for (uint32_t i = 0; i < std::min<uint32_t>(count, kK230CanBatchMaxFrames); ++i) {
    K230CanFrame &frame = batch.frames[i];
    frame.address = 0x340 + i;
    frame.src = i % 3;
    frame.bus_time = 1000 + i;
    frame.data_len = 8;
    frame.flags = i == 1 ? 0x2 : 0;
    for (int b = 0; b < 8; ++b) frame.data[b] = static_cast<uint8_t>(i * 8 + b);
  }
  return batch;
}

TEST(RecordingWriter, RouteOnDisk) {
  char root_template[] = "/tmp/gtest_recording_XXXXXX";
  const std::string root = mkdtemp(root_template);
  const std::string staging = root + "/staging";
  const std::string params = root + "/params";
  const std::string recordings = root + "/recordings";
  mkdir(params.c_str(), 0775);
  std::ofstream(params + "/steering.json") << "{\"steer_max\": 384}\n";
  std::ofstream(params + "/notes.txt") << "not a param file\n";
  setenv("K230_RECORD_STAGING", staging.c_str(), 1);

  const uint64_t t0 = 5'000'000'000ULL;
  {
    RecordingWriter writer(recordings, params, 1280, 720, 20, 8000000);
    writer.set_enabled(true, t0);
    const uint8_t codec_config[] = {'C', 'F', 'G', 0x01};
    writer.set_codec_config(codec_config, sizeof(codec_config));
    for (uint64_t i = 0; i < 3; ++i) {
      K230RoadAiFrame frame;
      frame.frame_id = 100 + i;
      frame.timestamp_ns = t0 + i * 50'000'000ULL;
      std::vector<uint8_t> packet(64 + i * 16, static_cast<uint8_t>(0xA0 + i));
      writer.write_encoded_frame(frame, packet.data(), packet.size(), i == 0);
    }
    writer.write_can(K230RecordType::CanRx, synthetic_batch(t0 + 10'000'000ULL, 3, 2));
    writer.write_can(K230RecordType::CanTx, synthetic_batch(t0 + 20'000'000ULL, 300, 0));
    writer.write_can(K230RecordType::ModelState, synthetic_batch(t0, 1, 0));  // 무시돼야 한다
    std::vector<uint8_t> control(240);
    for (size_t i = 0; i < control.size(); ++i) control[i] = static_cast<uint8_t>(i);
    writer.write_state(K230RecordType::ControlState, t0 + 30'000'000ULL, control.data(),
                       control.size());
    std::vector<uint8_t> panda(96, 0x5A);
    writer.write_state(K230RecordType::PandaState, t0 + 40'000'000ULL, panda.data(),
                       panda.size());
    writer.set_enabled(false, t0 + 50'000'000ULL);
    writer.close();
    // route를 끝낸 뒤 writer 카운터
    ASSERT_FALSE(writer.active());
    ASSERT_EQ(writer.video_frames(), 3);
    ASSERT_EQ(writer.queue_drops(), 0);
  }

  const std::vector<std::string> route_dirs = list_dir(recordings);
  ASSERT_EQ(route_dirs.size(), 1) << "route가 정확히 하나 생긴다";
  ASSERT_TRUE(list_dir(staging).empty()) << "close 뒤 스테이징 디렉터리는 빈다";
  const std::string route = recordings + "/" + route_dirs[0];

  // events/000.bin: 헤더 + CanRx(3) + CanTx(256으로 잘림) + 상태 2개
  const std::vector<uint8_t> events = read_file(route + "/events/000.bin");
  ASSERT_EQ(std::memcmp(events.data(), "K230LOG1", 8), 0) << "이벤트 로그 magic";
  ASSERT_EQ(read_at<uint32_t>(events, 8), kK230RecordingVersion) << "이벤트 로그 버전";
  const uint32_t header_size = read_at<uint32_t>(events, 12);
  // 이벤트 로그 헤더 크기와 route 시작 시각
  ASSERT_EQ(header_size, sizeof(K230EventFileHeader));
  ASSERT_EQ(read_at<uint64_t>(events, 16), t0);
  size_t offset = header_size;
  struct Expected { uint16_t type; uint64_t ts; uint32_t payload; };
  const size_t frame_bytes = sizeof(K230RecordedCanFrame);
  const size_t batch_header = sizeof(K230RecordedCanBatchHeader);
  const Expected expected[] = {
      {1, t0 + 10'000'000ULL, static_cast<uint32_t>(batch_header + 3 * frame_bytes)},
      {2, t0 + 20'000'000ULL, static_cast<uint32_t>(batch_header + 256 * frame_bytes)},
      {4, t0 + 30'000'000ULL, 240},
      {5, t0 + 40'000'000ULL, 96},
  };
  for (const Expected &record : expected) {
    const auto header = read_at<K230EventRecordHeader>(events, offset);
    // 이벤트 레코드 헤더 순서
    ASSERT_EQ(header.type, record.type);
    ASSERT_EQ(header.timestamp_ns, record.ts);
    ASSERT_EQ(header.payload_size, record.payload);
    offset += sizeof(header);
    if (record.type == 1) {
      const auto batch = read_at<K230RecordedCanBatchHeader>(events, offset);
      // CanRx 묶음 헤더
      ASSERT_EQ(batch.count, 3);
      ASSERT_EQ(batch.dropped, 2);
      const auto frame1 = read_at<K230RecordedCanFrame>(events, offset + batch_header + frame_bytes);
      // 기록된 CAN 프레임 필드
      ASSERT_EQ(frame1.address, 0x341);
      ASSERT_EQ(frame1.src, 1);
      ASSERT_EQ(frame1.bus_time, 1001);
      ASSERT_EQ(frame1.data_len, 8);
      ASSERT_EQ(frame1.flags, 0x2);
      ASSERT_EQ(frame1.data[3], 11);
    } else if (record.type == 2) {
      const auto batch = read_at<K230RecordedCanBatchHeader>(events, offset);
      // 256프레임을 넘는 묶음은 기록 형식의 최대치로 잘린다
      ASSERT_EQ(batch.count, 256);
      ASSERT_EQ(batch.dropped, 0);
    } else if (record.type == 4) {
      ASSERT_EQ(events[offset + 17], 17) << "상태 페이로드는 그대로 저장된다";
    }
    offset += record.payload;
  }
  ASSERT_EQ(offset, events.size()) << "마지막 레코드 뒤에 남는 바이트가 없다";

  // segments/000: 코덱 설정 + 패킷 3개, 인덱스 오프셋은 누적
  const std::vector<uint8_t> video = read_file(route + "/segments/000/road.hevc");
  // 영상 스트림은 코덱 설정 뒤에 패킷이 이어진다
  ASSERT_EQ(video.size(), 4 + 64 + 80 + 96);
  ASSERT_EQ(std::memcmp(video.data(), "CFG", 3), 0);
  const std::vector<uint8_t> index = read_file(route + "/segments/000/frames.bin");
  const auto index_header = read_at<K230FrameIndexHeader>(index, 0);
  // 프레임 인덱스 헤더
  ASSERT_EQ(std::memcmp(index_header.magic, "K230IDX1", 8), 0);
  ASSERT_EQ(index_header.width, 1280);
  ASSERT_EQ(index_header.height, 720);
  ASSERT_EQ(index_header.fps, 20);
  ASSERT_EQ(index_header.record_size, sizeof(K230FrameIndexRecord));
  ASSERT_EQ(index_header.segment_start_ns, t0);
  ASSERT_EQ(index.size(), sizeof(K230FrameIndexHeader) + 3 * sizeof(K230FrameIndexRecord))
      << "패킷마다 인덱스 레코드 하나";
  const auto second = read_at<K230FrameIndexRecord>(
      index, sizeof(K230FrameIndexHeader) + sizeof(K230FrameIndexRecord));
  // 프레임 인덱스 레코드의 오프셋이 스트림을 따른다
  ASSERT_EQ(second.frame_id, 101);
  ASSERT_EQ(second.encode_index, 1);
  ASSERT_EQ(second.file_offset, 4 + 64);
  ASSERT_EQ(second.packet_size, 80);
  ASSERT_EQ(second.flags, 0);

  const std::vector<uint8_t> manifest = read_file(route + "/manifest.json");
  const std::string text(manifest.begin(), manifest.end());
  // 매니페스트의 개수
  ASSERT_NE(text.find("\"complete\": true"), std::string::npos);
  ASSERT_NE(text.find("\"video_frames\": 3"), std::string::npos);
  ASSERT_NE(text.find("\"event_records\": 4"), std::string::npos);
  const auto snapshot = list_dir(route + "/params");
  // params 스냅샷은 json 파일만 복사한다
  ASSERT_EQ(snapshot.size(), 1);
  ASSERT_EQ(snapshot[0], "steering.json");

  std::system(("rm -rf '" + root + "'").c_str());
}

}  // namespace
