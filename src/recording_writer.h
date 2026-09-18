#pragma once

#include "ipc_messages.h"
#include "recording_format.h"

#include <atomic>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/* tmpfs 스테이징 → SD 이동자. 활성 세그먼트는 램(tmpfs)에 써서 SD 지연
 * 스파이크(세그먼트 로테이션 flush, writeback 쓰로틀링)가 기록 경로를 막지
 * 못하게 하고, 닫힌 파일만 큰 순차 복사로 SD에 옮긴다. 작업은 순서대로 처리한다. */
class StagingMover {
public:
  StagingMover() = default;
  ~StagingMover() { stop(); }

  // 이전 세션이 route 도중 죽어 스테이징에 남긴 route를 전부 SD로 회수한다.
  void recover(const std::string &staging_root, const std::string &root);
  void enqueue_file(std::string from, std::string to);
  void enqueue_tree(std::string from, std::string to);
  // 큐를 다 비운 뒤 스레드를 끝낸다. 여러 번 불러도 된다.
  void stop();
  uint64_t pending() const { return pending_.load(); }

private:
  struct Job {
    bool tree = false;  // true면 디렉토리 전체 이동
    std::string from;
    std::string to;
  };
  void enqueue(Job &&job);
  void loop();
  static void move_file(const std::string &from, const std::string &to);
  static void move_tree(const std::string &from, const std::string &to);

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Job> queue_;
  bool stop_ = false;
  std::atomic<uint64_t> pending_{0};
  std::thread thread_{&StagingMover::loop, this};  // 마지막 멤버: 위가 다 준비된 뒤 시작
};

class RecordingWriter {
public:
  RecordingWriter(std::string root, std::string params_directory,
                  unsigned width, unsigned height, unsigned fps,
                  unsigned bitrate);
  ~RecordingWriter();

  void set_enabled(bool enabled, uint64_t now_ns);
  void set_codec_config(const uint8_t *data, size_t size);
  void write_encoded_frame(const K230RoadAiFrame &frame, const uint8_t *data,
                           size_t size, bool keyframe);
  void write_can(K230RecordType type, const K230CanBatch &batch);
  void write_state(K230RecordType type, uint64_t timestamp_ns,
                   const void *data, size_t size);
  void close();

  bool requested_enabled() const { return requested_enabled_.load(); }
  bool active() const { return active_.load(); }
  bool blocked_for_space() const { return blocked_for_space_.load(); }
  uint64_t video_frames() const { return total_video_frames_.load(); }
  uint64_t queue_drops() const { return queue_drops_.load(); }
  uint64_t pending_moves() const { return mover_.pending(); }

private:
  struct PendingWrite {
    enum class Kind {
      Enable,
      Disable,
      CodecConfig,
      EncodedFrame,
      Can,
      State,
      Stop,
    };

    Kind kind = Kind::Stop;
    uint64_t timestamp_ns = 0;
    K230RecordType record_type = K230RecordType::ModelState;
    K230RoadAiFrame frame{};
    bool keyframe = false;
    /* 인코딩 패킷, 상태 스냅샷, 또는 이미 디스크 형식으로 직렬화한 CAN 배치.
     * 배치를 값으로 품으면 항목마다 21.5 KB라 큐 상한이 램 절벽이 된다. */
    std::vector<uint8_t> data;
  };

  void enqueue(PendingWrite &&write, bool force = false);
  void worker_loop();
  void process(PendingWrite &&write);
  void write_encoded_frame_impl(const K230RoadAiFrame &frame,
                                const uint8_t *data, size_t size,
                                bool keyframe);
  // 이벤트 레코드 하나(헤더 + 페이로드). CAN 배치와 상태 스냅샷이 같이 쓴다.
  void write_record_impl(K230RecordType type, uint64_t timestamp_ns,
                         const void *data, size_t size);
  bool start_route(uint64_t now_ns);
  bool open_segment(const K230RoadAiFrame &frame);
  void close_segment();
  bool open_event_chunk(uint64_t now_ns);
  void close_event_chunk();
  void close_route(bool complete);
  void write_manifest(bool complete) const;
  void snapshot_params() const;
  bool has_storage_reserve() const;
  bool write_event_header(K230RecordType type, uint64_t timestamp_ns,
                          uint32_t payload_size);
  static FILE *open_buffered(const std::string &path);

  std::string root_;
  std::string staging_root_;
  std::string final_route_path_;
  std::string segment_relative_;
  std::string params_directory_;
  std::string route_path_;
  unsigned width_ = 0;
  unsigned height_ = 0;
  unsigned fps_ = 0;
  unsigned bitrate_ = 0;
  std::atomic<bool> requested_enabled_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> blocked_for_space_{false};
  std::atomic<uint64_t> total_video_frames_{0};
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<PendingWrite> queue_;
  static constexpr size_t kMaximumPendingWrites = 1024;
  std::atomic<uint64_t> queue_drops_{0};
  FILE *event_file_ = nullptr;
  FILE *video_file_ = nullptr;
  FILE *index_file_ = nullptr;
  std::vector<uint8_t> codec_config_;
  uint64_t route_start_ns_ = 0;
  uint64_t segment_start_ns_ = 0;
  uint64_t event_chunk_start_ns_ = 0;
  uint64_t next_storage_check_ns_ = 0;
  uint64_t video_offset_ = 0;
  uint64_t event_records_ = 0;
  uint32_t segment_index_ = 0;
  uint32_t event_chunk_index_ = 0;
  StagingMover mover_;
  // 마지막 멤버: 위의 모든 상태가 초기화된 뒤에 워커가 시작된다.
  std::thread worker_{&RecordingWriter::worker_loop, this};
};
