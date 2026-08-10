#pragma once

#include "k230_ipc.h"
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
    K230CanBatch can_batch{};
    bool keyframe = false;
    std::vector<uint8_t> data;
  };

  void enqueue(PendingWrite &&write, bool force = false);
  void worker_loop();
  void process(PendingWrite &&write);
  void write_encoded_frame_impl(const K230RoadAiFrame &frame,
                                const uint8_t *data, size_t size,
                                bool keyframe);
  void write_can_impl(K230RecordType type, const K230CanBatch &batch);
  void write_state_impl(K230RecordType type, uint64_t timestamp_ns,
                        const void *data, size_t size);
  bool start_route(uint64_t now_ns);
  bool open_segment(const K230RoadAiFrame &frame);
  void close_segment();
  void close_route(bool complete);
  void write_manifest(bool complete) const;
  void snapshot_params() const;
  bool has_storage_reserve() const;
  bool write_event_header(K230RecordType type, uint64_t timestamp_ns,
                          uint32_t payload_size);
  static FILE *open_buffered(const std::string &path);

  std::string root_;
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
  std::thread worker_;
  static constexpr size_t kMaximumPendingWrites = 1024;
  std::atomic<uint64_t> queue_drops_{0};
  FILE *event_file_ = nullptr;
  FILE *video_file_ = nullptr;
  FILE *index_file_ = nullptr;
  std::vector<uint8_t> codec_config_;
  uint64_t segment_start_ns_ = 0;
  uint64_t next_storage_check_ns_ = 0;
  uint64_t video_offset_ = 0;
  uint64_t event_records_ = 0;
  uint32_t segment_index_ = 0;
};
