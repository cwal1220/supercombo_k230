#pragma once

#include "k230_ipc.h"

#include <linux/videodev2.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

class MvxV4l2Encoder {
public:
  struct Packet {
    const uint8_t *data = nullptr;
    size_t size = 0;
    bool codec_config = false;
    bool keyframe = false;
    const K230RoadAiFrame *frame = nullptr;
  };
  using PacketHandler = std::function<void(const Packet &)>;

  MvxV4l2Encoder() = default;
  ~MvxV4l2Encoder();

  bool open(const char *device, unsigned width, unsigned height,
            unsigned fps, unsigned bitrate);
  void close();
  bool submit_frame(const uint8_t *nv12, size_t size,
                    const K230RoadAiFrame &frame,
                    const PacketHandler &handler);
  void drain(const PacketHandler &handler);

  bool valid() const { return fd_ >= 0; }
  uint64_t submitted_frames() const { return submitted_frames_; }
  uint64_t encoded_frames() const { return encoded_frames_; }

private:
  struct Plane {
    void *address = nullptr;
    size_t length = 0;
    uint32_t offset = 0;
  };
  struct RawBuffer {
    std::vector<Plane> planes;
  };
  struct CaptureBuffer {
    Plane plane;
  };

  bool set_formats();
  bool set_controls();
  bool allocate_buffers();
  bool queue_capture(unsigned index);
  bool start_streaming();
  void reclaim_raw();
  void drain_capture(const PacketHandler &handler);
  void fail(const std::string &message);

  int fd_ = -1;
  unsigned width_ = 0;
  unsigned height_ = 0;
  unsigned fps_ = 0;
  unsigned bitrate_ = 0;
  unsigned raw_plane_count_ = 0;
  uint32_t raw_stride_[2] = {};
  bool streaming_ = false;
  std::vector<RawBuffer> raw_buffers_;
  std::vector<CaptureBuffer> capture_buffers_;
  std::vector<unsigned> free_raw_;
  std::deque<K230RoadAiFrame> pending_frames_;
  uint64_t submitted_frames_ = 0;
  uint64_t encoded_frames_ = 0;
};
