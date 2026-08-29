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
  /* 인코더 입력 버퍼를 호출자가 직접 채운다(중간 복사 제거). 목적지는
   * luma 높이 행, chroma 높이/2 행이고 각자 stride를 가진다. false를
   * 반환하면 버퍼는 반납되고 프레임은 인코딩되지 않는다. */
  using FrameFiller = std::function<bool(uint8_t *luma, size_t luma_stride,
                                         uint8_t *chroma, size_t chroma_stride)>;

  MvxV4l2Encoder() = default;
  ~MvxV4l2Encoder();

  bool open(const char *device, unsigned width, unsigned height,
            unsigned fps, unsigned bitrate);
  void close();
  bool submit_frame(const K230RoadAiFrame &frame, const FrameFiller &fill,
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
  /* 소스 플레인(Y, UV)이 메모리 플레인 어디에 놓이는지. NV12는 드라이버가
   * 연속 1플레인 또는 Y/UV 2플레인으로 주는데, 차이를 여기서 흡수한다. */
  struct PlaneLayout {
    unsigned buffer_plane = 0;
    size_t byte_offset = 0;
    size_t stride = 0;
  };

  bool set_formats();
  bool set_controls();
  bool allocate_buffers();
  void build_layout();
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
  uint32_t raw_plane_bytes_[2] = {};
  PlaneLayout layout_[2];
  bool streaming_ = false;
  std::vector<std::vector<Plane>> raw_buffers_;
  std::vector<Plane> capture_planes_;
  std::deque<unsigned> free_raw_;
  std::deque<K230RoadAiFrame> pending_frames_;
  uint64_t submitted_frames_ = 0;
  uint64_t encoded_frames_ = 0;
};
