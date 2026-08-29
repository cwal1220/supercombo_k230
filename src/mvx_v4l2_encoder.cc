#include "mvx_v4l2_encoder.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>

namespace {

constexpr unsigned kBufferCount = 8;
constexpr uint32_t kMvxCodecConfigFlag = 0xc1000000U;
constexpr uint32_t kMvxVendorFlagMask = 0xfff00000U;
constexpr uint32_t kMvxFrameRateControl = V4L2_CTRL_CLASS_MPEG + 0x2000;
constexpr uint32_t kMvxPFramesControl = V4L2_CTRL_CLASS_MPEG + 0x2009;
constexpr uint32_t kMvxGopTypeControl = V4L2_CTRL_CLASS_MPEG + 0x200a;

struct MvxRateControl {
  uint32_t type;
  uint32_t target_bitrate;
  uint32_t maximum_bitrate;
};

constexpr uint32_t kMvxConstantRateControl = 3;
#define VIDIOC_S_K230_MVX_RATE_CONTROL \
  _IOWR('V', BASE_VIDIOC_PRIVATE + 5, MvxRateControl)

int xioctl(int fd, unsigned long request, void *argument) {
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result < 0 && errno == EINTR);
  return result;
}

bool set_control(int fd, uint32_t id, int32_t value) {
  v4l2_control control{};
  control.id = id;
  control.value = value;
  return xioctl(fd, VIDIOC_S_CTRL, &control) == 0;
}

/* annex-B start code 길이. 아니면 0. */
size_t annex_b_header(const uint8_t *data, size_t size) {
  if (!data) return 0;
  if (size > 4 && data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1) return 4;
  if (size > 3 && data[0] == 0 && data[1] == 0 && data[2] == 1) return 3;
  return 0;
}

/* MVX는 패킷 하나에 NAL 하나만 담으므로 선두 NAL 타입이 곧 프레임 종류다.
 * 16~21은 IRAP(BLA/IDR/CRA)로, 세그먼트를 여기서 끊으면 독립 디코딩된다. */
bool is_hevc_irap(const uint8_t *data, size_t size) {
  const size_t header = annex_b_header(data, size);
  if (header == 0) return false;
  const uint8_t nal_type = (data[header] >> 1) & 0x3f;
  return nal_type >= 16 && nal_type <= 21;
}

}  // namespace

MvxV4l2Encoder::~MvxV4l2Encoder() {
  close();
}

void MvxV4l2Encoder::fail(const std::string &message) {
  std::fprintf(stderr, "recordd: MVX %s: %s\n", message.c_str(), std::strerror(errno));
}

bool MvxV4l2Encoder::open(const char *device, unsigned width, unsigned height,
                          unsigned fps, unsigned bitrate) {
  close();
  width_ = width;
  height_ = height;
  fps_ = fps;
  bitrate_ = bitrate;
  fd_ = ::open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    fail(std::string("open ") + device);
    return false;
  }

  v4l2_capability capability{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) != 0 ||
      !(capability.device_caps & V4L2_CAP_VIDEO_M2M_MPLANE ||
        capability.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE)) {
    fail("device is not a V4L2 M2M codec");
    close();
    return false;
  }
  if (!set_formats() || !set_controls() || !allocate_buffers()) {
    close();
    return false;
  }
  std::fprintf(stderr,
               "recordd: MVX ready device=%s frame=%ux%u fps=%u bitrate=%u "
               "planes=%u stride=%u/%u buffers=%zu/%zu\n",
               device, width_, height_, fps_, bitrate_, raw_plane_count_,
               raw_stride_[0], raw_stride_[1], raw_buffers_.size(),
               capture_planes_.size());
  return true;
}

bool MvxV4l2Encoder::set_formats() {
  /* 소스와 같은 NV12(단일 연속 플레인)를 그대로 쓴다. 3-plane YUV420M
   * 경로는 이 보드의 MVX 드라이버가 720p에서 소스 높이를 잘못 프로그래밍해
   * (384행 이후 가장자리 복제) 하단이 깨지고, U/V 분리 복사 비용도 있었다.
   * 드라이버는 NV12M(2-plane)은 지원하지 않는다. CTU 정렬은 펌웨어가
   * 처리하므로 실높이를 그대로 준다. */
  v4l2_format raw{};
  raw.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  raw.fmt.pix_mp.width = width_;
  raw.fmt.pix_mp.height = height_;
  raw.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
  raw.fmt.pix_mp.field = V4L2_FIELD_NONE;
  raw.fmt.pix_mp.num_planes = 1;
  if (xioctl(fd_, VIDIOC_S_FMT, &raw) != 0) {
    fail("set NV12 input format");
    return false;
  }
  if (raw.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12) {
    std::fprintf(stderr, "recordd: MVX rejected NV12, driver picked %08x\n",
                 raw.fmt.pix_mp.pixelformat);
    return false;
  }
  /* 이 드라이버는 NV12 fourcc에 메모리 플레인 2개(Y, UV)를 쓴다. 1개
   * (연속)와 2개 모두 지원한다. */
  raw_plane_count_ = raw.fmt.pix_mp.num_planes;
  if (raw_plane_count_ != 1 && raw_plane_count_ != 2) {
    std::fprintf(stderr, "recordd: MVX expected 1-2 NV12 planes, got %u\n",
                 raw_plane_count_);
    return false;
  }
  for (unsigned plane = 0; plane < raw_plane_count_; ++plane) {
    raw_stride_[plane] = raw.fmt.pix_mp.plane_fmt[plane].bytesperline;
    if (raw_stride_[plane] < width_) raw_stride_[plane] = width_;
  }

  v4l2_format encoded{};
  encoded.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  encoded.fmt.pix.width = width_;
  encoded.fmt.pix.height = height_;
  encoded.fmt.pix.pixelformat = V4L2_PIX_FMT_HEVC;
  encoded.fmt.pix.field = V4L2_FIELD_NONE;
  encoded.fmt.pix.sizeimage = 1024 * 1024;
  if (xioctl(fd_, VIDIOC_S_FMT, &encoded) != 0) {
    fail("set HEVC output format");
    return false;
  }
  build_layout();
  return true;
}

void MvxV4l2Encoder::build_layout() {
  const unsigned rows[2] = {height_, height_ / 2};
  for (unsigned source_plane = 0; source_plane < 2; ++source_plane) {
    PlaneLayout &layout = layout_[source_plane];
    layout.buffer_plane = raw_plane_count_ == 1 ? 0 : source_plane;
    layout.stride = raw_stride_[layout.buffer_plane];
    layout.byte_offset = raw_plane_count_ == 1 && source_plane == 1
        ? layout.stride * height_ : 0;
  }
  for (unsigned plane = 0; plane < raw_plane_count_; ++plane) {
    raw_plane_bytes_[plane] = raw_stride_[plane] *
        (raw_plane_count_ == 1 ? height_ + height_ / 2 : rows[plane]);
  }
}

bool MvxV4l2Encoder::set_controls() {
  if (!set_control(fd_, kMvxFrameRateControl, static_cast<int32_t>(fps_ << 16))) {
    fail("set frame rate");
    return false;
  }
  if (!set_control(fd_, kMvxPFramesControl, static_cast<int32_t>(fps_ - 1))) {
    fail("set P-frame interval");
    return false;
  }
  if (!set_control(fd_, kMvxGopTypeControl, 2)) {
    fail("set low-delay GOP");
    return false;
  }
  MvxRateControl rate_control{kMvxConstantRateControl, bitrate_, 0};
  if (xioctl(fd_, VIDIOC_S_K230_MVX_RATE_CONTROL, &rate_control) != 0) {
    fail("set constant rate control");
    return false;
  }
  return true;
}

bool MvxV4l2Encoder::allocate_buffers() {
  v4l2_requestbuffers output_request{};
  output_request.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  output_request.memory = V4L2_MEMORY_MMAP;
  output_request.count = kBufferCount;
  if (xioctl(fd_, VIDIOC_REQBUFS, &output_request) != 0 || output_request.count == 0) {
    fail("request YUV420 buffers");
    return false;
  }
  raw_buffers_.resize(output_request.count);
  for (unsigned index = 0; index < output_request.count; ++index) {
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    buffer.length = VIDEO_MAX_PLANES;
    buffer.m.planes = planes;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) != 0) {
      fail("query YUV420 buffer");
      return false;
    }
    if (buffer.length != raw_plane_count_) {
      std::fprintf(stderr, "recordd: MVX raw plane mismatch query=%u format=%u\n",
                   buffer.length, raw_plane_count_);
      return false;
    }
    raw_buffers_[index].resize(buffer.length);
    for (unsigned plane = 0; plane < buffer.length; ++plane) {
      Plane &mapped = raw_buffers_[index][plane];
      mapped.length = planes[plane].length;
      mapped.offset = planes[plane].m.mem_offset;
      mapped.address = mmap(nullptr, mapped.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd_, mapped.offset);
      if (mapped.address == MAP_FAILED) {
        mapped.address = nullptr;
        fail("map YUV420 buffer");
        return false;
      }
      /* 매 프레임 채우는 영역 밖(드라이버가 더 크게 잡은 꼬리)을 중립값
       * (Y=16, U/V=128)으로 한 번만 깔아 둔다. */
      std::memset(mapped.address, plane == 0 ? 0x10 : 0x80, mapped.length);
    }
    free_raw_.push_back(index);
  }

  v4l2_requestbuffers request{};
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  request.count = kBufferCount;
  if (xioctl(fd_, VIDIOC_REQBUFS, &request) != 0 || request.count == 0) {
    fail("request HEVC buffers");
    return false;
  }
  capture_planes_.resize(request.count);
  for (unsigned index = 0; index < request.count; ++index) {
    v4l2_buffer buffer{};
    buffer.type = request.type;
    buffer.memory = request.memory;
    buffer.index = index;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) != 0) {
      fail("query HEVC buffer");
      return false;
    }
    Plane &mapped = capture_planes_[index];
    mapped.length = buffer.length;
    mapped.offset = buffer.m.offset;
    mapped.address = mmap(nullptr, mapped.length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd_, mapped.offset);
    if (mapped.address == MAP_FAILED) {
      mapped.address = nullptr;
      fail("map HEVC buffer");
      return false;
    }
    if (!queue_capture(index)) return false;
  }
  return true;
}

bool MvxV4l2Encoder::queue_capture(unsigned index) {
  v4l2_buffer buffer{};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.index = index;
  if (xioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
    fail("queue HEVC buffer");
    return false;
  }
  return true;
}

bool MvxV4l2Encoder::start_streaming() {
  if (streaming_) return true;
  v4l2_buf_type raw_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  if (xioctl(fd_, VIDIOC_STREAMON, &raw_type) != 0) {
    fail("start YUV420 stream");
    return false;
  }
  v4l2_buf_type encoded_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &encoded_type) != 0) {
    fail("start HEVC stream");
    xioctl(fd_, VIDIOC_STREAMOFF, &raw_type);
    return false;
  }
  streaming_ = true;
  return true;
}

void MvxV4l2Encoder::reclaim_raw() {
  if (!streaming_) return;
  while (true) {
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.length = raw_plane_count_;
    buffer.m.planes = planes;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) != 0) {
      if (errno != EAGAIN) fail("dequeue YUV420 buffer");
      break;
    }
    free_raw_.push_back(buffer.index);
  }
}

void MvxV4l2Encoder::drain_capture(const PacketHandler &handler) {
  if (!streaming_) return;
  while (true) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) != 0) {
      if (errno != EAGAIN) fail("dequeue HEVC buffer");
      break;
    }

    /* MVX는 비트스트림을 버퍼 선두가 아니라 m.offset이 가리키는 위치에
     * 쓴다(2 KB 안팎). 선두부터 읽으면 프레임 끝 그만큼이 잘려 나가 화면
     * 하단이 깨진다. 오프셋이 실제로 start code를 가리킬 때만 받아들인다. */
    const Plane &capture_plane = capture_planes_[buffer.index];
    uint8_t *const capture_data = static_cast<uint8_t *>(capture_plane.address);
    size_t data_offset =
        capture_plane.length ? buffer.m.offset % capture_plane.length : 0;
    if (data_offset + buffer.bytesused > capture_plane.length ||
        annex_b_header(capture_data + data_offset, buffer.bytesused) == 0) {
      data_offset = 0;
    }

    const bool codec_config =
        (buffer.flags & kMvxVendorFlagMask) == kMvxCodecConfigFlag;
    K230RoadAiFrame frame{};
    const K230RoadAiFrame *frame_ptr = nullptr;
    if (!codec_config && buffer.bytesused > 0) {
      if (!pending_frames_.empty()) {
        frame = pending_frames_.front();
        pending_frames_.pop_front();
        frame_ptr = &frame;
        ++encoded_frames_;
      } else {
        std::fprintf(stderr, "recordd: MVX produced a frame without input metadata\n");
      }
    }
    if (buffer.bytesused > 0 && handler) {
      Packet packet;
      packet.data = capture_data + data_offset;
      packet.size = std::min<size_t>(buffer.bytesused,
                                     capture_plane.length - data_offset);
      packet.codec_config = codec_config;
      packet.keyframe = !codec_config && is_hevc_irap(packet.data, packet.size);
      packet.frame = frame_ptr;
      handler(packet);
    }
    if (!queue_capture(buffer.index)) break;
  }
}

void MvxV4l2Encoder::drain(const PacketHandler &handler) {
  drain_capture(handler);
  reclaim_raw();
}

bool MvxV4l2Encoder::submit_frame(const K230RoadAiFrame &frame,
                                  const FrameFiller &fill,
                                  const PacketHandler &handler) {
  if (!valid() || !fill || frame.width != width_ || frame.height != height_) {
    return false;
  }
  drain(handler);
  if (free_raw_.empty()) return false;

  /* 버퍼는 FIFO로 돌린다. 방금 반환된 버퍼를 곧바로 다시 쓰지 않으면
   * 드라이버 쪽 반납이 조금 이르더라도 안전 여유가 생긴다. */
  const unsigned index = free_raw_.front();
  free_raw_.pop_front();
  std::vector<Plane> &raw = raw_buffers_[index];
  uint8_t *destination[2];
  for (unsigned source_plane = 0; source_plane < 2; ++source_plane) {
    const PlaneLayout &layout = layout_[source_plane];
    destination[source_plane] =
        static_cast<uint8_t *>(raw[layout.buffer_plane].address) + layout.byte_offset;
  }
  if (!fill(destination[0], layout_[0].stride, destination[1], layout_[1].stride)) {
    free_raw_.push_front(index);
    return false;
  }

  v4l2_plane planes[VIDEO_MAX_PLANES]{};
  v4l2_buffer buffer{};
  buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.index = index;
  buffer.length = raw_plane_count_;
  buffer.m.planes = planes;
  for (unsigned plane = 0; plane < raw_plane_count_; ++plane) {
    planes[plane].length = raw[plane].length;
    planes[plane].m.mem_offset = raw[plane].offset;
    planes[plane].bytesused = raw_plane_bytes_[plane];
  }
  if (xioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
    fail("queue YUV420 frame");
    free_raw_.push_front(index);
    return false;
  }
  pending_frames_.push_back(frame);
  ++submitted_frames_;
  return start_streaming();
}

void MvxV4l2Encoder::close() {
  if (fd_ >= 0 && streaming_) {
    v4l2_buf_type raw_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    v4l2_buf_type encoded_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &raw_type);
    xioctl(fd_, VIDIOC_STREAMOFF, &encoded_type);
  }
  for (std::vector<Plane> &planes : raw_buffers_) {
    for (Plane &plane : planes) {
      if (plane.address) munmap(plane.address, plane.length);
    }
  }
  for (Plane &plane : capture_planes_) {
    if (plane.address) munmap(plane.address, plane.length);
  }
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
  streaming_ = false;
  raw_buffers_.clear();
  capture_planes_.clear();
  free_raw_.clear();
  pending_frames_.clear();
  submitted_frames_ = 0;
  encoded_frames_ = 0;
}
