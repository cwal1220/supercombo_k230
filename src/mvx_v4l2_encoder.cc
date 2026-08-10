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

bool is_hevc_idr(const uint8_t *data, size_t size) {
  if (!data || size < 5) return false;
  for (size_t index = 0; index + 4 < size; ++index) {
    size_t header = 0;
    if (data[index] == 0 && data[index + 1] == 0 && data[index + 2] == 1) {
      header = index + 3;
    } else if (index + 5 < size && data[index] == 0 && data[index + 1] == 0 &&
               data[index + 2] == 0 && data[index + 3] == 1) {
      header = index + 4;
    }
    if (header != 0 && header < size) {
      const uint8_t nal_type = (data[header] >> 1) & 0x3f;
      if (nal_type == 19 || nal_type == 20) return true;
    }
  }
  return false;
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
               "stride=%u/%u/%u buffers=%zu/%zu\n",
               device, width_, height_, fps_, bitrate_, raw_stride_[0],
               raw_stride_[1], raw_stride_[2], raw_buffers_.size(),
               capture_buffers_.size());
  return true;
}

bool MvxV4l2Encoder::set_formats() {
  v4l2_format raw{};
  raw.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  raw.fmt.pix_mp.width = width_;
  raw.fmt.pix_mp.height = height_;
  raw.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420M;
  raw.fmt.pix_mp.field = V4L2_FIELD_NONE;
  raw.fmt.pix_mp.num_planes = 3;
  if (xioctl(fd_, VIDIOC_S_FMT, &raw) != 0) {
    fail("set YUV420 input format");
    return false;
  }
  raw_plane_count_ = raw.fmt.pix_mp.num_planes;
  if (raw_plane_count_ != 3) {
    std::fprintf(stderr, "recordd: MVX expected 3 YUV420 planes, got %u\n",
                 raw_plane_count_);
    return false;
  }
  raw_stride_[0] = raw.fmt.pix_mp.plane_fmt[0].bytesperline;
  raw_stride_[1] = raw.fmt.pix_mp.plane_fmt[1].bytesperline;
  raw_stride_[2] = raw.fmt.pix_mp.plane_fmt[2].bytesperline;
  if (raw_stride_[0] < width_) raw_stride_[0] = width_;
  if (raw_stride_[1] < width_ / 2) raw_stride_[1] = width_ / 2;
  if (raw_stride_[2] < width_ / 2) raw_stride_[2] = width_ / 2;

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
  return true;
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
  v4l2_requestbuffers request{};
  request.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  request.memory = V4L2_MEMORY_MMAP;
  request.count = kBufferCount;
  if (xioctl(fd_, VIDIOC_REQBUFS, &request) != 0 || request.count == 0) {
    fail("request YUV420 buffers");
    return false;
  }
  raw_buffers_.resize(request.count);
  for (unsigned index = 0; index < request.count; ++index) {
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    v4l2_buffer buffer{};
    buffer.type = request.type;
    buffer.memory = request.memory;
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
    raw_buffers_[index].planes.resize(buffer.length);
    for (unsigned plane = 0; plane < buffer.length; ++plane) {
      Plane &mapped = raw_buffers_[index].planes[plane];
      mapped.length = planes[plane].length;
      mapped.offset = planes[plane].m.mem_offset;
      mapped.address = mmap(nullptr, mapped.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd_, mapped.offset);
      if (mapped.address == MAP_FAILED) {
        mapped.address = nullptr;
        fail("map YUV420 buffer");
        return false;
      }
    }
    free_raw_.push_back(index);
  }

  request = {};
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  request.count = kBufferCount;
  if (xioctl(fd_, VIDIOC_REQBUFS, &request) != 0 || request.count == 0) {
    fail("request HEVC buffers");
    return false;
  }
  capture_buffers_.resize(request.count);
  for (unsigned index = 0; index < request.count; ++index) {
    v4l2_buffer buffer{};
    buffer.type = request.type;
    buffer.memory = request.memory;
    buffer.index = index;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) != 0) {
      fail("query HEVC buffer");
      return false;
    }
    Plane &mapped = capture_buffers_[index].plane;
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
      packet.data = static_cast<const uint8_t *>(capture_buffers_[buffer.index].plane.address);
      packet.size = std::min<size_t>(buffer.bytesused,
                                     capture_buffers_[buffer.index].plane.length);
      packet.codec_config = codec_config;
      packet.keyframe = !codec_config &&
          is_hevc_idr(packet.data, packet.size);
      packet.frame = frame_ptr;
      handler(packet);
    }
    if (!queue_capture(buffer.index)) break;
  }
}

void MvxV4l2Encoder::drain(const PacketHandler &handler) {
  reclaim_raw();
  drain_capture(handler);
  reclaim_raw();
}

bool MvxV4l2Encoder::submit_frame(const uint8_t *nv12, size_t size,
                                  const K230RoadAiFrame &frame,
                                  const PacketHandler &handler) {
  if (!valid() || !nv12 || size < static_cast<size_t>(width_) * height_ * 3 / 2 ||
      frame.width != width_ || frame.height != height_) {
    return false;
  }
  drain(handler);
  if (free_raw_.empty()) return false;

  const unsigned index = free_raw_.back();
  free_raw_.pop_back();
  RawBuffer &raw = raw_buffers_[index];
  const size_t y_size = static_cast<size_t>(width_) * height_;
  const uint8_t *source_uv = nv12 + y_size;
  const unsigned rows[3] = {height_, height_ / 2, height_ / 2};
  for (unsigned plane = 0; plane < 3; ++plane) {
    uint8_t *destination = static_cast<uint8_t *>(raw.planes[plane].address);
    for (unsigned row = 0; row < rows[plane]; ++row) {
      uint8_t *dst = destination + static_cast<size_t>(row) * raw_stride_[plane];
      if (plane == 0) {
        std::memcpy(dst, nv12 + static_cast<size_t>(row) * width_, width_);
      } else {
        const uint8_t *src = source_uv + static_cast<size_t>(row) * width_;
        for (unsigned column = 0; column < width_ / 2; ++column) {
          dst[column] = src[column * 2 + (plane - 1)];
        }
      }
    }
  }

  v4l2_plane planes[VIDEO_MAX_PLANES]{};
  v4l2_buffer buffer{};
  buffer.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.index = index;
  buffer.length = raw_plane_count_;
  buffer.m.planes = planes;
  for (unsigned plane = 0; plane < raw_plane_count_; ++plane) {
    planes[plane].length = raw.planes[plane].length;
    planes[plane].m.mem_offset = raw.planes[plane].offset;
    planes[plane].bytesused = raw_stride_[plane] * rows[plane];
  }
  if (xioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
    fail("queue YUV420 frame");
    free_raw_.push_back(index);
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
  for (RawBuffer &buffer : raw_buffers_) {
    for (Plane &plane : buffer.planes) {
      if (plane.address) munmap(plane.address, plane.length);
    }
  }
  for (CaptureBuffer &buffer : capture_buffers_) {
    if (buffer.plane.address) munmap(buffer.plane.address, buffer.plane.length);
  }
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
  streaming_ = false;
  raw_buffers_.clear();
  capture_buffers_.clear();
  free_raw_.clear();
  pending_frames_.clear();
  submitted_frames_ = 0;
  encoded_frames_ = 0;
}
