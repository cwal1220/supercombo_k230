#include "json_utils.h"
#include "k230_ipc.h"
#include "mvx_v4l2_encoder.h"
#include "recording_writer.h"

#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;
constexpr unsigned kRecordingFps = 20;
/* 프레임이 비트 예산을 넘치면 VPU가 래스터 마지막 블록들을 깨뜨리므로
 * 720p에는 여유 있는 값이 필요하다. K230_RECORD_BITRATE로 조정 가능. */
constexpr unsigned kRecordingBitrate = 12000000;
constexpr uint64_t kConfigPollIntervalNs = 250000000ULL;
constexpr uint64_t kMaximumFrameAgeNs = 100000000ULL;

bool read_recording_enabled(const std::string &path, bool fallback) {
  std::ifstream file(path);
  if (!file) return fallback;
  const std::string text((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  bool enabled = fallback;
  if (!parse_json_bool_value(text, "enabled", &enabled)) {
    throw std::runtime_error("recording config has no 'enabled' value");
  }
  return enabled;
}

uint64_t file_revision(const std::string &path) {
  struct stat status {};
  if (stat(path.c_str(), &status) != 0) return 0;
#if defined(__APPLE__)
  const uint64_t nanoseconds = status.st_mtimespec.tv_nsec;
#else
  const uint64_t nanoseconds = status.st_mtim.tv_nsec;
#endif
  return static_cast<uint64_t>(status.st_ino) ^
      (static_cast<uint64_t>(status.st_mtime) << 24) ^ nanoseconds ^
      static_cast<uint64_t>(status.st_size);
}

void open_optional_channel(K230LatestChannel &channel, bool *opened,
                           const char *name, size_t size) {
  if (!*opened) *opened = channel.open(name, size, false);
}

}  // namespace

int main() {
  install_stop_signal_handlers(&g_stop);

  try {
    const std::string params_directory = env_string("K230_PARAMS_DIR", "params");
    const std::string config_path = env_string(
        "K230_RECORDING_PARAMS", (params_directory + "/recording.json").c_str());
    const std::string recording_root = env_string("K230_RECORD_ROOT", "recordings");
    const std::string codec_device = env_string("K230_RECORD_CODEC", "/dev/video0");
    const unsigned recording_bitrate =
        env_unsigned("K230_RECORD_BITRATE", kRecordingBitrate);

    K230FrameRing frame_ring;
    while (!g_stop && !frame_ring.open(false)) {
      std::fprintf(stderr, "recordd: waiting for road AI frame ring\n");
      usleep(250000);
    }
    if (!frame_ring.valid()) return 1;

    K230LatestChannel record_frame_sub;
    if (!record_frame_sub.open(kK230RecordFrameTopic, sizeof(K230RoadAiFrame), true))
      throw std::runtime_error("open recordFrame IPC failed");

    K230CanQueue can_log_sub;
    K230CanQueue sendcan_log_sub;
    if (!can_log_sub.open(kK230CanLogTopic, kK230CanQueueSlots, true) ||
        !sendcan_log_sub.open(kK230SendCanLogTopic, kK230CanQueueSlots, true)) {
      throw std::runtime_error("open CAN recording queues failed");
    }

    /* 인코더가 VPU의 DMA 꼬리 결함을 피하려고 코딩 높이에 패딩 행을
     * 붙인다(736행). 스트림 하단 16행은 중립 패딩이므로 시청 변환 시
     * 실높이(720)로 crop한다. */
    MvxV4l2Encoder encoder;
    if (!encoder.open(codec_device.c_str(), frame_ring.width(), frame_ring.height(),
                      kRecordingFps, recording_bitrate)) {
      throw std::runtime_error("open MVX hardware encoder failed");
    }
    RecordingWriter writer(recording_root, params_directory, frame_ring.width(),
                           frame_ring.height(), kRecordingFps, recording_bitrate);

    auto packet_handler = [&writer](const MvxV4l2Encoder::Packet &packet) {
      if (packet.codec_config) {
        writer.set_codec_config(packet.data, packet.size);
      } else if (packet.frame) {
        writer.write_encoded_frame(*packet.frame, packet.data, packet.size,
                                   packet.keyframe);
      }
    };
    auto drain_encoder = [&encoder, &packet_handler]() {
      const uint64_t deadline_ns = k230_now_ns() + 500000000ULL;
      while (encoder.submitted_frames() != encoder.encoded_frames() &&
             k230_now_ns() < deadline_ns) {
        encoder.drain(packet_handler);
        if (encoder.submitted_frames() != encoder.encoded_frames()) usleep(2000);
      }
    };

    K230LatestChannel model_sub;
    K230LatestChannel control_sub;
    K230LatestChannel panda_sub;
    bool model_open = false;
    bool control_open = false;
    bool panda_open = false;
    uint64_t model_seq = 0;
    uint64_t control_seq = 0;
    uint64_t panda_seq = 0;
    uint64_t frame_seq = 0;
    uint64_t config_revision = UINT64_MAX;
    uint64_t next_config_poll_ns = 0;
    uint64_t next_log_ns = k230_now_ns() + 1000000000ULL;
    uint64_t selected_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t stale_frames = 0;
    uint64_t frame_sync_failures = 0;
    bool warmed = false;
    std::vector<uint8_t> frame_copy(frame_ring.frame_bytes());

    while (!g_stop) {
      const uint64_t now_ns = k230_now_ns();
      if (now_ns >= next_config_poll_ns) {
        next_config_poll_ns = now_ns + kConfigPollIntervalNs;
        const uint64_t revision = file_revision(config_path);
        if (revision != config_revision) {
          config_revision = revision;
          try {
            const bool enabled = read_recording_enabled(config_path, false);
            if (!enabled && writer.requested_enabled()) drain_encoder();
            writer.set_enabled(enabled, now_ns);
          } catch (const std::exception &error) {
            std::fprintf(stderr, "recordd: config error: %s\n", error.what());
          }
        }
      }

      const bool need_video_frame = !warmed || writer.requested_enabled();
      K230RoadAiFrame frame;
      if (need_video_frame &&
          record_frame_sub.read_new(&frame_seq, &frame, sizeof(frame), 10)) {
        ++selected_frames;

        const uint64_t frame_now_ns = k230_now_ns();
        const uint64_t age_ns = frame_now_ns >= frame.timestamp_ns
            ? frame_now_ns - frame.timestamp_ns : UINT64_MAX;
        if (frame.slot < frame_ring.slot_count() &&
            frame.width == frame_ring.width() && frame.height == frame_ring.height()) {
          if (age_ns <= kMaximumFrameAgeNs) {
            if (frame_ring.copy_slot(frame.slot, frame.frame_id,
                                     frame_copy.data(), frame_copy.size())) {
              if (encoder.submit_frame(frame_copy.data(), frame_copy.size(), frame,
                                       packet_handler)) {
                warmed = true;
              } else {
                ++dropped_frames;
              }
            } else {
              ++frame_sync_failures;
            }
          } else {
            ++stale_frames;
          }
        }
      } else if (!need_video_frame) {
        usleep(10000);
      }
      if (encoder.submitted_frames() != encoder.encoded_frames())
        encoder.drain(packet_handler);

      K230CanBatch batch;
      while (can_log_sub.pop(&batch)) writer.write_can(K230RecordType::CanRx, batch);
      while (sendcan_log_sub.pop(&batch)) writer.write_can(K230RecordType::CanTx, batch);

      if (writer.requested_enabled()) {
        open_optional_channel(model_sub, &model_open, kK230ModelStateTopic,
                              sizeof(K230ModelState));
        open_optional_channel(control_sub, &control_open, kK230ControlStateTopic,
                              sizeof(K230ControlState));
        open_optional_channel(panda_sub, &panda_open, kK230PandaStateTopic,
                              sizeof(K230PandaState));
        K230ModelState model_state;
        if (model_open && model_sub.read_new(&model_seq, &model_state,
                                             sizeof(model_state), 0)) {
          writer.write_state(K230RecordType::ModelState, model_state.model_timestamp_ns,
                             &model_state, sizeof(model_state));
        }
        K230ControlState control_state;
        if (control_open && control_sub.read_new(&control_seq, &control_state,
                                                 sizeof(control_state), 0)) {
          writer.write_state(K230RecordType::ControlState, control_state.timestamp_ns,
                             &control_state, sizeof(control_state));
        }
        K230PandaState panda_state;
        if (panda_open && panda_sub.read_new(&panda_seq, &panda_state,
                                             sizeof(panda_state), 0)) {
          writer.write_state(K230RecordType::PandaState, panda_state.timestamp_ns,
                             &panda_state, sizeof(panda_state));
        }
      }

      if (now_ns >= next_log_ns) {
        next_log_ns = now_ns + 1000000000ULL;
        std::fprintf(stderr,
                     "recordd: enabled=%u active=%u warmed=%u selected=%llu "
                     "submitted=%llu encoded=%llu dropped=%llu stale=%llu sync=%llu queues=%llu/%llu "
                     "frames=%llu write_queue_drops=%llu moves=%llu%s\n",
                     writer.requested_enabled() ? 1 : 0, writer.active() ? 1 : 0,
                     warmed ? 1 : 0,
                     static_cast<unsigned long long>(selected_frames),
                     static_cast<unsigned long long>(encoder.submitted_frames()),
                     static_cast<unsigned long long>(encoder.encoded_frames()),
                     static_cast<unsigned long long>(dropped_frames),
                     static_cast<unsigned long long>(stale_frames),
                     static_cast<unsigned long long>(frame_sync_failures),
                     static_cast<unsigned long long>(can_log_sub.depth()),
                     static_cast<unsigned long long>(sendcan_log_sub.depth()),
                     static_cast<unsigned long long>(writer.video_frames()),
                     static_cast<unsigned long long>(writer.queue_drops()),
                     static_cast<unsigned long long>(writer.pending_moves()),
                     writer.blocked_for_space() ? " storage-blocked" : "");
      }
    }

    drain_encoder();
    writer.close();
    std::fprintf(stderr, "recordd: stopped\n");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "recordd error: %s\n", error.what());
    return 1;
  }
}
