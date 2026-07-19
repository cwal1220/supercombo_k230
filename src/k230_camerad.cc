#include "app_config.h"
#include "input_source.h"
#include "k230_ipc.h"
#include "setting.h"

#include <linux/videodev2.h>
#include <signal.h>
#include <sys/time.h>

#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace {

volatile sig_atomic_t g_stop = 0;

void signal_handler(int)
{
    g_stop = 1;
}

uint64_t timeval_us(const timeval &tv)
{
    return static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + tv.tv_usec;
}

} // namespace

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        AppConfig config = AppConfig::from_env_defaults();
        K230LatestChannel frame_pub;
        K230FrameRing frame_ring;

        if (!frame_pub.open(kK230RoadAiFrameTopic, sizeof(K230RoadAiFrame), true))
            throw std::runtime_error("open roadAiFrame ipc failed");
        if (!frame_ring.open(true, config.nv12_width, config.nv12_height, kK230FrameSlots))
            throw std::runtime_error("open road ai frame ring failed");

        std::fprintf(stderr, "camerad: opening /dev/video2\n");

        LiveNv12Source source(config, kd_mpi_get_vvcam_video00() + 1);
        Nv12Frame frame;
        uint64_t frame_id = 0;
        unsigned errors = 0;

        timeval start {};
        timeval last {};
        gettimeofday(&start, nullptr);
        last = start;
        unsigned last_frames = 0;
        unsigned last_errors = 0;

        while (!g_stop) {
            if (!source.read(frame)) {
                ++errors;
                if (source.eof()) break;
                continue;
            }

            const size_t expected = static_cast<size_t>(frame_ring.frame_bytes());
            if (frame.data.size() != expected) {
                std::fprintf(stderr, "\ncamerad: frame size mismatch got=%zu expected=%zu\n",
                             frame.data.size(), expected);
                ++errors;
                continue;
            }

            const unsigned slot = static_cast<unsigned>(frame_id % frame_ring.slot_count());
            uint8_t *dst = frame_ring.slot(slot);
            if (!dst) {
                ++errors;
                continue;
            }
            std::memcpy(dst, frame.data.data(), frame.data.size());

            K230RoadAiFrame msg;
            msg.frame_id = frame_id;
            msg.timestamp_ns = k230_now_ns();
            msg.slot = slot;
            msg.width = frame.width;
            msg.height = frame.height;
            msg.format = V4L2_PIX_FMT_NV12;
            msg.crop_x = config.nv12_crop_x;
            msg.crop_y = config.nv12_crop_y;
            msg.crop_width = config.nv12_crop_width;
            msg.crop_height = config.nv12_crop_height;
            if (!frame_pub.publish(&msg, sizeof(msg))) {
                std::fprintf(stderr, "\ncamerad: publish roadAiFrame failed\n");
                ++errors;
            }

            ++frame_id;
            if (config.max_frames > 0 && frame_id >= config.max_frames) break;

            timeval now {};
            gettimeofday(&now, nullptr);
            const uint64_t duration = timeval_us(now) - timeval_us(last);
            if (duration >= 1000000ULL) {
                const unsigned frames_delta = static_cast<unsigned>(frame_id) - last_frames;
                const unsigned errors_delta = errors - last_errors;
                std::fprintf(stderr, "camerad: fps=%.2f frames=%llu errors=%u(+%u)          \r",
                             frames_delta * 1000000.0 / duration,
                             static_cast<unsigned long long>(frame_id),
                             errors,
                             errors_delta);
                std::fflush(stderr);
                last = now;
                last_frames = static_cast<unsigned>(frame_id);
                last_errors = errors;
            }
        }

        timeval end {};
        gettimeofday(&end, nullptr);
        const uint64_t total_us = timeval_us(end) - timeval_us(start);
        const double fps = total_us > 0 ? frame_id * 1000000.0 / total_us : 0.0;
        std::fprintf(stderr, "\ncamerad done frames=%llu errors=%u fps=%.2f\n",
                     static_cast<unsigned long long>(frame_id), errors, fps);
        return errors == 0 ? 0 : 1;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "camerad error: %s\n", e.what());
        return 1;
    }
}
