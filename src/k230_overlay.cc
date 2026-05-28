#include "app_config.h"
#include "display.h"
#include "k230_ipc.h"
#include "overlay_renderer.h"
#include "projection.h"
#include "thead.h"
#include "v4l2-drm.h"

#include <drm/drm_fourcc.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace {

constexpr unsigned kSensorWidth = 1920;
constexpr unsigned kSensorHeight = 1080;

volatile sig_atomic_t g_stop = 0;

void signal_handler(int)
{
    g_stop = 1;
}

uint64_t timeval_us(const timeval &tv)
{
    return static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + tv.tv_usec;
}

int env_int(const char *name, int default_value, int min_value, int max_value)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') return default_value;
    char *end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value) return default_value;
    return std::max(min_value, std::min(max_value, static_cast<int>(parsed)));
}

bool env_bool(const char *name, bool default_value)
{
    const char *value = std::getenv(name);
    if (!value) return default_value;
    return value[0] != '\0' && std::strcmp(value, "0") != 0;
}

const char *display_ready_path()
{
    const char *path = std::getenv("K230_DISPLAY_READY_FILE");
    return path && path[0] != '\0' ? path : "/tmp/k230_display_ready";
}

struct StageStats {
    uint64_t total_ns = 0;
    uint32_t count = 0;

    void add(uint64_t elapsed_ns)
    {
        total_ns += elapsed_ns;
        ++count;
    }

    double avg_ms_and_reset()
    {
        const double avg = count > 0
            ? static_cast<double>(total_ns) / static_cast<double>(count) / 1000000.0
            : 0.0;
        total_ns = 0;
        count = 0;
        return avg;
    }
};

class K230OverlayDisplay;
K230OverlayDisplay *g_app = nullptr;

class K230OverlayDisplay {
public:
    explicit K230OverlayDisplay(const AppConfig &config)
        : config_(config),
          overlay_(config),
          video_device_(env_int("K230_OVERLAY_VIDEO_DEVICE", 1, 0, 16)),
          profile_(env_bool("K230_OVERLAY_PROFILE", false)),
          display_ready_preview_frames_(env_int("K230_DISPLAY_READY_PREVIEW_FRAMES", 30, 0, 300))
    {
        default_projection_ = make_projection_state(config.projection_mode,
                                                    config.manual_roll,
                                                    config.manual_pitch,
                                                    config.manual_yaw);
    }

    ~K230OverlayDisplay()
    {
        cleanup();
    }

    int run()
    {
        if (!model_state_sub_.open(kK230ModelStateTopic, sizeof(K230ModelState), true))
            throw std::runtime_error("open modelState ipc failed");

        display_ = display_init(0);
        if (!display_) throw std::runtime_error("display_init error");

        v4l2_drm_context context {};
        v4l2_drm_default_context(&context);
        context.device = static_cast<unsigned>(video_device_);
        context.video_format = V4L2_PIX_FMT_NV12;
        context.display_format = 0;

        if (display_->width > display_->height) {
            context.width = display_->width;
            context.height = (display_->width * kSensorHeight / kSensorWidth) & 0xfff8;
            context.drm_rotation = rotation_0;
        } else {
            context.width = display_->height;
            context.height = display_->width;
            context.drm_rotation = rotation_90;
        }

        if (v4l2_drm_setup(&context, 1, &display_) != 0)
            throw std::runtime_error("display v4l2_drm_setup failed");

        overlay_plane_ = display_get_plane(display_, DRM_FORMAT_ARGB8888);
        if (!overlay_plane_) throw std::runtime_error("display_get_plane ARGB failed");
        overlay_buffer_ = display_allocate_buffer(overlay_plane_, display_->width, display_->height);
        if (!overlay_buffer_) throw std::runtime_error("display_allocate_buffer ARGB failed");
        std::memset(overlay_buffer_->map, 0, overlay_buffer_->size);
        clean(overlay_buffer_);
        display_commit_buffer(overlay_buffer_, 0, 0);

        std::fprintf(stderr,
                     "k230_overlay: display=%ux%u preview=/dev/video%d %ux%u rotation=%d projection=%s\n",
                     display_->width, display_->height, video_device_,
                     context.width, context.height, static_cast<int>(context.drm_rotation),
                     projection_mode_name(config_.projection_mode));
        if (display_ready_preview_frames_ == 0) {
            publish_display_ready();
        } else {
            std::fprintf(stderr,
                         "k230_overlay: waiting %u displayed preview frames before ready\n",
                         display_ready_preview_frames_);
        }

        gettimeofday(&fps_tv_, nullptr);
        g_app = this;
        v4l2_drm_run(&context, 1, &K230OverlayDisplay::frame_handler);
        g_app = nullptr;

        std::fprintf(stderr, "\noverlay done errors=%u\n", errors_);
        return errors_ == 0 ? 0 : 1;
    }

private:
    static int frame_handler(v4l2_drm_context *context, bool displayed)
    {
        return g_app ? g_app->on_frame(context, displayed) : 'q';
    }

    int on_frame(v4l2_drm_context *context, bool displayed)
    {
        ++poll_count_;
        const bool model_updated = update_model();

        if (displayed && overlay_buffer_) {
            if (!ready_file_written_) {
                ++startup_preview_frames_;
                if (startup_preview_frames_ >= display_ready_preview_frames_)
                    publish_display_ready();
            }

            display_buffer *current = nullptr;
            if (context[0].buffer_hold[context[0].wp] >= 0)
                current = context[0].display_buffers[context[0].buffer_hold[context[0].wp]];

            if (current != last_preview_buffer_ || model_updated || force_redraw_) {
                force_redraw_ = false;
                redraw_overlay();
                ++overlay_frames_;
                last_preview_buffer_ = current;
            }
            ++display_frames_;
        }

        timeval now {};
        gettimeofday(&now, nullptr);
        const uint64_t duration = timeval_us(now) - timeval_us(fps_tv_);
        if (duration >= 1000000ULL) {
            const double poll_fps = poll_count_ * 1000000.0 / duration;
            const double display_fps = display_frames_ * 1000000.0 / duration;
            const double camera_fps = context[0].frame_count * 1000000.0 / duration;
            const double overlay_fps = overlay_frames_ * 1000000.0 / duration;
            if (profile_) {
                std::fprintf(stderr,
                             "overlay: poll=%.2f display=%.2f camera=%.2f overlay=%.2f model_seq=%llu draw=%.2fms present=%.2fms errors=%u          \r",
                             poll_fps, display_fps, camera_fps, overlay_fps,
                             static_cast<unsigned long long>(latest_model_seq_),
                             overlay_stats_.avg_ms_and_reset(),
                             present_stats_.avg_ms_and_reset(),
                             errors_);
            } else {
                std::fprintf(stderr,
                             "overlay: poll=%.2f display=%.2f camera=%.2f overlay=%.2f model_seq=%llu errors=%u          \r",
                             poll_fps, display_fps, camera_fps, overlay_fps,
                             static_cast<unsigned long long>(latest_model_seq_),
                             errors_);
            }
            std::fflush(stderr);
            poll_count_ = 0;
            display_frames_ = 0;
            overlay_frames_ = 0;
            context[0].frame_count = 0;
            fps_tv_ = now;
        }

        return g_stop ? 'q' : 0;
    }

    void cleanup()
    {
        if (overlay_buffer_) {
            display_free_buffer(overlay_buffer_);
            overlay_buffer_ = nullptr;
        }
        if (overlay_plane_) {
            display_free_plane(overlay_plane_);
            overlay_plane_ = nullptr;
        }
        if (display_) {
            display_exit(display_);
            display_ = nullptr;
        }
        if (ready_file_written_) {
            unlink(display_ready_path());
            ready_file_written_ = false;
        }
    }

    void clean(display_buffer *buffer)
    {
        thead_csi_dcache_clean_invalid_range(buffer->map, buffer->size);
    }

    bool update_model()
    {
        K230ModelState state;
        uint64_t seq = latest_model_seq_;
        if (!model_state_sub_.read(&state, sizeof(state), &seq) || seq == latest_model_seq_)
            return false;
        latest_model_seq_ = seq;
        have_model_state_ = state.valid != 0;
        latest_output_ = k230_parsed_from_model_state(state);
        latest_projection_ = k230_projection_from_model_state(state);
        return true;
    }

    void redraw_overlay()
    {
        const uint64_t draw_start = profile_ ? k230_now_ns() : 0;
        overlay_.draw(overlay_buffer_, have_model_state_ ? latest_output_ : ParsedModelOutput{},
                      have_model_state_ ? latest_projection_ : default_projection_);
        if (profile_) overlay_stats_.add(k230_now_ns() - draw_start);

        const uint64_t present_start = profile_ ? k230_now_ns() : 0;
        clean(overlay_buffer_);
        if (display_update_buffer(overlay_buffer_, 0, 0) != 0)
            ++errors_;
        if (profile_) present_stats_.add(k230_now_ns() - present_start);
    }

    void publish_display_ready()
    {
        FILE *file = std::fopen(display_ready_path(), "w");
        if (!file) {
            std::perror("k230_overlay display ready fopen");
            return;
        }
        std::fprintf(file, "%llu\n", static_cast<unsigned long long>(k230_now_ns()));
        std::fclose(file);
        ready_file_written_ = true;
        std::fprintf(stderr, "k230_overlay: display ready %s preview_frames=%u\n",
                     display_ready_path(), startup_preview_frames_);
    }

    AppConfig config_;
    OverlayRenderer overlay_;
    int video_device_ = 1;
    bool profile_ = false;
    unsigned display_ready_preview_frames_ = 30;

    K230LatestChannel model_state_sub_;

    display *display_ = nullptr;
    display_plane *overlay_plane_ = nullptr;
    display_buffer *overlay_buffer_ = nullptr;
    display_buffer *last_preview_buffer_ = nullptr;

    uint64_t latest_model_seq_ = 0;
    ParsedModelOutput latest_output_ {};
    ProjectionState latest_projection_ {};
    ProjectionState default_projection_ {};
    bool have_model_state_ = false;
    bool force_redraw_ = true;
    bool ready_file_written_ = false;
    unsigned startup_preview_frames_ = 0;
    unsigned errors_ = 0;

    timeval fps_tv_ {};
    unsigned poll_count_ = 0;
    unsigned display_frames_ = 0;
    unsigned overlay_frames_ = 0;

    StageStats overlay_stats_;
    StageStats present_stats_;
};

} // namespace

int main(int argc, char *argv[])
{
    (void)argc;
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        AppConfig config = AppConfig::from_env_defaults(argv[0]);
        K230OverlayDisplay app(config);
        return app.run();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "k230_overlay error: %s\n", e.what());
        return 1;
    }
}
