#include "supercombo_runtime.h"

#include "input_source.h"
#include "setting.h"
#include "supercombo_model.h"
#include "thead.h"

#include <linux/videodev2.h>
#include <poll.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using std::cerr;
using std::cout;
using std::endl;

namespace {

SupercomboRuntime *g_runtime = nullptr;

int frame_handler_bridge(v4l2_drm_context *context, bool displayed)
{
    return g_runtime ? g_runtime->frame_handler(context, displayed) : 'q';
}

} // namespace

SupercomboRuntime::SupercomboRuntime(const AppConfig &config)
    : config_(config),
      calibration_(config_)
{
    latest_projection_ = calibration_.projection();
}

SupercomboRuntime::~SupercomboRuntime()
{
    request_stop();
}

void SupercomboRuntime::request_stop()
{
    display_stop_ = true;
    ai_stop_ = true;
    start_cv_.notify_all();
}

void SupercomboRuntime::mark_display_ready_once()
{
    {
        std::lock_guard<std::mutex> lock(start_mutex_);
        if (display_ready_) return;
        display_ready_ = true;
    }
    start_cv_.notify_all();
}

int SupercomboRuntime::run_replay()
{
    try {
        ReplayNv12Source source(config_.replay_nv12_path);
        const unsigned target_frames = config_.max_frames > 0
            ? std::min(config_.max_frames, source.frame_count())
            : source.frame_count();

        Nv12Frame frame;
        std::fprintf(stderr,
                     "replay input format=NV12 frames=%u file=%s target=%u\n",
                     source.frame_count(), config_.replay_nv12_path.c_str(), target_frames);
        std::fprintf(stderr, "replay mode=headless camera=off display=off\n");

        SupercomboModel model(config_.kmodel_path.c_str(), config_.debug_mode, config_);
        std::vector<float> raw;
        unsigned processed = 0;
        unsigned errors = 0;
        timeval start{};
        timeval last{};
        gettimeofday(&start, nullptr);
        last = start;

        while (!ai_stop_ && source.read(frame)) {
            const bool ok = model.run_frame_nv12(frame.data.data(), frame.width, frame.height, raw);
            if (ok) {
                ParsedModelOutput parsed = ModelOutputParser::parse(raw);
                calibration_.update(parsed);
                const ProjectionState projection = calibration_.projection();
                lateral_control_.update(parsed.plan, projection);
                ++processed;
            } else {
                ++errors;
            }

            timeval now{};
            gettimeofday(&now, nullptr);
            const uint64_t since_last = 1000000ULL * (now.tv_sec - last.tv_sec) + now.tv_usec - last.tv_usec;
            if (since_last >= 1000000) {
                const uint64_t since_start = 1000000ULL * (now.tv_sec - start.tv_sec) + now.tv_usec - start.tv_usec;
                const double fps = since_start > 0 ? processed * 1000000.0 / since_start : 0.0;
                std::fprintf(stderr, "replay: frames=%u/%u fps=%.2f errors=%u          \r",
                             processed, target_frames, fps, errors);
                std::fflush(stderr);
                last = now;
            }

            if (config_.max_frames > 0 && processed >= config_.max_frames) break;
        }

        timeval end{};
        gettimeofday(&end, nullptr);
        const uint64_t duration = 1000000ULL * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
        const double fps = duration > 0 ? processed * 1000000.0 / duration : 0.0;
        std::fprintf(stderr, "\nreplay done frames=%u errors=%u fps=%.2f\n", processed, errors, fps);
        return processed > 0 && errors == 0 ? 0 : 1;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "replay error: %s\n", e.what());
        return 1;
    }
}

void SupercomboRuntime::ai_thread_proc()
{
    {
        std::unique_lock<std::mutex> lock(start_mutex_);
        while (!display_ready_ && !ai_stop_)
            start_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    if (ai_stop_) return;

    try {
        LiveNv12Source source(config_, kd_mpi_get_vvcam_video00() + 1);
        SupercomboModel model(config_.kmodel_path.c_str(), config_.debug_mode, config_);
        Nv12Frame frame;
        std::vector<float> raw;
        unsigned processed_frames = 0;

        while (!ai_stop_) {
            if (!source.read(frame)) {
                ++ai_error_count_;
                if (source.eof()) break;
                continue;
            }

            const bool ok = model.run_frame_nv12(frame.data.data(), frame.width, frame.height, raw);
            if (ok) {
                ParsedModelOutput parsed = ModelOutputParser::parse(raw);
                calibration_.update(parsed);
                const ProjectionState projection = calibration_.projection();
                lateral_control_.update(parsed.plan, projection);
                {
                    std::lock_guard<std::mutex> lock(result_mutex_);
                    latest_output_ = parsed;
                    latest_projection_ = projection;
                }
                ++kpu_frame_count_;
                ++processed_frames;
                if (config_.max_frames > 0 && processed_frames >= config_.max_frames) {
                    request_stop();
                }
            } else {
                ++ai_error_count_;
            }
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "AI thread error: %s\n", e.what());
        ++ai_error_count_;
        request_stop();
    }
}

int SupercomboRuntime::frame_handler(v4l2_drm_context *context, bool displayed)
{
    response_count_ += 1;

    if (displayed && !display_ready_) {
        ++startup_display_frames_;
        constexpr unsigned kAiStartPreviewFrames = 30;
        if (startup_display_frames_ >= kAiStartPreviewFrames) {
            std::fprintf(stderr, "display preview ready after %u frames; starting AI stream\n",
                         startup_display_frames_);
            mark_display_ready_once();
        }
    }

    if (displayed && context[0].buffer_hold[context[0].wp] >= 0 && draw_buffer_) {
        auto buffer = context[0].display_buffers[context[0].buffer_hold[context[0].wp]];
        if (buffer != last_drawn_buffer_) {
            ParsedModelOutput output;
            ProjectionState projection;
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                output = latest_output_;
                projection = latest_projection_;
            }
            overlay_.draw(draw_buffer_, output, projection);
            thead_csi_dcache_clean_invalid_range(draw_buffer_->map, draw_buffer_->size);
            display_update_buffer(draw_buffer_, 0, 0);
            last_drawn_buffer_ = buffer;
            display_frame_count_ += 1;
        }
    }

    timeval now{};
    gettimeofday(&now, nullptr);
    const uint64_t duration = 1000000ULL * (now.tv_sec - fps_tv_.tv_sec) + now.tv_usec - fps_tv_.tv_usec;
    if (duration >= 1000000) {
        std::fprintf(stderr, "poll: %.2f display: %.2f camera: %.2f KPU: %.2f errors: %u          \r",
                     response_count_ * 1000000.0 / duration,
                     display_frame_count_ * 1000000.0 / duration,
                     context[0].frame_count * 1000000.0 / duration,
                     kpu_frame_count_.exchange(0) * 1000000.0 / duration,
                     ai_error_count_.load());
        std::fflush(stderr);
        response_count_ = 0;
        display_frame_count_ = 0;
        context[0].frame_count = 0;
        fps_tv_ = now;
    }

    return display_stop_ ? 'q' : 0;
}

void SupercomboRuntime::display_thread_proc()
{
    v4l2_drm_context context;
    v4l2_drm_default_context(&context);
    context.device = kd_mpi_get_vvcam_video00();

    if (display_->width > display_->height) {
        context.width = display_->width;
        context.height = (display_->width * SENSOR_HEIGHT / SENSOR_WIDTH) & 0xfff8;
        context.video_format = V4L2_PIX_FMT_NV12;
        context.display_format = 0;
        context.drm_rotation = rotation_0;
    } else {
        context.width = display_->height;
        context.height = display_->width;
        context.video_format = V4L2_PIX_FMT_NV12;
        context.display_format = 0;
        context.drm_rotation = rotation_90;
    }

    if (v4l2_drm_setup(&context, 1, &display_)) {
        cerr << "display v4l2_drm_setup error" << endl;
        request_stop();
        return;
    }

    display_plane *plane = display_get_plane(display_, DRM_FORMAT_ARGB8888);
    draw_buffer_ = display_allocate_buffer(plane, display_->width, display_->height);
    if (!draw_buffer_) {
        cerr << "display_allocate_buffer error" << endl;
        request_stop();
        return;
    }
    std::memset(draw_buffer_->map, 0, draw_buffer_->size);
    thead_csi_dcache_clean_invalid_range(draw_buffer_->map, draw_buffer_->size);
    display_commit_buffer(draw_buffer_, 0, 0);

    cout << "display " << display_->width << "x" << display_->height
         << ", preview " << context.width << "x" << context.height << endl;
    gettimeofday(&fps_tv_, nullptr);

    g_runtime = this;
    v4l2_drm_run(&context, 1, frame_handler_bridge);
    g_runtime = nullptr;

    if (plane) display_free_plane(plane);
    if (display_) {
        display_exit(display_);
        display_ = nullptr;
    }
}

int SupercomboRuntime::run_live(volatile sig_atomic_t *signal_stop)
{
    display_ = display_init(0);
    if (!display_) {
        cerr << "display_init error" << endl;
        return 1;
    }

    std::thread ai_thread(&SupercomboRuntime::ai_thread_proc, this);
    std::thread display_thread(&SupercomboRuntime::display_thread_proc, this);

    cout << "press 'q' then Enter or Ctrl+C to exit" << endl;
    while (!display_stop_ && !ai_stop_) {
        if (signal_stop && *signal_stop) break;

        pollfd stdin_poll{};
        stdin_poll.fd = STDIN_FILENO;
        stdin_poll.events = POLLIN;
        const int poll_ret = poll(&stdin_poll, 1, 200);
        if (poll_ret < 0) {
            if (errno == EINTR || (signal_stop && *signal_stop)) break;
            std::perror("poll stdin");
            break;
        }
        if (poll_ret == 0) continue;

        if (stdin_poll.revents & (POLLERR | POLLHUP | POLLNVAL)) break;
        if (stdin_poll.revents & POLLIN) {
            std::string input;
            if (!std::getline(std::cin, input)) break;
            if (input == "q") break;
        }
    }

    request_stop();
    usleep(100000);

    display_thread.join();
    ai_thread.join();
    return ai_error_count_.load() == 0 ? 0 : 1;
}
