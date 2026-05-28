#ifndef SUPERCOMBO_RUNTIME_H
#define SUPERCOMBO_RUNTIME_H

#include "app_config.h"
#include "calibration_service.h"
#include "display.h"
#include "lateral_control.h"
#include "model_output.h"
#include "overlay_renderer.h"
#include "v4l2-drm.h"

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <mutex>
#include <sys/time.h>
#include <thread>

class SupercomboRuntime {
public:
    explicit SupercomboRuntime(const AppConfig &config);
    ~SupercomboRuntime();

    int run_replay();
    int run_live(volatile sig_atomic_t *signal_stop);
    void request_stop();
    int frame_handler(v4l2_drm_context *context, bool displayed);

private:
    void ai_thread_proc();
    void display_thread_proc();
    void mark_display_ready_once();

    AppConfig config_;
    OverlayRenderer overlay_;
    CalibrationService calibration_;
    LateralControlDraft lateral_control_;

    std::mutex result_mutex_;
    ParsedModelOutput latest_output_{};
    ProjectionState latest_projection_{};

    std::atomic<bool> ai_stop_{false};
    std::atomic<bool> display_stop_{false};
    std::atomic<unsigned> kpu_frame_count_{0};
    std::atomic<unsigned> ai_error_count_{0};

    std::mutex start_mutex_;
    std::condition_variable start_cv_;
    bool display_ready_ = false;

    display *display_ = nullptr;
    display_buffer *draw_buffer_ = nullptr;

    unsigned response_count_ = 0;
    unsigned display_frame_count_ = 0;
    unsigned startup_display_frames_ = 0;
    display_buffer *last_drawn_buffer_ = nullptr;
    timeval fps_tv_{};
};

#endif
