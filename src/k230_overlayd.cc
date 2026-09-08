#include "app_config.h"
#include "display.h"
#include "k230_ipc.h"
#include "overlay_renderer.h"
#include "piezo_buzzer.h"
#include "projection.h"
#include "system_monitor.h"
#include "thead.h"
#include "v4l2-drm.h"

#include <drm/drm_fourcc.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/videodev2.h>
#include <net/if.h>
#include <signal.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace {

constexpr unsigned kSensorWidth = 1920;
constexpr unsigned kSensorHeight = 1080;

volatile sig_atomic_t g_stop = 0;

constexpr const char *kDisplayReadyPath = "/tmp/k230_display_ready";
constexpr int kPreviewVideoDevice = 1;
constexpr unsigned kPreviewBufferCount = 8;
constexpr unsigned kDisplayReadyPreviewFrames = 30;
constexpr unsigned kOverlayBufferCount = 2;
constexpr uint64_t kStateFreshNs = 2000000000ULL;
// Leave margin below three 60 Hz display callbacks so redraws do not slip to 15 Hz.
constexpr uint64_t kOverlayIntervalNs = 45000000ULL;
// engage 거부 토스트 표시 시간.
constexpr uint64_t kEngageAlertNs = 3000000000ULL;
// 깜빡이 애니메이션 한 단계. 모델 갱신(≈20 Hz)마다 한 단계씩 나가던 속도를 유지한다.
constexpr uint64_t kTurnSignalStepNs = 50000000ULL;

std::string executable_dir()
{
    char path[512];
    const ssize_t length = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (length <= 0) return ".";
    path[length] = '\0';
    const std::string full(path);
    const size_t slash = full.rfind('/');
    return slash == std::string::npos ? "." : full.substr(0, slash);
}

OverlayTarget overlay_target(const display_buffer *buffer)
{
    return OverlayTarget{buffer->map, buffer->width, buffer->height, buffer->stride};
}

// 라벨 표는 overlay_renderer가 소유한다. 여기서는 토스트용 기본값만 얹는다.
const char *engage_block_text(const char *block)
{
    if (!block || block[0] == '\0') return "NOT READY";
    const char *label = engage_block_label(block);
    return label ? label : block;
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

class OverlayDisplay;
OverlayDisplay *g_app = nullptr;

class OverlayDisplay {
public:
    explicit OverlayDisplay(const AppConfig &config)
        : profile_(config.profile)
    {
        piezo_buzzer_ = piezo_buzzer_create();
        if (!piezo_buzzer_)
            std::fprintf(stderr, "k230_overlayd: piezo buzzer worker unavailable\n");
        default_projection_ = make_projection_state(config.manual_roll,
                                                    config.manual_pitch,
                                                    config.manual_yaw);
        overlay_.load_assets(executable_dir() + "/assets/ui");
    }

    ~OverlayDisplay()
    {
        piezo_buzzer_destroy(piezo_buzzer_);
        piezo_buzzer_ = nullptr;
        cleanup();
    }

    int run()
    {
        if (!model_state_sub_.open(kK230ModelStateTopic, sizeof(K230ModelState), true))
            throw std::runtime_error("open modelState ipc failed");
        if (!panda_state_sub_.open(kK230PandaStateTopic, sizeof(K230PandaState), true))
            throw std::runtime_error("open pandaState ipc failed");
        if (!control_state_sub_.open(kK230ControlStateTopic, sizeof(K230ControlState), true))
            throw std::runtime_error("open controlState ipc failed");
        if (!manager_state_sub_.open(kK230ManagerStateTopic, sizeof(K230ManagerState), true))
            throw std::runtime_error("open managerState ipc failed");

        display_ = display_init(0);
        if (!display_) throw std::runtime_error("display_init error");
        // 프리뷰 플레인과 같은 기준: 세로 패널이면 800x480 논리 화면을 회전해 그린다.
        rotate_landscape_ = display_->width < display_->height;

        v4l2_drm_context context {};
        v4l2_drm_default_context(&context);
        context.device = kPreviewVideoDevice;
        context.video_format = V4L2_PIX_FMT_NV12;
        context.display_format = 0;
        context.buffer_num = kPreviewBufferCount;

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
        overlay_plane_->drm_rotation = rotation_0;
        for (display_buffer *&buffer : overlay_buffers_) {
            buffer = display_allocate_buffer(overlay_plane_,
                                             display_->width,
                                             display_->height);
            if (!buffer) throw std::runtime_error("display_allocate_buffer ARGB failed");
            std::memset(buffer->map, 0, buffer->size);
            clean(buffer);
        }
        overlay_buffer_ = overlay_buffers_[0];
        overlay_.draw(overlay_target(overlay_buffer_), ParsedModelOutput{},
                      default_projection_, hud_, rotate_landscape_);
        clean(overlay_buffer_);
        display_->osd_disp_buffer = overlay_buffer_;

        std::fprintf(stderr,
                     "k230_overlayd: display=%ux%u logical=%ux%u preview=/dev/video%d %ux%u buffers=%u rotation=%d overlay=native-direct\n",
                     display_->width, display_->height,
                     rotate_landscape_ ? display_->height : display_->width,
                     rotate_landscape_ ? display_->width : display_->height, kPreviewVideoDevice,
                     context.width, context.height, context.buffer_num,
                     static_cast<int>(context.drm_rotation));
        std::fprintf(stderr,
                     "k230_overlayd: waiting %u displayed preview frames before ready\n",
                     kDisplayReadyPreviewFrames);

        gettimeofday(&fps_tv_, nullptr);
        g_app = this;
        v4l2_drm_run_v4l2_2_drm_need_run = true;
        v4l2_drm_run_v4l2_2_drm(&context, 1, &OverlayDisplay::frame_handler);
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
        pending_redraw_ = update_model() || pending_redraw_;
        pending_redraw_ = update_aux_state() || pending_redraw_;
        pending_redraw_ = update_turn_signal(k230_now_ns()) || pending_redraw_;

        if (displayed && overlay_buffer_) {
            display_buffer *current = nullptr;
            if (context[0].buffer_hold[context[0].wp] >= 0)
                current = context[0].display_buffers[context[0].buffer_hold[context[0].wp]];

            const bool preview_updated = current && current != last_preview_buffer_;
            if (preview_updated) {
                last_preview_buffer_ = current;
                if (!ready_file_written_) {
                    ++startup_preview_frames_;
                    if (startup_preview_frames_ >= kDisplayReadyPreviewFrames)
                        publish_display_ready();
                }
            }

            const uint64_t draw_now = k230_now_ns();
            const bool overlay_due = last_overlay_draw_ns_ == 0 ||
                draw_now - last_overlay_draw_ns_ >= kOverlayIntervalNs;
            if (pending_redraw_ && overlay_due) {
                pending_redraw_ = false;
                redraw_overlay();
                last_overlay_draw_ns_ = draw_now;
                ++overlay_frames_;
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
            const double model_fps = model_updates_ * 1000000.0 / duration;
            hud_.preview_fps = static_cast<float>(camera_fps);
            hud_.overlay_fps = static_cast<float>(overlay_fps);
            hud_.model_fps = static_cast<float>(model_fps);
            system_monitor_.sample(&hud_);
            refresh_hud_state();
            pending_redraw_ = true;
            char profile_text[64] = "";
            if (profile_) {
                std::snprintf(profile_text, sizeof(profile_text), " draw=%.2fms present=%.2fms",
                              overlay_stats_.avg_ms_and_reset(), present_stats_.avg_ms_and_reset());
            }
            std::fprintf(stderr,
                         "overlay: poll=%.2f display=%.2f preview=%.2f model=%.2f overlay=%.2f model_seq=%llu%s cpu=%.1f%% mem=%.1f%% disk=%.1f%% temp=%.1fC errors=%u          \r",
                         poll_fps, display_fps, camera_fps, model_fps, overlay_fps,
                         static_cast<unsigned long long>(latest_model_seq_), profile_text,
                         hud_.cpu_percent, hud_.memory_percent, hud_.storage_percent,
                         hud_.cpu_temp_c, errors_);
            std::fflush(stderr);
            poll_count_ = 0;
            display_frames_ = 0;
            overlay_frames_ = 0;
            model_updates_ = 0;
            context[0].frame_count = 0;
            fps_tv_ = now;
        }

        return g_stop ? 'q' : 0;
    }

    void cleanup()
    {
        for (display_buffer *&buffer : overlay_buffers_) {
            if (buffer) {
                display_free_buffer(buffer);
                buffer = nullptr;
            }
        }
        overlay_buffer_ = nullptr;
        if (overlay_plane_) {
            display_free_plane(overlay_plane_);
            overlay_plane_ = nullptr;
        }
        if (display_) {
            display_exit(display_);
            display_ = nullptr;
        }
        if (ready_file_written_) {
            unlink(kDisplayReadyPath);
            ready_file_written_ = false;
        }
    }

    uint32_t next_piezo_event_id()
    {
        if (++next_piezo_event_id_ == 0) next_piezo_event_id_ = 1;
        return next_piezo_event_id_;
    }

    void clean(display_buffer *buffer)
    {
        thead_csi_dcache_clean_invalid_range(buffer->map, buffer->size);
    }

    /* 새 스냅샷이면 저장하고 true. */
    template <typename State>
    static bool poll(K230LatestChannel &channel, State *state, uint64_t *seq)
    {
        State candidate;
        uint64_t candidate_seq = *seq;
        if (!channel.read(&candidate, sizeof(candidate), &candidate_seq) || candidate_seq == *seq)
            return false;
        *state = candidate;
        *seq = candidate_seq;
        return true;
    }

    bool update_model()
    {
        if (!poll(model_state_sub_, &latest_model_state_, &latest_model_seq_)) return false;
        ++model_updates_;
        have_model_state_ = latest_model_state_.valid != 0 &&
            fresh(latest_model_state_.model_timestamp_ns, k230_now_ns());
        latest_output_ = k230_parsed_from_model_state(latest_model_state_);
        latest_projection_ = k230_projection_from_model_state(latest_model_state_);
        return true;
    }

    /* 새 panda/control/manager 스냅샷이 있으면 true. 모델이 멈춰도 속도·토스트가
     * 제어 상태를 따라가도록 재그리기 트리거가 된다. */
    /* 새 panda/control/manager 스냅샷이 있으면 true. 모델이 멈춰도 속도·토스트가
     * 제어 상태를 따라가도록 재그리기 트리거가 된다. */
    bool update_aux_state()
    {
        bool changed = poll(panda_state_sub_, &latest_panda_state_, &latest_panda_seq_);
        changed = poll(control_state_sub_, &latest_control_state_, &latest_control_seq_) || changed;
        changed = poll(manager_state_sub_, &latest_manager_state_, &latest_manager_seq_) || changed;
        refresh_hud_state();
        return changed;
    }

    /* 깜빡이 단계는 켜진 시각 기준으로 나간다. 단계가 바뀌면 true. */
    bool update_turn_signal(uint64_t now_ns)
    {
        if (hud_.left_blinker != previous_left_blinker_ ||
            hud_.right_blinker != previous_right_blinker_) {
            previous_left_blinker_ = hud_.left_blinker;
            previous_right_blinker_ = hud_.right_blinker;
            turn_signal_start_ns_ = now_ns;
        }
        const bool blinking = hud_.left_blinker || hud_.right_blinker;
        const int step = blinking
            ? static_cast<int>(((now_ns - turn_signal_start_ns_) / kTurnSignalStepNs) %
                               kTurnSignalSteps)
            : 0;
        const bool changed = step != hud_.turn_signal_step;
        hud_.turn_signal_step = step;
        return changed && blinking;
    }

    static bool fresh(uint64_t timestamp_ns, uint64_t now)
    {
        return timestamp_ns != 0 && now >= timestamp_ns && now - timestamp_ns <= kStateFreshNs;
    }

    struct Freshness {
        bool model = false;
        bool panda = false;
        bool control = false;
        bool manager = false;
    };

    Freshness freshness(uint64_t now) const
    {
        return {fresh(latest_model_state_.model_timestamp_ns, now),
                fresh(latest_panda_state_.timestamp_ns, now),
                fresh(latest_control_state_.timestamp_ns, now),
                fresh(latest_manager_state_.timestamp_ns, now)};
    }

    /* 최신 스냅샷을 HUD 상태로 옮기고, control 이벤트 카운터로 토스트·부저를 낸다. */
    void refresh_hud_state()
    {
        const uint64_t now = k230_now_ns();
        const Freshness f = freshness(now);
        have_model_state_ = latest_model_state_.valid != 0 && f.model;
        hud_apply_panda_state(latest_panda_state_, f.panda, &hud_);
        hud_apply_control_state(latest_control_state_, f.control, &hud_);
        hud_apply_model_state(latest_model_state_, f.model, &hud_);
        hud_apply_manager_state(latest_manager_state_, f.manager, have_model_state_, &hud_);
        process_alert_events(f, now);
    }

    /* 이벤트 카운터는 공유 제어 상태에 있다. overlay가 독립적으로 재시작될
     * 수 있으므로 첫 번째 정상 스냅샷은 새 사용자 이벤트가 아니라 기준값으로
     * 처리한다. controlsd 재시작으로 카운터가 0부터 다시 시작한 경우에도
     * 전체 기준값을 다시 설정한다. 기준값을 잡은 프레임이면 true. */
    bool baseline_event_counters()
    {
        const auto counter_reset = [](uint32_t current, uint32_t previous) {
            return previous != 0 && current < previous;
        };
        const bool counters_reset =
            event_ids_initialized_ &&
            (counter_reset(latest_control_state_.engage_event_id, last_engage_event_id_) ||
             counter_reset(latest_control_state_.disengage_event_id, last_disengage_event_id_) ||
             counter_reset(latest_control_state_.engage_reject_event_id,
                           last_engage_reject_event_id_) ||
             counter_reset(latest_control_state_.departure_alert_event_id, last_departure_alert_event_id_));
        if (event_ids_initialized_ && !counters_reset) return false;
        last_engage_event_id_ = latest_control_state_.engage_event_id;
        last_disengage_event_id_ = latest_control_state_.disengage_event_id;
        last_engage_reject_event_id_ = latest_control_state_.engage_reject_event_id;
        last_departure_alert_event_id_ = latest_control_state_.departure_alert_event_id;
        event_ids_initialized_ = true;
        return true;
    }

    /* engage 거부 > engage > disengage 중 첫 새 이벤트 하나만. 울렸으면 true. */
    bool play_engagement_alert(uint64_t now)
    {
        const K230ControlState &c = latest_control_state_;
        if (c.engage_reject_event_id != 0 &&
            c.engage_reject_event_id != last_engage_reject_event_id_) {
            last_engage_reject_event_id_ = c.engage_reject_event_id;
            std::snprintf(hud_.engage_alert_message, sizeof(hud_.engage_alert_message),
                          "UNABLE TO ENGAGE: %s", engage_block_text(c.engage_reject_block));
            engage_alert_until_ns_ = now + kEngageAlertNs;
            piezo_buzzer_play(piezo_buzzer_, PIEZO_ALERT_UNABLE, last_engage_reject_event_id_);
            std::fprintf(stderr, "k230_overlayd: piezo alert=unable event=%u block=%s\n",
                         last_engage_reject_event_id_, c.engage_reject_block);
            return true;
        }
        if (c.engage_event_id != 0 && c.engage_event_id != last_engage_event_id_) {
            last_engage_event_id_ = c.engage_event_id;
            piezo_buzzer_play(piezo_buzzer_, PIEZO_ALERT_ENGAGE, last_engage_event_id_);
            std::fprintf(stderr, "k230_overlayd: piezo alert=engage event=%u\n",
                         last_engage_event_id_);
            return true;
        }
        if (c.disengage_event_id != 0 && c.disengage_event_id != last_disengage_event_id_) {
            last_disengage_event_id_ = c.disengage_event_id;
            piezo_buzzer_play(piezo_buzzer_, PIEZO_ALERT_DISENGAGE, last_disengage_event_id_);
            std::fprintf(stderr, "k230_overlayd: piezo alert=disengage event=%u\n",
                         last_disengage_event_id_);
            return true;
        }
        return false;
    }

    /* 두 가지 출발 감지는 모두 도로 상황의 변화로 처리한다. 울렸으면 true. */
    bool play_departure_alert()
    {
        if (hud_.departure_alert_type == DepartureAlertType::none ||
            latest_control_state_.departure_alert_event_id == 0 ||
            latest_control_state_.departure_alert_event_id == last_departure_alert_event_id_)
            return false;
        last_departure_alert_event_id_ = latest_control_state_.departure_alert_event_id;
        piezo_buzzer_play(piezo_buzzer_, PIEZO_ALERT_SIGNAL_CHANGED, latest_control_state_.departure_alert_event_id);
        std::fprintf(stderr, "k230_overlayd: piezo alert=signal_changed event=%u\n",
                     latest_control_state_.departure_alert_event_id);
        return true;
    }

    /* 가용 → 불가용 천이에만 울린다. active 천이는 정차 부근 path 깜빡임마다
     * 울리므로 소리내지 않는다. 같은 프레임에 다른 알림이 울렸으면 생략. */
    void play_availability_alert(const Freshness &f, bool suppressed)
    {
        const bool panda_unavailable =
            latest_panda_state_.timestamp_ns != 0 &&
            (!f.panda || !hud_.panda_connected || !hud_.panda_healthy ||
             latest_panda_state_.faults != 0);
        const bool unavailable =
            !f.control || panda_unavailable || latest_control_state_.steering_fault != 0;
        if (!alert_state_initialized_) {
            alert_state_initialized_ = true;
        } else if (unavailable && !previous_unavailable_ && !suppressed) {
            const uint32_t event_id = next_piezo_event_id();
            piezo_buzzer_play(piezo_buzzer_, PIEZO_ALERT_UNAVAILABLE, event_id);
            std::fprintf(stderr, "k230_overlayd: piezo alert=unavailable event=%u\n", event_id);
        }
        previous_unavailable_ = unavailable;
    }

    void process_alert_events(const Freshness &f, uint64_t now)
    {
        const bool process = f.control && !baseline_event_counters();
        const bool engagement = process && play_engagement_alert(now);
        if (now >= engage_alert_until_ns_) hud_.engage_alert_message[0] = '\0';
        const bool departure = process && !engagement && play_departure_alert();
        play_availability_alert(f, engagement || departure);
    }

    void redraw_overlay()
    {
        overlay_buffer_index_ = (overlay_buffer_index_ + 1) % kOverlayBufferCount;
        overlay_buffer_ = overlay_buffers_[overlay_buffer_index_];
        const uint64_t draw_start = profile_ ? k230_now_ns() : 0;
        overlay_.draw(overlay_target(overlay_buffer_),
                      have_model_state_ ? latest_output_ : ParsedModelOutput{},
                      have_model_state_ ? latest_projection_ : default_projection_, hud_,
                      rotate_landscape_);
        if (profile_) overlay_stats_.add(k230_now_ns() - draw_start);

        const uint64_t present_start = profile_ ? k230_now_ns() : 0;
        clean(overlay_buffer_);
        display_->osd_disp_buffer = overlay_buffer_;
        if (profile_) present_stats_.add(k230_now_ns() - present_start);
    }

    void publish_display_ready()
    {
        FILE *file = std::fopen(kDisplayReadyPath, "w");
        if (!file) {
            std::perror("k230_overlayd display ready fopen");
            return;
        }
        std::fprintf(file, "%llu\n", static_cast<unsigned long long>(k230_now_ns()));
        std::fclose(file);
        ready_file_written_ = true;
        std::fprintf(stderr, "k230_overlayd: display ready %s preview_frames=%u\n",
                     kDisplayReadyPath, startup_preview_frames_);
    }

    OverlayRenderer overlay_;
    bool profile_ = false;
    bool rotate_landscape_ = true;

    K230LatestChannel model_state_sub_;
    K230LatestChannel panda_state_sub_;
    K230LatestChannel control_state_sub_;
    K230LatestChannel manager_state_sub_;

    display *display_ = nullptr;
    display_plane *overlay_plane_ = nullptr;
    display_buffer *overlay_buffer_ = nullptr;
    std::array<display_buffer *, kOverlayBufferCount> overlay_buffers_ {};
    unsigned overlay_buffer_index_ = 0;
    display_buffer *last_preview_buffer_ = nullptr;
    uint64_t last_overlay_draw_ns_ = 0;

    uint64_t latest_model_seq_ = 0;
    uint64_t latest_panda_seq_ = 0;
    uint64_t latest_control_seq_ = 0;
    uint64_t latest_manager_seq_ = 0;
    K230ModelState latest_model_state_ {};
    K230PandaState latest_panda_state_ {};
    K230ControlState latest_control_state_ {};
    K230ManagerState latest_manager_state_ {};
    ParsedModelOutput latest_output_ {};
    ProjectionState latest_projection_ {};
    ProjectionState default_projection_ {};
    bool have_model_state_ = false;
    bool pending_redraw_ = true;
    bool ready_file_written_ = false;
    unsigned startup_preview_frames_ = 0;
    unsigned errors_ = 0;

    timeval fps_tv_ {};
    unsigned poll_count_ = 0;
    unsigned display_frames_ = 0;
    unsigned overlay_frames_ = 0;
    unsigned model_updates_ = 0;

    StageStats overlay_stats_;
    StageStats present_stats_;
    SystemMonitor system_monitor_;
    PiezoBuzzer *piezo_buzzer_ = nullptr;
    uint32_t last_departure_alert_event_id_ = 0;
    uint32_t last_engage_event_id_ = 0;
    uint32_t last_disengage_event_id_ = 0;
    uint32_t last_engage_reject_event_id_ = 0;
    bool event_ids_initialized_ = false;
    uint32_t next_piezo_event_id_ = 0;
    bool alert_state_initialized_ = false;
    bool previous_unavailable_ = false;
    uint64_t engage_alert_until_ns_ = 0;
    bool previous_left_blinker_ = false;
    bool previous_right_blinker_ = false;
    uint64_t turn_signal_start_ns_ = 0;
    OverlayHudState hud_;
};

} // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);

    try {
        AppConfig config = AppConfig::from_env_defaults();
        OverlayDisplay app(config);
        return app.run();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "k230_overlayd error: %s\n", e.what());
        return 1;
    }
}
