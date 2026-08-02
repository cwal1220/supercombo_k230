#include "app_config.h"
#include "alert_sound_player.h"
#include "display.h"
#include "k230_ipc.h"
#include "overlay_renderer.h"
#include "projection.h"
#include "thead.h"
#include "v4l2-drm.h"

#include <drm/drm_fourcc.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/videodev2.h>
#include <net/if.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace {

constexpr unsigned kSensorWidth = 1920;
constexpr unsigned kSensorHeight = 1080;
constexpr unsigned kLogicalDisplayWidth = 800;
constexpr unsigned kLogicalDisplayHeight = 480;

volatile sig_atomic_t g_stop = 0;

void signal_handler(int)
{
    g_stop = 1;
}

uint64_t timeval_us(const timeval &tv)
{
    return static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + tv.tv_usec;
}

constexpr const char *kDisplayReadyPath = "/tmp/k230_display_ready";
constexpr int kPreviewVideoDevice = 1;
constexpr unsigned kPreviewBufferCount = 8;
constexpr unsigned kDisplayReadyPreviewFrames = 30;
constexpr unsigned kOverlayBufferCount = 2;
constexpr uint64_t kStateFreshNs = 2000000000ULL;
// Leave margin below three 60 Hz display callbacks so redraws do not slip to 15 Hz.
constexpr uint64_t kOverlayIntervalNs = 45000000ULL;

const char *display_ready_path()
{
    return kDisplayReadyPath;
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

class SystemMonitor {
public:
    void sample(OverlayHudState *hud)
    {
        if (!hud) return;
        sample_cpu(&hud->cpu_percent);
        sample_memory(&hud->memory_percent);
        sample_temperature(&hud->cpu_temp_c);
        sample_network(hud);
    }

private:
    static float canaan_temperature_c(float raw_value)
    {
        const unsigned register_value = static_cast<unsigned>(raw_value);
        const float data = static_cast<float>(register_value & 0x0fffU);
        return ((((1.01472e-10f * data - 1.10063e-6f) * data + 4.36150e-3f) * data -
                 7.10128f) * data + 3565.87f);
    }

    void sample_cpu(float *percent)
    {
        FILE *file = std::fopen("/proc/stat", "r");
        if (!file) return;
        unsigned long long user = 0, nice = 0, system = 0, idle = 0;
        unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;
        const int count = std::fscanf(file, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                                      &user, &nice, &system, &idle, &iowait,
                                      &irq, &softirq, &steal);
        std::fclose(file);
        if (count < 4) return;

        const uint64_t idle_total = idle + iowait;
        const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
        if (previous_total_ != 0 && total > previous_total_) {
            const uint64_t total_delta = total - previous_total_;
            const uint64_t idle_delta = idle_total - previous_idle_;
            *percent = static_cast<float>(100.0 *
                (1.0 - static_cast<double>(std::min(idle_delta, total_delta)) / total_delta));
        }
        previous_total_ = total;
        previous_idle_ = idle_total;
    }

    void sample_memory(float *percent)
    {
        FILE *file = std::fopen("/proc/meminfo", "r");
        if (!file) return;
        unsigned long long total_kb = 0;
        unsigned long long available_kb = 0;
        char line[128];
        while (std::fgets(line, sizeof(line), file)) {
            std::sscanf(line, "MemTotal: %llu kB", &total_kb);
            std::sscanf(line, "MemAvailable: %llu kB", &available_kb);
        }
        std::fclose(file);
        if (total_kb > 0) {
            *percent = static_cast<float>(100.0 *
                (1.0 - static_cast<double>(std::min(available_kb, total_kb)) / total_kb));
        }
    }

    void sample_temperature(float *temperature_c)
    {
        float maximum = 0.0f;
        for (int i = 0; i < 16; ++i) {
            char path[96];
            std::snprintf(path, sizeof(path), "/sys/class/thermal/thermal_zone%d/temp", i);
            FILE *file = std::fopen(path, "r");
            if (!file) continue;
            float value = 0.0f;
            const bool read = std::fscanf(file, "%f", &value) == 1;
            std::fclose(file);
            if (!read) continue;

            char type_path[96];
            std::snprintf(type_path, sizeof(type_path),
                          "/sys/class/thermal/thermal_zone%d/type", i);
            FILE *type_file = std::fopen(type_path, "r");
            char type[64] = {};
            const bool type_read = type_file && std::fgets(type, sizeof(type), type_file);
            if (type_file) std::fclose(type_file);

            if (type_read && std::strncmp(type, "canaan_thermal_zone", 19) == 0 &&
                value >= 4096.0f) {
                value = canaan_temperature_c(value);
            } else if (value > 1000.0f) {
                value /= 1000.0f;
            }
            if (value > 0.0f && value < 200.0f) maximum = std::max(maximum, value);
        }
        *temperature_c = maximum;
    }

    void sample_network(OverlayHudState *hud)
    {
        hud->network_connected = false;
        hud->wifi_signal_dbm = 0;
        hud->network_interface[0] = '\0';
        hud->network_ipv4[0] = '\0';

        ifaddrs *addresses = nullptr;
        if (getifaddrs(&addresses) != 0) return;

        int best_score = -1;
        for (const ifaddrs *address = addresses; address; address = address->ifa_next) {
            if (!address->ifa_addr || address->ifa_addr->sa_family != AF_INET) continue;
            if ((address->ifa_flags & IFF_UP) == 0 ||
                (address->ifa_flags & IFF_LOOPBACK) != 0) {
                continue;
            }

            const char *name = address->ifa_name ? address->ifa_name : "";
            const int score = std::strcmp(name, "wlan0") == 0 ? 3 :
                              (std::strncmp(name, "wlan", 4) == 0 ? 2 : 1);
            if (score <= best_score) continue;

            char ipv4[INET_ADDRSTRLEN] = {};
            const sockaddr_in *socket_address =
                reinterpret_cast<const sockaddr_in *>(address->ifa_addr);
            if (!inet_ntop(AF_INET, &socket_address->sin_addr,
                           ipv4, sizeof(ipv4))) {
                continue;
            }

            best_score = score;
            hud->network_connected = true;
            std::snprintf(hud->network_interface, sizeof(hud->network_interface),
                          "%s", name);
            std::snprintf(hud->network_ipv4, sizeof(hud->network_ipv4),
                          "%s", ipv4);
        }
        freeifaddrs(addresses);

        if (!hud->network_connected ||
            std::strncmp(hud->network_interface, "wlan", 4) != 0) {
            return;
        }

        FILE *file = std::fopen("/proc/net/wireless", "r");
        if (!file) return;
        char line[160];
        while (std::fgets(line, sizeof(line), file)) {
            char interface_name[16] = {};
            unsigned status = 0;
            float link = 0.0f;
            float level = 0.0f;
            float noise = 0.0f;
            if (std::sscanf(line, " %15[^:]: %x %f %f %f",
                            interface_name, &status, &link, &level, &noise) == 5 &&
                std::strcmp(interface_name, hud->network_interface) == 0) {
                hud->wifi_signal_dbm = static_cast<int>(std::lround(level));
                break;
            }
        }
        std::fclose(file);
    }

    uint64_t previous_total_ = 0;
    uint64_t previous_idle_ = 0;
};

class K230OverlayDisplay;
K230OverlayDisplay *g_app = nullptr;

class K230OverlayDisplay {
public:
    explicit K230OverlayDisplay(const AppConfig &config)
        : profile_(config.profile)
    {
        default_projection_ = make_projection_state(config.manual_roll,
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
        if (!panda_state_sub_.open(kK230PandaStateTopic, sizeof(K230PandaState), true))
            throw std::runtime_error("open pandaState ipc failed");
        if (!control_state_sub_.open(kK230ControlStateTopic, sizeof(K230ControlState), true))
            throw std::runtime_error("open controlState ipc failed");
        if (!manager_state_sub_.open(kK230ManagerStateTopic, sizeof(K230ManagerState), true))
            throw std::runtime_error("open managerState ipc failed");

        display_ = display_init(0);
        if (!display_) throw std::runtime_error("display_init error");

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
        overlay_.draw(overlay_buffer_, ParsedModelOutput{}, default_projection_, hud_, true);
        clean(overlay_buffer_);
        display_->osd_disp_buffer = overlay_buffer_;

        std::fprintf(stderr,
                     "k230_overlay: display=%ux%u logical=%ux%u preview=/dev/video%d %ux%u buffers=%u rotation=%d overlay=native-direct\n",
                     display_->width, display_->height,
                     kLogicalDisplayWidth, kLogicalDisplayHeight, kPreviewVideoDevice,
                     context.width, context.height, context.buffer_num,
                     static_cast<int>(context.drm_rotation));
        std::fprintf(stderr,
                     "k230_overlay: waiting %u displayed preview frames before ready\n",
                     kDisplayReadyPreviewFrames);

        gettimeofday(&fps_tv_, nullptr);
        g_app = this;
        v4l2_drm_run_v4l2_2_drm_need_run = true;
        v4l2_drm_run_v4l2_2_drm(&context, 1, &K230OverlayDisplay::frame_handler);
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
        update_aux_state();

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
            if (profile_) {
                std::fprintf(stderr,
                             "overlay: poll=%.2f display=%.2f preview=%.2f model=%.2f overlay=%.2f model_seq=%llu draw=%.2fms present=%.2fms cpu=%.1f%% mem=%.1f%% temp=%.1fC errors=%u          \r",
                             poll_fps, display_fps, camera_fps, model_fps, overlay_fps,
                             static_cast<unsigned long long>(latest_model_seq_),
                             overlay_stats_.avg_ms_and_reset(),
                             present_stats_.avg_ms_and_reset(),
                             hud_.cpu_percent, hud_.memory_percent, hud_.cpu_temp_c,
                             errors_);
            } else {
                std::fprintf(stderr,
                             "overlay: poll=%.2f display=%.2f preview=%.2f model=%.2f overlay=%.2f model_seq=%llu cpu=%.1f%% mem=%.1f%% temp=%.1fC errors=%u          \r",
                             poll_fps, display_fps, camera_fps, model_fps, overlay_fps,
                             static_cast<unsigned long long>(latest_model_seq_),
                             hud_.cpu_percent, hud_.memory_percent, hud_.cpu_temp_c,
                             errors_);
            }
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
        latest_model_state_ = state;
        ++model_updates_;
        const uint64_t now = k230_now_ns();
        const bool model_fresh = state.model_timestamp_ns != 0 &&
            now >= state.model_timestamp_ns && now - state.model_timestamp_ns <= kStateFreshNs;
        have_model_state_ = state.valid != 0 && model_fresh;
        latest_output_ = k230_parsed_from_model_state(state);
        latest_projection_ = k230_projection_from_model_state(state);
        return true;
    }

    void update_aux_state()
    {
        uint64_t seq = latest_panda_seq_;
        if (panda_state_sub_.read(&latest_panda_state_, sizeof(latest_panda_state_), &seq) &&
            seq != latest_panda_seq_) {
            latest_panda_seq_ = seq;
        }

        seq = latest_control_seq_;
        if (control_state_sub_.read(&latest_control_state_, sizeof(latest_control_state_), &seq) &&
            seq != latest_control_seq_) {
            latest_control_seq_ = seq;
        }

        seq = latest_manager_seq_;
        if (manager_state_sub_.read(&latest_manager_state_, sizeof(latest_manager_state_), &seq) &&
            seq != latest_manager_seq_) {
            latest_manager_seq_ = seq;
        }
        refresh_hud_state();
    }

    void refresh_hud_state()
    {
        const uint64_t now = k230_now_ns();
        const bool model_fresh = latest_model_state_.model_timestamp_ns != 0 &&
            now >= latest_model_state_.model_timestamp_ns &&
            now - latest_model_state_.model_timestamp_ns <= kStateFreshNs;
        const bool panda_fresh = latest_panda_state_.timestamp_ns != 0 &&
            now >= latest_panda_state_.timestamp_ns &&
            now - latest_panda_state_.timestamp_ns <= kStateFreshNs;
        const bool control_fresh = latest_control_state_.timestamp_ns != 0 &&
            now >= latest_control_state_.timestamp_ns &&
            now - latest_control_state_.timestamp_ns <= kStateFreshNs;
        const bool manager_fresh = latest_manager_state_.timestamp_ns != 0 &&
            now >= latest_manager_state_.timestamp_ns &&
            now - latest_manager_state_.timestamp_ns <= kStateFreshNs;

        have_model_state_ = latest_model_state_.valid != 0 && model_fresh;

        hud_.panda_connected = panda_fresh && latest_panda_state_.connected != 0;
        hud_.panda_healthy = panda_fresh && latest_panda_state_.comms_healthy != 0;
        hud_.panda_tx_enabled = latest_panda_state_.tx_enabled != 0;
        hud_.panda_controls_allowed = panda_fresh && latest_panda_state_.controls_allowed != 0;
        hud_.panda_faults = panda_fresh ? latest_panda_state_.faults : 0;

        hud_.controller_enabled = control_fresh && latest_control_state_.enabled != 0;
        hud_.controller_engaged = control_fresh && latest_control_state_.engaged != 0;
        hud_.controller_active = control_fresh && latest_control_state_.active != 0;
        hud_.lateral_mode_available = control_fresh;
        hud_.laneless_mode =
            control_fresh &&
            (latest_control_state_.hud_flags & kK230HudFlagLaneless) != 0;
        hud_.vehicle_fresh = control_fresh && latest_control_state_.vehicle_fresh != 0;
        hud_.steering_fault = control_fresh && latest_control_state_.steering_fault != 0;
        hud_.left_blinker = control_fresh && latest_control_state_.left_blinker != 0;
        hud_.right_blinker = control_fresh && latest_control_state_.right_blinker != 0;
        hud_.cruise_active = control_fresh && latest_control_state_.cruise_active != 0;
        hud_.brake_hold =
            control_fresh &&
            (latest_control_state_.hud_flags & kK230HudFlagBrakeHold) != 0;
        hud_.gear = control_fresh ? latest_control_state_.gear : 0;
        hud_.speed_kph = control_fresh ? latest_control_state_.speed_kph : 0.0f;
        hud_.cruise_max_speed_kph =
            control_fresh ? latest_control_state_.cruise_max_speed_kph : 0.0f;
        hud_.cruise_command_speed_kph =
            control_fresh ? latest_control_state_.cruise_command_speed_kph
                          : 0.0f;
        hud_.radar_lead_valid =
            control_fresh && latest_control_state_.radar_lead_valid != 0;
        hud_.radar_lead_distance_m =
            control_fresh ? latest_control_state_.radar_lead_distance_m : 0.0f;
        hud_.radar_lead_relative_speed_mps =
            control_fresh
                ? latest_control_state_.radar_lead_relative_speed_mps
                : 0.0f;
        hud_.departure_alert_type = control_fresh
            ? static_cast<DepartureAlertType>(
                  latest_control_state_.departure_alert_type)
            : DepartureAlertType::none;
        hud_.departure_alert_event_id =
            control_fresh ? latest_control_state_.departure_alert_event_id : 0;
        hud_.green_light_alert_armed =
            control_fresh &&
            latest_control_state_.green_light_alert_armed != 0;
        hud_.tpms_valid =
            control_fresh && latest_control_state_.tpms_valid != 0;
        hud_.tpms_unit =
            control_fresh ? static_cast<int>(latest_control_state_.tpms_unit) : 0;
        hud_.tpms_pressure_fl =
            control_fresh ? latest_control_state_.tpms_pressure_fl : 0.0f;
        hud_.tpms_pressure_fr =
            control_fresh ? latest_control_state_.tpms_pressure_fr : 0.0f;
        hud_.tpms_pressure_rl =
            control_fresh ? latest_control_state_.tpms_pressure_rl : 0.0f;
        hud_.tpms_pressure_rr =
            control_fresh ? latest_control_state_.tpms_pressure_rr : 0.0f;
        hud_.tpms_warning =
            control_fresh && latest_control_state_.tpms_warning != 0;
        if (!control_fresh) {
            last_alert_sound_event_id_ = 0;
        } else if (hud_.departure_alert_type != DepartureAlertType::none &&
                   hud_.departure_alert_event_id != 0 &&
                   hud_.departure_alert_event_id !=
                       last_alert_sound_event_id_) {
            last_alert_sound_event_id_ = hud_.departure_alert_event_id;
            alert_sound_player_.play(hud_.departure_alert_type,
                                     hud_.departure_alert_event_id);
        }
        hud_.steering_angle_deg = control_fresh ? latest_control_state_.steering_angle_deg : 0.0f;
        hud_.normalized_output = control_fresh ? latest_control_state_.normalized_output : 0.0f;
        hud_.desired_torque = control_fresh ? latest_control_state_.desired_torque : 0;
        hud_.apply_torque = control_fresh ? latest_control_state_.apply_torque : 0;
        hud_.driver_torque = control_fresh ? latest_control_state_.driver_torque : 0;
        std::snprintf(hud_.active_block, sizeof(hud_.active_block), "%s",
                      control_fresh ? latest_control_state_.active_block : "control_stale");

        hud_.calibration_available = model_fresh;
        hud_.calibration_status = latest_model_state_.calibration.status;
        hud_.calibration_valid_blocks = latest_model_state_.calibration.valid_blocks;
        hud_.calibration_roll_deg = rad_to_deg(latest_model_state_.calibration.roll);
        hud_.calibration_pitch_deg = rad_to_deg(latest_model_state_.calibration.pitch);
        hud_.calibration_yaw_deg = rad_to_deg(latest_model_state_.calibration.yaw);

        const unsigned total_processes = manager_fresh
            ? std::min<unsigned>(latest_manager_state_.process_count, kK230MaxProcesses)
            : 0;
        unsigned running_processes = 0;
        for (unsigned i = 0; i < total_processes; ++i)
            running_processes += latest_manager_state_.processes[i].running ? 1U : 0U;
        hud_.services_healthy = manager_fresh && total_processes >= 3 &&
            running_processes == total_processes && have_model_state_;
    }

    void redraw_overlay()
    {
        overlay_buffer_index_ = (overlay_buffer_index_ + 1) % kOverlayBufferCount;
        overlay_buffer_ = overlay_buffers_[overlay_buffer_index_];
        const uint64_t draw_start = profile_ ? k230_now_ns() : 0;
        overlay_.draw(overlay_buffer_,
                      have_model_state_ ? latest_output_ : ParsedModelOutput{},
                      have_model_state_ ? latest_projection_ : default_projection_, hud_, true);
        if (profile_) overlay_stats_.add(k230_now_ns() - draw_start);

        const uint64_t present_start = profile_ ? k230_now_ns() : 0;
        clean(overlay_buffer_);
        display_->osd_disp_buffer = overlay_buffer_;
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

    OverlayRenderer overlay_;
    bool profile_ = false;

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
    AlertSoundPlayer alert_sound_player_;
    uint32_t last_alert_sound_event_id_ = 0;
    OverlayHudState hud_;
};

} // namespace

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        AppConfig config = AppConfig::from_env_defaults();
        K230OverlayDisplay app(config);
        return app.run();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "k230_overlay error: %s\n", e.what());
        return 1;
    }
}
