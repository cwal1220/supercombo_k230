#include "app_config.h"
#include "calibration_service.h"
#include "input_source.h"
#include "k230_ipc.h"
#include "lateral_control.h"
#include "model_output.h"
#include "supercombo_model.h"

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;
constexpr uint64_t kControlStateTimeoutNs = 500000000ULL;

void signal_handler(int)
{
    g_stop = 1;
}

uint64_t steady_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

void sleep_until_ns(uint64_t deadline_ns)
{
    timespec deadline {};
    deadline.tv_sec = static_cast<time_t>(deadline_ns / 1000000000ULL);
    deadline.tv_nsec = static_cast<long>(deadline_ns % 1000000000ULL);
    while (!g_stop) {
        const int result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
        if (result != EINTR) break;
    }
}

void print_latency_summary(const std::vector<double> &samples_ms, size_t warmup)
{
    if (samples_ms.size() <= warmup) return;
    std::vector<double> values(samples_ms.begin() + warmup, samples_ms.end());
    std::sort(values.begin(), values.end());
    auto percentile = [&](double fraction) {
        const size_t index = static_cast<size_t>(
            std::ceil(fraction * static_cast<double>(values.size())) - 1.0);
        return values[std::min(index, values.size() - 1)];
    };
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    const size_t deadline_misses = static_cast<size_t>(std::count_if(
        values.begin(), values.end(), [](double value) { return value > 50.0; }));
    std::fprintf(stderr,
                 "modeld latency steady ms: n=%zu mean=%.3f p50=%.3f p95=%.3f p99=%.3f max=%.3f over50=%zu\n",
                 values.size(), sum / values.size(), percentile(0.50), percentile(0.95),
                 percentile(0.99), values.back(), deadline_misses);
}

void print_cadence_summary(const std::vector<double> &intervals_ms, size_t warmup)
{
    if (intervals_ms.size() <= warmup) return;
    std::vector<double> values(intervals_ms.begin() + warmup, intervals_ms.end());
    std::sort(values.begin(), values.end());
    auto percentile = [&](double fraction) {
        const size_t index = static_cast<size_t>(
            std::ceil(fraction * static_cast<double>(values.size())) - 1.0);
        return values[std::min(index, values.size() - 1)];
    };
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    const double mean = sum / values.size();
    const size_t late = static_cast<size_t>(std::count_if(
        values.begin(), values.end(), [](double value) { return value > 55.0; }));
    std::fprintf(stderr,
                 "modeld cadence steady: n=%zu fps=%.3f mean_ms=%.3f p95=%.3f p99=%.3f max=%.3f over55=%zu\n",
                 values.size(), 1000.0 / mean, mean, percentile(0.95),
                 percentile(0.99), values.back(), late);
}

bool publish_output(K230LatestChannel &model_pub, SupercomboModel &model, const ParsedModelOutput &parsed,
                    CalibrationService &calibration, LateralControlDraft &lateral_control,
                    uint64_t frame_id, uint64_t capture_timestamp_ns, float model_ms,
                    float v_ego)
{
    calibration.update(parsed, v_ego);
    float input_rpy[3];
    calibration.input_rpy(input_rpy);
    model.set_input_calibration(input_rpy);

    const ProjectionState projection = calibration.projection();
    const LateralTarget lateral = lateral_control.update(parsed.plan);

    K230ModelState state;
    k230_fill_model_state(state, parsed, projection, calibration.snapshot(), lateral,
                          frame_id, capture_timestamp_ns, model_ms);
    return model_pub.publish(&state, sizeof(state));
}

int run_replay(const AppConfig &config, K230LatestChannel &model_pub)
{
    ReplayNv12Source source(config.replay_nv12_path);
    AppConfig replay_config = config;
    replay_config.nv12_width = source.width();
    replay_config.nv12_height = source.height();
    configure_k230_camera(replay_config, source.width(), source.height());
    const unsigned target_frames = config.max_frames > 0
        ? std::min(config.max_frames, source.frame_count())
        : source.frame_count();
    std::fprintf(stderr,
                 "modeld replay input format=NV12 frames=%u file=%s target=%u camera=k230_ov5647\n",
                 source.frame_count(), config.replay_nv12_path.c_str(), target_frames);

    SupercomboModel model(config.kmodel_path.c_str(), config.debug_mode, replay_config);
    CalibrationService calibration(replay_config);
    LateralControlDraft lateral_control;
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);

    Nv12Frame frame;
    std::vector<float> raw;
    std::fstream raw_dump;
    uint32_t raw_dump_frames = 0;
    if (!config.raw_output_dump_path.empty()) {
        raw_dump.open(config.raw_output_dump_path,
                      std::ios::binary | std::ios::out | std::ios::trunc);
        if (!raw_dump) throw std::runtime_error("open raw output dump failed");
        const char magic[8] = {'S', 'C', 'O', 'D', 'M', 'P', '1', '\0'};
        const uint32_t output_floats = 2576;
        raw_dump.write(magic, sizeof(magic));
        raw_dump.write(reinterpret_cast<const char *>(&output_floats), sizeof(output_floats));
        raw_dump.write(reinterpret_cast<const char *>(&raw_dump_frames), sizeof(raw_dump_frames));
    }
    unsigned processed = 0;
    unsigned errors = 0;
    std::vector<double> frame_latency_ms;
    const uint64_t start_ns = steady_ns();
    uint64_t last_report_ns = start_ns;

    while (!g_stop && source.read(frame)) {
        const uint64_t t0 = steady_ns();
        const bool ok = model.run_frame_nv12(frame.data.data(), frame.width, frame.height, raw);
        const uint64_t t1 = steady_ns();
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            if (!parsed.valid || !parsed.plan.valid) {
                std::fprintf(stderr, "\nmodeld: rejected unhealthy model output at frame %u\n", processed);
                ++errors;
                continue;
            }
            if (raw_dump.is_open()) {
                raw_dump.write(reinterpret_cast<const char *>(raw.data()),
                               static_cast<std::streamsize>(raw.size() * sizeof(float)));
                if (!raw_dump) throw std::runtime_error("write raw output dump failed");
                ++raw_dump_frames;
            }
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration, lateral_control,
                                processed, k230_now_ns(), model_ms, 0.0f)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            frame_latency_ms.push_back((steady_ns() - t0) / 1000000.0);
            ++processed;
        } else {
            ++errors;
        }

        const uint64_t now_ns = steady_ns();
        const uint64_t since_last_ns = now_ns - last_report_ns;
        if (since_last_ns >= 1000000000ULL) {
            const uint64_t since_start_ns = now_ns - start_ns;
            const double fps = since_start_ns > 0 ? processed * 1000000000.0 / since_start_ns : 0.0;
            std::fprintf(stderr, "modeld replay: frames=%u/%u fps=%.2f errors=%u          \r",
                         processed, target_frames, fps, errors);
            std::fflush(stderr);
            last_report_ns = now_ns;
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;
    }

    const uint64_t duration_ns = steady_ns() - start_ns;
    const double fps = duration_ns > 0 ? processed * 1000000000.0 / duration_ns : 0.0;
    std::fprintf(stderr, "\nmodeld replay done frames=%u errors=%u fps=%.2f\n",
                 processed, errors, fps);
    print_latency_summary(frame_latency_ms, 5);
    if (raw_dump.is_open()) {
        raw_dump.seekp(12, std::ios::beg);
        raw_dump.write(reinterpret_cast<const char *>(&raw_dump_frames), sizeof(raw_dump_frames));
        raw_dump.close();
    }
    return processed > 0 && errors == 0 ? 0 : 1;
}

int run_live(const AppConfig &config, K230LatestChannel &model_pub,
             K230LatestChannel &record_frame_pub)
{
    K230LatestChannel frame_sub;
    K230FrameRing frame_ring;
    if (!frame_sub.open(kK230RoadAiFrameTopic, sizeof(K230RoadAiFrame), true))
        throw std::runtime_error("open roadAiFrame ipc failed");

    while (!g_stop && !frame_ring.open(false)) {
        std::fprintf(stderr, "modeld: waiting for road ai frame ring\n");
        usleep(500000);
    }
    if (!frame_ring.valid()) return 1;

    SupercomboModel model(config.kmodel_path.c_str(), config.debug_mode, config);
    CalibrationService calibration(config);
    LateralControlDraft lateral_control;
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);
    K230LatestChannel control_sub;
    bool control_sub_open = false;
    float v_ego = 0.0f;
    int desire = 0;
    std::vector<float> raw;
    uint64_t last_frame_seq = 0;
    unsigned processed = 0;
    unsigned errors = 0;
    unsigned missed = 0;
    unsigned frame_sync_failures = 0;
    uint64_t last_frame_id = 0;
    bool have_last_frame_id = false;
    const unsigned target_fps = std::max(1U, std::min(config.model_fps, 30U));
    const uint64_t model_interval_ns = 1000000000ULL / target_fps;
    uint64_t next_model_start_ns = 0;
    std::vector<double> validation_latency_ms;
    std::vector<double> validation_cadence_ms;
    uint64_t previous_publish_ns = 0;
    std::vector<uint8_t> frame_copy(frame_ring.frame_bytes());

    const uint64_t start_ns = steady_ns();
    uint64_t last_report_ns = start_ns;
    unsigned last_processed = 0;
    unsigned last_errors = 0;

    std::fprintf(stderr, "modeld: live shared ring slots=%u frame=%ux%u bytes=%u target=%uHz\n",
                 frame_ring.slot_count(), frame_ring.width(), frame_ring.height(),
                 frame_ring.frame_bytes(), target_fps);

    while (!g_stop) {
        const uint64_t now_ns = steady_ns();
        if (next_model_start_ns > now_ns) {
            sleep_until_ns(next_model_start_ns);
        }
        if (g_stop) break;

        K230RoadAiFrame meta;
        if (!frame_sub.read_new(&last_frame_seq, &meta, sizeof(meta), 1000)) {
            std::fprintf(stderr, "modeld: waiting for roadAiFrame\n");
            continue;
        }
        if (meta.slot >= frame_ring.slot_count()) {
            ++errors;
            continue;
        }

        if (have_last_frame_id && meta.frame_id > last_frame_id + 1)
            missed += static_cast<unsigned>(meta.frame_id - last_frame_id - 1);
        have_last_frame_id = true;
        last_frame_id = meta.frame_id;

        if (!frame_ring.copy_slot(meta.slot, meta.frame_id,
                                  frame_copy.data(), frame_copy.size())) {
            ++frame_sync_failures;
            ++errors;
            continue;
        }
        const uint8_t *nv12 = frame_copy.data();

        if (!control_sub_open)
            control_sub_open = control_sub.open(kK230ControlStateTopic,
                                                sizeof(K230ControlState), false);
        bool control_state_fresh = false;
        if (control_sub_open) {
            K230ControlState control_state;
            if (control_sub.read(&control_state, sizeof(control_state))) {
                const uint64_t now_ns = k230_now_ns();
                control_state_fresh = control_state.timestamp_ns != 0 &&
                    now_ns >= control_state.timestamp_ns &&
                    now_ns - control_state.timestamp_ns <= kControlStateTimeoutNs;
                if (control_state_fresh) {
                    // 휠속도 0은 정차 상태로 유효하므로 CLU 표시속도로 대체하지 않는다.
                    v_ego = std::max(0.0f, control_state.ego_speed_kph / 3.6f);
                    desire = static_cast<int>(control_state.desire);
                }
            }
        }
        if (!control_state_fresh) {
            v_ego = 0.0f;
            desire = 0;
        }
        model.set_desire(desire);

        // The recorder follows the exact frame selected by modeld, rather than
        // sampling camerad's higher-rate latest-frame stream independently.
        if (!record_frame_pub.publish(&meta, sizeof(meta))) {
            std::fprintf(stderr, "\nmodeld: publish recordFrame failed\n");
        }

        const uint64_t t0 = steady_ns();
        const bool ok = model.run_frame_nv12(nv12, meta.width, meta.height, raw);
        const uint64_t t1 = steady_ns();
        if (next_model_start_ns == 0) {
            next_model_start_ns = t0 + model_interval_ns;
        } else {
            next_model_start_ns += model_interval_ns;
            // Keep an absolute 20 Hz phase so sleep overshoot does not accumulate.
            // If the process falls more than one full cycle behind, drop that cycle
            // rather than creating an unbounded catch-up burst.
            if (next_model_start_ns + model_interval_ns < t0)
                next_model_start_ns = t0 + model_interval_ns;
        }
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            if (!parsed.valid || !parsed.plan.valid) {
                std::fprintf(stderr, "\nmodeld: rejected unhealthy model output at frame %llu\n",
                             static_cast<unsigned long long>(meta.frame_id));
                ++errors;
                continue;
            }
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration, lateral_control,
                                meta.frame_id, meta.timestamp_ns, model_ms, v_ego)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            if (config.max_frames > 0)
                validation_latency_ms.push_back((steady_ns() - t0) / 1000000.0);
            if (config.max_frames > 0) {
                const uint64_t publish_ns = steady_ns();
                if (previous_publish_ns != 0)
                    validation_cadence_ms.push_back((publish_ns - previous_publish_ns) / 1000000.0);
                previous_publish_ns = publish_ns;
            }
            ++processed;
        } else {
            ++errors;
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;

        const uint64_t report_now_ns = steady_ns();
        const uint64_t duration_ns = report_now_ns - last_report_ns;
        if (duration_ns >= 1000000000ULL) {
            const unsigned frames_delta = processed - last_processed;
            const unsigned errors_delta = errors - last_errors;
            std::fprintf(stderr,
                         "modeld: fps=%.2f frames=%u missed=%u sync=%u errors=%u(+%u) last_ms=%.2f          \r",
                         frames_delta * 1000000000.0 / duration_ns,
                         processed,
                         missed,
                         frame_sync_failures,
                         errors,
                         errors_delta,
                         ok ? (t1 - t0) / 1000000.0 : 0.0);
            std::fflush(stderr);
            last_report_ns = report_now_ns;
            last_processed = processed;
            last_errors = errors;
        }
    }

    const uint64_t total_ns = steady_ns() - start_ns;
    const double fps = total_ns > 0 ? processed * 1000000000.0 / total_ns : 0.0;
    std::fprintf(stderr, "\nmodeld done frames=%u missed=%u sync=%u errors=%u fps=%.2f\n",
                 processed, missed, frame_sync_failures, errors, fps);
    print_latency_summary(validation_latency_ms, 5);
    print_cadence_summary(validation_cadence_ms, 5);
    return processed > 0 && errors == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        AppConfig config = AppConfig::from_env(argc, argv);
        K230LatestChannel model_pub;
        K230LatestChannel record_frame_pub;
        if (!model_pub.open(kK230ModelStateTopic, sizeof(K230ModelState), true))
            throw std::runtime_error("open modelState ipc failed");
        if (!record_frame_pub.open(kK230RecordFrameTopic, sizeof(K230RoadAiFrame), true))
            throw std::runtime_error("open recordFrame ipc failed");

        if (config.replay_enabled()) return run_replay(config, model_pub);
        return run_live(config, model_pub, record_frame_pub);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "modeld error: %s\n", e.what());
        std::fprintf(stderr, "%s\n", AppConfig::usage(argc > 0 ? argv[0] : "k230_modeld").c_str());
        return 1;
    }
}
