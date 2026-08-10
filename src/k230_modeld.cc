#include "app_config.h"
#include "calibration_service.h"
#include "input_source.h"
#include "k230_ipc.h"
#include "lateral_control.h"
#include "model_output.h"
#include "supercombo_model.h"

#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;

uint64_t steady_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
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
    if (!env_present("SUPERCOMBO_INPUT_WARP_FX"))
        replay_config.input_warp_fx = default_input_warp_fx(source.width());
    if (!env_present("SUPERCOMBO_INPUT_WARP_FY"))
        replay_config.input_warp_fy = default_input_warp_fy(source.height());
    if (!env_present("SUPERCOMBO_INPUT_WARP_CX"))
        replay_config.input_warp_cx = default_input_warp_cx(source.width());
    if (!env_present("SUPERCOMBO_INPUT_WARP_CY"))
        replay_config.input_warp_cy = default_input_warp_cy(source.height());
    const unsigned target_frames = config.max_frames > 0
        ? std::min(config.max_frames, source.frame_count())
        : source.frame_count();
    std::fprintf(stderr, "modeld replay input format=NV12 frames=%u file=%s target=%u\n",
                 source.frame_count(), config.replay_nv12_path.c_str(), target_frames);

    SupercomboModel model(config.kmodel_path.c_str(), config.debug_mode, replay_config);
    CalibrationService calibration(config);
    LateralControlDraft lateral_control;
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);

    Nv12Frame frame;
    std::vector<float> raw;
    unsigned processed = 0;
    unsigned errors = 0;
    timeval start {};
    timeval last {};
    gettimeofday(&start, nullptr);
    last = start;

    while (!g_stop && source.read(frame)) {
        const uint64_t t0 = steady_ns();
        const bool ok = model.run_frame_nv12_stable(frame.data.data(), frame.width, frame.height, raw);
        const uint64_t t1 = steady_ns();
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration, lateral_control,
                                processed, k230_now_ns(), model_ms, 0.0f)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            ++processed;
        } else {
            ++errors;
        }

        timeval now {};
        gettimeofday(&now, nullptr);
        const uint64_t since_last = timeval_us(now) - timeval_us(last);
        if (since_last >= 1000000ULL) {
            const uint64_t since_start = timeval_us(now) - timeval_us(start);
            const double fps = since_start > 0 ? processed * 1000000.0 / since_start : 0.0;
            std::fprintf(stderr, "modeld replay: frames=%u/%u fps=%.2f errors=%u          \r",
                         processed, target_frames, fps, errors);
            std::fflush(stderr);
            last = now;
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;
    }

    timeval end {};
    gettimeofday(&end, nullptr);
    const uint64_t duration = timeval_us(end) - timeval_us(start);
    const double fps = duration > 0 ? processed * 1000000.0 / duration : 0.0;
    std::fprintf(stderr, "\nmodeld replay done frames=%u errors=%u fps=%.2f\n",
                 processed, errors, fps);
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

    timeval start {};
    timeval last {};
    gettimeofday(&start, nullptr);
    last = start;
    unsigned last_processed = 0;
    unsigned last_errors = 0;
    std::vector<uint8_t> frame_copy(frame_ring.frame_bytes());

    std::fprintf(stderr, "modeld: live shared ring slots=%u frame=%ux%u bytes=%u target=%uHz\n",
                 frame_ring.slot_count(), frame_ring.width(), frame_ring.height(),
                 frame_ring.frame_bytes(), target_fps);

    while (!g_stop) {
        const uint64_t now_ns = steady_ns();
        if (next_model_start_ns > now_ns) {
            const uint64_t sleep_us = (next_model_start_ns - now_ns) / 1000ULL;
            if (sleep_us > 0) usleep(static_cast<useconds_t>(sleep_us));
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
        if (control_sub_open) {
            K230ControlState control_state;
            if (control_sub.read(&control_state, sizeof(control_state))) {
                v_ego = std::max(0.0f, control_state.speed_kph / 3.6f);
                desire = static_cast<int>(control_state.desire);
            }
        }
        model.set_desire(desire);

        // The recorder follows the exact frame selected by modeld, rather than
        // sampling camerad's higher-rate latest-frame stream independently.
        if (!record_frame_pub.publish(&meta, sizeof(meta))) {
            std::fprintf(stderr, "\nmodeld: publish recordFrame failed\n");
        }

        const uint64_t t0 = steady_ns();
        const bool ok = model.run_frame_nv12_stable(nv12, meta.width, meta.height, raw);
        const uint64_t t1 = steady_ns();
        next_model_start_ns = t0 + model_interval_ns;
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration, lateral_control,
                                meta.frame_id, meta.timestamp_ns, model_ms, v_ego)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            ++processed;
        } else {
            ++errors;
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;

        timeval now {};
        gettimeofday(&now, nullptr);
        const uint64_t duration = timeval_us(now) - timeval_us(last);
        if (duration >= 1000000ULL) {
            const unsigned frames_delta = processed - last_processed;
            const unsigned errors_delta = errors - last_errors;
            std::fprintf(stderr,
                         "modeld: fps=%.2f frames=%u missed=%u sync=%u errors=%u(+%u) last_ms=%.2f          \r",
                         frames_delta * 1000000.0 / duration,
                         processed,
                         missed,
                         frame_sync_failures,
                         errors,
                         errors_delta,
                         ok ? (t1 - t0) / 1000000.0 : 0.0);
            std::fflush(stderr);
            last = now;
            last_processed = processed;
            last_errors = errors;
        }
    }

    timeval end {};
    gettimeofday(&end, nullptr);
    const uint64_t total_us = timeval_us(end) - timeval_us(start);
    const double fps = total_us > 0 ? processed * 1000000.0 / total_us : 0.0;
    std::fprintf(stderr, "\nmodeld done frames=%u missed=%u sync=%u errors=%u fps=%.2f\n",
                 processed, missed, frame_sync_failures, errors, fps);
    return processed > 0 && errors == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char *argv[])
{
    install_stop_signal_handlers(&g_stop);

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
