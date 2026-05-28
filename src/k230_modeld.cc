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
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

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

uint64_t steady_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

class RawDumpWriter {
public:
    explicit RawDumpWriter(const std::string &path)
    {
        if (!path.empty())
            file_ = std::fopen(path.c_str(), "wb+");
        if (!file_ && !path.empty())
            std::perror("fopen SUPERCOMBO_DUMP_RAW");
    }

    ~RawDumpWriter()
    {
        close();
    }

    void write(const std::vector<float> &raw)
    {
        if (!file_) return;
        if (!header_written_) {
            raw_size_ = static_cast<uint32_t>(raw.size());
            const char magic[8] = {'S', 'C', 'O', 'D', 'M', 'P', '1', '\0'};
            std::fwrite(magic, 1, sizeof(magic), file_);
            std::fwrite(&raw_size_, sizeof(raw_size_), 1, file_);
            std::fwrite(&frame_count_, sizeof(frame_count_), 1, file_);
            header_written_ = true;
        }
        if (raw.size() != raw_size_) {
            std::fprintf(stderr, "raw dump size mismatch: first=%u current=%zu\n",
                         raw_size_, raw.size());
            return;
        }
        std::fwrite(raw.data(), sizeof(float), raw.size(), file_);
        ++frame_count_;
    }

    void close()
    {
        if (!file_) return;
        if (header_written_) {
            std::fflush(file_);
            std::fseek(file_, 8 + static_cast<long>(sizeof(raw_size_)), SEEK_SET);
            std::fwrite(&frame_count_, sizeof(frame_count_), 1, file_);
        }
        std::fclose(file_);
        file_ = nullptr;
    }

private:
    FILE *file_ = nullptr;
    bool header_written_ = false;
    uint32_t raw_size_ = 0;
    uint32_t frame_count_ = 0;
};

bool publish_output(K230LatestChannel &model_pub, const ParsedModelOutput &parsed,
                    CalibrationService &calibration, LateralControlDraft &lateral_control,
                    uint64_t frame_id, uint64_t capture_timestamp_ns, float model_ms)
{
    calibration.update(parsed);
    const ProjectionState projection = calibration.projection();
    const LateralTarget lateral = lateral_control.update(parsed.plan, projection);

    K230ModelState state;
    k230_fill_model_state(state, parsed, projection, calibration.snapshot(), lateral,
                          frame_id, capture_timestamp_ns, model_ms);
    return model_pub.publish(&state, sizeof(state));
}

int run_replay(const AppConfig &config, K230LatestChannel &model_pub)
{
    ReplayNv12Source source(config.replay_nv12_path);
    const unsigned target_frames = config.max_frames > 0
        ? std::min(config.max_frames, source.frame_count())
        : source.frame_count();
    std::fprintf(stderr, "modeld replay input format=NV12 frames=%u file=%s target=%u\n",
                 source.frame_count(), config.replay_nv12_path.c_str(), target_frames);

    SupercomboModel model(config.kmodel_path.c_str(), config.debug_mode);
    CalibrationService calibration(config);
    LateralControlDraft lateral_control(config);
    RawDumpWriter raw_dump(config.raw_dump_path);

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
        const bool ok = model.run_frame_nv12(frame.data.data(), frame.width, frame.height, raw);
        const uint64_t t1 = steady_ns();
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, parsed, calibration, lateral_control,
                                processed, k230_now_ns(), model_ms)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            raw_dump.write(raw);
            ModelOutputParser::maybe_log_pose(parsed, config.log_pose);
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

int run_live(const AppConfig &config, K230LatestChannel &model_pub)
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

    SupercomboModel model(config.kmodel_path.c_str(), config.debug_mode);
    CalibrationService calibration(config);
    LateralControlDraft lateral_control(config);
    RawDumpWriter raw_dump(config.raw_dump_path);
    std::vector<float> raw;
    uint64_t last_frame_seq = 0;
    unsigned processed = 0;
    unsigned errors = 0;
    unsigned missed = 0;
    uint64_t last_frame_id = 0;
    bool have_last_frame_id = false;

    timeval start {};
    timeval last {};
    gettimeofday(&start, nullptr);
    last = start;
    unsigned last_processed = 0;
    unsigned last_errors = 0;

    std::fprintf(stderr, "modeld: live shared ring slots=%u frame=%ux%u bytes=%u\n",
                 frame_ring.slot_count(), frame_ring.width(), frame_ring.height(),
                 frame_ring.frame_bytes());

    while (!g_stop) {
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

        const uint8_t *nv12 = frame_ring.slot(meta.slot);
        if (!nv12) {
            ++errors;
            continue;
        }

        const uint64_t t0 = steady_ns();
        const bool ok = model.run_frame_nv12(nv12, meta.width, meta.height, raw);
        const uint64_t t1 = steady_ns();
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, parsed, calibration, lateral_control,
                                meta.frame_id, meta.timestamp_ns, model_ms)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            raw_dump.write(raw);
            ModelOutputParser::maybe_log_pose(parsed, config.log_pose);
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
                         "modeld: fps=%.2f frames=%u missed=%u errors=%u(+%u) last_ms=%.2f          \r",
                         frames_delta * 1000000.0 / duration,
                         processed,
                         missed,
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
    std::fprintf(stderr, "\nmodeld done frames=%u missed=%u errors=%u fps=%.2f\n",
                 processed, missed, errors, fps);
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
        if (!model_pub.open(kK230ModelStateTopic, sizeof(K230ModelState), true))
            throw std::runtime_error("open modelState ipc failed");

        if (config.replay_enabled()) return run_replay(config, model_pub);
        return run_live(config, model_pub);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "modeld error: %s\n", e.what());
        std::fprintf(stderr, "%s\n", AppConfig::usage(argc > 0 ? argv[0] : "k230_modeld").c_str());
        return 1;
    }
}
