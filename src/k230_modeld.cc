#include "app_config.h"
#include "calibration_service.h"
#include "input_source.h"
#include "utils_process.h"
#include "utils_time.h"
#include "ipc_channels.h"
#include "ipc_messages.h"
#include "model_output.h"
#include "supercombo_model.h"

#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;

/* SUPERCOMBO_RAW_DUMP: replay 중 모델 raw 출력을 SCODMP1로 남긴다.
 * gtest/gtest_model_output_parser가 이 포맷을 읽어 보드 출력과 호스트
 * 기준을 프레임 단위로 비교할 수 있다(모델 교체 검증용). */
class RawOutputDump
{
public:
    explicit RawOutputDump(const char *path)
    {
        if (!path || !path[0]) return;
        file_ = std::fopen(path, "wb");
        if (!file_) {
            std::fprintf(stderr, "modeld: cannot open raw dump %s\n", path);
            return;
        }
        std::fwrite("SCODMP1\0", 1, 8, file_);
        const uint32_t placeholder = 0;
        std::fwrite(&placeholder, sizeof(placeholder), 1, file_);  // raw floats
        std::fwrite(&placeholder, sizeof(placeholder), 1, file_);  // frames
    }

    ~RawOutputDump()
    {
        if (!file_) return;
        std::fseek(file_, 8, SEEK_SET);
        std::fwrite(&raw_size_, sizeof(raw_size_), 1, file_);
        std::fwrite(&frames_, sizeof(frames_), 1, file_);
        std::fclose(file_);
        std::fprintf(stderr, "modeld: raw dump wrote %u frames x %u floats\n",
                     frames_, raw_size_);
    }

    void append(const std::vector<float> &raw)
    {
        if (!file_ || raw.empty()) return;
        if (raw_size_ == 0) raw_size_ = static_cast<uint32_t>(raw.size());
        if (raw.size() != raw_size_) return;
        std::fwrite(raw.data(), sizeof(float), raw.size(), file_);
        ++frames_;
    }

private:
    std::FILE *file_ = nullptr;
    uint32_t raw_size_ = 0;
    uint32_t frames_ = 0;
};

/* 1초 창 통계. 두 루프(라이브·리플레이)가 같은 시계로 fps를 센다. */
struct RateWindow
{
    timeval start{};
    timeval last{};
    unsigned last_processed = 0;
    unsigned last_errors = 0;

    RateWindow()
    {
        gettimeofday(&start, nullptr);
        last = start;
    }

    // 1초가 지났으면 창을 닫고 true. window_us는 닫힌 창의 길이.
    bool close_if_due(uint64_t *window_us)
    {
        timeval now{};
        gettimeofday(&now, nullptr);
        const uint64_t elapsed = timeval_us(now) - timeval_us(last);
        if (elapsed < 1000000ULL) return false;
        *window_us = elapsed;
        last = now;
        return true;
    }

    double total_fps(unsigned processed) const
    {
        timeval now{};
        gettimeofday(&now, nullptr);
        const uint64_t since_start = timeval_us(now) - timeval_us(start);
        return since_start > 0 ? processed * 1000000.0 / since_start : 0.0;
    }
};

/* controlsd 스냅샷에서 모델 입력용 자차 속도와 desire를 읽는다. controlsd가
 * 아직 없으면 열릴 때까지 마지막 값을 유지한다. */
class EgoStateReader
{
public:
    void poll()
    {
        if (!open_) open_ = sub_.open(kK230ControlStateTopic, sizeof(K230ControlState), false);
        if (!open_) return;
        K230ControlState control_state;
        if (!sub_.read(&control_state, sizeof(control_state))) return;
        const float ego_speed_kph = control_state.ego_speed_kph > 0.0f
            ? control_state.ego_speed_kph
            : control_state.cluster_speed_kph;
        v_ego_ = std::max(0.0f, ego_speed_kph / 3.6f);
        desire_ = static_cast<int>(control_state.desire);
    }
    float v_ego() const { return v_ego_; }
    int desire() const { return desire_; }

private:
    K230LatestChannel sub_;
    bool open_ = false;
    float v_ego_ = 0.0f;
    int desire_ = 0;
};

bool publish_output(K230LatestChannel &model_pub, SupercomboModel &model, const ParsedModelOutput &parsed,
                    CalibrationService &calibration,
                    uint64_t frame_id, uint64_t capture_timestamp_ns, float model_ms,
                    float v_ego)
{
    calibration.update(parsed, v_ego);
    float input_rpy[3];
    calibration.input_rpy(input_rpy);
    model.set_input_calibration(input_rpy);

    const ProjectionState projection = calibration.projection();

    K230ModelState state;
    k230_fill_model_state(state, parsed, projection, calibration.snapshot(),
                          frame_id, capture_timestamp_ns, model_ms);
    return model_pub.publish(&state, sizeof(state));
}

int run_replay(const AppConfig &config, K230LatestChannel &model_pub)
{
    ReplayNv12Source source(config.replay_nv12_path);
    /* 재생 소스 해상도에 맞춘 기본 워프. GPU 워프도 이 크기로 만들어져야
     * 라이브와 같은 경로를 탄다. */
    AppConfig replay_config = config;
    replay_config.nv12_width = source.width();
    replay_config.nv12_height = source.height();
    replay_config.input_warp_fx = default_input_warp_fx(source.width());
    replay_config.input_warp_fy = default_input_warp_fy(source.height());
    replay_config.input_warp_cx = default_input_warp_cx(source.width());
    replay_config.input_warp_cy = default_input_warp_cy(source.height());
    const unsigned target_frames = config.max_frames > 0
        ? std::min(config.max_frames, source.frame_count())
        : source.frame_count();
    std::fprintf(stderr, "modeld replay input format=NV12 frames=%u file=%s target=%u\n",
                 source.frame_count(), config.replay_nv12_path.c_str(), target_frames);

    SupercomboModel model(config.kmodel_path.c_str(), replay_config);
    CalibrationService calibration(config);
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);

    Nv12Frame frame;
    std::vector<float> raw;
    RawOutputDump raw_dump(std::getenv("SUPERCOMBO_RAW_DUMP"));
    unsigned processed = 0;
    unsigned errors = 0;
    RateWindow window;

    while (!g_stop && source.read(frame)) {
        const uint64_t t0 = k230_now_ns();
        const bool ok = model.run_frame_nv12(frame.data.data(), frame.width, frame.height, raw);
        const uint64_t t1 = k230_now_ns();
        if (ok) {
            raw_dump.append(raw);
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration,
                                processed, k230_now_ns(), model_ms, 0.0f)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            ++processed;
        } else {
            ++errors;
        }

        uint64_t window_us = 0;
        if (window.close_if_due(&window_us)) {
            std::fprintf(stderr, "modeld replay: frames=%u/%u fps=%.2f errors=%u          \r",
                         processed, target_frames, window.total_fps(processed), errors);
            std::fflush(stderr);
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;
    }

    std::fprintf(stderr, "\nmodeld replay done frames=%u errors=%u fps=%.2f\n",
                 processed, errors, window.total_fps(processed));
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

    SupercomboModel model(config.kmodel_path.c_str(), config);
    CalibrationService calibration(config);
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);
    EgoStateReader ego;
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
    RateWindow window;
    /* GPU 워프를 쓰면 링 슬롯을 워프 소스 평면으로 곧장 복사해 중간 버퍼를 없앤다. */
    GpuWarp::Planes planes;
    const bool preload_planes = model.frame_planes(&planes);
    std::vector<uint8_t> frame_copy(preload_planes ? 0 : frame_ring.frame_bytes());

    std::fprintf(stderr, "modeld: live shared ring slots=%u frame=%ux%u bytes=%u target=%uHz\n",
                 frame_ring.slot_count(), frame_ring.width(), frame_ring.height(),
                 frame_ring.frame_bytes(), target_fps);

    while (!g_stop) {
        const uint64_t now_ns = k230_now_ns();
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

        const bool frame_ready = preload_planes
            ? frame_ring.copy_slot_planes(meta.slot, meta.frame_id, planes.luma,
                                          planes.luma_stride, planes.chroma,
                                          planes.chroma_stride)
            : frame_ring.copy_slot(meta.slot, meta.frame_id,
                                   frame_copy.data(), frame_copy.size());
        if (!frame_ready) {
            ++frame_sync_failures;
            ++errors;
            continue;
        }
        ego.poll();
        model.set_desire(ego.desire());

        // The recorder follows the exact frame selected by modeld, rather than
        // sampling camerad's higher-rate latest-frame stream independently.
        if (!record_frame_pub.publish(&meta, sizeof(meta))) {
            std::fprintf(stderr, "\nmodeld: publish recordFrame failed\n");
        }

        const uint64_t t0 = k230_now_ns();
        const bool ok = preload_planes
            ? model.run_frame_preloaded(meta.width, meta.height, raw)
            : model.run_frame_nv12(frame_copy.data(), meta.width, meta.height, raw);
        const uint64_t t1 = k230_now_ns();
        next_model_start_ns = t0 + model_interval_ns;
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            if (!publish_output(model_pub, model, parsed, calibration,
                                meta.frame_id, meta.timestamp_ns, model_ms, ego.v_ego())) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            ++processed;
        } else {
            ++errors;
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;

        uint64_t window_us = 0;
        if (window.close_if_due(&window_us)) {
            std::fprintf(stderr,
                         "modeld: fps=%.2f frames=%u missed=%u sync=%u errors=%u(+%u) last_ms=%.2f          \r",
                         (processed - window.last_processed) * 1000000.0 / window_us,
                         processed,
                         missed,
                         frame_sync_failures,
                         errors,
                         errors - window.last_errors,
                         ok ? (t1 - t0) / 1000000.0 : 0.0);
            std::fflush(stderr);
            window.last_processed = processed;
            window.last_errors = errors;
        }
    }

    std::fprintf(stderr, "\nmodeld done frames=%u missed=%u sync=%u errors=%u fps=%.2f\n",
                 processed, missed, frame_sync_failures, errors, window.total_fps(processed));
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
