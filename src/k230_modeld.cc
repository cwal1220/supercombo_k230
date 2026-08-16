#include "app_config.h"
#include "calibration_service.h"
#include "input_source.h"
#include "k230_ipc.h"
#include "ld_model.h"
#include "ld_fusion.h"
#include "model_output.h"
#include "supercombo_model.h"

#include <sys/stat.h>

#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;

void fill_lane_meta(K230LaneMetaState *state, const std::vector<DecodedLine> &lines,
                    uint64_t frame_id, const LdModelTiming &timing);

/* LD(차선 종류) 모델 워커. supercombo 발행 직후 프레임 사본을 받아 자기
 * 스레드에서 변환→AI2D→추론→발행을 수행한다. supercombo가 다음 프레임의
 * CPU 워프를 도는 동안 NPU 유휴 구간에 LD 추론이 겹치고, 공유 NPU 뮤텍스가
 * 두 모델의 제출 순서를 보장하므로 모델 pacing에는 영향이 없다. 워커가
 * 바쁘면 프레임을 건너뛴다. */
class LdWorker {
public:
    LdWorker(std::unique_ptr<LdModel> model, K230LatestChannel *lane_meta_pub)
        : model_(std::move(model)), lane_meta_pub_(lane_meta_pub),
          thread_(&LdWorker::run, this) {}

    ~LdWorker()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        condition_.notify_one();
        thread_.join();
    }

    // 워커가 쉬고 있을 때만 프레임 사본을 넘긴다. 넘겼으면 true를 돌려준다.
    bool submit(const uint8_t *nv12, size_t bytes, uint64_t frame_id)
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || pending_ || busy_) return false;
        frame_.assign(nv12, nv12 + bytes);
        frame_id_ = frame_id;
        pending_ = true;
        lock.unlock();
        condition_.notify_one();
        return true;
    }

    // 섀도우 융합용 최근 디코드 결과 사본. 없으면 false.
    bool latest_lines(std::vector<DecodedLine> *out)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_lines_) return false;
        *out = latest_lines_;
        return true;
    }

private:
    void run()
    {
        unsigned failures = 0;
        LdModelTiming timing_sum;
        unsigned runs = 0;
        auto last_log = std::chrono::steady_clock::now();
        while (true) {
            uint64_t frame_id = 0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] { return stop_ || pending_; });
                if (stop_) return;
                pending_ = false;
                busy_ = true;
                frame_id = frame_id_;
            }
            try {
                LdModelTiming timing;
                const std::vector<DecodedLine> lines = model_->run(frame_.data(), &timing);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_lines_ = lines;
                    have_lines_ = true;
                }
                K230LaneMetaState lane_meta;
                fill_lane_meta(&lane_meta, lines, frame_id, timing);
                if (!lane_meta_pub_->publish(&lane_meta, sizeof(lane_meta))) {
                    std::fprintf(stderr, "\nmodeld: publish laneMeta failed\n");
                }
                timing_sum.convert_ms += timing.convert_ms;
                timing_sum.ai2d_ms += timing.ai2d_ms;
                timing_sum.infer_ms += timing.infer_ms;
                timing_sum.decode_ms += timing.decode_ms;
                timing_sum.total_ms += timing.total_ms;
                ++runs;
                failures = 0;
            } catch (const std::exception &error) {
                if (++failures >= 3) {
                    std::fprintf(stderr, "\nmodeld: LD model disabled: %s\n", error.what());
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        busy_ = false;
                        stop_ = true;
                    }
                    return;
                }
            }
            const auto now = std::chrono::steady_clock::now();
            if (runs > 0 && now - last_log >= std::chrono::seconds(5)) {
                const double n = static_cast<double>(runs);
                std::fprintf(stderr,
                             "\nmodeld ld avg[%u] ms: convert=%.2f ai2d=%.2f "
                             "infer=%.2f decode=%.2f total=%.2f\n",
                             runs, timing_sum.convert_ms / n, timing_sum.ai2d_ms / n,
                             timing_sum.infer_ms / n, timing_sum.decode_ms / n,
                             timing_sum.total_ms / n);
                last_log = now;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                busy_ = false;
            }
        }
    }

    std::unique_ptr<LdModel> model_;
    K230LatestChannel *lane_meta_pub_;
    std::vector<uint8_t> frame_;
    std::vector<DecodedLine> latest_lines_;
    bool have_lines_ = false;
    uint64_t frame_id_ = 0;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool pending_ = false;
    bool busy_ = false;
    bool stop_ = false;
    std::thread thread_;
};

// 디코드된 LD 라인들을 IPC lane meta 상태로 채운다.
void fill_lane_meta(K230LaneMetaState *state, const std::vector<DecodedLine> &lines,
                    uint64_t frame_id, const LdModelTiming &timing)
{
    *state = K230LaneMetaState{};
    state->timestamp_ns = k230_now_ns();
    state->frame_id = frame_id;
    state->total_ms = static_cast<float>(timing.total_ms);
    state->infer_ms = static_cast<float>(timing.infer_ms);
    for (const DecodedLine &line : lines) {
        K230LaneMetaLine *dst = nullptr;
        if (line.kind == DecodedLineKind::Lane &&
            line.slot >= 0 && line.slot < static_cast<int>(kK230LaneMetaLaneSlots)) {
            dst = &state->lanes[line.slot];
        } else if (line.kind == DecodedLineKind::Boundary &&
                   line.slot >= 0 &&
                   line.slot < static_cast<int>(kK230LaneMetaBoundarySlots)) {
            dst = &state->boundaries[line.slot];
        }
        if (!dst) continue;
        dst->valid = 1;
        dst->validity = line.validity;
        dst->confidence = line.confidence;
        dst->pattern = lane_class_index(line.marker_pattern);
        dst->color = lane_class_index(line.marker_color);
        dst->double_shape = lane_class_index(line.double_shape);
        dst->boundary_type = lane_class_index(line.boundary_type);
        const size_t stride =
            (line.points.size() + kK230LaneMetaMaxPoints - 1) / kK230LaneMetaMaxPoints;
        uint32_t count = 0;
        for (size_t i = 0; i < line.points.size() && count < kK230LaneMetaMaxPoints;
             i += std::max<size_t>(1, stride)) {
            const LanePoint &point = line.points[i];
            if (!lane_point_valid(point)) continue;
            dst->x[count] = point.x;
            dst->y[count] = point.y;
            ++count;
        }
        dst->point_count = count;
    }
}

uint64_t steady_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool publish_output(K230LatestChannel &model_pub, SupercomboModel &model, const ParsedModelOutput &parsed,
                    CalibrationService &calibration,
                    uint64_t frame_id, uint64_t capture_timestamp_ns, float model_ms,
                    float v_ego, bool ld_promoted = false)
{
    calibration.update(parsed, v_ego);
    float input_rpy[3];
    calibration.input_rpy(input_rpy);
    model.set_input_calibration(input_rpy);

    const ProjectionState projection = calibration.projection();

    K230ModelState state;
    k230_fill_model_state(state, parsed, projection, calibration.snapshot(),
                          frame_id, capture_timestamp_ns, model_ms);
    state.ld_promoted = ld_promoted ? 1 : 0;
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
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);

    /* 리플레이에서도 LD 워커를 라이브와 동일하게 돌려 종합 처리량과
     * 섀도우 융합을 오프라인에서 측정할 수 있게 한다. */
    static std::mutex npu_mutex;
    K230LatestChannel lane_meta_pub;
    std::unique_ptr<LdWorker> ld_worker;
    const char *ld_path_env = std::getenv("SUPERCOMBO_LD_KMODEL");
    const std::string ld_path = ld_path_env && ld_path_env[0] != '\0'
        ? ld_path_env : std::string("model/lane_detect.kmodel");
    /* AI2D 직결 전처리로 매 프레임(20Hz) LD가 예산 안에 들어와 기본값 1.
     * (구 CPU 변환 시절의 10Hz 기본은 SUPERCOMBO_LD_INTERVAL=2로 재현) */
    const unsigned ld_interval =
        std::max(1U, env_unsigned("SUPERCOMBO_LD_INTERVAL", 1));
    struct stat ld_stat = {};
    if (ld_path != "0" && ld_path != "off" && stat(ld_path.c_str(), &ld_stat) == 0) {
        try {
            std::unique_ptr<LdModel> ld_model(
                new LdModel(ld_path, source.width(), source.height()));
            ld_model->set_npu_mutex(&npu_mutex);
            model.set_npu_mutex(&npu_mutex);
            if (!lane_meta_pub.open(kK230LaneMetaTopic, sizeof(K230LaneMetaState), true))
                throw std::runtime_error("open laneMeta ipc failed");
            ld_worker.reset(new LdWorker(std::move(ld_model), &lane_meta_pub));
            std::fprintf(stderr, "modeld replay: LD model %s interval=%u (worker)\n",
                         ld_path.c_str(), ld_interval);
        } catch (const std::exception &error) {
            std::fprintf(stderr, "modeld replay: LD model disabled: %s\n", error.what());
            ld_worker.reset();
        }
    }
    LdFusionResult fusion_last;
    float fusion_trim_ema = 0.0f;
    unsigned fusion_trim_n = 0;
    unsigned ld_gate_streak = 0;
    const bool ld_promote_enabled = env_unsigned("SUPERCOMBO_LD_PROMOTE", 1) != 0;

    /* 오프라인 융합 실험용: 프레임별 차선/플랜 횡위치를 텍스트로 덤프한다. */
    FILE *dump = nullptr;
    if (const char *dump_path = std::getenv("SUPERCOMBO_REPLAY_DUMP")) {
        dump = std::fopen(dump_path, "w");
        if (!dump) std::fprintf(stderr, "modeld replay: cannot open dump %s\n", dump_path);
    }

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
            bool promote_active = false;
            if (ld_worker && processed % ld_interval == 0)
                ld_worker->submit(frame.data.data(), frame.data.size(), processed);
            if (ld_worker) {
                std::vector<DecodedLine> ld_lines;
                if (ld_worker->latest_lines(&ld_lines)) {
                    float rpy[3];
                    calibration.input_rpy(rpy);
                    const LdFusionResult fusion = ld_fusion_compute(
                        ld_lines, parsed, rpy, frame.width, frame.height,
                        replay_config.input_warp_height);
                    if (fusion.valid) {
                        fusion_trim_ema = fusion_trim_n == 0
                            ? fusion.trim
                            : 0.9f * fusion_trim_ema + 0.1f * fusion.trim;
                        ++fusion_trim_n;
                        fusion_last = fusion;
                        if (dump)
                            std::fprintf(dump, "FUSION %u trim=%.3f ld_c=%.3f sc_c=%.3f width=%.2f ema=%.3f\n",
                                         processed, fusion.trim, fusion.ld_center,
                                         fusion.sc_center, fusion.ld_width, fusion_trim_ema);
                    }
                    if (ld_promote_enabled) {
                        bool gates = false;
                        const float sc_prob_before =
                            std::max(parsed.lanes[1].probability, parsed.lanes[2].probability);
                        ParsedModelOutput candidate = parsed;
                        const bool promoted = ld_promote_lanes(
                            ld_lines, rpy, frame.width, frame.height,
                            replay_config.input_warp_height, &candidate, &gates);
                        ld_gate_streak = gates ? ld_gate_streak + 1 : 0;
                        /* 리플레이는 v_ego/desire가 없어 저속 게이트를 생략한다. */
                        if (promoted && ld_gate_streak >= 5) {
                            parsed = candidate;
                            promote_active = true;
                            if (dump)
                                std::fprintf(dump, "PROMOTE %u lprob_before=%.2f\n",
                                             processed, sc_prob_before);
                        }
                    }
                }
            }
            if (!publish_output(model_pub, model, parsed, calibration,
                                processed, k230_now_ns(), model_ms, 0.0f,
                                promote_active)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            if (dump) {
                std::fprintf(dump, "SCFRAME %u\n", processed);
                for (int li = 0; li < 4; ++li) {
                    const auto &lane = parsed.lanes[li];
                    std::fprintf(dump, "SCLANE %d valid=%d prob=%.3f std=%.3f",
                                 li, lane.valid ? 1 : 0, lane.probability, lane.std);
                    for (int pi = 0; pi < kTrajectorySize; ++pi)
                        std::fprintf(dump, " %.1f,%.3f", lane.points[pi].x, lane.points[pi].y);
                    std::fprintf(dump, "\n");
                }
                std::fprintf(dump, "SCPLAN valid=%d", parsed.plan.valid ? 1 : 0);
                for (int pi = 0; pi < kTrajectorySize; ++pi)
                    std::fprintf(dump, " %.1f,%.3f", parsed.plan.points[pi].x, parsed.plan.points[pi].y);
                std::fprintf(dump, "\n");
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
    LdFusionResult fusion_last;
    float fusion_trim_ema = 0.0f;
    unsigned fusion_trim_n = 0;
    unsigned fusion_log_n = 0;
    unsigned ld_gate_streak = 0;
    unsigned ld_promote_frames = 0;
    const bool ld_promote_enabled = env_unsigned("SUPERCOMBO_LD_PROMOTE", 1) != 0;
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

    /* LD(차선 종류) 모델: kmodel 파일이 있으면 워커 스레드에서
     * SUPERCOMBO_LD_INTERVAL 프레임마다 한 번 돌리고 lane meta를 발행한다. */
    static std::mutex npu_mutex;
    K230LatestChannel lane_meta_pub;
    std::unique_ptr<LdWorker> ld_worker;
    const char *ld_path_env = std::getenv("SUPERCOMBO_LD_KMODEL");
    const std::string ld_path = ld_path_env && ld_path_env[0] != '\0'
        ? ld_path_env : std::string("model/lane_detect.kmodel");
    /* AI2D 직결 전처리로 매 프레임(20Hz) LD가 예산 안에 들어와 기본값 1.
     * (구 CPU 변환 시절의 10Hz 기본은 SUPERCOMBO_LD_INTERVAL=2로 재현) */
    const unsigned ld_interval =
        std::max(1U, env_unsigned("SUPERCOMBO_LD_INTERVAL", 1));
    struct stat ld_stat = {};
    if (ld_path != "0" && ld_path != "off" && stat(ld_path.c_str(), &ld_stat) == 0) {
        try {
            std::unique_ptr<LdModel> ld_model(
                new LdModel(ld_path, frame_ring.width(), frame_ring.height()));
            ld_model->set_npu_mutex(&npu_mutex);
            model.set_npu_mutex(&npu_mutex);
            if (!lane_meta_pub.open(kK230LaneMetaTopic, sizeof(K230LaneMetaState), true))
                throw std::runtime_error("open laneMeta ipc failed");
            ld_worker.reset(new LdWorker(std::move(ld_model), &lane_meta_pub));
            std::fprintf(stderr, "modeld: LD model %s interval=%u (worker)\n",
                         ld_path.c_str(), ld_interval);
        } catch (const std::exception &error) {
            std::fprintf(stderr, "modeld: LD model disabled: %s\n", error.what());
            ld_worker.reset();
        }
    }

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
                const float ego_speed_kph =
                    control_state.ego_speed_kph > 0.0f
                        ? control_state.ego_speed_kph
                        : control_state.cluster_speed_kph;
                v_ego = std::max(0.0f, ego_speed_kph / 3.6f);
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
            bool promote_active = false;

            /* LD 융합: 발행 전에 수행해야 승격된 차선이 플래너에 전달된다.
             * 섀도우 trim은 관측만, 승격은 SC host 확률이 낮고 LD 게이트가
             * 연속으로 열려 있을 때 lanes[1]/[2]를 LD 투영으로 교체한다. */
            if (ld_worker) {
                std::vector<DecodedLine> ld_lines;
                if (ld_worker->latest_lines(&ld_lines)) {
                    float rpy[3];
                    calibration.input_rpy(rpy);
                    const LdFusionResult fusion = ld_fusion_compute(
                        ld_lines, parsed, rpy, meta.width, meta.height,
                        config.input_warp_height);
                    if (fusion.valid) {
                        fusion_trim_ema = fusion_trim_n == 0
                            ? fusion.trim
                            : 0.9f * fusion_trim_ema + 0.1f * fusion.trim;
                        ++fusion_trim_n;
                        fusion_last = fusion;
                    }
                    if (ld_promote_enabled) {
                        bool gates = false;
                        /* 저속(60kph 이하)·차선변경 의도 없음에서만 승격.
                         * LD 유효거리(~30m)가 고속 지평선을 못 덮는다. */
                        const bool state_ok = v_ego <= 16.7f && desire == 0;
                        ParsedModelOutput candidate = parsed;
                        const bool promoted = ld_promote_lanes(
                            ld_lines, rpy, meta.width, meta.height,
                            config.input_warp_height, &candidate, &gates);
                        ld_gate_streak = gates ? ld_gate_streak + 1 : 0;
                        if (promoted && state_ok && ld_gate_streak >= 5) {
                            parsed = candidate;
                            promote_active = true;
                            ++ld_promote_frames;
                        }
                    }
                }
            }

            if (!publish_output(model_pub, model, parsed, calibration,
                                meta.frame_id, meta.timestamp_ns, model_ms, v_ego,
                                promote_active)) {
                std::fprintf(stderr, "\nmodeld: publish modelState failed\n");
                ++errors;
            }
            ++processed;

            /* supercombo 발행 직후 프레임 사본을 워커에 넘긴다. 이후의 NPU
             * 유휴 구간(다음 프레임 CPU 워프)에 LD 추론이 겹친다. */
            if (ld_worker && processed % ld_interval == 0) {
                ld_worker->submit(nv12, frame_copy.size(), meta.frame_id);
            }
        } else {
            ++errors;
        }

        if (config.max_frames > 0 && processed >= config.max_frames) break;

        timeval now {};
        gettimeofday(&now, nullptr);
        const uint64_t duration = timeval_us(now) - timeval_us(last);
        if (duration >= 1000000ULL) {
            if (fusion_trim_n > 0 && ++fusion_log_n % 5 == 0) {
                std::fprintf(stderr,
                             "\nmodeld fusion: trim=%+.2fm (ema) ld_c=%+.2f sc_c=%+.2f width=%.2f n=%u promote=%u\n",
                             fusion_trim_ema, fusion_last.ld_center,
                             fusion_last.sc_center, fusion_last.ld_width, fusion_trim_n,
                             ld_promote_frames);
            }
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
