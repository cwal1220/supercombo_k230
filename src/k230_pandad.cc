#include "utils_process.h"
#include "utils_time.h"
#include "ipc_channels.h"
#include "panda_can_codec.h"
#include "panda_client.h"

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;
constexpr uint64_t kCanPublishIntervalNs = 10000000ULL;
constexpr uint64_t kMaxSendCanAgeNs = 100000000ULL;

uint16_t parse_safety_model(const char *name, uint16_t *default_param)
{
    const char *value = std::getenv(name);
    const std::string mode = value && value[0] ? value : "nooutput";
    *default_param = 0;
    if (mode == "silent") return kPandaSafetySilent;
    if (mode == "elm327") return kPandaSafetyElm327;
    if (mode == "hyundai") {
        *default_param = 2;  // KIA K7 YG HEV is a Hyundai/Kia hybrid safety-param path.
        return kPandaSafetyHyundai;
    }
    if (mode == "hyundaiCommunity") return kPandaSafetyHyundaiCommunity;
    if (mode == "allOutput") return kPandaSafetyAllOutput;
    return kPandaSafetyNoOutput;
}

K230CanBatch rx_can_batch(const std::vector<PandaCanFrame> &frames)
{
    return k230_make_can_batch(frames, [](K230CanFrame *dst, const PandaCanFrame &src) {
        dst->address = src.address;
        dst->src = src.bus;
        dst->data_len = src.data_len;
        dst->flags = (src.returned ? 0x1U : 0U) | (src.rejected ? 0x2U : 0U);
        std::memcpy(dst->data, src.data, std::min<size_t>(src.data_len, sizeof(dst->data)));
    });
}

std::vector<PandaCanFrame> frames_from_batch(const K230CanBatch &batch)
{
    std::vector<PandaCanFrame> frames;
    if (!batch.valid) return frames;
    const uint32_t count = std::min<uint32_t>(batch.count, kK230CanBatchMaxFrames);
    frames.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        const K230CanFrame &src = batch.frames[i];
        if (src.flags != 0) continue;
        if (src.address > kPandaCanMaxAddress) continue;
        if (src.src > kPandaCanMaxTxBus) continue;
        if (src.data_len > kPandaCanMaxDataLen) continue;
        if (!panda_can_is_valid_data_len(static_cast<uint8_t>(src.data_len))) continue;
        PandaCanFrame frame;
        frame.address = src.address;
        frame.bus = static_cast<uint8_t>(src.src);
        frame.data_len = static_cast<uint8_t>(src.data_len);
        std::memcpy(frame.data, src.data, frame.data_len);
        frames.push_back(frame);
    }
    return frames;
}

void publish_disconnected(K230LatestChannel &state_pub, bool tx_enabled)
{
    K230PandaState state;
    state.timestamp_ns = k230_now_ns();
    state.tx_enabled = tx_enabled ? 1 : 0;
    state_pub.publish(&state, sizeof(state));
}

void publish_health(K230LatestChannel &state_pub, PandaClient &panda, bool tx_enabled)
{
    PandaHealth health;
    K230PandaState state;
    state.timestamp_ns = k230_now_ns();
    state.connected = panda.connected() ? 1 : 0;
    state.comms_healthy = panda.comms_healthy() ? 1 : 0;
    state.tx_enabled = tx_enabled ? 1 : 0;
    state.panda_type = panda.hw_type();
    if (panda.get_health(&health)) {
        state.controls_allowed = health.controls_allowed;
        state.ignition_line = health.ignition_line;
        state.ignition_can = health.ignition_can;
        state.safety_mode = health.safety_mode;
        state.safety_param = health.safety_param;
        state.can_rx_errs = health.can_rx_errs;
        state.can_send_errs = health.can_send_errs;
        state.can_fwd_errs = health.can_fwd_errs;
        state.blocked_msg_cnt = health.blocked_msg_cnt;
        state.heartbeat_lost = health.heartbeat_lost;
        state.usb_tx_timeouts = panda.usb_tx_timeouts();
        state.usb_tx_retries = panda.usb_tx_retries();
        state.malformed_rx_batches = panda.malformed_rx_batches();
        state.faults = health.faults;
        state.fault_status = health.fault_status;
        state.voltage = health.voltage;
        state.current = health.current;
    }
    state_pub.publish(&state, sizeof(state));
}

/* 1초 창의 브리지 통계. 창이 끝나면 panda 헬스와 함께 한 줄로 찍고 비운다. */
struct BridgeStats {
    unsigned rx_frames = 0;
    unsigned tx_frames = 0;
    unsigned tx_batches = 0;
    unsigned rx_queue_full = 0;
    unsigned rx_log_queue_full = 0;
    unsigned tx_log_queue_full = 0;
    unsigned tx_stale = 0;
    unsigned tx_blocked = 0;
    unsigned rx_rejected = 0;
    unsigned errors = 0;
    std::map<std::pair<uint32_t, uint8_t>, unsigned> rejected_frames;

    void log(PandaClient &panda, unsigned long long tx_depth, unsigned long long rx_depth)
    {
        PandaHealth health;
        const bool got_health = panda.get_health(&health);
        std::fprintf(stderr,
                     "k230_pandad: rx=%u tx=%u batches=%u stale=%u "
                     "queue=%llu/%llu rxFull=%u logFull=%u/%u "
                     "blocked=%u rejected=%u errors=%u "
                     "canerr=%u/%u/%u pandaBlocked=%u "
                     "heartbeatLost=%u controls=%u usb=%u/%u malformed=%u "
                     "safety=%u:%u ign=%u/%u voltage=%umV current=%umA faults=0x%x\n",
                     rx_frames, tx_frames, tx_batches, tx_stale,
                     tx_depth, rx_depth,
                     rx_queue_full,
                     rx_log_queue_full, tx_log_queue_full,
                     tx_blocked, rx_rejected, errors,
                     got_health ? health.can_rx_errs : 0,
                     got_health ? health.can_send_errs : 0,
                     got_health ? health.can_fwd_errs : 0,
                     got_health ? health.blocked_msg_cnt : 0,
                     got_health ? health.heartbeat_lost : 0,
                     got_health ? health.controls_allowed : 0,
                     panda.usb_tx_timeouts(), panda.usb_tx_retries(),
                     panda.malformed_rx_batches(),
                     got_health ? health.safety_mode : 0,
                     got_health ? health.safety_param : 0,
                     got_health ? health.ignition_line : 0,
                     got_health ? health.ignition_can : 0,
                     got_health ? health.voltage : 0,
                     got_health ? health.current : 0,
                     got_health ? health.faults : 0);
        for (const auto &[key, count] : rejected_frames) {
            std::fprintf(stderr,
                         "k230_pandad: rejected addr=0x%x bus=%u count=%u\n",
                         key.first, key.second, count);
        }
        *this = BridgeStats{};
    }
};

/* 수신 프레임을 10 ms 또는 256개 단위로 모아 CAN 토픽과 로그 토픽에 올린다. */
class RxBatcher {
public:
    RxBatcher() { pending_.reserve(kK230CanBatchMaxFrames); }

    void clear()
    {
        pending_.clear();
        dropped_ = 0;
    }

    void reset(uint64_t now_ns)
    {
        clear();
        last_publish_ns_ = now_ns;
    }

    void add(const std::vector<PandaCanFrame> &frames)
    {
        pending_.insert(pending_.end(), frames.begin(), frames.end());
    }

    bool due(uint64_t now_ns) const
    {
        return !pending_.empty() &&
               (now_ns - last_publish_ns_ >= kCanPublishIntervalNs ||
                pending_.size() >= kK230CanBatchMaxFrames);
    }

    void publish(uint64_t now_ns, K230CanQueue &can_pub, K230CanQueue &can_log_pub,
                 BridgeStats *stats)
    {
        if (pending_.size() > kK230CanBatchMaxFrames) {
            const size_t overflow = pending_.size() - kK230CanBatchMaxFrames;
            pending_.erase(pending_.begin(), pending_.begin() + overflow);
            dropped_ += static_cast<unsigned>(overflow);
        }
        K230CanBatch batch = rx_can_batch(pending_);
        batch.dropped += dropped_;
        if (!can_pub.push(batch)) {
            ++stats->rx_queue_full;
            ++stats->errors;
        }
        if (!can_log_pub.push(batch)) ++stats->rx_log_queue_full;
        reset(now_ns);
    }

private:
    std::vector<PandaCanFrame> pending_;
    unsigned dropped_ = 0;
    uint64_t last_publish_ns_ = 0;
};

/* USB 연결과 safety 모델 설정. 실패하면 false를 돌려주고 호출자가 다음 루프에서
 * 다시 시도한다. 연결 자체가 안 되면 1초 쉰다. */
bool connect_and_configure(PandaClient &panda, uint16_t safety_model, uint16_t safety_param,
                           BridgeStats *stats)
{
    if (!panda.connect()) {
        std::fprintf(stderr, "k230_pandad: waiting for panda\n");
        sleep(1);
        return false;
    }
    std::fprintf(stderr,
                 "k230_pandad: connected serial=%s hw_type=%u health_v=%u can_v=%u\n",
                 panda.usb_serial().c_str(), panda.hw_type(),
                 panda.health_packet_version(), panda.can_packet_version());
    PandaHealth configured_health;
    if (!panda.set_safety_model(safety_model, safety_param) ||
        !panda.get_health(&configured_health) ||
        configured_health.safety_mode != safety_model ||
        configured_health.safety_param != safety_param) {
        std::fprintf(stderr,
                     "k230_pandad: safety setup failed expected=%u:%u actual=%u:%u\n",
                     safety_model, safety_param,
                     configured_health.safety_mode,
                     configured_health.safety_param);
        ++stats->errors;
        panda.close();
        return false;
    }
    return true;
}

enum class RxResult { Idle, Frames, Error };

// USB 수신 한 번. Error면 호출자가 연결을 끊고 다시 잇는다.
RxResult service_rx(PandaClient &panda, bool log_can, RxBatcher *rx, BridgeStats *stats)
{
    std::vector<PandaCanFrame> frames;
    if (!panda.receive(&frames, 10)) {
        ++stats->errors;
        return RxResult::Error;
    }
    if (frames.empty()) return RxResult::Idle;
    for (const PandaCanFrame &frame : frames) {
        if (frame.rejected) {
            ++stats->rx_rejected;
            ++stats->rejected_frames[{frame.address, frame.bus}];
        }
    }
    rx->add(frames);
    stats->rx_frames += static_cast<unsigned>(frames.size());
    if (log_can) {
        for (const PandaCanFrame &frame : frames) {
            std::fprintf(stderr,
                         "can rx bus=%u addr=0x%x len=%u returned=%u rejected=%u\n",
                         frame.bus, frame.address, frame.data_len,
                         frame.returned ? 1 : 0, frame.rejected ? 1 : 0);
        }
    }
    return RxResult::Frames;
}

// sendcan 큐를 비워 panda로 보낸다. 배치가 하나라도 있었으면 true.
bool service_tx(K230CanQueue &sendcan_sub, K230CanQueue &sendcan_log_pub, PandaClient &panda,
                bool tx_enabled, BridgeStats *stats)
{
    bool had_sendcan = false;
    K230CanBatch send_batch;
    while (sendcan_sub.pop(&send_batch)) {
        had_sendcan = true;
        ++stats->tx_batches;
        // The producer can publish after the loop sampled its clock. Use a
        // fresh timestamp here so a new batch is never mistaken for a
        // future/stale batch and skipped from the torque sequence.
        const uint64_t tx_now = k230_now_ns();
        if (!k230_can_batch_is_fresh(send_batch, tx_now, kMaxSendCanAgeNs)) {
            ++stats->tx_stale;
            continue;
        }
        const std::vector<PandaCanFrame> tx = frames_from_batch(send_batch);
        if (!sendcan_log_pub.push(send_batch)) ++stats->tx_log_queue_full;
        if (tx_enabled) {
            if (panda.send(tx)) {
                stats->tx_frames += static_cast<unsigned>(tx.size());
            } else {
                ++stats->errors;
            }
        } else {
            stats->tx_blocked += static_cast<unsigned>(tx.size());
        }
    }
    return had_sendcan;
}

} // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);

    try {
        K230CanQueue can_pub;
        K230CanQueue sendcan_sub;
        K230CanQueue can_log_pub;
        K230CanQueue sendcan_log_pub;
        K230LatestChannel panda_state_pub;
        if (!can_pub.open(kK230CanTopic, kK230CanQueueSlots, true))
            throw std::runtime_error("open can ipc failed");
        if (!sendcan_sub.open(kK230SendCanTopic, kK230CanQueueSlots, true))
            throw std::runtime_error("open sendcan ipc failed");
        if (!can_log_pub.open(kK230CanLogTopic, kK230CanQueueSlots, true))
            throw std::runtime_error("open CAN log ipc failed");
        if (!sendcan_log_pub.open(kK230SendCanLogTopic, kK230CanQueueSlots, true))
            throw std::runtime_error("open sendcan log ipc failed");
        if (!panda_state_pub.open(kK230PandaStateTopic, sizeof(K230PandaState), true))
            throw std::runtime_error("open pandaState ipc failed");
        can_pub.reset();
        can_log_pub.reset();
        sendcan_log_pub.reset();

        const bool tx_enabled = env_flag("K230_PANDA_TX", false);
        const bool heartbeat_engaged = env_flag("K230_PANDA_ENGAGED", false);
        const bool log_can = env_flag("K230_PANDA_LOG_CAN", false);
        const uint16_t idle_us = static_cast<uint16_t>(env_unsigned("K230_PANDA_IDLE_US", 5000));
        uint16_t default_safety_param = 0;
        const uint16_t safety_model = parse_safety_model("K230_PANDA_SAFETY", &default_safety_param);
        const uint16_t safety_param = static_cast<uint16_t>(env_unsigned("K230_PANDA_SAFETY_PARAM", default_safety_param));

        if (tx_enabled) {
            std::fprintf(stderr,
                         "k230_pandad: TX enabled safety=%u param=%u engaged=%u\n",
                         safety_model, safety_param, heartbeat_engaged ? 1 : 0);
        } else {
            std::fprintf(stderr,
                         "k230_pandad: shadow mode TX disabled safety=%u param=%u\n",
                         safety_model, safety_param);
        }

        PandaClient panda;
        RxBatcher rx;
        BridgeStats stats;
        uint64_t last_health_ns = 0;
        uint64_t last_heartbeat_ns = 0;
        uint64_t last_log_ns = 0;

        while (!g_stop) {
            if (!panda.connected()) {
                publish_disconnected(panda_state_pub, tx_enabled);
                if (!connect_and_configure(panda, safety_model, safety_param, &stats)) continue;
                last_health_ns = 0;
                last_heartbeat_ns = 0;
                rx.reset(k230_now_ns());
            }

            const RxResult received = service_rx(panda, log_can, &rx, &stats);
            if (received == RxResult::Error) {
                rx.clear();
                panda.close();
                continue;
            }

            const uint64_t now = k230_now_ns();
            if (rx.due(now)) rx.publish(now, can_pub, can_log_pub, &stats);
            const bool had_sendcan =
                service_tx(sendcan_sub, sendcan_log_pub, panda, tx_enabled, &stats);

            if (now - last_heartbeat_ns >= 500000000ULL) {
                panda.send_heartbeat(heartbeat_engaged && tx_enabled);
                last_heartbeat_ns = now;
            }
            if (now - last_health_ns >= 500000000ULL) {
                publish_health(panda_state_pub, panda, tx_enabled);
                last_health_ns = now;
            }
            if (now - last_log_ns >= 1000000000ULL) {
                stats.log(panda, static_cast<unsigned long long>(sendcan_sub.depth()),
                          static_cast<unsigned long long>(can_pub.depth()));
                last_log_ns = now;
            }
            if (received == RxResult::Idle && !had_sendcan && idle_us > 0) {
                usleep(idle_us);
            }
        }

        std::fprintf(stderr, "\nk230_pandad: stopping\n");
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "k230_pandad error: %s\n", e.what());
        return 1;
    }
}
