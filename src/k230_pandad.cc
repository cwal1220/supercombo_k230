#include "k230_ipc.h"
#include "panda_can_codec.h"
#include "panda_client.h"

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
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

bool env_enabled(const char *name, bool default_value = false)
{
    const char *value = std::getenv(name);
    if (!value) return default_value;
    return std::strcmp(value, "0") != 0 &&
           std::strcmp(value, "false") != 0 &&
           std::strcmp(value, "FALSE") != 0;
}

uint16_t env_u16(const char *name, uint16_t default_value)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') return default_value;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(value, &end, 0);
    if (end == value || parsed > 0xffffUL) return default_value;
    return static_cast<uint16_t>(parsed);
}

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

void fill_can_batch(K230CanBatch *batch, const std::vector<PandaCanFrame> &frames)
{
    *batch = K230CanBatch{};
    batch->timestamp_ns = k230_now_ns();
    batch->valid = 1;
    batch->count = static_cast<uint32_t>(std::min<size_t>(frames.size(), kK230CanBatchMaxFrames));
    batch->dropped = frames.size() > kK230CanBatchMaxFrames
        ? static_cast<uint32_t>(frames.size() - kK230CanBatchMaxFrames)
        : 0;
    for (uint32_t i = 0; i < batch->count; ++i) {
        const PandaCanFrame &src = frames[i];
        K230CanFrame &dst = batch->frames[i];
        dst.address = src.address;
        dst.src = src.bus;
        dst.data_len = src.data_len;
        dst.flags = (src.returned ? 0x1U : 0U) | (src.rejected ? 0x2U : 0U);
        std::memcpy(dst.data, src.data, std::min<size_t>(src.data_len, sizeof(dst.data)));
    }
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
        state.blocked_msg_cnt = health.blocked_msg_cnt;
        state.faults = health.faults;
        state.fault_status = health.fault_status;
        state.voltage = health.voltage;
        state.current = health.current;
    }
    state_pub.publish(&state, sizeof(state));
}

} // namespace

int main()
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        K230LatestChannel can_pub;
        K230LatestChannel sendcan_sub;
        K230LatestChannel panda_state_pub;
        if (!can_pub.open(kK230CanTopic, sizeof(K230CanBatch), true))
            throw std::runtime_error("open can ipc failed");
        if (!sendcan_sub.open(kK230SendCanTopic, sizeof(K230CanBatch), true))
            throw std::runtime_error("open sendcan ipc failed");
        if (!panda_state_pub.open(kK230PandaStateTopic, sizeof(K230PandaState), true))
            throw std::runtime_error("open pandaState ipc failed");

        const bool tx_enabled = env_enabled("K230_PANDA_TX", false) ||
                                env_enabled("K230_PANDA_ENABLE_TX", false);
        const bool heartbeat_engaged = env_enabled("K230_PANDA_ENGAGED", false);
        const bool log_can = env_enabled("K230_PANDA_LOG_CAN", false);
        const char *serial_env = std::getenv("K230_PANDA_SERIAL");
        const std::string serial = serial_env ? serial_env : "";
        const uint16_t idle_us = env_u16("K230_PANDA_IDLE_US", 5000);
        uint16_t default_safety_param = 0;
        const uint16_t safety_model = parse_safety_model("K230_PANDA_SAFETY", &default_safety_param);
        const uint16_t safety_param = env_u16("K230_PANDA_SAFETY_PARAM", default_safety_param);

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
        uint64_t last_sendcan_seq = 0;
        uint64_t last_health_ns = 0;
        uint64_t last_heartbeat_ns = 0;
        uint64_t last_log_ns = 0;
        unsigned rx_frames = 0;
        unsigned tx_frames = 0;
        unsigned tx_blocked = 0;
        unsigned errors = 0;

        while (!g_stop) {
            if (!panda.connected()) {
                publish_disconnected(panda_state_pub, tx_enabled);
                if (!panda.connect(serial)) {
                    std::fprintf(stderr, "k230_pandad: waiting for panda\n");
                    sleep(1);
                    continue;
                }
                std::fprintf(stderr, "k230_pandad: connected serial=%s hw_type=%u\n",
                             panda.usb_serial().c_str(), panda.hw_type());
                panda.set_safety_model(safety_model, safety_param);
                last_health_ns = 0;
                last_heartbeat_ns = 0;
            }

            std::vector<PandaCanFrame> frames;
            bool had_rx = false;
            bool had_sendcan = false;
            if (panda.receive(&frames, 10)) {
                if (!frames.empty()) {
                    had_rx = true;
                    K230CanBatch batch;
                    fill_can_batch(&batch, frames);
                    if (!can_pub.publish(&batch, sizeof(batch))) ++errors;
                    rx_frames += static_cast<unsigned>(frames.size());
                    if (log_can) {
                        for (const PandaCanFrame &frame : frames) {
                            std::fprintf(stderr, "can rx bus=%u addr=0x%x len=%u\n",
                                         frame.bus, frame.address, frame.data_len);
                        }
                    }
                }
            } else {
                ++errors;
                panda.close();
                continue;
            }

            K230CanBatch send_batch;
            uint64_t send_seq = last_sendcan_seq;
            if (sendcan_sub.read(&send_batch, sizeof(send_batch), &send_seq) &&
                send_seq != last_sendcan_seq) {
                last_sendcan_seq = send_seq;
                had_sendcan = true;
                const std::vector<PandaCanFrame> tx = frames_from_batch(send_batch);
                if (tx_enabled) {
                    if (panda.send(tx)) {
                        tx_frames += static_cast<unsigned>(tx.size());
                    } else {
                        ++errors;
                    }
                } else {
                    tx_blocked += static_cast<unsigned>(tx.size());
                }
            }

            const uint64_t now = k230_now_ns();
            if (now - last_heartbeat_ns >= 500000000ULL) {
                panda.send_heartbeat(heartbeat_engaged && tx_enabled);
                last_heartbeat_ns = now;
            }
            if (now - last_health_ns >= 500000000ULL) {
                publish_health(panda_state_pub, panda, tx_enabled);
                last_health_ns = now;
            }
            if (now - last_log_ns >= 1000000000ULL) {
                PandaHealth health;
                const bool got_health = panda.get_health(&health);
                std::fprintf(stderr,
                             "k230_pandad: rx=%u tx=%u blocked=%u errors=%u controls=%u "
                             "safety=%u:%u ign=%u/%u voltage=%umV current=%umA faults=0x%x\n",
                             rx_frames, tx_frames, tx_blocked, errors,
                             got_health ? health.controls_allowed : 0,
                             got_health ? health.safety_mode : 0,
                             got_health ? health.safety_param : 0,
                             got_health ? health.ignition_line : 0,
                             got_health ? health.ignition_can : 0,
                             got_health ? health.voltage : 0,
                             got_health ? health.current : 0,
                             got_health ? health.faults : 0);
                rx_frames = tx_frames = tx_blocked = errors = 0;
                last_log_ns = now;
            }
            if (!had_rx && !had_sendcan && idle_us > 0) {
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
