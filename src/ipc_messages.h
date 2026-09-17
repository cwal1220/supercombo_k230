#ifndef IPC_MESSAGES_H
#define IPC_MESSAGES_H

/* 프로세스 사이를 /dev/shm으로 건너가는 메시지 전부: 토픽 이름, 매직/버전,
 * 채널 헤더, 상태 스냅샷, 그리고 ParsedModelOutput <-> K230ModelState 변환.
 * 채널 구현은 ipc_channels.h에 있다. 메시지를 쓰기만 하는 코드는 이 헤더만 본다. */

#include "app_config.h"
#include "model_output.h"
#include "projection.h"
#include "recording_format.h"
#include "utils_time.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

constexpr uint32_t kK230IpcMagic = 0x4b323349;
constexpr uint32_t kK230IpcVersion = 1;
constexpr uint32_t kK230FrameRingMagic = 0x4b465249;
constexpr uint32_t kK230FrameRingVersion = 3;
constexpr uint32_t kK230CanQueueMagic = 0x4b435151;
constexpr uint32_t kK230CanQueueVersion = 1;
constexpr unsigned kK230FrameSlots = 8;
constexpr unsigned kK230MaxProcesses = 7;
constexpr unsigned kK230AiWidth = kDefaultAiWidth;
constexpr unsigned kK230AiHeight = kDefaultAiHeight;
constexpr unsigned kK230AiFrameBytes = kK230AiWidth * kK230AiHeight * 3 / 2;
constexpr char kK230RoadAiFrameTopic[] = "/k230_road_ai_frame";
constexpr char kK230RecordFrameTopic[] = "/k230_record_frame";
constexpr char kK230ModelStateTopic[] = "/k230_model_state";
constexpr char kK230ManagerStateTopic[] = "/k230_manager_state";
constexpr char kK230CanTopic[] = "/k230_can";
constexpr char kK230SendCanTopic[] = "/k230_sendcan";
constexpr char kK230CanLogTopic[] = "/k230_can_log";
constexpr char kK230SendCanLogTopic[] = "/k230_sendcan_log";
constexpr char kK230PandaStateTopic[] = "/k230_panda_state";
constexpr char kK230ControlStateTopic[] = "/k230_control_state";

constexpr uint32_t kK230HudFlagLaneless = 1U << 0;
constexpr uint32_t kK230HudFlagBrakeHold = 1U << 1;
constexpr char kK230RoadAiFrameRing[] = "/k230_road_ai";
constexpr unsigned kK230CanBatchMaxFrames = 256;
constexpr unsigned kK230CanQueueSlots = 64;

struct K230IpcHeader {
    uint32_t magic = kK230IpcMagic;
    uint32_t version = kK230IpcVersion;
    uint32_t payload_capacity = 0;
    uint32_t reserved0 = 0;
    std::atomic<uint64_t> seq{0};
    std::atomic<uint64_t> timestamp_ns{0};
    std::atomic<uint32_t> payload_size{0};
    uint32_t reserved1 = 0;
};

static_assert(sizeof(K230IpcHeader) == 40,
              "K230IpcHeader layout is part of the Python manager ABI");

struct K230FrameRingHeader {
    uint32_t magic = kK230FrameRingMagic;
    uint32_t version = kK230FrameRingVersion;
    uint32_t slot_count = kK230FrameSlots;
    uint32_t width = kK230AiWidth;
    uint32_t height = kK230AiHeight;
    uint32_t frame_bytes = kK230AiFrameBytes;
    uint32_t reserved0 = 0;
    uint32_t reserved1 = 0;
    std::atomic<uint64_t> slot_seq[kK230FrameSlots]{};
    std::atomic<uint64_t> slot_frame_id[kK230FrameSlots]{};
};

struct K230RoadAiFrame {
    uint64_t frame_id = 0;
    uint64_t timestamp_ns = 0;
    uint32_t slot = 0;
    uint32_t width = kK230AiWidth;
    uint32_t height = kK230AiHeight;
    uint32_t format = 0;
    uint32_t crop_x = 0;
    uint32_t crop_y = 0;
    uint32_t crop_width = 0;
    uint32_t crop_height = 0;
};

struct K230IpcPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct K230LeadState {
    uint32_t valid = 0;
    float probability = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
};

struct K230PoseState {
    uint32_t valid = 0;
    float trans[3] = {};
    float rot[3] = {};
    float trans_std[3] = {};
    float rot_std[3] = {};
};

struct K230CalibrationState {
    uint32_t status = 0;
    int32_t valid_blocks = 0;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float spread[3] = {};
};

struct K230ModelState {
    uint64_t frame_id = 0;
    uint64_t capture_timestamp_ns = 0;
    uint64_t model_timestamp_ns = 0;
    float model_execution_ms = 0.0f;
    uint32_t valid = 0;
    int32_t best_plan = 0;
    float plan_probability = 0.0f;
    float model_t[kTrajectorySize] = {};
    float lane_t[kTrajectorySize] = {};
    K230IpcPoint plan[kTrajectorySize] = {};
    K230IpcPoint lanes[4][kTrajectorySize] = {};
    float lane_probabilities[4] = {};
    float lane_stds[4] = {};
    K230IpcPoint road_edges[2][kTrajectorySize] = {};
    float road_edge_stds[2] = {};
    float desire_state[kDesireLen] = {};
    K230LeadState lead;
    K230PoseState pose;
    K230CalibrationState calibration;
};

/* 이 크기가 녹화 ModelState 레코드의 페이로드 크기다. 바뀌면 기존 녹화를
 * 읽는 tools/model/k230_route.py와 어긋나므로 recording_format.h의
 * kK230RecordingVersion도 함께 올려야 한다. */
static_assert(sizeof(K230ModelState) == 3256,
              "K230ModelState layout is shared with the recording reader");

struct K230ProcessState {
    char name[16] = {};
    uint32_t running = 0;
};

static_assert(sizeof(K230ProcessState) == 20,
              "K230ProcessState layout is shared with the Python manager");

struct K230ManagerState {
    uint64_t timestamp_ns = 0;
    uint32_t process_count = 0;
    uint32_t reserved = 0;
    K230ProcessState processes[kK230MaxProcesses] = {};
};

static_assert(sizeof(K230ManagerState) == 160,
              "K230ManagerState layout is shared with the Python manager");

struct K230CanFrame {
    uint32_t address = 0;
    uint32_t src = 0;
    uint32_t bus_time = 0;
    uint32_t data_len = 0;
    uint32_t flags = 0;
    uint8_t data[64] = {};
};

struct K230CanBatch {
    uint64_t timestamp_ns = 0;
    uint32_t valid = 0;
    uint32_t count = 0;
    uint32_t dropped = 0;
    uint32_t reserved = 0;
    K230CanFrame frames[kK230CanBatchMaxFrames] = {};
};

inline bool k230_can_batch_is_fresh(const K230CanBatch &batch, uint64_t now_ns,
                                    uint64_t max_age_ns)
{
    return batch.valid && timestamp_fresh_ns(batch.timestamp_ns, now_ns, max_age_ns);
}

struct K230CanQueueHeader {
    uint32_t magic = kK230CanQueueMagic;
    uint32_t version = kK230CanQueueVersion;
    uint32_t slot_count = kK230CanQueueSlots;
    uint32_t reserved0 = 0;
    std::atomic<uint64_t> write_seq{0};
    std::atomic<uint64_t> read_seq{0};
    uint64_t reserved1 = 0;
    uint64_t reserved2 = 0;
};

struct K230PandaState {
    uint64_t timestamp_ns = 0;
    uint32_t connected = 0;
    uint32_t comms_healthy = 0;
    uint32_t tx_enabled = 0;
    uint32_t controls_allowed = 0;
    uint32_t ignition_line = 0;
    uint32_t ignition_can = 0;
    uint32_t safety_mode = 0;
    uint32_t safety_param = 0;
    uint32_t panda_type = 0;
    uint32_t can_rx_errs = 0;
    uint32_t can_send_errs = 0;
    uint32_t can_fwd_errs = 0;
    uint32_t blocked_msg_cnt = 0;
    uint32_t heartbeat_lost = 0;
    uint32_t usb_tx_timeouts = 0;
    uint32_t usb_tx_retries = 0;
    uint32_t malformed_rx_batches = 0;
    uint32_t faults = 0;
    uint32_t fault_status = 0;
    uint32_t voltage = 0;
    uint32_t current = 0;
};

struct K230ControlState {
    uint64_t timestamp_ns = 0;
    uint32_t enabled = 0;
    uint32_t engaged = 0;
    uint32_t active = 0;
    uint32_t should_send = 0;
    uint32_t path_usable = 0;
    uint32_t seeds_ready = 0;
    uint32_t vehicle_fresh = 0;
    uint32_t steering_fault = 0;
    uint32_t left_blinker = 0;
    uint32_t right_blinker = 0;
    uint32_t cruise_active = 0;
    int32_t gear = 0;
    float cluster_speed_kph = 0.0f;
    float cruise_max_speed_kph = 0.0f;
    float cruise_command_speed_kph = 0.0f;
    float steering_angle_deg = 0.0f;
    float desired_curvature = 0.0f;
    float actual_curvature = 0.0f;
    float normalized_output = 0.0f;
    int32_t desired_torque = 0;
    int32_t apply_torque = 0;
    int32_t driver_torque = 0;
    uint32_t desire = 0;
    char active_block[32] = {};
    uint32_t radar_lead_valid = 0;
    float radar_lead_distance_m = 0.0f;
    float radar_lead_relative_speed_mps = 0.0f;
    uint32_t departure_alert_type = 0;
    uint32_t departure_alert_event_id = 0;
    uint32_t green_light_alert_armed = 0;
    uint32_t tpms_valid = 0;
    uint32_t tpms_unit = 0;
    float tpms_pressure_fl = 0.0f;
    float tpms_pressure_fr = 0.0f;
    float tpms_pressure_rl = 0.0f;
    float tpms_pressure_rr = 0.0f;
    uint32_t tpms_warning = 0;
    uint32_t hud_flags = 0;
    uint32_t engage_event_id = 0;
    uint32_t disengage_event_id = 0;
    uint32_t engage_reject_event_id = 0;
    char engage_reject_block[32] = {};
    float ego_speed_kph = 0.0f;
};

/* controlsd가 발행하고 overlayd/recordd가 읽는 공유 레이아웃이다. 기록 v5는 이
 * 구조체를 그대로 저장하고 tools/model/k230_route.py가 위치로 디코드하므로 필드
 * 순서까지 전부 고정한다. 같은 크기 필드 둘을 맞바꿔도 여기서 걸린다. */
#define K230_CONTROL_STATE_AT(field, expected) \
    static_assert(offsetof(K230ControlState, field) == (expected), \
                  "K230ControlState." #field " moved: recording v5 layout")
K230_CONTROL_STATE_AT(timestamp_ns, 0);
K230_CONTROL_STATE_AT(enabled, 8);
K230_CONTROL_STATE_AT(engaged, 12);
K230_CONTROL_STATE_AT(active, 16);
K230_CONTROL_STATE_AT(should_send, 20);
K230_CONTROL_STATE_AT(path_usable, 24);
K230_CONTROL_STATE_AT(seeds_ready, 28);
K230_CONTROL_STATE_AT(vehicle_fresh, 32);
K230_CONTROL_STATE_AT(steering_fault, 36);
K230_CONTROL_STATE_AT(left_blinker, 40);
K230_CONTROL_STATE_AT(right_blinker, 44);
K230_CONTROL_STATE_AT(cruise_active, 48);
K230_CONTROL_STATE_AT(gear, 52);
K230_CONTROL_STATE_AT(cluster_speed_kph, 56);
K230_CONTROL_STATE_AT(cruise_max_speed_kph, 60);
K230_CONTROL_STATE_AT(cruise_command_speed_kph, 64);
K230_CONTROL_STATE_AT(steering_angle_deg, 68);
K230_CONTROL_STATE_AT(desired_curvature, 72);
K230_CONTROL_STATE_AT(actual_curvature, 76);
K230_CONTROL_STATE_AT(normalized_output, 80);
K230_CONTROL_STATE_AT(desired_torque, 84);
K230_CONTROL_STATE_AT(apply_torque, 88);
K230_CONTROL_STATE_AT(driver_torque, 92);
K230_CONTROL_STATE_AT(desire, 96);
K230_CONTROL_STATE_AT(active_block, 100);
K230_CONTROL_STATE_AT(radar_lead_valid, 132);
K230_CONTROL_STATE_AT(radar_lead_distance_m, 136);
K230_CONTROL_STATE_AT(radar_lead_relative_speed_mps, 140);
K230_CONTROL_STATE_AT(departure_alert_type, 144);
K230_CONTROL_STATE_AT(departure_alert_event_id, 148);
K230_CONTROL_STATE_AT(green_light_alert_armed, 152);
K230_CONTROL_STATE_AT(tpms_valid, 156);
K230_CONTROL_STATE_AT(tpms_unit, 160);
K230_CONTROL_STATE_AT(tpms_pressure_fl, 164);
K230_CONTROL_STATE_AT(tpms_pressure_fr, 168);
K230_CONTROL_STATE_AT(tpms_pressure_rl, 172);
K230_CONTROL_STATE_AT(tpms_pressure_rr, 176);
K230_CONTROL_STATE_AT(tpms_warning, 180);
K230_CONTROL_STATE_AT(hud_flags, 184);
K230_CONTROL_STATE_AT(engage_event_id, 188);
K230_CONTROL_STATE_AT(disengage_event_id, 192);
K230_CONTROL_STATE_AT(engage_reject_event_id, 196);
K230_CONTROL_STATE_AT(engage_reject_block, 200);
K230_CONTROL_STATE_AT(ego_speed_kph, 232);
#undef K230_CONTROL_STATE_AT
static_assert(sizeof(K230ControlState) == 240,
              "K230ControlState layout is shared by controlsd, overlay and recording v5");
/* recordd가 K230RecordType::PandaState로 그대로 저장한다. */
static_assert(sizeof(K230PandaState) == 96,
              "K230PandaState is recorded as-is: bump kK230RecordingVersion");
/* 기록 버전과 저장 구조체 크기를 한 줄에 묶어, 둘 중 하나만 바꾸면 컴파일이 깨진다. */
static_assert(kK230RecordingVersion == 5 && sizeof(K230ModelState) == 3256 &&
                  sizeof(K230ControlState) == 240 && sizeof(K230PandaState) == 96,
              "recording v5 pins these payloads; bump kK230RecordingVersion together");

/* modeld가 발행 직전에, overlayd와 hud_snapshot이 소비 직후에 쓴다. */
void k230_fill_model_state(K230ModelState &state, const ParsedModelOutput &parsed,
                           const ProjectionState &projection,
                           const OnlineCalibrator::Snapshot &calibration,
                           uint64_t frame_id, uint64_t capture_timestamp_ns,
                           float model_execution_ms);
ParsedModelOutput k230_parsed_from_model_state(const K230ModelState &state);
ProjectionState k230_projection_from_model_state(const K230ModelState &state);

#endif
