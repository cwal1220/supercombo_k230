#ifndef K230_IPC_H
#define K230_IPC_H

#include "app_config.h"
#include "common_utils.h"
#include "lateral_control.h"
#include "model_output.h"
#include "online_calibrator.h"
#include "projection.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

constexpr uint32_t kK230IpcMagic = 0x4b323349;
constexpr uint32_t kK230IpcVersion = 1;
constexpr uint32_t kK230FrameRingMagic = 0x4b465249;
constexpr uint32_t kK230CanQueueMagic = 0x4b435151;
constexpr uint32_t kK230CanQueueVersion = 1;
constexpr unsigned kK230FrameSlots = 4;
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

struct K230FrameRingHeader {
    uint32_t magic = kK230FrameRingMagic;
    uint32_t version = kK230IpcVersion;
    uint32_t slot_count = kK230FrameSlots;
    uint32_t width = kK230AiWidth;
    uint32_t height = kK230AiHeight;
    uint32_t frame_bytes = kK230AiFrameBytes;
    uint32_t reserved0 = 0;
    uint32_t reserved1 = 0;
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

struct K230StopLineState {
    uint32_t valid = 0;
    float probability = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float speed = 0.0f;
    float time = 0.0f;
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

struct K230LateralTargetState {
    uint32_t valid = 0;
    float lookahead_x = 0.0f;
    float target_y = 0.0f;
    float heading = 0.0f;
    float curvature = 0.0f;
};

struct K230LateralPlanState {
    uint32_t valid = 0;
    uint32_t mpc_solution_valid = 0;
    float psis[kLateralControlN] = {};
    float curvatures[kLateralControlN] = {};
    float curvature_rates[kLateralControlN] = {};
    float d_path_points[kLateralControlN] = {};
    float output_scale = 0.0f;
    uint32_t reserved = 0;
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
    K230IpcPoint plan_position_stds[kTrajectorySize] = {};
    K230IpcPoint plan_orientations[kTrajectorySize] = {};
    K230IpcPoint lanes[4][kTrajectorySize] = {};
    float lane_probabilities[4] = {};
    float lane_stds[4] = {};
    K230IpcPoint road_edges[2][kTrajectorySize] = {};
    float road_edge_stds[2] = {};
    float desire_state[kDesireLen] = {};
    K230LeadState lead;
    K230StopLineState stop_line;
    K230PoseState pose;
    K230CalibrationState calibration;
    K230LateralTargetState lateral_target;
    K230LateralPlanState lateral_plan;
};

struct K230ProcessState {
    char name[16] = {};
    uint32_t running = 0;
    int32_t pid = 0;
    int32_t exit_code = 0;
    uint32_t restart_count = 0;
    uint64_t last_start_ns = 0;
};

static_assert(sizeof(K230ProcessState) == 40,
              "K230ProcessState layout is shared with the Python manager");

struct K230ManagerState {
    uint64_t timestamp_ns = 0;
    uint32_t process_count = 0;
    uint32_t reserved = 0;
    K230ProcessState processes[kK230MaxProcesses] = {};
};

static_assert(sizeof(K230ManagerState) == 296,
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

bool k230_can_batch_is_fresh(const K230CanBatch &batch, uint64_t now_ns,
                             uint64_t max_age_ns);

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
    float speed_kph = 0.0f;
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
};

class K230LatestChannel {
public:
    K230LatestChannel() = default;
    ~K230LatestChannel();

    bool open(const char *name, size_t payload_capacity, bool create);
    void close();
    bool publish(const void *payload, size_t payload_size);
    bool read(void *payload, size_t payload_capacity, uint64_t *seq = nullptr) const;
    bool read_new(uint64_t *last_seq, void *payload, size_t payload_capacity, int timeout_ms) const;
    bool valid() const { return header_ != nullptr; }

private:
    std::string name_;
    int fd_ = -1;
    size_t map_size_ = 0;
    K230IpcHeader *header_ = nullptr;
    uint8_t *payload_ = nullptr;
};

class K230CanQueue {
public:
    K230CanQueue() = default;
    ~K230CanQueue();

    bool open(const char *name, unsigned slot_count = kK230CanQueueSlots,
              bool create = true);
    void close();
    void reset();
    bool push(const K230CanBatch &batch);
    bool pop(K230CanBatch *batch);
    uint64_t depth() const;
    unsigned capacity() const { return header_ ? header_->slot_count : 0; }
    bool valid() const { return header_ != nullptr; }

private:
    std::string name_;
    int fd_ = -1;
    size_t map_size_ = 0;
    K230CanQueueHeader *header_ = nullptr;
    K230CanBatch *slots_ = nullptr;
};

class K230FrameRing {
public:
    K230FrameRing() = default;
    ~K230FrameRing();

    bool open(bool create, unsigned width = kK230AiWidth, unsigned height = kK230AiHeight,
              unsigned slots = kK230FrameSlots);
    void close();
    uint8_t *slot(unsigned index);
    const uint8_t *slot(unsigned index) const;
    unsigned slot_count() const { return header_ ? header_->slot_count : 0; }
    unsigned frame_bytes() const { return header_ ? header_->frame_bytes : 0; }
    unsigned width() const { return header_ ? header_->width : 0; }
    unsigned height() const { return header_ ? header_->height : 0; }
    bool valid() const { return header_ != nullptr; }

private:
    int fd_ = -1;
    size_t map_size_ = 0;
    K230FrameRingHeader *header_ = nullptr;
    uint8_t *frames_ = nullptr;
};

void k230_fill_model_state(K230ModelState &state, const ParsedModelOutput &parsed,
                           const ProjectionState &projection,
                           const OnlineCalibrator::Snapshot &calibration,
                           const LateralTarget &lateral_target,
                           uint64_t frame_id, uint64_t capture_timestamp_ns,
                           float model_execution_ms);
ParsedModelOutput k230_parsed_from_model_state(const K230ModelState &state);
ProjectionState k230_projection_from_model_state(const K230ModelState &state);

#endif
