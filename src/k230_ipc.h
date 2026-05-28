#ifndef K230_IPC_H
#define K230_IPC_H

#include "lateral_control.h"
#include "model_output.h"
#include "projection.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

constexpr uint32_t kK230IpcMagic = 0x4b323349;
constexpr uint32_t kK230IpcVersion = 1;
constexpr uint32_t kK230FrameRingMagic = 0x4b465249;
constexpr unsigned kK230FrameSlots = 4;
constexpr unsigned kK230AiWidth = 512;
constexpr unsigned kK230AiHeight = 256;
constexpr unsigned kK230AiFrameBytes = kK230AiWidth * kK230AiHeight * 3 / 2;
constexpr char kK230RoadAiFrameTopic[] = "/k230_road_ai_frame";
constexpr char kK230ModelStateTopic[] = "/k230_model_state";
constexpr char kK230ManagerStateTopic[] = "/k230_manager_state";
constexpr char kK230RoadAiFrameRing[] = "/k230_road_ai";

uint64_t k230_now_ns();

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

struct K230ModelState {
    uint64_t frame_id = 0;
    uint64_t capture_timestamp_ns = 0;
    uint64_t model_timestamp_ns = 0;
    float model_execution_ms = 0.0f;
    uint32_t valid = 0;
    uint32_t projection_mode = static_cast<uint32_t>(ProjectionMode::Legacy);
    int32_t best_plan = 0;
    float plan_probability = 0.0f;
    K230IpcPoint plan[kTrajectorySize] = {};
    K230IpcPoint lanes[4][kTrajectorySize] = {};
    float lane_probabilities[4] = {};
    K230IpcPoint road_edges[2][kTrajectorySize] = {};
    K230LeadState lead;
    K230PoseState pose;
    K230CalibrationState calibration;
    K230LateralTargetState lateral_target;
};

struct K230ProcessState {
    char name[16] = {};
    uint32_t running = 0;
    int32_t pid = 0;
    int32_t exit_code = 0;
    uint32_t restart_count = 0;
    uint64_t last_start_ns = 0;
};

struct K230ManagerState {
    uint64_t timestamp_ns = 0;
    uint32_t process_count = 0;
    uint32_t reserved = 0;
    K230ProcessState processes[4] = {};
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
