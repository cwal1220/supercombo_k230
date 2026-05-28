#include "k230_ipc.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

static_assert(sizeof(K230IpcHeader) == 40, "K230IpcHeader layout is part of the Python manager ABI");

namespace {

size_t frame_ring_size(unsigned width, unsigned height, unsigned slots)
{
    return sizeof(K230FrameRingHeader) + static_cast<size_t>(width) * height * 3 / 2 * slots;
}

void sleep_ms(int ms)
{
    if (ms <= 0) return;
    usleep(static_cast<useconds_t>(ms) * 1000);
}

} // namespace

uint64_t k230_now_ns()
{
    timespec ts{};
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

K230LatestChannel::~K230LatestChannel()
{
    close();
}

bool K230LatestChannel::open(const char *name, size_t payload_capacity, bool create)
{
    close();
    name_ = name ? name : "";
    const int flags = O_RDWR | (create ? O_CREAT : 0);
    fd_ = shm_open(name_.c_str(), flags, 0664);
    if (fd_ < 0) return false;

    map_size_ = sizeof(K230IpcHeader) + payload_capacity;
    if (create && ftruncate(fd_, static_cast<off_t>(map_size_)) != 0) {
        std::perror("ftruncate ipc channel");
        close();
        return false;
    }

    void *map = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (map == MAP_FAILED) {
        std::perror("mmap ipc channel");
        close();
        return false;
    }

    header_ = static_cast<K230IpcHeader *>(map);
    payload_ = reinterpret_cast<uint8_t *>(header_) + sizeof(K230IpcHeader);
    if (create && (header_->magic != kK230IpcMagic ||
                   header_->version != kK230IpcVersion ||
                   header_->payload_capacity != payload_capacity)) {
        header_->magic = kK230IpcMagic;
        header_->version = kK230IpcVersion;
        header_->payload_capacity = static_cast<uint32_t>(payload_capacity);
        header_->reserved0 = 0;
        header_->seq.store(0, std::memory_order_release);
        header_->timestamp_ns.store(0, std::memory_order_release);
        header_->payload_size.store(0, std::memory_order_release);
        header_->reserved1 = 0;
        std::memset(payload_, 0, payload_capacity);
    }
    return header_->magic == kK230IpcMagic &&
        header_->version == kK230IpcVersion &&
        header_->payload_capacity >= payload_capacity;
}

void K230LatestChannel::close()
{
    if (header_) {
        munmap(header_, map_size_);
        header_ = nullptr;
        payload_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    map_size_ = 0;
}

bool K230LatestChannel::publish(const void *payload, size_t payload_size)
{
    if (!header_ || !payload || payload_size > header_->payload_capacity) return false;

    uint64_t seq = header_->seq.load(std::memory_order_acquire);
    if ((seq & 1ULL) != 0) ++seq;
    header_->seq.store(seq + 1, std::memory_order_release);
    std::memcpy(payload_, payload, payload_size);
    header_->payload_size.store(static_cast<uint32_t>(payload_size), std::memory_order_release);
    header_->timestamp_ns.store(k230_now_ns(), std::memory_order_release);
    header_->seq.store(seq + 2, std::memory_order_release);
    return true;
}

bool K230LatestChannel::read(void *payload, size_t payload_capacity, uint64_t *seq) const
{
    if (!header_ || !payload) return false;
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint64_t before = header_->seq.load(std::memory_order_acquire);
        if (before == 0 || (before & 1ULL) != 0) return false;
        const uint32_t payload_size = header_->payload_size.load(std::memory_order_acquire);
        if (payload_size == 0 || payload_size > payload_capacity) return false;
        std::memcpy(payload, payload_, payload_size);
        const uint64_t after = header_->seq.load(std::memory_order_acquire);
        if (before == after && (after & 1ULL) == 0) {
            if (seq) *seq = after;
            return true;
        }
    }
    return false;
}

bool K230LatestChannel::read_new(uint64_t *last_seq, void *payload, size_t payload_capacity, int timeout_ms) const
{
    const uint64_t start = k230_now_ns();
    const uint64_t timeout_ns = timeout_ms < 0 ? UINT64_MAX : static_cast<uint64_t>(timeout_ms) * 1000000ULL;
    while (true) {
        uint64_t seq = 0;
        if (read(payload, payload_capacity, &seq) && (!last_seq || seq != *last_seq)) {
            if (last_seq) *last_seq = seq;
            return true;
        }
        if (timeout_ms == 0) return false;
        if (timeout_ms > 0 && k230_now_ns() - start >= timeout_ns) return false;
        sleep_ms(1);
    }
}

K230FrameRing::~K230FrameRing()
{
    close();
}

bool K230FrameRing::open(bool create, unsigned width, unsigned height, unsigned slots)
{
    close();
    const int flags = O_RDWR | (create ? O_CREAT : 0);
    fd_ = shm_open(kK230RoadAiFrameRing, flags, 0664);
    if (fd_ < 0) return false;

    map_size_ = frame_ring_size(width, height, slots);
    if (!create) {
        struct stat st {};
        if (fstat(fd_, &st) == 0 && st.st_size > 0)
            map_size_ = static_cast<size_t>(st.st_size);
    }
    if (create && ftruncate(fd_, static_cast<off_t>(map_size_)) != 0) {
        std::perror("ftruncate frame ring");
        close();
        return false;
    }

    void *map = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (map == MAP_FAILED) {
        std::perror("mmap frame ring");
        close();
        return false;
    }

    header_ = static_cast<K230FrameRingHeader *>(map);
    frames_ = reinterpret_cast<uint8_t *>(header_) + sizeof(K230FrameRingHeader);
    if (create && (header_->magic != kK230FrameRingMagic ||
                   header_->width != width ||
                   header_->height != height ||
                   header_->slot_count != slots)) {
        header_->magic = kK230FrameRingMagic;
        header_->version = kK230IpcVersion;
        header_->slot_count = slots;
        header_->width = width;
        header_->height = height;
        header_->frame_bytes = static_cast<uint32_t>(width * height * 3 / 2);
        header_->reserved0 = 0;
        header_->reserved1 = 0;
        std::memset(frames_, 0, static_cast<size_t>(header_->frame_bytes) * slots);
    }
    return header_->magic == kK230FrameRingMagic && header_->frame_bytes > 0;
}

void K230FrameRing::close()
{
    if (header_) {
        munmap(header_, map_size_);
        header_ = nullptr;
        frames_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    map_size_ = 0;
}

uint8_t *K230FrameRing::slot(unsigned index)
{
    if (!header_ || index >= header_->slot_count) return nullptr;
    return frames_ + static_cast<size_t>(index) * header_->frame_bytes;
}

const uint8_t *K230FrameRing::slot(unsigned index) const
{
    if (!header_ || index >= header_->slot_count) return nullptr;
    return frames_ + static_cast<size_t>(index) * header_->frame_bytes;
}

void k230_fill_model_state(K230ModelState &state, const ParsedModelOutput &parsed,
                           const ProjectionState &projection,
                           const OnlineCalibrator::Snapshot &calibration,
                           const LateralTarget &lateral_target,
                           uint64_t frame_id, uint64_t capture_timestamp_ns,
                           float model_execution_ms)
{
    state = K230ModelState{};
    state.frame_id = frame_id;
    state.capture_timestamp_ns = capture_timestamp_ns;
    state.model_timestamp_ns = k230_now_ns();
    state.model_execution_ms = model_execution_ms;
    state.valid = parsed.valid ? 1 : 0;
    state.projection_mode = static_cast<uint32_t>(projection.mode);
    state.best_plan = parsed.plan.best_index;
    state.plan_probability = parsed.plan.probability;

    for (int i = 0; i < kTrajectorySize; ++i) {
        state.plan[i] = {parsed.plan.points[i].x, parsed.plan.points[i].y, parsed.plan.points[i].z};
        for (int lane = 0; lane < 4; ++lane) {
            state.lanes[lane][i] = {
                parsed.lanes[lane].points[i].x,
                parsed.lanes[lane].points[i].y,
                parsed.lanes[lane].points[i].z,
            };
        }
        for (int edge = 0; edge < 2; ++edge) {
            state.road_edges[edge][i] = {
                parsed.road_edges[edge].points[i].x,
                parsed.road_edges[edge].points[i].y,
                parsed.road_edges[edge].points[i].z,
            };
        }
    }
    for (int lane = 0; lane < 4; ++lane)
        state.lane_probabilities[lane] = parsed.lanes[lane].probability;

    ParsedLeadPoint lead;
    float lead_prob = 0.0f;
    if (parsed.leads.primary(0, 0.0f, &lead, &lead_prob)) {
        state.lead.valid = 1;
        state.lead.probability = lead_prob;
        state.lead.x = lead.x;
        state.lead.y = lead.y;
        state.lead.velocity = lead.velocity;
        state.lead.acceleration = lead.acceleration;
    }

    state.pose.valid = parsed.has_pose ? 1 : 0;
    if (parsed.has_pose) {
        for (int i = 0; i < 3; ++i) {
            state.pose.trans[i] = parsed.pose.trans[i];
            state.pose.rot[i] = parsed.pose.rot[i];
            state.pose.trans_std[i] = parsed.pose.trans_std[i];
            state.pose.rot_std[i] = parsed.pose.rot_std[i];
        }
    }

    state.calibration.status = static_cast<uint32_t>(calibration.status);
    state.calibration.valid_blocks = calibration.valid_blocks;
    state.calibration.roll = projection.roll;
    state.calibration.pitch = projection.pitch;
    state.calibration.yaw = projection.yaw;
    for (int i = 0; i < 3; ++i)
        state.calibration.spread[i] = calibration.spread[i];

    state.lateral_target.valid = lateral_target.valid ? 1 : 0;
    state.lateral_target.lookahead_x = lateral_target.lookahead_x;
    state.lateral_target.target_y = lateral_target.target_y;
    state.lateral_target.heading = lateral_target.heading;
    state.lateral_target.curvature = lateral_target.curvature;
}

ParsedModelOutput k230_parsed_from_model_state(const K230ModelState &state)
{
    ParsedModelOutput parsed;
    parsed.valid = state.valid != 0;
    parsed.plan.valid = parsed.valid;
    parsed.plan.best_index = state.best_plan;
    parsed.plan.probability = state.plan_probability;
    for (int i = 0; i < kTrajectorySize; ++i) {
        parsed.plan.points[i] = {state.plan[i].x, state.plan[i].y, state.plan[i].z};
        for (int lane = 0; lane < 4; ++lane) {
            parsed.lanes[lane].valid = parsed.valid;
            parsed.lanes[lane].probability = state.lane_probabilities[lane];
            parsed.lanes[lane].points[i] = {
                state.lanes[lane][i].x,
                state.lanes[lane][i].y,
                state.lanes[lane][i].z,
            };
        }
        for (int edge = 0; edge < 2; ++edge) {
            parsed.road_edges[edge].valid = parsed.valid;
            parsed.road_edges[edge].points[i] = {
                state.road_edges[edge][i].x,
                state.road_edges[edge][i].y,
                state.road_edges[edge][i].z,
            };
        }
    }

    if (state.lead.valid) {
        parsed.leads.valid = true;
        parsed.leads.global_probabilities[0] = state.lead.probability;
        parsed.leads.predictions[0].probabilities[0] = state.lead.probability;
        parsed.leads.predictions[0].points[0] = {
            state.lead.x,
            state.lead.y,
            state.lead.velocity,
            state.lead.acceleration,
        };
    }

    parsed.has_pose = state.pose.valid != 0;
    if (parsed.has_pose) {
        for (int i = 0; i < 3; ++i) {
            parsed.pose.trans[i] = state.pose.trans[i];
            parsed.pose.rot[i] = state.pose.rot[i];
            parsed.pose.trans_std[i] = state.pose.trans_std[i];
            parsed.pose.rot_std[i] = state.pose.rot_std[i];
        }
    }
    return parsed;
}

ProjectionState k230_projection_from_model_state(const K230ModelState &state)
{
    const ProjectionMode mode = state.projection_mode == static_cast<uint32_t>(ProjectionMode::Openpilot)
        ? ProjectionMode::Openpilot
        : ProjectionMode::Legacy;
    return make_projection_state(mode, state.calibration.roll, state.calibration.pitch, state.calibration.yaw);
}
