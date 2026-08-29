#include "k230_ipc.h"
#include "common_utils.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

K230FrameRing::~K230FrameRing()
{
    close();
}

bool K230FrameRing::open(bool create, unsigned width, unsigned height, unsigned slots)
{
    close();
    if (width == 0 || height == 0 || slots == 0 || slots > kK230FrameSlots)
        return false;

    const int flags = O_RDWR | (create ? O_CREAT : 0);
    fd_ = shm_open(kK230RoadAiFrameRing, flags, 0664);
    if (fd_ < 0) return false;

    const size_t expected_size = sizeof(K230FrameRingHeader) +
        static_cast<size_t>(width) * height * 3 / 2 * slots;
    map_size_ = expected_size;
    if (!create) {
        struct stat st {};
        if (fstat(fd_, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(K230FrameRingHeader))) {
            close();
            return false;
        }
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
                   header_->version != kK230FrameRingVersion ||
                   header_->width != width ||
                   header_->height != height ||
                   header_->slot_count != slots)) {
        header_->magic = kK230FrameRingMagic;
        header_->version = kK230FrameRingVersion;
        header_->slot_count = slots;
        header_->width = width;
        header_->height = height;
        header_->frame_bytes = static_cast<uint32_t>(width * height * 3 / 2);
        header_->reserved0 = 0;
        header_->reserved1 = 0;
        for (unsigned index = 0; index < kK230FrameSlots; ++index) {
            header_->slot_seq[index].store(0, std::memory_order_release);
            header_->slot_frame_id[index].store(UINT64_MAX, std::memory_order_release);
        }
        std::memset(frames_, 0, static_cast<size_t>(header_->frame_bytes) * slots);
    }
    const bool valid = header_->magic == kK230FrameRingMagic &&
        header_->version == kK230FrameRingVersion &&
        header_->slot_count > 0 && header_->slot_count <= kK230FrameSlots &&
        header_->frame_bytes > 0 &&
        map_size_ >= sizeof(K230FrameRingHeader) +
            static_cast<size_t>(header_->slot_count) * header_->frame_bytes;
    if (!valid) close();
    return valid;
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

bool K230FrameRing::write_slot(unsigned index, uint64_t frame_id,
                               const uint8_t *source, size_t size)
{
    if (!header_ || !source || index >= header_->slot_count ||
        size != header_->frame_bytes) return false;

    std::atomic<uint64_t> &sequence = header_->slot_seq[index];
    uint64_t next = sequence.load(std::memory_order_relaxed);
    if (next & 1ULL) ++next;
    sequence.store(next + 1, std::memory_order_release);
    std::memcpy(frames_ + static_cast<size_t>(index) * header_->frame_bytes,
                source, size);
    header_->slot_frame_id[index].store(frame_id, std::memory_order_release);
    sequence.store(next + 2, std::memory_order_release);
    return true;
}

namespace {

/* seqlock 재시도 껍데기. copy는 슬롯이 안정적일 때만 호출되고, 복사 도중
 * 생산자가 슬롯을 덮었으면 결과를 버리고 다시 시도한다. */
template <typename Copy>
bool copy_slot_guarded(const K230FrameRingHeader &header, unsigned index,
                       uint64_t frame_id, const uint8_t *source, Copy copy)
{
    constexpr unsigned kCopyAttempts = 8;
    const std::atomic<uint64_t> &sequence = header.slot_seq[index];
    const std::atomic<uint64_t> &stored_frame_id = header.slot_frame_id[index];
    for (unsigned attempt = 0; attempt < kCopyAttempts; ++attempt) {
        const uint64_t before = sequence.load(std::memory_order_acquire);
        if (before == 0 || (before & 1ULL) != 0) continue;
        const uint64_t before_frame_id = stored_frame_id.load(std::memory_order_acquire);
        if (before_frame_id != frame_id) {
            if (before_frame_id > frame_id) return false;
            continue;
        }

        copy(source);

        const uint64_t after = sequence.load(std::memory_order_acquire);
        const uint64_t after_frame_id = stored_frame_id.load(std::memory_order_acquire);
        if (before == after && (after & 1ULL) == 0 && after_frame_id == frame_id)
            return true;
    }
    return false;
}

void copy_plane(uint8_t *destination, size_t destination_stride,
                const uint8_t *source, unsigned width, unsigned rows)
{
    if (destination_stride == width) {
        std::memcpy(destination, source, static_cast<size_t>(width) * rows);
        return;
    }
    for (unsigned row = 0; row < rows; ++row) {
        std::memcpy(destination + row * destination_stride,
                    source + static_cast<size_t>(row) * width, width);
    }
}

}  // namespace

bool K230FrameRing::copy_slot(unsigned index, uint64_t frame_id,
                              uint8_t *destination, size_t size) const
{
    if (!header_ || !destination || index >= header_->slot_count ||
        size != header_->frame_bytes) return false;

    const uint8_t *source = frames_ + static_cast<size_t>(index) * header_->frame_bytes;
    return copy_slot_guarded(*header_, index, frame_id, source,
                             [&](const uint8_t *from) {
                                 std::memcpy(destination, from, size);
                             });
}

bool K230FrameRing::copy_slot_planes(unsigned index, uint64_t frame_id,
                                     uint8_t *luma, size_t luma_stride,
                                     uint8_t *chroma, size_t chroma_stride) const
{
    if (!header_ || !luma || !chroma || index >= header_->slot_count ||
        luma_stride < header_->width || chroma_stride < header_->width) return false;

    const unsigned width = header_->width;
    const unsigned height = header_->height;
    const uint8_t *source = frames_ + static_cast<size_t>(index) * header_->frame_bytes;
    return copy_slot_guarded(*header_, index, frame_id, source,
                             [&](const uint8_t *from) {
                                 copy_plane(luma, luma_stride, from, width, height);
                                 copy_plane(chroma, chroma_stride,
                                            from + static_cast<size_t>(width) * height,
                                            width, height / 2);
                             });
}

void k230_fill_model_state(K230ModelState &state, const ParsedModelOutput &parsed,
                           const ProjectionState &projection,
                           const OnlineCalibrator::Snapshot &calibration,
                           uint64_t frame_id, uint64_t capture_timestamp_ns,
                           float model_execution_ms)
{
    state = K230ModelState{};
    state.frame_id = frame_id;
    state.capture_timestamp_ns = capture_timestamp_ns;
    state.model_timestamp_ns = k230_now_ns();
    state.model_execution_ms = model_execution_ms;
    state.valid = parsed.valid ? 1 : 0;
    state.best_plan = parsed.plan.best_index;
    state.plan_probability = parsed.plan.probability;

    for (int i = 0; i < kTrajectorySize; ++i) {
        state.model_t[i] = model_t_idx(i);
        state.plan[i] = {parsed.plan.points[i].x, parsed.plan.points[i].y, parsed.plan.points[i].z};
        state.plan_position_stds[i] = {
            parsed.plan.position_stds[i].x,
            parsed.plan.position_stds[i].y,
            parsed.plan.position_stds[i].z,
        };
        state.plan_orientations[i] = {
            parsed.plan.orientations[i].x,
            parsed.plan.orientations[i].y,
            parsed.plan.orientations[i].z,
        };
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
    std::fill_n(state.lane_t, kTrajectorySize, NAN);
    state.lane_t[0] = 0.0f;
    for (int xidx = 1, tidx = 0; xidx < kTrajectorySize; ++xidx) {
        for (int next_tid = tidx + 1;
             next_tid < kTrajectorySize && parsed.plan.points[next_tid].x < model_x_idx_double(xidx);
             ++next_tid) {
            ++tidx;
        }
        if (tidx == kTrajectorySize - 1) {
            state.lane_t[xidx] = model_t_idx(kTrajectorySize - 1);
            break;
        }

        const float current_x = parsed.plan.points[tidx].x;
        const float next_x = parsed.plan.points[tidx + 1].x;
        const float p = static_cast<float>(
            (model_x_idx_double(xidx) - current_x) / (next_x - current_x));
        state.lane_t[xidx] = static_cast<float>(
            p * model_t_idx_double(tidx + 1) + (1.0f - p) * model_t_idx_double(tidx));
    }
    for (int lane = 0; lane < 4; ++lane) {
        state.lane_probabilities[lane] = parsed.lanes[lane].probability;
        state.lane_stds[lane] = parsed.lanes[lane].std;
    }
    for (int edge = 0; edge < 2; ++edge)
        state.road_edge_stds[edge] = parsed.road_edges[edge].std;
    for (int i = 0; i < kDesireLen; ++i)
        state.desire_state[i] = parsed.meta.desire_state[i];

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

    state.stop_line.valid = parsed.stop_line.valid ? 1 : 0;
    state.stop_line.probability = parsed.stop_line.probability;
    state.stop_line.x = parsed.stop_line.position.x;
    state.stop_line.y = parsed.stop_line.position.y;
    state.stop_line.z = parsed.stop_line.position.z;
    state.stop_line.speed = parsed.stop_line.speed;
    state.stop_line.time = parsed.stop_line.time;

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
        parsed.plan.position_stds[i] = {
            state.plan_position_stds[i].x,
            state.plan_position_stds[i].y,
            state.plan_position_stds[i].z,
        };
        parsed.plan.orientations[i] = {
            state.plan_orientations[i].x,
            state.plan_orientations[i].y,
            state.plan_orientations[i].z,
        };
        for (int lane = 0; lane < 4; ++lane) {
            parsed.lanes[lane].valid = parsed.valid;
            parsed.lanes[lane].probability = state.lane_probabilities[lane];
            parsed.lanes[lane].std = state.lane_stds[lane];
            parsed.lanes[lane].points[i] = {
                state.lanes[lane][i].x,
                state.lanes[lane][i].y,
                state.lanes[lane][i].z,
            };
        }
        for (int edge = 0; edge < 2; ++edge) {
            parsed.road_edges[edge].valid = parsed.valid;
            parsed.road_edges[edge].std = state.road_edge_stds[edge];
            parsed.road_edges[edge].points[i] = {
                state.road_edges[edge][i].x,
                state.road_edges[edge][i].y,
                state.road_edges[edge][i].z,
            };
        }
    }
    for (int i = 0; i < kDesireLen; ++i)
        parsed.meta.desire_state[i] = state.desire_state[i];

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

    parsed.stop_line.valid = state.stop_line.valid != 0;
    parsed.stop_line.probability = state.stop_line.probability;
    parsed.stop_line.position = {
        state.stop_line.x,
        state.stop_line.y,
        state.stop_line.z,
    };
    parsed.stop_line.speed = state.stop_line.speed;
    parsed.stop_line.time = state.stop_line.time;

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
    return make_projection_state(state.calibration.roll, state.calibration.pitch, state.calibration.yaw);
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
    if (create) {
        if (ftruncate(fd_, static_cast<off_t>(map_size_)) != 0) {
            std::perror("ftruncate ipc channel");
            close();
            return false;
        }
    } else {
        // 생산자가 O_CREAT 직후 ftruncate 전이면 shm이 요청보다 작을 수 있다.
        // 그대로 mmap하면 header를 읽는 순간 SIGBUS가 난다.
        struct stat st {};
        if (fstat(fd_, &st) != 0 || static_cast<size_t>(st.st_size) < map_size_) {
            std::fprintf(stderr,
                         "ipc channel size mismatch name=%s actual=%lld expected=%zu\n",
                         name_.c_str(), static_cast<long long>(st.st_size), map_size_);
            close();
            return false;
        }
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

bool K230LatestChannel::read_new(uint64_t *last_seq, void *payload,
                                 size_t payload_capacity, int timeout_ms) const
{
    const uint64_t start = k230_now_ns();
    const uint64_t timeout_ns = timeout_ms < 0
        ? UINT64_MAX
        : static_cast<uint64_t>(timeout_ms) * 1000000ULL;
    while (true) {
        uint64_t seq = 0;
        if (read(payload, payload_capacity, &seq) && (!last_seq || seq != *last_seq)) {
            if (last_seq) *last_seq = seq;
            return true;
        }
        if (timeout_ms == 0) return false;
        if (timeout_ms > 0 && k230_now_ns() - start >= timeout_ns) return false;
        usleep(1000);
    }
}

namespace {

size_t queue_map_size(unsigned slot_count) {
    return sizeof(K230CanQueueHeader) +
        static_cast<size_t>(slot_count) * sizeof(K230CanBatch);
}

}  // namespace

bool k230_can_batch_is_fresh(const K230CanBatch &batch, uint64_t now_ns,
                             uint64_t max_age_ns) {
    return batch.valid && batch.timestamp_ns != 0 &&
        now_ns >= batch.timestamp_ns &&
        now_ns - batch.timestamp_ns <= max_age_ns;
}

K230CanQueue::~K230CanQueue() {
    close();
}

bool K230CanQueue::open(const char *name, unsigned slot_count, bool create) {
    close();
    if (!name || name[0] == '\0' || slot_count == 0) return false;

    name_ = name;
    fd_ = shm_open(name_.c_str(), O_RDWR | (create ? O_CREAT : 0), 0664);
    if (fd_ < 0) {
        std::perror("shm_open CAN queue");
        return false;
    }

    map_size_ = queue_map_size(slot_count);
    if (create) {
        struct stat st {};
        if (fstat(fd_, &st) != 0) {
            std::perror("fstat CAN queue");
            close();
            return false;
        }
        if (static_cast<size_t>(st.st_size) < map_size_ &&
            ftruncate(fd_, static_cast<off_t>(map_size_)) != 0) {
            std::fprintf(stderr,
                         "CAN queue resize failed name=%s actual=%lld expected=%zu\n",
                         name_.c_str(), static_cast<long long>(st.st_size), map_size_);
            std::perror("ftruncate CAN queue");
            close();
            return false;
        }
    } else {
        struct stat st {};
        if (fstat(fd_, &st) != 0 || static_cast<size_t>(st.st_size) < map_size_) {
            std::fprintf(stderr,
                         "CAN queue size mismatch name=%s actual=%lld expected=%zu\n",
                         name_.c_str(), static_cast<long long>(st.st_size), map_size_);
            close();
            return false;
        }
    }

    void *map = mmap(nullptr, map_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (map == MAP_FAILED) {
        std::perror("mmap CAN queue");
        close();
        return false;
    }

    header_ = static_cast<K230CanQueueHeader *>(map);
    slots_ = reinterpret_cast<K230CanBatch *>(
        reinterpret_cast<uint8_t *>(map) + sizeof(K230CanQueueHeader));
    if (create && (header_->magic != kK230CanQueueMagic ||
                   header_->version != kK230CanQueueVersion ||
                   header_->slot_count != slot_count)) {
        std::memset(map, 0, map_size_);
        header_->magic = kK230CanQueueMagic;
        header_->version = kK230CanQueueVersion;
        header_->slot_count = slot_count;
        header_->write_seq.store(0, std::memory_order_release);
        header_->read_seq.store(0, std::memory_order_release);
    }

    if (header_->magic != kK230CanQueueMagic ||
        header_->version != kK230CanQueueVersion ||
        header_->slot_count != slot_count) {
        std::fprintf(stderr,
                     "CAN queue header mismatch name=%s magic=0x%x version=%u slots=%u\n",
                     name_.c_str(), header_->magic, header_->version,
                     header_->slot_count);
        close();
        return false;
    }
    return true;
}

void K230CanQueue::close() {
    if (header_) {
        munmap(header_, map_size_);
        header_ = nullptr;
        slots_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    map_size_ = 0;
}

void K230CanQueue::reset() {
    if (!header_) return;
    header_->read_seq.store(0, std::memory_order_release);
    header_->write_seq.store(0, std::memory_order_release);
    for (unsigned i = 0; i < header_->slot_count; ++i) {
        slots_[i] = K230CanBatch{};
    }
}

bool K230CanQueue::push(const K230CanBatch &batch) {
    if (!header_ || !slots_) return false;
    const uint64_t write_seq = header_->write_seq.load(std::memory_order_relaxed);
    const uint64_t read_seq = header_->read_seq.load(std::memory_order_acquire);
    if (write_seq - read_seq >= header_->slot_count) return false;

    slots_[write_seq % header_->slot_count] = batch;
    header_->write_seq.store(write_seq + 1, std::memory_order_release);
    return true;
}

bool K230CanQueue::pop(K230CanBatch *batch) {
    if (!header_ || !slots_ || !batch) return false;
    const uint64_t read_seq = header_->read_seq.load(std::memory_order_relaxed);
    const uint64_t write_seq = header_->write_seq.load(std::memory_order_acquire);
    if (read_seq == write_seq) return false;
    if (write_seq < read_seq || write_seq - read_seq > header_->slot_count) {
        return false;
    }

    *batch = slots_[read_seq % header_->slot_count];
    header_->read_seq.store(read_seq + 1, std::memory_order_release);
    return true;
}

uint64_t K230CanQueue::depth() const {
    if (!header_) return 0;
    const uint64_t write_seq = header_->write_seq.load(std::memory_order_acquire);
    const uint64_t read_seq = header_->read_seq.load(std::memory_order_acquire);
    if (write_seq < read_seq) return 0;
    return std::min<uint64_t>(write_seq - read_seq, header_->slot_count);
}
