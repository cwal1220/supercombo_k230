#include "k230_ipc.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

static_assert(sizeof(K230IpcHeader) == 40, "K230IpcHeader layout is part of the Python manager ABI");
static_assert(sizeof(K230ControlState) == 232,
              "K230ControlState layout is shared by controlsd and overlay");
static_assert(offsetof(K230ControlState, hud_flags) == 184,
              "K230ControlState HUD flag offset is part of the shared ABI");
static_assert(offsetof(K230ControlState, engage_event_id) == 188,
              "K230ControlState engagement event offset is part of the shared ABI");

namespace {

float model_t_idx(int i)
{
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return static_cast<float>(10.0 * t * t);
}

double model_t_idx_double(int i)
{
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return 10.0 * t * t;
}

double model_x_idx_double(int i)
{
    const double t = static_cast<double>(i) / static_cast<double>(kTrajectorySize - 1);
    return 192.0 * t * t;
}

} // namespace

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

    map_size_ = sizeof(K230FrameRingHeader) +
        static_cast<size_t>(width) * height * 3 / 2 * slots;
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

    state.lateral_target.valid = lateral_target.valid ? 1 : 0;
    state.lateral_target.lookahead_x = lateral_target.lookahead_x;
    state.lateral_target.target_y = lateral_target.target_y;
    state.lateral_target.heading = lateral_target.heading;
    state.lateral_target.curvature = lateral_target.curvature;

    state.lateral_plan.valid = lateral_target.valid ? 1 : 0;
    state.lateral_plan.mpc_solution_valid = lateral_target.mpc_solution_valid ? 1 : 0;
    state.lateral_plan.output_scale = lateral_target.output_scale;
    for (int i = 0; i < kLateralControlN; ++i) {
        state.lateral_plan.psis[i] = lateral_target.psis[i];
        state.lateral_plan.curvatures[i] = lateral_target.curvatures[i];
        state.lateral_plan.curvature_rates[i] = lateral_target.curvature_rates[i];
        state.lateral_plan.d_path_points[i] = lateral_target.d_path_points[i];
    }
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
