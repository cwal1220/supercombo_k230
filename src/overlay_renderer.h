#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include "display.h"
#include "model_output.h"
#include "projection.h"

struct OverlayHudState {
    bool panda_connected = false;
    bool panda_healthy = false;
    bool panda_tx_enabled = false;
    bool panda_controls_allowed = false;
    bool controller_enabled = false;
    bool controller_engaged = false;
    bool controller_active = false;
    bool vehicle_fresh = false;
    bool steering_fault = false;
    bool services_healthy = false;
    unsigned panda_faults = 0;
    float speed_kph = 0.0f;
    float steering_angle_deg = 0.0f;
    float normalized_output = 0.0f;
    int desired_torque = 0;
    int apply_torque = 0;
    int driver_torque = 0;
    float cpu_percent = 0.0f;
    float memory_percent = 0.0f;
    float cpu_temp_c = 0.0f;
    float preview_fps = 0.0f;
    float model_fps = 0.0f;
    float overlay_fps = 0.0f;
    bool calibration_available = false;
    unsigned calibration_status = 0;
    int calibration_valid_blocks = 0;
    float calibration_roll_deg = 0.0f;
    float calibration_pitch_deg = 0.0f;
    float calibration_yaw_deg = 0.0f;
};

class OverlayRenderer {
public:
    OverlayRenderer() = default;

    void draw(display_buffer *buffer, const ParsedModelOutput &output,
              const ProjectionState &projection,
              const OverlayHudState &hud = OverlayHudState{},
              bool rotate_landscape = false) const;
};

#endif
