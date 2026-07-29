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
    bool left_blinker = false;
    bool right_blinker = false;
    bool cruise_active = false;
    bool services_healthy = false;
    bool network_connected = false;
    unsigned panda_faults = 0;
    int gear = 0;
    int wifi_signal_dbm = 0;
    float speed_kph = 0.0f;
    float cruise_set_speed_kph = 0.0f;
    bool radar_lead_valid = false;
    float radar_lead_distance_m = 0.0f;
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
    char active_block[32] = {};
    char network_interface[16] = {};
    char network_ipv4[16] = {};
};

class OverlayRenderer {
public:
    OverlayRenderer() = default;

    void draw(display_buffer *buffer, const ParsedModelOutput &output,
              const ProjectionState &projection,
              const OverlayHudState &hud = OverlayHudState{},
              bool rotate_landscape = false) const;

private:
    mutable bool previous_left_blinker_ = false;
    mutable bool previous_right_blinker_ = false;
    mutable int blinker_blinking_rate_ = 120;
};

#endif
