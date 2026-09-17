#include "overlay_state.h"

#include "utils_math.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {

struct EngageBlockLabel {
    const char *reason;
    const char *label;
};

constexpr EngageBlockLabel kEngageBlockLabels[] = {
    {"brake_error", "BRAKE ERROR"},
    {"control_stale", "CONTROL STALE"},
    {"controller_disabled", "CONTROL OFF"},
    {"door_open", "DOOR OPEN"},
    {"esp_disabled", "ESP OFF"},
    {"esp_stale", "ESP STALE"},
    {"gear_not_drive", "GEAR NOT D"},
    {"lateral_plan_invalid", "PLAN INVALID"},
    {"lateral_plan_stale", "PLAN STALE"},
    {"mdps_fault", "MDPS FAULT"},
    {"not_engaged", "STANDBY"},
    {"panda_controls_off", "PANDA CTRL OFF"},
    {"panda_not_ready", "PANDA NOT READY"},
    {"park_brake", "PARK BRAKE"},
    {"path_invalid", "PATH INVALID"},
    {"seatbelt_unlatched", "SEATBELT"},
    {"seeds_missing", "CAN SEEDS"},
    {"speed_invalid", "SPEED INVALID"},
    {"stopped", "STOPPED"},
    {"steering_angle_limit", "ANGLE LIMIT"},
    {"vehicle_state_stale", "CAR STALE"},
    {"yaw_rate_invalid", "YAW INVALID"},
};

} // namespace

const char *engage_block_label(const char *block)
{
    if (!block || block[0] == '\0') return nullptr;
    for (const EngageBlockLabel &entry : kEngageBlockLabels) {
        if (std::strcmp(block, entry.reason) == 0) return entry.label;
    }
    return nullptr;
}

/* ---- 공유 상태 → HUD 상태 ---- */

void hud_apply_panda_state(const K230PandaState &panda, bool fresh, OverlayHudState *hud)
{
    hud->panda_connected = fresh && panda.connected != 0;
    hud->panda_healthy = fresh && panda.comms_healthy != 0;
    hud->panda_faults = fresh ? panda.faults : 0;
}

void hud_apply_control_state(const K230ControlState &c, bool fresh, OverlayHudState *hud)
{
    hud->controller_enabled = fresh && c.enabled != 0;
    hud->controller_engaged = fresh && c.engaged != 0;
    hud->controller_active = fresh && c.active != 0;
    hud->lateral_mode_available = fresh;
    hud->laneless_mode = fresh && (c.hud_flags & kK230HudFlagLaneless) != 0;
    hud->vehicle_fresh = fresh && c.vehicle_fresh != 0;
    hud->steering_fault = fresh && c.steering_fault != 0;
    hud->left_blinker = fresh && c.left_blinker != 0;
    hud->right_blinker = fresh && c.right_blinker != 0;
    hud->cruise_active = fresh && c.cruise_active != 0;
    hud->brake_hold = fresh && (c.hud_flags & kK230HudFlagBrakeHold) != 0;
    hud->gear = fresh ? c.gear : 0;
    hud->cluster_speed_kph = fresh ? c.cluster_speed_kph : 0.0f;
    hud->ego_speed_kph = fresh ? c.ego_speed_kph : 0.0f;
    hud->cruise_max_speed_kph = fresh ? c.cruise_max_speed_kph : 0.0f;
    hud->cruise_command_speed_kph = fresh ? c.cruise_command_speed_kph : 0.0f;
    hud->radar_lead_valid = fresh && c.radar_lead_valid != 0;
    hud->radar_lead_distance_m = fresh ? c.radar_lead_distance_m : 0.0f;
    hud->radar_lead_relative_speed_mps = fresh ? c.radar_lead_relative_speed_mps : 0.0f;
    hud->departure_alert_type = fresh
        ? static_cast<DepartureAlertType>(c.departure_alert_type)
        : DepartureAlertType::none;
    hud->green_light_alert_armed = fresh && c.green_light_alert_armed != 0;
    hud->tpms_valid = fresh && c.tpms_valid != 0;
    hud->tpms_unit = fresh ? static_cast<int>(c.tpms_unit) : 0;
    hud->tpms_pressure_fl = fresh ? c.tpms_pressure_fl : 0.0f;
    hud->tpms_pressure_fr = fresh ? c.tpms_pressure_fr : 0.0f;
    hud->tpms_pressure_rl = fresh ? c.tpms_pressure_rl : 0.0f;
    hud->tpms_pressure_rr = fresh ? c.tpms_pressure_rr : 0.0f;
    hud->tpms_warning = fresh && c.tpms_warning != 0;
    hud->steering_angle_deg = fresh ? c.steering_angle_deg : 0.0f;
    hud->normalized_output = fresh ? c.normalized_output : 0.0f;
    hud->desired_torque = fresh ? c.desired_torque : 0;
    hud->apply_torque = fresh ? c.apply_torque : 0;
    hud->driver_torque = fresh ? c.driver_torque : 0;
    std::snprintf(hud->active_block, sizeof(hud->active_block), "%s",
                  fresh ? c.active_block : "control_stale");
}

void hud_apply_model_state(const K230ModelState &model, bool fresh, OverlayHudState *hud)
{
    const K230CalibrationState &calibration = model.calibration;
    hud->calibration_available = fresh;
    hud->calibration_status = calibration.status;
    hud->calibration_valid_blocks = calibration.valid_blocks;
    hud->calibration_roll_deg = rad_to_deg(calibration.roll);
    hud->calibration_pitch_deg = rad_to_deg(calibration.pitch);
    hud->calibration_yaw_deg = rad_to_deg(calibration.yaw);
}

void hud_apply_manager_state(const K230ManagerState &manager, bool fresh, bool model_ok,
                             OverlayHudState *hud)
{
    const unsigned total = fresh
        ? std::min<unsigned>(manager.process_count, kK230MaxProcesses) : 0;
    unsigned running = 0;
    for (unsigned i = 0; i < total; ++i) running += manager.processes[i].running ? 1U : 0U;
    hud->services_healthy = fresh && total >= 3 && running == total && model_ok;
}
