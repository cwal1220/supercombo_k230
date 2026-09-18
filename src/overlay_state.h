#ifndef OVERLAY_STATE_H
#define OVERLAY_STATE_H

/* 공유 상태 스냅샷(K230*State) → HUD 표시 상태. 그리기와 무관해 OpenCV 없이
 * 컴파일되며, k230_overlayd와 hud_snapshot이 같은 매핑을 쓴다. */

#include "departure_alert.h"
#include "ipc_messages.h"

/* engage 차단 사유 → HUD 라벨. HUD 상태줄과 engage 거부 토스트가 같은 표를
 * 쓰도록 여기 한 벌만 둔다. 모르는 사유면 nullptr을 돌려주고, 표시 방식은
 * 호출부가 정한다. */
const char *engage_block_label(const char *block);

struct OverlayHudState {
    bool panda_connected = false;
    bool panda_healthy = false;
    bool controller_enabled = false;
    bool controller_engaged = false;
    bool controller_active = false;
    bool lateral_mode_available = false;
    bool laneless_mode = false;
    bool vehicle_fresh = false;
    bool steering_fault = false;
    bool left_blinker = false;
    bool right_blinker = false;
    int turn_signal_step = 0;
    bool cruise_active = false;
    bool brake_hold = false;
    bool services_healthy = false;
    bool network_connected = false;
    unsigned panda_faults = 0;
    int gear = 0;
    int wifi_signal_dbm = 0;
    float cluster_speed_kph = 0.0f;
    float ego_speed_kph = 0.0f;
    float cruise_max_speed_kph = 0.0f;
    float cruise_command_speed_kph = 0.0f;
    bool radar_lead_valid = false;
    float radar_lead_distance_m = 0.0f;
    float radar_lead_relative_speed_mps = 0.0f;
    DepartureAlertType departure_alert_type = DepartureAlertType::none;
    bool green_light_alert_armed = false;
    char engage_alert_message[64] = {};
    bool tpms_valid = false;
    int tpms_unit = 0;
    float tpms_pressure_fl = 0.0f;
    float tpms_pressure_fr = 0.0f;
    float tpms_pressure_rl = 0.0f;
    float tpms_pressure_rr = 0.0f;
    bool tpms_warning = false;
    float steering_angle_deg = 0.0f;
    float normalized_output = 0.0f;
    int desired_torque = 0;
    int apply_torque = 0;
    int driver_torque = 0;
    float cpu_percent = 0.0f;
    float memory_percent = 0.0f;
    float storage_percent = 0.0f;
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

/* 공유 상태 스냅샷 → HUD 표시 상태. fresh가 아니면 값을 0/false로 두어 HUD가 "--"를
 * 그린다. k230_overlayd와 hud_snapshot이 같은 매핑을 쓴다. */
void hud_apply_panda_state(const K230PandaState &panda, bool fresh, OverlayHudState *hud);
void hud_apply_control_state(const K230ControlState &control, bool fresh, OverlayHudState *hud);
void hud_apply_model_state(const K230ModelState &model, bool fresh, OverlayHudState *hud);
/* model_ok: 유효하고 신선한 모델 출력이 있는지. services_healthy의 조건 중 하나. */
void hud_apply_manager_state(const K230ManagerState &manager, bool fresh, bool model_ok,
                             OverlayHudState *hud);

/* 오버레이가 울리는 알림. 열거 순서가 같은 프레임 안의 우선순위다. */
enum class OverlayAlert { none, unable, engage, disengage, signal_changed };

/* controlsd 이벤트 카운터 → 이 프레임에 울릴 알림 하나. 신선한 제어 스냅샷에만
 * 부른다. overlay가 독립적으로 재시작될 수 있으므로 첫 스냅샷은 사용자 이벤트가
 * 아니라 기준값이고, controlsd 재시작으로 카운터가 줄어들면 기준값을 다시 잡는다.
 * 거부 > engage > disengage > 출발 순으로 새 이벤트 하나만 고르고, 같은 프레임의
 * 나머지는 다음 프레임에 잡힌다. 출발은 표시 중인 알림 유형이 있을 때만 소비한다. */
class OverlayAlertEvents {
public:
    struct Decision {
        OverlayAlert alert = OverlayAlert::none;
        uint32_t event_id = 0;
    };
    Decision update(const K230ControlState &control, DepartureAlertType departure_type);

private:
    // 기준값을 (다시) 잡은 프레임이면 true.
    bool baseline(const K230ControlState &control);

    bool initialized_ = false;
    uint32_t last_engage_ = 0;
    uint32_t last_disengage_ = 0;
    uint32_t last_reject_ = 0;
    uint32_t last_departure_ = 0;
};

#endif
