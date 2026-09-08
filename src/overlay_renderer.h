#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include "departure_alert.h"
#include "k230_ipc.h"
#include "model_output.h"
#include "projection.h"

#include <cstdint>
#include <memory>
#include <string>

/* engage 차단 사유 → HUD 라벨. HUD 상태줄과 engage 거부 토스트가 같은 표를
 * 쓰도록 여기 한 벌만 둔다. 모르는 사유면 nullptr을 돌려주고, 표시 방식은
 * 호출부가 정한다. */
const char *engage_block_label(const char *block);

/* 깜빡이 애니메이션은 켜진 순간을 0으로 하는 단계 수로 그린다. 단계 진행은
 * k230_overlayd가 시각 기준으로 계산하므로 재그리기 빈도에 영향받지 않는다. */
constexpr int kTurnSignalSteps = 25;

/* ARGB8888 CPU 버퍼. 렌더러는 DRM 헤더 없이 이 뷰만 본다. */
struct OverlayTarget {
    void *map = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
};

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

struct TrafficSignalSprites;

class OverlayRenderer {
public:
    /* <dir>/traffic_*_retro-270x155-v3.png. 없으면 신호등 표시만 빠진다. */
    bool load_assets(const std::string &dir);

    void draw(const OverlayTarget &target, const ParsedModelOutput &output,
              const ProjectionState &projection,
              const OverlayHudState &hud = OverlayHudState{},
              bool rotate_landscape = false) const;

private:
    std::shared_ptr<const TrafficSignalSprites> sprites_;
};

#endif
