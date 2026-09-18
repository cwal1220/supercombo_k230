/* overlay_state: 제어 스냅샷 → HUD 상태 매핑과 알림 선택 정책. 보드 없이,
 * OpenCV 없이 돈다. */
#include "check_harness.h"
#include "control_block.h"
#include "overlay_state.h"

#include <cstring>

namespace {

K230ControlState control_with_events(uint32_t engage, uint32_t disengage, uint32_t reject,
                                     uint32_t departure) {
  K230ControlState c;
  c.engage_event_id = engage;
  c.disengage_event_id = disengage;
  c.engage_reject_event_id = reject;
  c.departure_alert_event_id = departure;
  return c;
}

void verify_first_snapshot_is_a_baseline() {
  OverlayAlertEvents events;
  const auto first = events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::none);
  require(first.alert == OverlayAlert::none, "the first snapshot only sets the baseline");
  const auto same = events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::none);
  require(same.alert == OverlayAlert::none, "unchanged counters are not events");
  const auto engaged = events.update(control_with_events(6, 3, 2, 1), DepartureAlertType::none);
  require(engaged.alert == OverlayAlert::engage && engaged.event_id == 6,
          "a new engage id plays the engage tone with that id");
}

void verify_priority_and_one_alert_per_frame() {
  OverlayAlertEvents events;
  events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::none);
  const K230ControlState burst = control_with_events(6, 4, 3, 2);
  const auto first = events.update(burst, DepartureAlertType::lead_departed);
  require(first.alert == OverlayAlert::unable && first.event_id == 3,
          "engage rejection wins over everything else in the same frame");
  const auto second = events.update(burst, DepartureAlertType::lead_departed);
  require(second.alert == OverlayAlert::engage && second.event_id == 6,
          "the engage tone follows on the next frame");
  const auto third = events.update(burst, DepartureAlertType::lead_departed);
  require(third.alert == OverlayAlert::disengage && third.event_id == 4,
          "then the disengage tone");
  const auto fourth = events.update(burst, DepartureAlertType::lead_departed);
  require(fourth.alert == OverlayAlert::signal_changed && fourth.event_id == 2,
          "then the departure tone");
  require(events.update(burst, DepartureAlertType::lead_departed).alert == OverlayAlert::none,
          "every event is consumed exactly once");
}

void verify_departure_waits_for_a_displayed_type() {
  OverlayAlertEvents events;
  events.update(control_with_events(0, 0, 0, 0), DepartureAlertType::none);
  const K230ControlState departed = control_with_events(0, 0, 0, 7);
  require(events.update(departed, DepartureAlertType::none).alert == OverlayAlert::none,
          "a departure id without a displayed alert type is not consumed");
  const auto later = events.update(departed, DepartureAlertType::green_light);
  require(later.alert == OverlayAlert::signal_changed && later.event_id == 7,
          "the same id plays once the alert type is displayed");
}

void verify_controlsd_restart_rebaselines() {
  OverlayAlertEvents events;
  events.update(control_with_events(40, 39, 12, 9), DepartureAlertType::none);
  const auto restarted = events.update(control_with_events(1, 0, 0, 0), DepartureAlertType::none);
  require(restarted.alert == OverlayAlert::none,
          "counters that went backwards mean controlsd restarted: rebaseline, no tone");
  const auto next = events.update(control_with_events(2, 0, 0, 0), DepartureAlertType::none);
  require(next.alert == OverlayAlert::engage && next.event_id == 2,
          "events after the restart are detected against the new baseline");
}

void verify_control_state_mapping() {
  K230ControlState c;
  c.enabled = 1;
  c.engaged = 1;
  c.cluster_speed_kph = 63.5f;
  c.hud_flags = kK230HudFlagLaneless;
  c.departure_alert_type = static_cast<uint32_t>(DepartureAlertType::green_light);
  c.apply_torque = -120;
  std::snprintf(c.active_block, sizeof(c.active_block), "%s", "not_engaged");

  OverlayHudState hud;
  hud_apply_control_state(c, true, &hud);
  require(hud.controller_enabled && hud.controller_engaged && hud.laneless_mode &&
              hud.cluster_speed_kph == 63.5f && hud.apply_torque == -120 &&
              hud.departure_alert_type == DepartureAlertType::green_light &&
              std::strcmp(hud.active_block, "not_engaged") == 0,
          "a fresh control snapshot maps field for field");
  hud_apply_control_state(c, false, &hud);
  require(!hud.controller_enabled && !hud.laneless_mode && hud.cluster_speed_kph == 0.0f &&
              hud.apply_torque == 0 && hud.departure_alert_type == DepartureAlertType::none &&
              std::strcmp(hud.active_block, "control_stale") == 0,
          "a stale snapshot zeroes the HUD and names the staleness");
  require(engage_block_label("panda_not_ready") != nullptr &&
              std::strcmp(engage_block_label("panda_not_ready"), "PANDA NOT READY") == 0 &&
              engage_block_label("no_such_reason") == nullptr &&
              engage_block_label("") == nullptr,
          "engage block labels resolve known reasons only");
  for (const BlockReasonRow &row : kBlockReasons) {
    if (row.reason == BlockReason::None) continue;
    const char *label = engage_block_label(row.name);
    require(label != nullptr && label[0] != '\0' && std::strcmp(label, row.label) == 0,
            "every block reason reaches the HUD with its own label");
  }
}

}  // namespace

int main() {
  return run_checks("OVERLAY_STATE_OK", [] {
    verify_first_snapshot_is_a_baseline();
    verify_priority_and_one_alert_per_frame();
    verify_departure_waits_for_a_displayed_type();
    verify_controlsd_restart_rebaselines();
    verify_control_state_mapping();
  });
}
