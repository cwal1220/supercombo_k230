/* overlay_state: 제어 스냅샷 → HUD 상태 매핑과 알림 선택 정책. 보드 없이,
 * OpenCV 없이 돈다. */
#include "control_block.h"
#include "overlay_state.h"

#include <gtest/gtest.h>
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

TEST(OverlayState, FirstSnapshotIsABaseline) {
  OverlayAlertEvents events;
  const auto first = events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::none);
  ASSERT_EQ(first.alert, OverlayAlert::none) << "첫 스냅샷은 기준값만 잡는다";
  const auto same = events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::none);
  ASSERT_EQ(same.alert, OverlayAlert::none) << "그대로인 카운터는 이벤트가 아니다";
  const auto engaged = events.update(control_with_events(6, 3, 2, 1), DepartureAlertType::none);
  // 새 engage id는 그 id로 engage 알림음을 낸다
  ASSERT_EQ(engaged.alert, OverlayAlert::engage);
  ASSERT_EQ(engaged.event_id, 6);
}

TEST(OverlayState, PriorityAndOneAlertPerFrame) {
  OverlayAlertEvents events;
  events.update(control_with_events(5, 3, 2, 1), DepartureAlertType::none);
  const K230ControlState burst = control_with_events(6, 4, 3, 2);
  const auto first = events.update(burst, DepartureAlertType::lead_departed);
  // 같은 프레임에서는 engage 거부가 가장 먼저다
  ASSERT_EQ(first.alert, OverlayAlert::unable);
  ASSERT_EQ(first.event_id, 3);
  const auto second = events.update(burst, DepartureAlertType::lead_departed);
  // 다음 프레임에 engage 알림음
  ASSERT_EQ(second.alert, OverlayAlert::engage);
  ASSERT_EQ(second.event_id, 6);
  const auto third = events.update(burst, DepartureAlertType::lead_departed);
  // 그다음 disengage 알림음
  ASSERT_EQ(third.alert, OverlayAlert::disengage);
  ASSERT_EQ(third.event_id, 4);
  const auto fourth = events.update(burst, DepartureAlertType::lead_departed);
  // 그다음 출발 알림음
  ASSERT_EQ(fourth.alert, OverlayAlert::signal_changed);
  ASSERT_EQ(fourth.event_id, 2);
  ASSERT_EQ(events.update(burst, DepartureAlertType::lead_departed).alert, OverlayAlert::none)
      << "이벤트는 모두 한 번씩만 소비된다";
}

TEST(OverlayState, DepartureWaitsForADisplayedType) {
  OverlayAlertEvents events;
  events.update(control_with_events(0, 0, 0, 0), DepartureAlertType::none);
  const K230ControlState departed = control_with_events(0, 0, 0, 7);
  ASSERT_EQ(events.update(departed, DepartureAlertType::none).alert, OverlayAlert::none)
      << "표시할 알림 종류가 없으면 출발 id를 소비하지 않는다";
  const auto later = events.update(departed, DepartureAlertType::green_light);
  // 알림 종류가 표시되면 같은 id로 울린다
  ASSERT_EQ(later.alert, OverlayAlert::signal_changed);
  ASSERT_EQ(later.event_id, 7);
}

TEST(OverlayState, ControlsdRestartRebaselines) {
  OverlayAlertEvents events;
  events.update(control_with_events(40, 39, 12, 9), DepartureAlertType::none);
  const auto restarted = events.update(control_with_events(1, 0, 0, 0), DepartureAlertType::none);
  ASSERT_EQ(restarted.alert, OverlayAlert::none)
      << "카운터가 줄면 controlsd 재시작이다. 기준을 다시 잡고 울리지 않는다";
  const auto next = events.update(control_with_events(2, 0, 0, 0), DepartureAlertType::none);
  // 재시작 뒤 이벤트는 새 기준으로 잡는다
  ASSERT_EQ(next.alert, OverlayAlert::engage);
  ASSERT_EQ(next.event_id, 2);
}

TEST(OverlayState, ControlStateMapping) {
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
  // 신선한 제어 스냅샷은 필드 그대로 옮긴다
  ASSERT_TRUE(hud.controller_enabled);
  ASSERT_TRUE(hud.controller_engaged);
  ASSERT_TRUE(hud.laneless_mode);
  ASSERT_EQ(hud.cluster_speed_kph, 63.5f);
  ASSERT_EQ(hud.apply_torque, -120);
  ASSERT_EQ(hud.departure_alert_type, DepartureAlertType::green_light);
  ASSERT_STREQ(hud.active_block, "not_engaged");
  hud_apply_control_state(c, false, &hud);
  // 낡은 스냅샷은 HUD를 비우고 control_stale로 표시한다
  ASSERT_FALSE(hud.controller_enabled);
  ASSERT_FALSE(hud.laneless_mode);
  ASSERT_EQ(hud.cluster_speed_kph, 0.0f);
  ASSERT_EQ(hud.apply_torque, 0);
  ASSERT_EQ(hud.departure_alert_type, DepartureAlertType::none);
  ASSERT_STREQ(hud.active_block, "control_stale");
  // engage 차단 라벨은 아는 사유에만 있다
  ASSERT_NE(engage_block_label("panda_not_ready"), nullptr);
  ASSERT_STREQ(engage_block_label("panda_not_ready"), "PANDA NOT READY");
  ASSERT_EQ(engage_block_label("no_such_reason"), nullptr);
  ASSERT_EQ(engage_block_label(""), nullptr);
  for (const BlockReasonRow &row : kBlockReasons) {
    if (row.reason == BlockReason::None) continue;
    const char *label = engage_block_label(row.name);
    // 모든 차단 사유가 제 라벨로 HUD에 뜬다
    ASSERT_NE(label, nullptr);
    ASSERT_NE(label[0], '\0');
    ASSERT_STREQ(label, row.label);
  }
}

}  // namespace
