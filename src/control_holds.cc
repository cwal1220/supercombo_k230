#include "control_holds.h"

#include "utils_time.h"

PandaGateOutput PandaHealthGate::update(const K230PandaState &state, uint64_t now_ns,
                                        bool force_engaged) {
  PandaGateOutput out;
  out.state_fresh = timestamp_fresh_ns(state.timestamp_ns, now_ns, kPandaStateTimeoutNs);
  const bool transport_ready = out.state_fresh && state.connected != 0 &&
                               state.comms_healthy != 0 && state.tx_enabled != 0;
  const bool safety_ready = state.heartbeat_lost == 0 &&
                            state.safety_mode == kExpectedPandaSafetyModel &&
                            state.safety_param == kExpectedPandaSafetyParam;
  out.ready_raw = transport_ready && safety_ready;
  const bool controls_allowed_raw = out.ready_raw && state.controls_allowed != 0;
  out.controls_off_explicit = out.ready_raw && state.controls_allowed == 0;
  if (out.ready_raw) {
    have_last_ready_ = true;
    last_controls_allowed_ = controls_allowed_raw;
    last_ready_ns_ = now_ns;
  }
  /* 홀드는 판정 자체가 불가능할 때(!ready_raw)만 든다. 신선하고 전송 준비된
   * controls_allowed=0은 ready_raw라 여기 오지 않으므로 절대 유지되지 않는다. */
  out.hold_applied = !out.ready_raw && have_last_ready_ &&
                     now_ns >= last_ready_ns_ && now_ns - last_ready_ns_ <= kPandaHealthHoldNs;
  out.ready = force_engaged || out.ready_raw || out.hold_applied;
  out.controls_allowed = force_engaged || controls_allowed_raw ||
                         (out.hold_applied && last_controls_allowed_);
  return out;
}

PathHoldOutput PathHoldGate::update(const K230ModelState &model, uint64_t now_ns,
                                    uint64_t model_timeout_ns) {
  PathHoldOutput out;
  out.raw = path_from_model_state(model, now_ns, model_timeout_ns);
  out.path = out.raw;
  if (out.raw.usable_for_steering) {
    last_usable_ = out.raw;
    last_usable_model_timestamp_ns_ = model.model_timestamp_ns;
  } else if (out.raw.invalid_reason == "path_invalid" && last_usable_.usable_for_steering &&
             timestamp_fresh_ns(model.model_timestamp_ns, now_ns, model_timeout_ns) &&
             last_usable_model_timestamp_ns_ != 0 &&
             now_ns >= last_usable_model_timestamp_ns_ &&
             now_ns - last_usable_model_timestamp_ns_ <= kPathInvalidHoldNs) {
    out.path = last_usable_;
    out.path.invalid_reason.clear();
    out.hold_applied = true;
  }
  return out;
}
