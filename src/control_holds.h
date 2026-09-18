#pragma once

/* controlsd의 입력 안전 홀드 둘. 문서(source-layout.md "Control safety holds")가
 * 말하는 바로 그 동작이며, 100 Hz 루프에서 분리해 호스트 검사가 경계값을 본다.
 * - Panda 헬스 스냅샷의 짧은 공백은 100 ms까지 마지막 판정을 유지한다. 신선하고
 *   전송 준비된 controls_allowed=0은 절대 유지하지 않는다.
 * - 잘못된 plan 프레임 하나는 150 ms까지 마지막 유효 경로로 덮는다. 모델
 *   freshness 타임아웃(기본 250 ms)은 그대로 하드 게이트다. */

#include <cstdint>

#include "ipc_messages.h"
#include "lateral_path.h"

constexpr uint32_t kExpectedPandaSafetyModel = 24;
constexpr uint32_t kExpectedPandaSafetyParam = 0;
constexpr uint64_t kPandaStateTimeoutNs = 1100000000ULL;
constexpr uint64_t kPandaHealthHoldNs = 100000000ULL;
constexpr uint64_t kPathInvalidHoldNs = 150000000ULL;

struct PandaGateOutput {
  bool state_fresh = false;
  bool ready_raw = false;
  bool controls_off_explicit = false;
  bool hold_applied = false;
  bool ready = false;
  bool controls_allowed = false;
};

class PandaHealthGate {
public:
  PandaGateOutput update(const K230PandaState &state, uint64_t now_ns, bool force_engaged);

private:
  bool have_last_ready_ = false;
  bool last_controls_allowed_ = false;
  uint64_t last_ready_ns_ = 0;
};

struct PathHoldOutput {
  LateralPath raw;
  LateralPath path;
  bool hold_applied = false;
};

class PathHoldGate {
public:
  PathHoldOutput update(const K230ModelState &model, uint64_t now_ns, uint64_t model_timeout_ns);

private:
  LateralPath last_usable_;
  uint64_t last_usable_model_timestamp_ns_ = 0;
};
