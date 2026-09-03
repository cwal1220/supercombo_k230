#include "departure_alert.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kDriveGear = 5;
constexpr float kStoppedSpeedMps = 0.1f;
constexpr float kMovingResetSpeedMps = 0.5f;
constexpr double kMinimumStopTimeS = 1.0;

constexpr float kMinimumLeadDistanceM = 1.0f;
constexpr float kMaximumLeadDistanceM = 30.0f;
constexpr double kLeadArmTimeS = 1.0;
constexpr float kLeadDistanceChangeM = 0.5f;
constexpr float kLeadRelativeSpeedMps = 0.5f;
constexpr double kLeadConfirmTimeS = 0.3;

constexpr float kMinimumStoppedPlanDistanceM = -5.0f;
constexpr float kStoppedPlanDistanceM = 5.0f;
// 094 plan은 정차 중 중앙 1.3 m로 붕괴하고 출발 예고 시 10 m를 넘긴 뒤 25 m까지
// 0.8 s 더 걸린다(09-01 루트 13회 정차, 오경보 차이 없음).
constexpr float kOpenPlanDistanceM = 10.0f;
constexpr double kTrafficSignalDisplayStopTimeS = 3.0;
constexpr double kGreenLightConfirmTimeS = 0.3;
constexpr double kAlertDisplayTimeS = 3.0;

bool elapsed(double now_s, double since_s, double duration_s) {
  return since_s >= 0.0 && now_s >= since_s &&
         now_s - since_s >= duration_s;
}

}  // namespace

const char *departure_alert_name(DepartureAlertType type) {
  switch (type) {
    case DepartureAlertType::lead_departed:
      return "lead_departed";
    case DepartureAlertType::green_light:
      return "green_light";
    case DepartureAlertType::none:
    default:
      return "none";
  }
}

DepartureAlertOutput DepartureAlertDetector::update(
    const DepartureAlertInput &input) {
  if (active_type_ != DepartureAlertType::none &&
      input.now_s >= active_until_s_) {
    active_type_ = DepartureAlertType::none;
  }

  const bool reset =
      !input.vehicle_valid || input.gear != kDriveGear || input.gas_pressed ||
      !std::isfinite(input.speed_mps) ||
      input.speed_mps > kMovingResetSpeedMps;
  if (reset) {
    reset_cycle();
  } else if (input.speed_mps <= kStoppedSpeedMps) {
    if (stopped_since_s_ < 0.0) stopped_since_s_ = input.now_s;
  } else {
    stopped_since_s_ = -1.0;
    reset_lead();
    reset_green_light();
  }

  const bool stopped_long_enough =
      elapsed(input.now_s, stopped_since_s_, kMinimumStopTimeS);
  const bool stopped = stopped_since_s_ >= 0.0;
  if (stopped && !consumed_) {
    bool lead_departed = false;
    bool green_light_changed = false;
    const bool lead_present =
        input.lead_valid && std::isfinite(input.lead_distance_m) &&
        input.lead_distance_m >= kMinimumLeadDistanceM &&
        input.lead_distance_m <= kMaximumLeadDistanceM;

    if (lead_present) {
      if (stopped_long_enough && input.lead_updated) {
        if (lead_seen_since_s_ < 0.0) {
          lead_seen_since_s_ = input.now_s;
          lead_baseline_distance_m_ = input.lead_distance_m;
        } else {
          lead_baseline_distance_m_ =
              std::min(lead_baseline_distance_m_, input.lead_distance_m);
        }

        lead_armed_ =
            elapsed(input.now_s, lead_seen_since_s_, kLeadArmTimeS);
        const bool departing =
            lead_armed_ &&
            input.lead_distance_m - lead_baseline_distance_m_ >
                kLeadDistanceChangeM &&
            input.lead_relative_speed_mps > kLeadRelativeSpeedMps;
        if (departing) {
          if (lead_depart_candidate_since_s_ < 0.0)
            lead_depart_candidate_since_s_ = input.now_s;
          lead_departed = elapsed(input.now_s, lead_depart_candidate_since_s_,
                                  kLeadConfirmTimeS);
        } else {
          lead_depart_candidate_since_s_ = -1.0;
        }
      }
    } else {
      reset_lead();
    }

    if (input.model_updated && input.model_valid) {
      const bool model_stopped =
          std::isfinite(input.plan_distance_m) &&
          input.plan_distance_m > kMinimumStoppedPlanDistanceM &&
          input.plan_distance_m < kStoppedPlanDistanceM;

      /* 모델이 여기서 멈추겠다고 계획한 채 3 s가 지나면 무장한다. vision lead는
       * 무장을 막지 않는다: 094 lead 확률은 상수 0.6에 가까워 앞차 유무를
       * 가르지 못하고(drive15: 9회 정차 중 4회가 가짜 lead로 무장 실패),
       * 앞차가 출발해도 plan이 같은 식으로 열려 출발 알림으로는 옳다. */
      if (!green_light_armed_) {
        green_light_armed_ =
            model_stopped && elapsed(input.now_s, stopped_since_s_,
                                     kTrafficSignalDisplayStopTimeS);
      }

      const bool road_open =
          green_light_armed_ && input.plan_distance_m > kOpenPlanDistanceM;
      if (road_open) {
        if (green_light_candidate_since_s_ < 0.0)
          green_light_candidate_since_s_ = input.now_s;
        green_light_changed =
            elapsed(input.now_s, green_light_candidate_since_s_,
                    kGreenLightConfirmTimeS);
      } else {
        green_light_candidate_since_s_ = -1.0;
      }
    }

    // 둘이 같은 프레임에 성립하면 더 구체적인 사유(앞차 출발)를 쓴다.
    if (lead_departed) {
      trigger(DepartureAlertType::lead_departed, input.now_s);
    } else if (green_light_changed) {
      trigger(DepartureAlertType::green_light, input.now_s);
    }
  }

  return {
      active_type_,
      event_id_,
      lead_armed_,
      green_light_armed_,
  };
}

void DepartureAlertDetector::reset_cycle() {
  consumed_ = false;
  stopped_since_s_ = -1.0;
  reset_lead();
  reset_green_light();
}

void DepartureAlertDetector::reset_lead() {
  lead_seen_since_s_ = -1.0;
  lead_depart_candidate_since_s_ = -1.0;
  lead_baseline_distance_m_ = 0.0f;
  lead_armed_ = false;
}

void DepartureAlertDetector::reset_green_light() {
  green_light_candidate_since_s_ = -1.0;
  green_light_armed_ = false;
}

void DepartureAlertDetector::trigger(DepartureAlertType type, double now_s) {
  active_type_ = type;
  active_until_s_ = now_s + kAlertDisplayTimeS;
  consumed_ = true;
  if (++event_id_ == 0) ++event_id_;
  reset_lead();
  reset_green_light();
}
