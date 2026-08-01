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

constexpr float kStopLineProbability = 0.5f;
constexpr float kMaximumStopLineDistanceM = 5.0f;
constexpr float kMinimumStoppedPlanDistanceM = -5.0f;
constexpr float kStoppedPlanDistanceM = 5.0f;
constexpr float kOpenPlanDistanceM = 25.0f;
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

    if (!consumed_ && input.model_updated && input.model_valid) {
      const bool stop_line_near =
          input.stop_line_valid &&
          input.stop_line_probability > kStopLineProbability &&
          std::isfinite(input.stop_line_distance_m) &&
          input.stop_line_distance_m < kMaximumStopLineDistanceM;
      const bool model_stopped =
          std::isfinite(input.plan_distance_m) &&
          input.plan_distance_m > kMinimumStoppedPlanDistanceM &&
          input.plan_distance_m < kStoppedPlanDistanceM;

      if (!green_light_armed_) {
        green_light_armed_ =
            stop_line_near && model_stopped &&
            elapsed(input.now_s, stopped_since_s_,
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

    if (green_light_changed) {
      trigger(DepartureAlertType::green_light, input.now_s);
    } else if (lead_departed && !green_light_armed_) {
      trigger(DepartureAlertType::lead_departed, input.now_s);
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
