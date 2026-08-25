#include "hyundai_steering.h"

#include "common_utils.h"

#include <algorithm>
#include <cmath>

namespace {

}  // namespace

int apply_hyundai_steer_torque_limits(int desired_torque, int last_torque, int driver_torque,
                                      const HyundaiSteeringLimits &limits) {
  const int driver_max_torque = limits.steer_max +
      (limits.steer_driver_allowance + driver_torque * limits.steer_driver_factor) *
      limits.steer_driver_multiplier;
  const int driver_min_torque = -limits.steer_max +
      (-limits.steer_driver_allowance + driver_torque * limits.steer_driver_factor) *
      limits.steer_driver_multiplier;

  const int max_steer_allowed = std::max(std::min(limits.steer_max, driver_max_torque), 0);
  const int min_steer_allowed = std::min(std::max(-limits.steer_max, driver_min_torque), 0);
  int apply_torque = clamp_int(desired_torque, min_steer_allowed, max_steer_allowed);

  if (last_torque > 0) {
    apply_torque = clamp_int(apply_torque,
                            std::max(last_torque - limits.steer_delta_down, -limits.steer_delta_up),
                            last_torque + limits.steer_delta_up);
  } else {
    apply_torque = clamp_int(apply_torque,
                            last_torque - limits.steer_delta_up,
                            std::min(last_torque + limits.steer_delta_down, limits.steer_delta_up));
  }
  return apply_torque;
}

float mdps_speed_for_lkas(float cluster_speed_raw, bool lkas_active, bool is_mph,
                          float spoof_speed_kph) {
  if (!std::isfinite(cluster_speed_raw) || cluster_speed_raw < 0.0f) return 0.0f;
  const float safe_spoof_kph = std::isfinite(spoof_speed_kph)
      ? std::clamp(spoof_speed_kph, 30.0f, 100.0f)
      : 60.0f;
  const float threshold = is_mph ? safe_spoof_kph / 1.609344f : safe_spoof_kph;
  if (!lkas_active || cluster_speed_raw > threshold) return cluster_speed_raw;
  return threshold;
}
