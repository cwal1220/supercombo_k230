#pragma once

#include <cstdint>

enum class DepartureAlertType : uint32_t {
  none = 0,
  lead_departed = 1,
  green_light = 2,
};

const char *departure_alert_name(DepartureAlertType type);

struct DepartureAlertInput {
  double now_s = 0.0;
  bool vehicle_valid = false;
  int gear = 0;
  float speed_mps = 0.0f;
  bool gas_pressed = false;

  bool lead_updated = false;
  bool lead_valid = false;
  float lead_distance_m = 0.0f;
  float lead_relative_speed_mps = 0.0f;

  bool model_updated = false;
  bool model_valid = false;
  float plan_distance_m = 0.0f;
};

struct DepartureAlertOutput {
  DepartureAlertType type = DepartureAlertType::none;
  uint32_t event_id = 0;
  bool lead_armed = false;
  bool green_light_armed = false;
};

class DepartureAlertDetector {
public:
  DepartureAlertOutput update(const DepartureAlertInput &input);

private:
  void reset_cycle();
  void reset_lead();
  void reset_green_light();
  void trigger(DepartureAlertType type, double now_s);

  bool consumed_ = false;
  double stopped_since_s_ = -1.0;

  double lead_seen_since_s_ = -1.0;
  double lead_depart_candidate_since_s_ = -1.0;
  float lead_baseline_distance_m_ = 0.0f;
  bool lead_armed_ = false;

  double green_light_candidate_since_s_ = -1.0;
  bool green_light_armed_ = false;

  DepartureAlertType active_type_ = DepartureAlertType::none;
  double active_until_s_ = -1.0;
  uint32_t event_id_ = 0;
};
