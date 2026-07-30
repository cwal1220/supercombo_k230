#include "vehicle_can.h"

#include <algorithm>
#include <cmath>

#include "hyundai_can.h"

namespace {

constexpr int kMdpsToiUnavailableFaultFrames = 100;
constexpr double kBlinkerHoldSeconds = 0.5;

// openpilot K7 parser와 같은 bus에서 온 frame만 차량 상태에 반영한다.
bool expected_openpilot_k7_bus(uint32_t address, uint8_t bus) {
  switch (address) {
    case kHyundaiLkas11Address:
      return bus == kK7CameraBus;
    case kHyundaiClu11Address:
    case kHyundaiEsp12Address:
    case kHyundaiScc11Address:
    case kHyundaiScc12Address:
    case kHyundaiTcs13Address:
    case kHyundaiTcs15Address:
    case kHyundaiEEms11Address:
    case kHyundaiElectGearAddress:
    case kHyundaiCgw1Address:
    case kHyundaiCgw2Address:
    case kHyundaiLca11Address:
    case kHyundaiTpms11Address:
      return bus == kK7PowertrainBus;
    case kHyundaiMdps12Address:
      return bus == kK7MdpsBus;
    case kHyundaiSas11Address:
      return bus == kK7PowertrainBus || bus == kK7MdpsBus;
    default:
      return true;
  }
}

// little-endian DBC 신호를 raw integer로 읽는다.
uint32_t get_signal_le(const uint8_t *data, int start_bit, int length) {
  uint32_t raw = 0;
  for (int i = 0; i < length; ++i) {
    const int bit = start_bit + i;
    if (data[bit / 8] & (1U << (bit % 8))) {
      raw |= 1U << i;
    }
  }
  return raw;
}

// signed raw 값을 지정 bit 수로 sign extension한다.
int32_t sign_extend(uint32_t raw, int bits) {
  const uint32_t sign_bit = 1U << (bits - 1);
  if ((raw & sign_bit) == 0U) return static_cast<int32_t>(raw);
  return static_cast<int32_t>(raw - (1U << bits));
}

// 특정 timestamp가 freshness timeout 안에 있는지 확인한다.
bool fresh_time(double timestamp_s, double now_s, double timeout_s) {
  return timestamp_s >= 0.0 && now_s >= timestamp_s && now_s - timestamp_s <= timeout_s;
}

}  // namespace

int panda_driver_torque_from_raw_signal(int raw_signal) {
  return static_cast<int>(static_cast<float>(raw_signal) * 0.79f - 808.0f);
}

Sas11Values decode_sas11(const std::array<uint8_t, 8> &data) {
  Sas11Values values;
  values.steering_angle_deg =
      static_cast<float>(sign_extend(get_signal_le(data.data(), 0, 16), 16)) * 0.1f;
  values.steering_rate_deg =
      static_cast<float>(get_signal_le(data.data(), 16, 8)) * 4.0f;
  return values;
}

Mdps12Values decode_mdps12(const std::array<uint8_t, 8> &data) {
  Mdps12Values values;
  values.driver_torque_raw_signal = static_cast<int>(get_signal_le(data.data(), 0, 11));
  values.driver_torque = values.driver_torque_raw_signal - 1024;
  values.panda_driver_torque =
      panda_driver_torque_from_raw_signal(values.driver_torque_raw_signal);
  values.toi_unavailable = get_signal_le(data.data(), 12, 1) != 0;
  values.toi_active = get_signal_le(data.data(), 13, 1) != 0;
  values.toi_fault = get_signal_le(data.data(), 14, 1) != 0;
  values.fail_state = get_signal_le(data.data(), 15, 1) != 0;
  values.msg_count = static_cast<int>(get_signal_le(data.data(), 16, 8));
  values.checksum = static_cast<int>(get_signal_le(data.data(), 24, 8));
  values.sensor_error = get_signal_le(data.data(), 37, 1) != 0;
  return values;
}

Esp12Values decode_esp12(const std::array<uint8_t, 8> &data) {
  Esp12Values values;
  const float yaw_rate_deg_s =
      static_cast<float>(get_signal_le(data.data(), 40, 13)) * 0.01f - 40.95f;
  values.yaw_rate_deg_s = yaw_rate_deg_s;
  values.yaw_rate_rad_s = yaw_rate_deg_s * 0.017453292519943295f;
  values.yaw_rate_valid = get_signal_le(data.data(), 54, 1) == 0;
  values.lat_accel_mps2 =
      static_cast<float>(get_signal_le(data.data(), 0, 11)) * 0.01f - 10.23f;
  return values;
}

Scc11Values decode_scc11(const std::array<uint8_t, 8> &data) {
  Scc11Values values;
  values.main_mode = get_signal_le(data.data(), 0, 1) != 0;
  values.set_speed = static_cast<float>(get_signal_le(data.data(), 8, 8));
  values.object_valid = get_signal_le(data.data(), 22, 2) != 0;
  values.object_distance_m =
      static_cast<float>(get_signal_le(data.data(), 33, 11)) * 0.1f;
  values.object_relative_speed_mps =
      static_cast<float>(get_signal_le(data.data(), 44, 12)) * 0.1f - 170.0f;
  return values;
}

Tcs13Values decode_tcs13(const std::array<uint8_t, 8> &data) {
  Tcs13Values values;
  const int acc_enable = static_cast<int>(get_signal_le(data.data(), 43, 2));
  values.brake_light = get_signal_le(data.data(), 11, 1) != 0;
  values.brake_error = acc_enable == 3;
  values.driver_override = static_cast<int>(get_signal_le(data.data(), 45, 2));
  values.park_brake = get_signal_le(data.data(), 53, 1) != 0;
  values.brake_pressed = get_signal_le(data.data(), 55, 1) != 0;
  return values;
}

Tcs15Values decode_tcs15(const std::array<uint8_t, 8> &data) {
  Tcs15Values values;
  values.esp_disabled = get_signal_le(data.data(), 8, 2) != 0;
  values.brake_hold = get_signal_le(data.data(), 29, 3) == 2;
  return values;
}

EEms11Values decode_e_ems11(const std::array<uint8_t, 8> &data) {
  EEms11Values values;
  values.gas = static_cast<int>(get_signal_le(data.data(), 56, 8));
  values.gas_pressed = values.gas > 0;
  return values;
}

ElectGearValues decode_elect_gear(const std::array<uint8_t, 8> &data) {
  ElectGearValues values;
  values.gear = static_cast<int>(get_signal_le(data.data(), 16, 4));
  return values;
}

Cgw1Values decode_cgw1(const std::array<uint8_t, 8> &data) {
  Cgw1Values values;
  values.driver_door_open = get_signal_le(data.data(), 8, 2) != 0;
  values.passenger_door_open = get_signal_le(data.data(), 35, 1) != 0;
  values.front_door_open = values.driver_door_open || values.passenger_door_open;
  values.seatbelt_unlatched = get_signal_le(data.data(), 10, 2) == 0;
  values.left_blinker = get_signal_le(data.data(), 19, 2) != 0;
  values.right_blinker = get_signal_le(data.data(), 62, 2) != 0;
  values.hazard = get_signal_le(data.data(), 33, 2) != 0;
  return values;
}

Cgw2Values decode_cgw2(const std::array<uint8_t, 8> &data) {
  Cgw2Values values;
  values.rear_right_door_open = get_signal_le(data.data(), 23, 1) != 0;
  values.rear_left_door_open = get_signal_le(data.data(), 24, 1) != 0;
  values.rear_door_open = values.rear_left_door_open || values.rear_right_door_open;
  return values;
}

Lca11Values decode_lca11(const std::array<uint8_t, 8> &data) {
  Lca11Values values;
  values.left_blindspot = get_signal_le(data.data(), 8, 2) != 0;
  values.right_blindspot = get_signal_le(data.data(), 16, 2) != 0;
  return values;
}

Tpms11Values decode_tpms11(const std::array<uint8_t, 8> &data) {
  Tpms11Values values;
  values.unit = static_cast<int>(get_signal_le(data.data(), 11, 2));
  const float factor =
      values.unit == 1 ? 0.72519f : (values.unit == 2 ? 0.1f : 1.0f);
  auto pressure = [factor](uint8_t raw) {
    return raw == 0xff ? 0.0f : static_cast<float>(raw) * factor;
  };
  values.pressure_fl = pressure(data[2]);
  values.pressure_fr = pressure(data[3]);
  values.pressure_rl = pressure(data[4]);
  values.pressure_rr = pressure(data[5]);
  values.warning =
      get_signal_le(data.data(), 0, 2) != 0 ||
      get_signal_le(data.data(), 4, 1) != 0 ||
      get_signal_le(data.data(), 5, 1) != 0 ||
      get_signal_le(data.data(), 6, 1) != 0 ||
      get_signal_le(data.data(), 7, 1) != 0;
  return values;
}

void update_k7_vehicle_can_state(K7VehicleCanState *state, uint32_t address,
                                 const std::array<uint8_t, 8> &data,
                                 uint8_t length, uint8_t bus,
                                 double now_s) {
  if (!state) return;
  if (!expected_openpilot_k7_bus(address, bus)) return;

  if (address == kHyundaiLkas11Address && length >= 8) {
    state->lkas11_seed = data;
    state->has_lkas11_seed = true;
    state->lkas11_time_s = now_s;
  } else if (address == kHyundaiClu11Address && length >= 4) {
    state->clu11_seed = {{data[0], data[1], data[2], data[3]}};
    state->has_clu11_seed = true;
    const HyundaiClu11Values clu = decode_clu11(state->clu11_seed);
    state->clu_button = clu.cruise_sw_state;
    state->cluster_speed = clu.speed + clu.speed_decimal;
    state->speed_unit_mph = clu.speed_unit_mph;
    state->clu_alive_count = clu.alive_count;
    state->clu11_time_s = now_s;
  } else if (address == kHyundaiSas11Address && length >= 5) {
    const Sas11Values sas = decode_sas11(data);
    state->steering_angle_deg = sas.steering_angle_deg;
    state->steering_rate_deg = sas.steering_rate_deg;
    state->sas11_time_s = now_s;
  } else if (address == kHyundaiEsp12Address && length >= 8) {
    const Esp12Values esp = decode_esp12(data);
    state->yaw_rate_rad_s = esp.yaw_rate_rad_s;
    state->yaw_rate_valid = esp.yaw_rate_valid;
    state->lat_accel_mps2 = esp.lat_accel_mps2;
    state->esp12_time_s = now_s;
  } else if (address == kHyundaiMdps12Address && length >= 8) {
    state->mdps12_seed = data;
    state->has_mdps12_seed = true;
    const Mdps12Values mdps = decode_mdps12(data);
    state->driver_torque = mdps.driver_torque;
    state->panda_driver_torque = mdps.panda_driver_torque;
    state->mdps_toi_unavailable = mdps.toi_unavailable;
    state->mdps_error_count = mdps.toi_unavailable ? state->mdps_error_count + 1 : 0;
    state->mdps_hard_fault = mdps.toi_fault || mdps.fail_state || mdps.sensor_error;
    state->steering_fault = state->mdps_error_count > kMdpsToiUnavailableFaultFrames;
    state->mdps12_time_s = now_s;
  } else if (address == kHyundaiScc11Address && length >= 8) {
    const Scc11Values scc = decode_scc11(data);
    state->cruise_main = scc.main_mode;
    state->cruise_set_speed = scc.set_speed;
    state->radar_lead_valid = scc.object_valid;
    state->radar_lead_distance_m = scc.object_distance_m;
    state->radar_lead_relative_speed_mps = scc.object_relative_speed_mps;
    state->scc11_time_s = now_s;
  } else if (address == kHyundaiScc12Address && length >= 8) {
    state->acc_mode = static_cast<int>(get_signal_le(data.data(), 13, 2));
    state->cruise_active = state->acc_mode != 0;
  } else if (address == kHyundaiTcs13Address && length >= 8) {
    const Tcs13Values tcs = decode_tcs13(data);
    state->brake_light = tcs.brake_light;
    state->brake_error = tcs.brake_error;
    state->park_brake = tcs.park_brake;
    state->brake_pressed = tcs.brake_pressed;
    state->tcs13_time_s = now_s;
  } else if (address == kHyundaiTcs15Address && length >= 4) {
    const Tcs15Values tcs = decode_tcs15(data);
    state->esp_disabled = tcs.esp_disabled;
    state->brake_hold = tcs.brake_hold;
    state->tcs15_time_s = now_s;
  } else if (address == kHyundaiEEms11Address && length >= 8) {
    const EEms11Values ems = decode_e_ems11(data);
    state->gas = ems.gas;
    state->gas_pressed = ems.gas_pressed;
    state->e_ems11_time_s = now_s;
  } else if (address == kHyundaiElectGearAddress && length >= 8) {
    state->gear = decode_elect_gear(data).gear;
    state->elect_gear_time_s = now_s;
  } else if (address == kHyundaiCgw1Address && length >= 8) {
    const Cgw1Values cgw = decode_cgw1(data);
    state->driver_door_open = cgw.driver_door_open;
    state->passenger_door_open = cgw.passenger_door_open;
    state->door_open = state->driver_door_open || state->passenger_door_open ||
                       state->rear_left_door_open || state->rear_right_door_open;
    state->seatbelt_unlatched = cgw.seatbelt_unlatched;
    if (cgw.left_blinker) state->left_blinker_until_s = now_s + kBlinkerHoldSeconds;
    if (cgw.right_blinker) state->right_blinker_until_s = now_s + kBlinkerHoldSeconds;
    state->left_blinker = now_s < state->left_blinker_until_s;
    state->right_blinker = now_s < state->right_blinker_until_s;
    state->hazard = cgw.hazard;
    state->cgw1_time_s = now_s;
  } else if (address == kHyundaiCgw2Address && length >= 8) {
    const Cgw2Values cgw = decode_cgw2(data);
    state->rear_left_door_open = cgw.rear_left_door_open;
    state->rear_right_door_open = cgw.rear_right_door_open;
    state->door_open = state->driver_door_open || state->passenger_door_open ||
                       state->rear_left_door_open || state->rear_right_door_open;
    state->cgw2_time_s = now_s;
  } else if (address == kHyundaiLca11Address && length >= 8) {
    const Lca11Values lca = decode_lca11(data);
    state->left_blindspot = lca.left_blindspot;
    state->right_blindspot = lca.right_blindspot;
    state->lca11_time_s = now_s;
  } else if (address == kHyundaiTpms11Address && length >= 6) {
    const Tpms11Values tpms = decode_tpms11(data);
    state->tpms_unit = tpms.unit;
    state->tpms_pressure_fl = tpms.pressure_fl;
    state->tpms_pressure_fr = tpms.pressure_fr;
    state->tpms_pressure_rl = tpms.pressure_rl;
    state->tpms_pressure_rr = tpms.pressure_rr;
    state->tpms_warning = tpms.warning;
    state->tpms11_time_s = now_s;
  }
}

bool k7_vehicle_state_fresh(const K7VehicleCanState &state, double now_s,
                            double timeout_s) {
  return fresh_time(state.lkas11_time_s, now_s, timeout_s) &&
         fresh_time(state.clu11_time_s, now_s, timeout_s) &&
         fresh_time(state.sas11_time_s, now_s, timeout_s) &&
         fresh_time(state.mdps12_time_s, now_s, timeout_s) &&
         fresh_time(state.tcs13_time_s, now_s, timeout_s) &&
         fresh_time(state.tcs15_time_s, now_s, timeout_s) &&
         fresh_time(state.e_ems11_time_s, now_s, timeout_s) &&
         fresh_time(state.elect_gear_time_s, now_s, timeout_s) &&
         fresh_time(state.cgw1_time_s, now_s, timeout_s) &&
         fresh_time(state.cgw2_time_s, now_s, timeout_s);
}

bool k7_seed_frames_ready(const K7VehicleCanState &state) {
  return state.has_lkas11_seed && state.has_clu11_seed && state.has_mdps12_seed;
}

bool k7_tpms_state_fresh(const K7VehicleCanState &state, double now_s,
                         double timeout_s) {
  return fresh_time(state.tpms11_time_s, now_s, timeout_s);
}

float k7_cruise_set_speed_kph(const K7VehicleCanState &state) {
  if (state.cruise_set_speed <= 0.0f || state.cruise_set_speed >= 255.0f) return 0.0f;
  return state.cruise_set_speed * (state.speed_unit_mph ? 1.609344f : 1.0f);
}
