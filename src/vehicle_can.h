#pragma once

#include <array>
#include <cstdint>

constexpr uint32_t kHyundaiSas11Address = 688;       // 0x2b0
constexpr uint32_t kHyundaiEsp12Address = 544;       // 0x220
constexpr uint32_t kHyundaiScc12Address = 1057;      // 0x421
constexpr uint32_t kHyundaiTcs13Address = 916;       // 0x394
constexpr uint32_t kHyundaiTcs15Address = 1287;      // 0x507
constexpr uint32_t kHyundaiEEms11Address = 881;      // 0x371
constexpr uint32_t kHyundaiElectGearAddress = 882;   // 0x372
constexpr uint32_t kHyundaiCgw1Address = 1345;       // 0x541
constexpr uint32_t kHyundaiCgw2Address = 1363;       // 0x553
constexpr uint32_t kHyundaiLca11Address = 1419;      // 0x58b

constexpr uint8_t kK7PowertrainBus = 0;
constexpr uint8_t kK7MdpsBus = 1;
constexpr uint8_t kK7CameraBus = 2;

struct Sas11Values {
  float steering_angle_deg = 0.0f;
  float steering_rate_deg = 0.0f;
};

struct Mdps12Values {
  int driver_torque = 0;
  int driver_torque_raw_signal = 0;
  int panda_driver_torque = 0;
  bool toi_unavailable = false;
  bool toi_active = false;
  bool toi_fault = false;
  bool fail_state = false;
  bool sensor_error = false;
  int msg_count = 0;
  int checksum = 0;
};

struct Esp12Values {
  float yaw_rate_deg_s = 0.0f;
  float yaw_rate_rad_s = 0.0f;
  bool yaw_rate_valid = true;
  float lat_accel_mps2 = 0.0f;
};

struct Tcs13Values {
  bool brake_light = false;
  bool brake_error = false;
  int driver_override = 0;
  bool park_brake = false;
  bool brake_pressed = false;
};

struct Tcs15Values {
  bool esp_disabled = false;
  bool brake_hold = false;
};

struct EEms11Values {
  int gas = 0;
  bool gas_pressed = false;
};

struct ElectGearValues {
  int gear = 0;
};

struct Cgw1Values {
  bool driver_door_open = false;
  bool passenger_door_open = false;
  bool front_door_open = false;
  bool seatbelt_unlatched = false;
  bool left_blinker = false;
  bool right_blinker = false;
  bool hazard = false;
};

struct Cgw2Values {
  bool rear_left_door_open = false;
  bool rear_right_door_open = false;
  bool rear_door_open = false;
};

struct Lca11Values {
  bool left_blindspot = false;
  bool right_blindspot = false;
};

struct K7VehicleCanState {
  std::array<uint8_t, 8> lkas11_seed{};
  std::array<uint8_t, 4> clu11_seed{};
  std::array<uint8_t, 8> mdps12_seed{};

  bool has_lkas11_seed = false;
  bool has_clu11_seed = false;
  bool has_mdps12_seed = false;

  double lkas11_time_s = -1.0;
  double clu11_time_s = -1.0;
  double sas11_time_s = -1.0;
  double esp12_time_s = -1.0;
  double mdps12_time_s = -1.0;
  double tcs13_time_s = -1.0;
  double tcs15_time_s = -1.0;
  double e_ems11_time_s = -1.0;
  double elect_gear_time_s = -1.0;
  double cgw1_time_s = -1.0;
  double cgw2_time_s = -1.0;
  double lca11_time_s = -1.0;

  int clu_button = 0;
  float cluster_speed = 0.0f;
  bool speed_unit_mph = false;
  int clu_alive_count = 0;

  float steering_angle_deg = 0.0f;
  float steering_rate_deg = 0.0f;
  float yaw_rate_rad_s = 0.0f;
  bool yaw_rate_valid = true;
  float lat_accel_mps2 = 0.0f;
  int driver_torque = 0;
  int panda_driver_torque = 0;
  bool mdps_toi_unavailable = false;
  bool mdps_hard_fault = false;
  int mdps_error_count = 0;
  bool steering_fault = false;

  int gear = 0;
  bool brake_pressed = false;
  bool brake_light = false;
  bool brake_error = false;
  bool park_brake = false;
  bool esp_disabled = false;
  bool brake_hold = false;
  int gas = 0;
  bool gas_pressed = false;
  bool driver_door_open = false;
  bool passenger_door_open = false;
  bool rear_left_door_open = false;
  bool rear_right_door_open = false;
  bool door_open = false;
  bool seatbelt_unlatched = false;
  bool left_blinker = false;
  bool right_blinker = false;
  double left_blinker_until_s = -1.0;
  double right_blinker_until_s = -1.0;
  bool hazard = false;
  bool left_blindspot = false;
  bool right_blindspot = false;
  int acc_mode = 0;
  bool cruise_active = false;
};

// Panda safety의 MDPS12 driver torque scale과 같은 값을 계산한다.
int panda_driver_torque_from_raw_signal(int raw_signal);

// K7 SAS11 steering angle/rate를 해석한다.
Sas11Values decode_sas11(const std::array<uint8_t, 8> &data);

// K7 MDPS12 steering feedback/fault 값을 해석한다.
Mdps12Values decode_mdps12(const std::array<uint8_t, 8> &data);

// K7 ESP12 yaw/lateral acceleration 값을 해석한다.
Esp12Values decode_esp12(const std::array<uint8_t, 8> &data);

// K7 TCS13 brake 관련 상태를 해석한다.
Tcs13Values decode_tcs13(const std::array<uint8_t, 8> &data);

// K7 TCS15 ESP/brake-hold 상태를 해석한다.
Tcs15Values decode_tcs15(const std::array<uint8_t, 8> &data);

// K7 E_EMS11 hybrid gas 값을 해석한다.
EEms11Values decode_e_ems11(const std::array<uint8_t, 8> &data);

// K7 ELECT_GEAR gear 값을 해석한다.
ElectGearValues decode_elect_gear(const std::array<uint8_t, 8> &data);

// K7 CGW1 door/seatbelt/blinker 값을 해석한다.
Cgw1Values decode_cgw1(const std::array<uint8_t, 8> &data);

// K7 CGW2 rear door 값을 해석한다.
Cgw2Values decode_cgw2(const std::array<uint8_t, 8> &data);

// K7 LCA11 blind-spot indicators를 해석한다.
Lca11Values decode_lca11(const std::array<uint8_t, 8> &data);

// raw CAN frame을 K7 vehicle state에 반영한다.
void update_k7_vehicle_can_state(K7VehicleCanState *state, uint32_t address,
                                 const std::array<uint8_t, 8> &data,
                                 uint8_t length, uint8_t bus,
                                 double now_s);

// K7 lateral 제어에 필요한 필수 CAN이 모두 최신인지 확인한다.
bool k7_vehicle_state_fresh(const K7VehicleCanState &state, double now_s,
                            double timeout_s = 0.5);

// LKAS/CLU/MDPS seed frame이 모두 준비됐는지 확인한다.
bool k7_seed_frames_ready(const K7VehicleCanState &state);
