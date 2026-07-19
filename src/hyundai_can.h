#pragma once

#include <array>
#include <cstdint>
#include <vector>

constexpr uint32_t kHyundaiLkas11Address = 832;   // 0x340
constexpr uint32_t kHyundaiClu11Address = 1265;   // 0x4f1
constexpr uint32_t kHyundaiMdps12Address = 593;   // 0x251
constexpr uint8_t kHyundaiMdps12TxBus = 2;

struct CanFrame {
  uint32_t address = 0;
  uint8_t bus = 0;
  uint8_t length = 0;
  std::array<uint8_t, 8> data = {};
};

struct HyundaiLkas11Values {
  int ldws_active_mode = 0;
  int ldws_sys_state = 0;
  int sys_warning = 0;
  int left_lane_depart = 0;
  int right_lane_depart = 0;
  int hba_lamp = 0;
  int fcw_bas_req = 0;
  int steer_torque = 0;
  bool steer_req = false;
  bool toi_fault = false;
  int hba_sys_state = 0;
  int fcw_opt = 0;
  int hba_opt = 0;
  int msg_count = 0;
  int fcw_sys_state = 0;
  int fcw_collision_warning = 0;
  int fusion_state = 0;
  int unknown1 = 0;
  int fcw_opt_usm = 0;
  int ldws_opt_usm = 0;
  int unknown2 = 0;
};

struct HyundaiClu11Values {
  int cruise_sw_state = 0;
  int cruise_sw_main = 0;
  int sld_main_sw = 0;
  int parity_bit = 0;
  float speed_decimal = 0.0f;
  float speed = 0.0f;
  bool speed_unit_mph = false;
  int detent_out = 0;
  int rheostat_level = 0;
  int clu_info = 0;
  int amp_info = 0;
  int alive_count = 0;
};

struct HyundaiLkasCommand {
  int apply_steer = 0;
  bool steer_req = false;
  bool cut_steer_temp = false;
  bool sys_warning = false;
  int sys_state = 0;
  bool left_lane = false;
  bool right_lane = false;
  int left_lane_depart = 0;
  int right_lane_depart = 0;
  int lkas_msg_count = 0;
  bool ldws_fix = false;
};

struct HyundaiCluCommand {
  int button = 0;
  float speed = 0.0f;
  int frame = 0;
};

struct HyundaiCanConfig {
  uint8_t main_bus = 0;
  uint8_t scc_bus = 0;
  uint8_t mdps_bus = 0;
  bool send_lkas_on_scc_bus = true;
  bool send_lkas_on_mdps_bus = true;
  bool send_clu11_speed_to_mdps = true;
  float mdps_speed_spoof_kph = 60.0f;
};

HyundaiLkas11Values decode_lkas11(const std::array<uint8_t, 8> &data);
HyundaiClu11Values decode_clu11(const std::array<uint8_t, 4> &data);

uint8_t hyundai_lkas11_checksum(const std::array<uint8_t, 8> &data);
CanFrame create_lkas11_frame(const HyundaiLkas11Values &seed, const HyundaiLkasCommand &command,
                             uint8_t bus);
CanFrame create_clu11_frame(const HyundaiClu11Values &seed, const HyundaiCluCommand &command,
                            uint8_t bus);
// 최신 MDPS12 seed에서 openpilot create_mdps12와 같은 오류 회피 frame을 만든다.
CanFrame create_mdps12_frame(const std::array<uint8_t, 8> &seed, int frame);

std::vector<CanFrame> build_k7_hev_lateral_can_frames(const HyundaiLkas11Values &lkas_seed,
                                                      const HyundaiClu11Values &clu_seed,
                                                      const HyundaiLkasCommand &lkas_command,
                                                      const HyundaiCanConfig &config,
                                                      bool lkas_active,
                                                      float cluster_speed,
                                                      bool is_mph,
                                                      int frame);
