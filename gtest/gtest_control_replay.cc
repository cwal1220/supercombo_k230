/* 횡제어와 K7 CAN: 신호 해석(vehicle_can), engage 게이트와 홀드, 토크 한계와 MDPS 고장 회피,
 * 곡률 제한, 학습값 소비. CanFixture는 녹화 CAN 픽스처를 컨트롤러에 흘려 openpilot 참조식과
 * 대조하며, 픽스처를 인자로 줄 때만 돈다. */
#include "control_holds.h"
#include "hyundai_can.h"
#include "ipc_messages.h"
#include "lateral_controller.h"
#include "model_output.h"
#include "lateral_path.h"
#include "lateral_torque.h"
#include "vehicle_can.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace {

/* 컨트롤러가 보는 수신 메시지 시각을 한 번에 t로 맞춘다. */
void stamp_can_times(VehicleCanState *vehicle, double t) {
  vehicle->lkas11_time_s = t;
  vehicle->clu11_time_s = t;
  vehicle->sas11_time_s = t;
  vehicle->esp12_time_s = t;
  vehicle->mdps12_time_s = t;
  vehicle->tcs13_time_s = t;
  vehicle->tcs15_time_s = t;
  vehicle->e_ems11_time_s = t;
  vehicle->elect_gear_time_s = t;
  vehicle->whl_spd11_time_s = t;
  vehicle->cgw1_time_s = t;
  vehicle->cgw2_time_s = t;
}

VehicleCanState ready_vehicle(double timestamp_s = 1.0) {
  VehicleCanState vehicle;
  vehicle.has_lkas11_seed = true;
  vehicle.has_clu11_seed = true;
  vehicle.has_mdps12_seed = true;
  stamp_can_times(&vehicle, timestamp_s);
  // 저속 조향 게이트를 넘는 주행 상태가 헬퍼의 기본값이다.
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  vehicle.cluster_speed_raw = 63.0f;
  vehicle.gear = 5;
  return vehicle;
}

LateralPath replay_path() {
  LateralPath path;
  path.left_valid = true;
  path.right_valid = true;
  path.usable_for_steering = true;
  path.point_count = 30;
  path.reach_m = 60.0f;
  return path;
}

LateralTarget replay_target() {
  LateralTarget target;
  target.valid = true;
  target.mpc_solution_valid = true;
  for (int i = 0; i < kLateralControlN; ++i) {
    target.psis[i] = 0.0008f * 20.0f * (10.0f * i * i / (32.0f * 32.0f));
    target.curvatures[i] = 0.0008f;
  }
  return target;
}

/* lag 보상 곡률과 clip_curvature의 독립 전사본. 한계값은 lateral_controller.h에서 그대로
 * 가져온다 — 숫자를 복제하면 구현이 바뀔 때 이 검증이 조용히 썩는다.
 * 픽스처 target은 capture_timestamp_ns=0이라 plan 나이 보정은 0이다. */
float reference_plan_curvature(const LateralTarget &target, float speed_mps,
                               float actuator_delay) {
  const float delay = std::max(0.01f, actuator_delay);
  float psi = target.psis[kLateralControlN - 1];
  if (delay <= 0.0f) {
    psi = target.psis[0];
  } else {
    for (int i = 1; i < kLateralControlN; ++i) {
      if (delay <= model_t_idx(i)) {
        const float p = (delay - model_t_idx(i - 1)) /
                        (model_t_idx(i) - model_t_idx(i - 1));
        psi = target.psis[i - 1] + p * (target.psis[i] - target.psis[i - 1]);
        break;
      }
    }
  }
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  const float current = target.curvatures[0];
  return current + 2.0f * (psi / (speed * delay) - current);
}

float reference_clip_curvature(float speed_mps, float prev_curvature, float desired) {
  const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
  const float rate_limit = kMaxLateralJerk / (speed * speed);
  desired = std::clamp(desired,
                       prev_curvature - rate_limit * kCurvatureRateWindowS,
                       prev_curvature + rate_limit * kCurvatureRateWindowS);
  const float accel_speed = std::max(speed, 1.0f);
  desired = std::clamp(desired,
                       -kMaxLateralAccel / (accel_speed * accel_speed),
                       kMaxLateralAccel / (accel_speed * accel_speed));
  return std::clamp(desired, -kMaxCurvature, kMaxCurvature);
}

TEST(ControlReplay, MdpsSpeedSpoof) {
  HyundaiCanConfig config;
  config.main_bus = 0;
  config.mdps_bus = 1;
  config.send_lkas_on_scc_bus = false;
  HyundaiLkas11Values lkas;
  HyundaiClu11Values clu;
  clu.speed_decimal = 0.375f;
  HyundaiLkasCommand command;
  command.steer_req = true;
  const auto frames = build_lateral_can_frames(
      lkas, clu, command, config, true, 20.0f, false, 1);
  ASSERT_EQ(frames.size(), 3) << "저속 프레임 구성(LKAS11, MDPS12, CLU11)";
  std::array<uint8_t, 4> bytes = {};
  std::copy_n(frames[2].data.begin(), bytes.size(), bytes.begin());
  const HyundaiClu11Values decoded = decode_clu11(bytes);
  // MDPS용 CLU11은 버스 1로 나간다
  ASSERT_EQ(frames[2].address, kHyundaiClu11Address);
  ASSERT_EQ(frames[2].bus, 1);
  ASSERT_NEAR(decoded.speed, 60.0f, 0.001f) << "MDPS용 CLU11 속도는 60 km/h로 바꿔 보낸다";
  ASSERT_NEAR(decoded.speed_decimal, 0.375f, 0.001f) << "MDPS용 CLU11의 소수부는 그대로 둔다";

  config.mdps_speed_spoof_kph = 72.0f;
  const auto custom_frames = build_lateral_can_frames(
      lkas, clu, command, config, true, 20.0f, false, 1);
  std::copy_n(custom_frames[2].data.begin(), bytes.size(), bytes.begin());
  ASSERT_NEAR(decode_clu11(bytes).speed, 72.0f, 0.001f) << "설정한 MDPS 속도로 바꿔 보낸다";
}

TEST(ControlReplay, Lca11) {
  std::array<uint8_t, 8> bytes{};
  bytes[1] = 1;
  bytes[2] = 2;
  const Lca11Values decoded = decode_lca11(bytes);
  // LCA11 사각지대 해석
  ASSERT_TRUE(decoded.left_blindspot);
  ASSERT_TRUE(decoded.right_blindspot);

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiLca11Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  // LCA11로 차량 상태 갱신
  ASSERT_TRUE(vehicle.left_blindspot);
  ASSERT_TRUE(vehicle.right_blindspot);
}

TEST(ControlReplay, WhlSpd11) {
  std::array<uint8_t, 8> bytes{};
  const auto set_speed_raw = [&bytes](int start_bit, float speed_kph) {
    set_signal_le(&bytes, start_bit, 14,
                  static_cast<uint32_t>(std::lround(speed_kph / 0.03125f)));
  };
  set_speed_raw(0, 40.0f);
  set_speed_raw(16, 41.0f);
  set_speed_raw(32, 39.0f);
  set_speed_raw(48, 40.5f);

  const WhlSpd11Values decoded = decode_whl_spd11(bytes);
  // WHL_SPD11 휠 속도 해석
  ASSERT_NEAR(decoded.speed_fl_kph, 40.0f, 0.001f);
  ASSERT_NEAR(decoded.speed_fr_kph, 41.0f, 0.001f);
  ASSERT_NEAR(decoded.speed_rl_kph, 39.0f, 0.001f);
  ASSERT_NEAR(decoded.speed_rr_kph, 40.5f, 0.001f);

  VehicleCanState vehicle;
  vehicle.cluster_speed_raw = 72.0f;
  update_vehicle_can_state(&vehicle, kHyundaiWhlSpd11Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  // WHL_SPD11 네 바퀴 평균이 차속이다
  ASSERT_EQ(vehicle.whl_spd11_time_s, 1.0);
  ASSERT_NEAR(vehicle_speed_kph(vehicle, 1.2), 40.125f, 0.001f);
  /* 클러스터로 대체하지 않는다: 도메인이 달라 최소 조향 속도 게이트가
   * 뒤집힌다. 대신 낡은 휠 속도는 vehicle_state_fresh에서 막힌다. */
  ASSERT_FALSE(std::isfinite(vehicle_speed_kph(vehicle, 1.6)))
      << "낡은 WHL_SPD11을 CLU 속도로 대체하지 않는다";
  ASSERT_FALSE(vehicle_state_fresh(vehicle, 1.6, 0.5)) << "낡은 WHL_SPD11은 제어를 막는다";
}

TEST(ControlReplay, Tpms11) {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 11, 2, 2);
  bytes[2] = 23;
  bytes[3] = 24;
  bytes[4] = 25;
  bytes[5] = 26;
  const Tpms11Values decoded = decode_tpms11(bytes);
  // TPMS11 공기압 해석
  ASSERT_EQ(decoded.unit, 2);
  ASSERT_NEAR(decoded.pressure_fl, 2.3f, 0.001f);
  ASSERT_NEAR(decoded.pressure_fr, 2.4f, 0.001f);
  ASSERT_NEAR(decoded.pressure_rl, 2.5f, 0.001f);
  ASSERT_NEAR(decoded.pressure_rr, 2.6f, 0.001f);
  ASSERT_FALSE(decoded.warning);

  bytes[0] |= 1U << 4;
  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiTpms11Address, bytes, 6,
                           kPowertrainBus, 1.0);
  // TPMS11로 차량 상태 갱신
  ASSERT_TRUE(tpms_state_fresh(vehicle, 2.0));
  ASSERT_TRUE(vehicle.tpms_warning);
  ASSERT_FALSE(tpms_state_fresh(vehicle, 7.0)) << "TPMS11 신선도 시한";

  bytes[2] = 0xff;
  update_vehicle_can_state(&vehicle, kHyundaiTpms11Address, bytes, 6,
                           kPowertrainBus, 8.0);
  // TPMS11에서 값이 없는 바퀴는 0
  ASSERT_TRUE(tpms_state_fresh(vehicle, 8.0));
  ASSERT_EQ(vehicle.tpms_pressure_fl, 0.0f);
  ASSERT_GT(vehicle.tpms_pressure_fr, 0.0f);
}

TEST(ControlReplay, Tcs15) {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 29, 3, 2);
  const Tcs15Values decoded = decode_tcs15(bytes);
  // TCS15 Auto Hold 작동 해석
  ASSERT_TRUE(decoded.brake_hold);
  ASSERT_FALSE(decoded.esp_disabled);

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiTcs15Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  // TCS15 Auto Hold로 차량 상태 갱신
  ASSERT_TRUE(vehicle.brake_hold);
  ASSERT_EQ(vehicle.tcs15_time_s, 1.0);

  set_signal_le(&bytes, 29, 3, 3);
  update_vehicle_can_state(&vehicle, kHyundaiTcs15Address, bytes,
                           bytes.size(), kPowertrainBus, 1.1);
  ASSERT_FALSE(vehicle.brake_hold) << "TCS15 대기 상태는 Auto Hold 작동이 아니다";
}

TEST(ControlReplay, Tcs13DriverOverride) {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 45, 2, 2);
  const Tcs13Values decoded = decode_tcs13(bytes);
  ASSERT_EQ(decoded.driver_override, 2) << "TCS13 운전자 가속 개입 해석";

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  ASSERT_EQ(vehicle.driver_override, 2) << "TCS13 운전자 가속 개입으로 차량 상태 갱신";
}

TEST(ControlReplay, Scc11) {
  std::array<uint8_t, 8> bytes{};
  bytes[0] = 1;
  bytes[1] = 88;
  set_signal_le(&bytes, 22, 2, 1);
  set_signal_le(&bytes, 33, 11, 54);
  set_signal_le(&bytes, 44, 12, 1715);
  const Scc11Values decoded = decode_scc11(bytes);
  // SCC11 크루즈 상태 해석
  ASSERT_TRUE(decoded.main_mode);
  ASSERT_NEAR(decoded.set_speed_raw, 88.0f, 0.001f);
  ASSERT_TRUE(decoded.object_valid);
  ASSERT_NEAR(decoded.object_distance_m, 5.4f, 0.001f);
  ASSERT_NEAR(decoded.object_relative_speed_mps, 1.5f, 0.001f);

  VehicleCanState vehicle;
  update_vehicle_can_state(&vehicle, kHyundaiScc11Address, bytes,
                           bytes.size(), kPowertrainBus, 1.0);
  // SCC11로 차량 상태 갱신
  ASSERT_TRUE(vehicle.cruise_main);
  ASSERT_NEAR(vehicle.cruise_set_speed_raw, 88.0f, 0.001f);
  ASSERT_TRUE(vehicle.radar_lead_valid);
  ASSERT_NEAR(vehicle.radar_lead_distance_m, 5.4f, 0.001f);
  ASSERT_NEAR(vehicle.radar_lead_relative_speed_mps, 1.5f, 0.001f);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 88.0f, 0.001f) << "SCC11 km/h 설정 속도";
  vehicle.speed_unit_mph = true;
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 141.622272f, 0.001f)
      << "SCC11 mph 설정 속도 변환";
}

void update_clu11(VehicleCanState *vehicle, float speed, int button,
                  bool unit_mph, double now_s) {
  std::array<uint8_t, 8> bytes{};
  set_signal_le(&bytes, 0, 3, static_cast<uint32_t>(button));
  set_signal_le(&bytes, 8, 9,
                static_cast<uint32_t>(std::lround(speed * 2.0f)));
  set_signal_le(&bytes, 17, 1, unit_mph ? 1U : 0U);
  update_vehicle_can_state(vehicle, kHyundaiClu11Address, bytes, 4,
                           kPowertrainBus, now_s);
}

void release_cruise_button(VehicleCanState *vehicle, float speed,
                           bool unit_mph, double now_s) {
  update_clu11(vehicle, speed, 0, unit_mph, now_s);
}

TEST(ControlReplay, FixedCruiseSpeedEstimate) {
  VehicleCanState vehicle;
  update_clu11(&vehicle, 64.0f, 2, false, 1.0);
  // 고정형 크루즈 SET은 클러스터 속도를 잡는다
  ASSERT_TRUE(vehicle.cruise_active);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 64.0f, 0.001f);

  release_cruise_button(&vehicle, 64.0f, false, 1.1);
  update_clu11(&vehicle, 64.0f, 1, false, 1.2);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 66.0f, 0.001f)
      << "작동 중 RES는 목표 속도를 올린다";
  update_clu11(&vehicle, 64.0f, 1, false, 1.3);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 66.0f, 0.001f)
      << "누르고 있는 버튼은 떼기 전까지 반복하지 않는다";

  release_cruise_button(&vehicle, 64.0f, false, 1.4);
  update_clu11(&vehicle, 64.0f, 2, false, 1.5);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 64.0f, 0.001f)
      << "작동 중 SET은 목표 속도를 내린다";
  release_cruise_button(&vehicle, 64.0f, false, 1.6);
  update_clu11(&vehicle, 64.0f, 4, false, 1.7);
  // CANCEL은 목표 속도를 남긴다
  ASSERT_FALSE(vehicle.cruise_active);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 64.0f, 0.001f);
  release_cruise_button(&vehicle, 70.0f, false, 1.8);
  update_clu11(&vehicle, 70.0f, 2, false, 1.9);
  // CANCEL 뒤 SET은 현재 속도를 잡는다
  ASSERT_TRUE(vehicle.cruise_active);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 70.0f, 0.001f);

  std::array<uint8_t, 8> tcs13{};
  set_signal_le(&tcs13, 55, 1, 1);
  update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13,
                           tcs13.size(), kPowertrainBus, 2.0);
  ASSERT_FALSE(vehicle.cruise_active) << "브레이크는 추정한 크루즈 작동을 끈다";
  set_signal_le(&tcs13, 55, 1, 0);
  update_vehicle_can_state(&vehicle, kHyundaiTcs13Address, tcs13,
                           tcs13.size(), kPowertrainBus, 2.1);
  release_cruise_button(&vehicle, 60.0f, false, 2.2);
  update_clu11(&vehicle, 60.0f, 1, false, 2.3);
  // 브레이크 뒤 RES는 추정 목표 속도를 되살린다
  ASSERT_TRUE(vehicle.cruise_active);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 70.0f, 0.001f);

  release_cruise_button(&vehicle, 70.0f, false, 2.4);
  std::array<uint8_t, 8> clu_main{};
  set_signal_le(&clu_main, 3, 1, 1);
  set_signal_le(&clu_main, 8, 9, 140);
  update_vehicle_can_state(&vehicle, kHyundaiClu11Address, clu_main, 4,
                           kPowertrainBus, 2.5);
  // MAIN을 누르면 작동을 끄고 목표 속도는 남긴다
  ASSERT_FALSE(vehicle.cruise_active);
  ASSERT_NEAR(cruise_set_speed_kph(vehicle), 70.0f, 0.001f);

  VehicleCanState imperial;
  update_clu11(&imperial, 40.0f, 2, true, 1.0);
  ASSERT_NEAR(cruise_set_speed_kph(imperial), 64.37376f, 0.001f)
      << "mph SET 변환";
  release_cruise_button(&imperial, 40.0f, true, 1.1);
  update_clu11(&imperial, 40.0f, 1, true, 1.2);
  ASSERT_NEAR(cruise_set_speed_kph(imperial), 67.592448f, 0.001f)
      << "mph 증가 단위";

  std::array<uint8_t, 8> scc11{};
  scc11[1] = 88;
  update_vehicle_can_state(&imperial, kHyundaiScc11Address, scc11,
                           scc11.size(), kPowertrainBus, 1.3);
  ASSERT_NEAR(cruise_set_speed_kph(imperial), 141.622272f, 0.001f)
      << "유효한 SCC 설정 속도가 고정형 추정보다 우선한다";
}

TEST(ControlReplay, MdpsFaultFilter) {
  VehicleCanState vehicle;
  std::array<uint8_t, 8> bytes{};
  bytes[1] = (1U << 6) | (1U << 7);
  update_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                           bytes.size(), kMdpsBus, 1.0);
  // MDPS ToiFlt/FailStat 일시 신호는 openpilot과 같게 거른다
  ASSERT_TRUE(vehicle.mdps_hard_fault);
  ASSERT_FALSE(vehicle.steering_fault);

  bytes[1] = 1U << 4;
  for (int frame = 0; frame < 100; ++frame) {
    update_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                             bytes.size(), kMdpsBus, 1.0 + frame * 0.02);
  }
  ASSERT_FALSE(vehicle.steering_fault) << "MDPS 사용 불가는 디바운스 기준까지 고장이 아니다";
  update_vehicle_can_state(&vehicle, kHyundaiMdps12Address, bytes,
                           bytes.size(), kMdpsBus, 3.0);
  ASSERT_TRUE(vehicle.steering_fault) << "MDPS 사용 불가가 이어지면 고장이다";
}

TEST(ControlReplay, BrakingDoesNotDisengage) {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음
  vehicle.brake_light = true;

  const auto brake_light_result =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  ASSERT_TRUE(brake_light_result.active) << "DriverBraking 없는 브레이크등만으로는 활성을 유지한다";

  vehicle.brake_pressed = true;
  const auto brake_pressed_result =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  ASSERT_TRUE(brake_pressed_result.active) << "DriverBraking도 횡제어를 disengage하지 않는다";
}

/* K7 MDPS는 steer 요청이 켜진 채 85도 위에 1초 머물면 fault를 낸다(2026-09-18 실측).
 * 85도 위: 토크를 램프로 0까지 내리고 steer 요청은 유지, 89프레임마다 2프레임 컷.
 * 램프는 fault 실측 하한(98프레임)보다 먼저 끝나야 하고, 컷과 복귀가 토크 0에서
 * 일어나야 어시스트가 빠졌다 돌아오는 충격이 없다. 즉시 0으로 떨어뜨리지 않는 것은
 * 짧게 스치는 커브에서 어시스트를 유지하기 위해서다. */
TEST(ControlReplay, LargeAngleFaultAvoidance) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 72.0f;
  int frame = 0;
  auto step = [&]() {
    const int f = frame++;
    stamp_can_times(&vehicle, 1.0 + f * 0.01);  // 4초 넘게 돌리므로 CAN 신선도를 유지한다
    return controller.update(replay_path(), replay_target(), vehicle, 1.0 + f * 0.01, f);
  };

  vehicle.steering_angle_deg = 20.0f;
  LateralControlResult result;
  for (int i = 0; i < 100; ++i) result = step();
  // MDPS 고장 각도 아래에서는 토크가 나간다
  ASSERT_TRUE(result.active);
  ASSERT_NE(result.apply_torque, 0);

  vehicle.steering_angle_deg = 100.0f;
  const int crossed = frame;
  const int ramp = config.steering_params.avoid_lkas_fault_max_frames - 20;
  while (frame < crossed + 89) {
    result = step();
    const int since = frame - crossed;
    // RK 고장 한계 전까지는 큰 조향각에서도 요청을 유지한다
    ASSERT_TRUE(result.active);
    ASSERT_FALSE(result.cut_steer_temp);
    if (since == 1)
      ASSERT_NE(result.desired_torque, 0) << "고장 각도를 넘은 첫 프레임은 보조를 유지한다";
    if (since >= ramp)
      ASSERT_EQ(result.desired_torque, 0)
          << "각도 램프는 실측 고장 시각보다 충분히 먼저 0이 된다";
    if (frame > crossed + 80)
      ASSERT_EQ(result.apply_torque, 0) << "첫 차단 전에 토크가 0까지 내려와 있다";
  }
  for (int i = 0; i < config.steering_params.avoid_lkas_fault_cut_frames; ++i) {
    result = step();
    // 큰 조향각 고장 회피는 disengage 없이 요청만 끊는다
    ASSERT_TRUE(result.active);
    ASSERT_TRUE(result.cut_steer_temp);
    ASSERT_FALSE(result.frames.empty());
    const HyundaiLkas11Values lkas = decode_lkas11(result.frames.front().data);
    // 고장 회피 중 LKAS11 요청 비트와 일시 고장 비트
    ASSERT_FALSE(lkas.steer_req);
    ASSERT_TRUE(lkas.toi_fault);
    // 차단은 토크 0에서 일어난다
    ASSERT_EQ(lkas.steer_torque, 0);
    ASSERT_EQ(result.apply_torque, 0);
  }
  result = step();
  // 설정한 차단 길이 뒤 조향 요청을 다시 낸다
  ASSERT_TRUE(result.active);
  ASSERT_FALSE(result.cut_steer_temp);
  ASSERT_FALSE(result.frames.empty());
  const HyundaiLkas11Values lkas = decode_lkas11(result.frames.front().data);
  // 재개한 LKAS11 요청은 고장 각도 위에서 토크 0을 유지한다
  ASSERT_TRUE(lkas.steer_req);
  ASSERT_FALSE(lkas.toi_fault);
  ASSERT_EQ(result.apply_torque, 0);

  vehicle.steering_angle_deg = 20.0f;
  for (int i = 0; i < 30; ++i) result = step();
  // 고장 각도 아래로 오면 평소 램프로 토크를 되살린다
  ASSERT_TRUE(result.active);
  ASSERT_NE(result.apply_torque, 0);
}

// 정지 부근 path 깜빡임: active 재진입은 0.5s 연속 유효 후에만.
TEST(ControlReplay, PathFlickerDebounce) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  LateralPath bad = replay_path();
  bad.usable_for_steering = false;
  double t = 1.0;
  auto step = [&](const LateralPath &path) {
    VehicleCanState vehicle = ready_vehicle(t);
    const auto r = controller.update(path, replay_target(), vehicle, t, 0);
    t += 0.01;
    return r;
  };
  ASSERT_TRUE(step(replay_path()).active) << "처음 유효한 경로로 활성이 된다";
  ASSERT_FALSE(step(bad).active) << "경로가 무효가 되면 바로 비활성이 된다";
  int reactivated = 0;
  for (int i = 0; i < 20; ++i) reactivated += step(replay_path()).active ? 1 : 0;
  ASSERT_EQ(reactivated, 0) << "경로가 깜박여도 홀드 전에는 다시 활성이 되지 않는다";
  ASSERT_FALSE(step(bad).active) << "다음 끊김에서도 비활성";
  int active_after = 0;
  for (int i = 0; i < 60; ++i) active_after = step(replay_path()).active ? 1 : 0;
  ASSERT_EQ(active_after, 1) << "경로가 계속 유효하면 홀드 뒤 다시 활성이 된다";
}

// 정차(path 무효)에서도 engage는 받아야 한다 — 조향만 쉰다.
TEST(ControlReplay, EngageAllowedWithUnavailablePath) {
  LateralControllerConfig config;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  LateralPath bad = replay_path();
  bad.usable_for_steering = false;
  VehicleCanState vehicle = ready_vehicle(1.0);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
  vehicle.clu_button = 2;
  controller.update(bad, replay_target(), vehicle, 1.0, 0, true, true);
  vehicle.clu_button = 0;
  const auto engaged =
      controller.update(bad, replay_target(), vehicle, 1.01, 1, true, true);
  // 정차 중에는 경로가 없어도 engage를 받는다
  ASSERT_TRUE(engaged.engaged);
  ASSERT_FALSE(engaged.engage_rejected);
  // 정지 + path 무효는 오류가 아니라 대기 상태로 보고한다
  {
    // 경로 없는 정차는 오류가 아니라 정차로 알린다
    ASSERT_FALSE(engaged.active);
    ASSERT_EQ(engaged.active_block, BlockReason::Stopped);
  }
  // 대기 중에도 steer_req/스푸프는 유지(토크 0) — 정차 천이 부저 방지
  {
    // 사용 가능해질 때까지 steer_req를 토크 0으로 유지한다
    ASSERT_FALSE(engaged.frames.empty());
    ASSERT_TRUE(decode_lkas11(engaged.frames.front().data).steer_req);
    ASSERT_EQ(decode_lkas11(engaged.frames.front().data).steer_torque, 0);
  }
  // 주행 중 path 무효는 진짜 문제로 보고한다
  vehicle = ready_vehicle(1.02);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
  const auto rolling =
      controller.update(bad, replay_target(), vehicle, 1.02, 2, true, true);
  // 주행 중 쓸 수 없는 경로는 path_invalid로 알린다
  ASSERT_FALSE(rolling.active);
  ASSERT_EQ(rolling.active_block, BlockReason::PathInvalid);
  // 결함은 가용성 대기보다 우선한다 (정차 중 문 열림 -> hard disengage)
  vehicle = ready_vehicle(1.03);
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 0.0f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 0.0f;
  vehicle.door_open = true;
  const auto door =
      controller.update(bad, replay_target(), vehicle, 1.03, 3, true, true);
  // 고장은 사용 가능 여부보다 우선하고 정차 중에도 바로 disengage한다
  ASSERT_EQ(door.active_block, BlockReason::DoorOpen);
  ASSERT_FALSE(door.engaged);
}

TEST(ControlReplay, FixedMaxCurvature) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 3.6f;
  vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 3.6f;
  vehicle.cluster_speed_raw = 3.6f;

  LateralTarget target = replay_target();
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = 0.5f;
    target.psis[i] = 0.23f;
  }
  /* 틱당 변화율 제한이 걸리므로 한 번에 상한까지 뛰지 않는다. 1 m/s에서
   * 창은 5/(1*1)*0.01 = 0.05 1/m 다. */
  const auto first = controller.update(replay_path(), target, vehicle, 1.0, 0);
  // 첫 틱은 한계로 뛰지 않고 횡저크 한 스텝만 움직인다
  ASSERT_TRUE(first.active);
  ASSERT_NEAR(first.desired_curvature, 0.05f, 1e-6f);
  LateralControlResult result = first;
  for (int tick = 1; tick < 40; ++tick)
    result = controller.update(replay_path(), target, vehicle,
                               1.0 + 0.01 * tick, tick);
  ASSERT_NEAR(result.desired_curvature, 0.2f, 1e-6f)
      << "최대 곡률은 openpilot의 0.2 1/m 고정";
}

/* v0.11식 지연 보정: 요청 스텝 직후 delay 동안은 P가 과거 요청(0)과 현재
 * 측정(0)을 비교해 오차가 없어야 하고, 토크는 FF만으로 나와야 한다. */
TEST(ControlReplay, DelayCompensatedError) {
  TorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  params.torque_use_angle = true;
  // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
  params.angle_offset_deg = 0.0f;
  const float v = 20.0f;

  // 요청 0으로 버퍼를 채운다
  for (int i = 0; i < 120; ++i)
    torque.update(true, v, 0.0f, 0.0f, false, false, params);
  ASSERT_LT(std::fabs(torque.error()), 1e-6f) << "0 요청이 이어지면 오차가 없다";

  // 곡률 스텝. 조향각은 아직 0(차가 반응 전).
  torque.update(true, v, 0.01f, 0.0f, false, false, params);
  ASSERT_LT(std::fabs(torque.error()), 1e-4f)
      << "스텝 직후 오차는 거의 0이다(지연 보상)";
  ASSERT_GT(torque.feedforward(), 1.0f) << "스텝은 feedforward가 바로 싣는다";

  // delay(31프레임)를 넘겨도 차가 반응하지 않으면 그때 오차가 나타난다
  for (int i = 0; i < 40; ++i)
    torque.update(true, v, 0.01f, 0.0f, false, false, params);
  ASSERT_GT(torque.error(), 1.0f) << "지연이 지나도 못 따라간 만큼은 오차로 나타난다";
}

// inactive 동안에도 요청 버퍼가 갱신되어야 재engage 때 낡은 요청과 비교되지 않는다.
TEST(ControlReplay, ReengageHasNoStaleBufferSpike) {
  TorqueController torque;
  SteeringParams params;
  params.enabled = true;
  params.steer_actuator_delay = 0.30f;
  params.torque_use_angle = true;
  // 검증 대상은 요청 버퍼/지연 보상이다. 차량별 센서 트림은 배제한다.
  params.angle_offset_deg = 0.0f;
  const float v = 20.0f;

  // 커브 요청으로 버퍼를 채운 뒤 disengage
  for (int i = 0; i < 120; ++i)
    torque.update(true, v, 0.01f, 0.0f, false, false, params);
  // inactive 동안 요청은 0으로 돌아간다 (직선 수동 주행)
  for (int i = 0; i < 120; ++i)
    torque.update(false, v, 0.0f, 0.0f, false, false, params);
  // 직선에서 re-engage: 버퍼가 신선하면 오차 ~0, 얼었다면 큰 스파이크
  torque.update(true, v, 0.0f, 0.0f, false, false, params);
  ASSERT_LT(std::fabs(torque.error()), 1e-4f)
      << "다시 engage할 때 disengage 전의 낡은 요청과 비교하지 않는다";
}

/* 속도별 비례 이득(openpilot KP_INTERP). 오차는 횡가속도만으로 내고 저속 보강은
 * 이 곡선이 맡는다. 곡률 오차 항(LOW_SPEED_Y)은 더 이상 없다. */
TEST(ControlReplay, KpSpeedSchedule) {
  SteeringParams base;
  base.enabled = true;
  base.torque_use_angle = true;
  base.angle_offset_deg = 0.0f;
  base.torque_friction = 0.0f;   // P항만 남긴다
  base.torque_ki = 0.0f;
  /* 조향각 3도를 실제 곡률로 두고 요청 곡률 0을 준다. 오차 = -actual_lat_accel이라
   * 출력은 kp(v) x kf x v^2 x |actual_curvature|에 비례한다. */
  const auto run = [&](float v, float kp) {
    SteeringParams p = base;
    p.torque_kp = kp;
    TorqueController torque;
    for (int i = 0; i < 120; ++i) torque.update(true, v, 0.0f, 3.0f, false, false, p);
    return torque.normalized_output();
  };
  // 이득 곡선의 노드에서 출력비가 KP_INTERP 비율 x v^2 비율과 맞아야 한다.
  const auto gain_at = [&](float v) {
    const float out = run(v, 0.8f);
    TorqueController probe;
    SteeringParams p = base;
    for (int i = 0; i < 120; ++i) probe.update(true, v, 0.0f, 3.0f, false, false, p);
    return std::fabs(out / (probe.error() == 0.0f ? 1.0f : probe.error()));
  };
  // 5 m/s 노드는 11.5, 10 m/s 노드는 3.5 -> 이득비 3.2857
  const float g5 = gain_at(5.0f), g10 = gain_at(10.0f);
  // 두 속도 노드 모두에서 이득이 나온다
  ASSERT_GT(g5, 0.0f);
  ASSERT_GT(g10, 0.0f);
  const float ratio = g5 / g10;
  // 5 m/s 노드는 10 m/s 노드의 11.5/3.5배
  ASSERT_GT(ratio, 3.2f);
  ASSERT_LT(ratio, 3.4f);
  // 30 m/s 위는 torque_kp가 그대로 끝점이다.
  const float top8 = std::fabs(run(35.0f, 0.8f));
  const float top16 = std::fabs(run(35.0f, 1.6f));
  // 커브 정점 경우는 포화하지 않는다
  ASSERT_GT(top8, 1e-4f);
  ASSERT_LT(top8, 0.95f);
  ASSERT_NEAR(top16 / top8, 2.0f, 0.05f) << "30 m/s 위에서는 이득이 torque_kp에 비례한다";
  /* LOW_SPEED_Y가 남아 있으면 곡률 항이 저속에서 오차를 수십 배로 키운다.
   * 오차가 순수 횡가속도인지 확인한다. */
  TorqueController t;
  SteeringParams p = base;
  for (int i = 0; i < 120; ++i) t.update(true, 4.0f, 0.002f, 3.0f, false, false, p);
  const float curvature_error = 0.002f - t.actual_curvature();
  ASSERT_NEAR(t.error(), curvature_error * 16.0f, 1e-4f)
      << "오차는 횡가속도뿐이고 저속 곡률 항이 없다";
}

// 라이브 뱅크: 편경사에 해당하는 만큼 FF가 이동해야 한다.
TEST(ControlReplay, LiveBankCompensation) {
  TorqueController with_bank, without_bank;
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  params.live_bank_compensation = true;
  SteeringParams off = params;
  off.live_bank_compensation = false;
  for (int i = 0; i < 120; ++i) {
    with_bank.update(true, 20.0f, 0.002f, 1.0f, false, false, params,
                     0.0f, false, -0.117f);
    without_bank.update(true, 20.0f, 0.002f, 1.0f, false, false, off,
                        0.0f, false, -0.117f);
  }
  // bank -0.117(우측 기움) -> 중력이 우로 끄니 FF는 좌로 0.117 이동해야 한다
  const float diff = with_bank.feedforward() - without_bank.feedforward();
  ASSERT_LT(std::fabs(diff + 0.117f), 1e-3f)
      << "실시간 뱅크는 feedforward를 +bank만큼 옮긴다(중력이 반대로 작용)";
}

// 라이브 뱅크: 커브(|yaw*v| >= 0.4)에서는 갱신을 멈추고 직선 값을 유지해야 한다.
TEST(ControlReplay, BankHoldsDuringCurves) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController controller(config);
  double t = 1.0;
  auto step = [&](float yaw_rate_rad_s, float lat_accel_mps2) {
    VehicleCanState vehicle = ready_vehicle(t);
    vehicle.wheel_speed_fl_kph = vehicle.wheel_speed_fr_kph = 60.0f;
    vehicle.wheel_speed_rl_kph = vehicle.wheel_speed_rr_kph = 60.0f;
    vehicle.cluster_speed_raw = 63.0f;
    vehicle.yaw_rate_valid = true;
    vehicle.yaw_rate_rad_s = yaw_rate_rad_s;
    vehicle.lat_accel_valid = true;
    vehicle.lat_accel_mps2 = lat_accel_mps2;
    controller.update(replay_path(), replay_target(), vehicle, t, 0);
    t += 0.01;
  };
  for (int i = 0; i < 1500; ++i) step(0.0f, -0.117f);  // 직선 크라운
  const float straight_bank = controller.road_bank_lat_accel();
  ASSERT_LT(std::fabs(straight_bank + 0.117f), 5e-3f) << "직선에서 뱅크는 노면 경사(크라운)로 수렴한다";
  // 커브: yaw*v = +1.2, 롤 누설 +0.5 (기구학 성분 상쇄 후 잔여)
  const float v = 60.0f / 3.6f;
  for (int i = 0; i < 500; ++i) step(1.2f / v, -1.2f + 0.5f);
  ASSERT_NEAR(controller.road_bank_lat_accel(), straight_bank, 1e-4f)
      << "커브 중에는 뱅크를 유지하고 롤 누설을 따라가지 않는다";
}

// latAccelOffset: 상수 편향이 FF에서 그대로 빠져야 한다.
TEST(ControlReplay, LatAccelOffsetShiftsFeedforward) {
  TorqueController a, b;
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  SteeringParams offset_params = params;
  offset_params.torque_lat_accel_offset = 0.25f;
  for (int i = 0; i < 120; ++i) {
    a.update(true, 20.0f, 0.002f, 1.0f, false, false, params);
    b.update(true, 20.0f, 0.002f, 1.0f, false, false, offset_params);
  }
  const float diff = a.feedforward() - b.feedforward();
  ASSERT_NEAR(diff, 0.25f, 1e-3f) << "lat_accel_offset은 feedforward에서 정확히 빠진다";
}

TEST(ControlReplay, RuntimeParamsApplyImmediately) {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 72.0f;

  const auto active =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0);
  ASSERT_TRUE(active.active) << "런타임 파라미터 검사는 활성 상태에서 시작한다";

  SteeringParams steering = config.steering_params;
  steering.enabled = false;
  controller.update_params(steering, config.driving_params);
  const auto disabled =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1);
  // 런타임 조향 파라미터는 다음 제어 틱에 적용된다
  ASSERT_FALSE(disabled.active);
  ASSERT_EQ(disabled.active_block, BlockReason::ControllerDisabled);

  steering.enabled = true;
  controller.update_params(steering, config.driving_params);
  const auto resumed =
      controller.update(replay_path(), replay_target(), vehicle, 1.02, 2);
  ASSERT_TRUE(resumed.active) << "런타임 파라미터를 바꿔도 컨트롤러 동작이 이어진다";
}

TEST(ControlReplay, LkasHudStateStability) {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

  LateralPath no_lane_path = replay_path();
  no_lane_path.left_valid = false;
  no_lane_path.right_valid = false;
  const auto active =
      controller.update(no_lane_path, replay_target(), vehicle, 1.0, 0);
  // 활성 상태의 HUD 프레임
  ASSERT_TRUE(active.active);
  ASSERT_FALSE(active.frames.empty());
  ASSERT_EQ(decode_lkas11(active.frames.front().data).ldws_sys_state, 3)
      << "차선 확률이 흔들려도 HUD는 활성으로 남는다";

  /* 클러스터는 sys_state 천이마다 부저를 울리므로, sys_state는 active가
   * 아니라 engaged만 따른다. enable/disable에서만 천이가 생긴다. */
  LateralTarget invalid_target = replay_target();
  invalid_target.mpc_solution_valid = false;
  const auto inactive =
      controller.update(no_lane_path, invalid_target, vehicle, 2.5, 2);
  // 비활성 상태의 HUD 프레임
  ASSERT_FALSE(inactive.active);
  ASSERT_FALSE(inactive.frames.empty());
  ASSERT_EQ(decode_lkas11(inactive.frames.front().data).ldws_sys_state, 3)
      << "engage 중 비활성이어도 부저가 울리지 않게 sys_state를 유지한다";
}

TEST(ControlReplay, PandaGateAndHandoff) {
  LateralControllerConfig config;
  LateralController controller(config);
  VehicleCanState vehicle = ready_vehicle();
  vehicle.cluster_speed_raw = 0.0f;  // 클러스터 속도 없음

  vehicle.clu_button = 2;
  const auto set_press =
      controller.update(replay_path(), replay_target(), vehicle, 1.0, 0, true, true);
  ASSERT_FALSE(set_press.engaged) << "SET은 떼기 전에 engage하지 않는다";

  vehicle.clu_button = 0;
  const auto panda_blocked =
      controller.update(replay_path(), replay_target(), vehicle, 1.01, 1, true, false);
  // Panda controls 게이트
  ASSERT_TRUE(panda_blocked.engaged);
  ASSERT_FALSE(panda_blocked.active);
  ASSERT_EQ(panda_blocked.active_block, BlockReason::PandaControlsOff);
  ASSERT_FALSE(panda_blocked.engage_rejected)
      << "Panda controls 핸드셰이크 대기 중에도 유효한 SET을 거부하지 않는다";
  // Panda 불일치 중에도 토크 0 대체 스트림을 계속 보낸다
  ASSERT_TRUE(panda_blocked.should_send);
  ASSERT_FALSE(panda_blocked.frames.empty());
  const HyundaiLkas11Values zero_lkas =
      decode_lkas11(panda_blocked.frames.front().data);
  // Panda controls가 꺼져 있으면 LKAS 토크 0
  ASSERT_EQ(zero_lkas.steer_torque, 0);
  ASSERT_FALSE(zero_lkas.steer_req);

  LateralController panda_timeout_controller(config);
  /* 이 블록은 t=10 에서 돈다. CAN 타임스탬프를 함께 옮기지 않으면 차량 상태가
   * 9초 낡아 vehicle_state_stale 이 되고, 검증하려던 Panda 유예가 아니라
   * freshness 게이트를 보게 된다. */
  VehicleCanState timeout_vehicle = ready_vehicle(10.0);
  timeout_vehicle.clu_button = 2;
  panda_timeout_controller.update(replay_path(), replay_target(), timeout_vehicle,
                                  10.0, 0, true, true);
  timeout_vehicle.clu_button = 0;
  const auto panda_waiting = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle, 10.01, 1, true, false);
  // Panda 핸드셰이크 유예 동안 유효한 요청을 붙잡아 둔다
  ASSERT_TRUE(panda_waiting.engaged);
  ASSERT_FALSE(panda_waiting.engage_rejected);
  // 유예(1초)를 넘기며 CAN 은 계속 신선하게 유지한다.
  VehicleCanState timeout_vehicle_later = ready_vehicle(11.0);
  const auto panda_timeout = panda_timeout_controller.update(
      replay_path(), replay_target(), timeout_vehicle_later, 11.02, 102, true, false);
  // Panda 불일치가 이어지면 결국 engage를 거부한다
  ASSERT_FALSE(panda_timeout.engaged);
  ASSERT_TRUE(panda_timeout.engage_rejected);
  ASSERT_EQ(panda_timeout.active_block, BlockReason::PandaControlsOff);

  LateralController deferred_static_controller(config);
  // t=12 시점 검사이므로 차량 데이터도 신선해야 한다(낡으면 stale이 우선).
  VehicleCanState deferred_vehicle = ready_vehicle(12.0);
  deferred_vehicle.wheel_speed_fl_kph = deferred_vehicle.wheel_speed_fr_kph = 60.0f;
  deferred_vehicle.wheel_speed_rl_kph = deferred_vehicle.wheel_speed_rr_kph = 60.0f;
  deferred_vehicle.clu_button = 2;
  deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle,
                                    12.0, 0, true, true);
  deferred_vehicle.clu_button = 0;
  deferred_static_controller.update(replay_path(), replay_target(), deferred_vehicle,
                                    12.01, 1, true, false);
  LateralTarget deferred_invalid_target = replay_target();
  deferred_invalid_target.mpc_solution_valid = false;
  const auto deferred_static = deferred_static_controller.update(
      replay_path(), deferred_invalid_target, deferred_vehicle, 12.02, 2, true, true);
  // Panda가 회복되면 정적 engage 게이트를 다시 본다
  ASSERT_FALSE(deferred_static.engaged);
  ASSERT_TRUE(deferred_static.engage_rejected);
  ASSERT_EQ(deferred_static.active_block, BlockReason::LateralPlanInvalid);

  const auto active =
      controller.update(replay_path(), replay_target(), vehicle, 1.02, 2, true, true);
  // Panda controls가 켜지면 활성이 된다
  ASSERT_TRUE(active.engaged);
  ASSERT_TRUE(active.active);

  vehicle.clu_button = 4;
  const auto cancel =
      controller.update(replay_path(), replay_target(), vehicle, 1.03, 3, true, false);
  // CANCEL을 누르면 토크 0 프레임 인계를 시작한다
  ASSERT_FALSE(cancel.engaged);
  ASSERT_TRUE(cancel.should_send);
  const auto release_tail =
      controller.update(replay_path(), replay_target(), vehicle, 4.02, 302, true, false);
  ASSERT_TRUE(release_tail.should_send) << "인계는 3000 ms 이어진다";
  const auto stock_handoff =
      controller.update(replay_path(), replay_target(), vehicle, 4.04, 304, true, false);
  ASSERT_FALSE(stock_handoff.should_send) << "인계는 3000 ms 뒤 멈춘다";

  vehicle.lkas11_seed[4] = 9U << 4;
  stamp_can_times(&vehicle, 4.05);
  vehicle.clu_button = 2;
  controller.update(replay_path(), replay_target(), vehicle, 4.05, 305, true, true);
  vehicle.clu_button = 0;
  const auto reengaged =
      controller.update(replay_path(), replay_target(), vehicle, 4.06, 306, true, true);
  // 인계 뒤 다시 engage
  ASSERT_TRUE(reengaged.active);
  ASSERT_FALSE(reengaged.frames.empty());
  ASSERT_EQ(decode_lkas11(reengaged.frames.front().data).msg_count, 10)
      << "다시 engage하면 순정 카메라의 LKAS 카운터에서 이어간다";

  LateralController rejected_controller(config);
  VehicleCanState rejected_vehicle = ready_vehicle();
  rejected_vehicle.clu_button = 2;
  rejected_controller.update(replay_path(), replay_target(), rejected_vehicle,
                              1.0, 0, true, true);
  rejected_vehicle.clu_button = 0;
  LateralTarget invalid_target = replay_target();
  invalid_target.mpc_solution_valid = false;
  const auto rejected = rejected_controller.update(
      replay_path(), invalid_target, rejected_vehicle, 1.01, 1, true, true);
  // 정적 engage 게이트는 engaged 상태를 남기지 않고 거부한다
  ASSERT_FALSE(rejected.engaged);
  ASSERT_TRUE(rejected.engage_rejected);
  ASSERT_EQ(rejected.active_block, BlockReason::LateralPlanInvalid);
}

/* 시동 직후 첫 engage: Panda health 가 아직 없고 안전벨트/기어가 막고 있을 때.
 * 실차(2026-09-12)에서 engage 톤이 울린 뒤 해제되고, 두 번째 시도부터만
 * 거절음이 났다. 하드 결함이 panda_not_ready 뒤로 밀려 가려졌기 때문이다. */
TEST(ControlReplay, ColdStartEngageReportsHardBlock) {
  const auto cold_start_attempt = [](bool panda_ready, bool seatbelt_unlatched,
                                     int gear) {
    LateralControllerConfig config;
    LateralController controller(config);
    VehicleCanState vehicle = ready_vehicle();
    vehicle.seatbelt_unlatched = seatbelt_unlatched;
    vehicle.gear = gear;
    vehicle.clu_button = 2;
    controller.update(replay_path(), replay_target(), vehicle, 1.0, 0,
                      panda_ready, panda_ready);
    vehicle.clu_button = 0;
    return controller.update(replay_path(), replay_target(), vehicle, 1.01, 1,
                             panda_ready, panda_ready);
  };

  const auto belt = cold_start_attempt(false, true, 5);
  // 시동 직후 안전벨트 미착용 SET은 panda를 기다리지 않고 안전벨트를 알린다
  ASSERT_FALSE(belt.engaged);
  ASSERT_TRUE(belt.engage_rejected);
  ASSERT_EQ(belt.active_block, BlockReason::SeatbeltUnlatched);

  const auto gear = cold_start_attempt(false, false, 0);
  // 시동 직후 D가 아닌 SET은 panda를 기다리지 않고 기어를 알린다
  ASSERT_FALSE(gear.engaged);
  ASSERT_TRUE(gear.engage_rejected);
  ASSERT_EQ(gear.active_block, BlockReason::GearNotDrive);

  // 차량이 정상이면 Panda 핸드셰이크 유예는 그대로 살아 있어야 한다.
  const auto handshake = cold_start_attempt(false, false, 5);
  // 시동 직후 차가 정상이면 SET은 panda 핸드셰이크를 기다린다
  ASSERT_TRUE(handshake.engaged);
  ASSERT_FALSE(handshake.engage_rejected);
  ASSERT_EQ(handshake.active_block, BlockReason::PandaNotReady);
}

/* t=1.0 s에 나온, 60 m 이상 뻗은 조향 가능 plan. */
K230ModelState usable_model_state() {
  K230ModelState state;
  state.valid = 1;
  state.model_timestamp_ns = 1000000000ULL;
  state.plan_probability = 0.9f;
  state.lane_probabilities[1] = 0.8f;
  state.lane_probabilities[2] = 0.7f;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float x = 2.0f * static_cast<float>(i + 1);
    state.plan[i].x = x;
    state.plan[i].y = -0.0004f * x * x;
  }
  return state;
}

// 점 수는 충분하지만 몇 미터로 주저앉은 plan.
void collapse_plan(K230ModelState *state) {
  for (int i = 0; i < kTrajectorySize; ++i)
    state->plan[i].x = 1.0f + 0.1f * static_cast<float>(i);
}

TEST(ControlReplay, ModelPathAdapter) {
  const K230ModelState state = usable_model_state();
  const LateralPath path =
      path_from_model_state(state, 1100000000ULL, 250000000ULL);
  // 모델 경로 변환의 유효 판정
  ASSERT_TRUE(path.usable_for_steering);
  ASSERT_TRUE(path.left_valid);
  ASSERT_TRUE(path.right_valid);
  // 모델 경로 변환은 앞쪽 plan 점을 모두 센다
  ASSERT_EQ(path.point_count, kTrajectorySize);
  ASSERT_GE(path.reach_m, 60.0f);

  /* 정차에서 plan이 몇 미터로 주저앉으면 점 수는 충분해도 조향에 못 쓴다. */
  K230ModelState short_state = state;
  collapse_plan(&short_state);
  const LateralPath short_path =
      path_from_model_state(short_state, 1100000000ULL, 250000000ULL);
  // plan 도달 거리가 짧으면 조향 게이트에서 막힌다
  ASSERT_FALSE(short_path.usable_for_steering);
  ASSERT_EQ(short_path.invalid_reason, "path_invalid");
}

/* 문서화된 안전 홀드 1: Panda 헬스 스냅샷 공백은 100 ms까지만, 신선한
 * controls_allowed=0은 절대 유지하지 않는다. */
TEST(ControlReplay, PandaHealthHold) {
  const uint64_t t0 = 1000000000ULL;
  K230PandaState ready;
  ready.timestamp_ns = t0;
  ready.connected = ready.comms_healthy = ready.tx_enabled = 1;
  ready.controls_allowed = 1;
  ready.safety_mode = kExpectedPandaSafetyModel;
  ready.safety_param = kExpectedPandaSafetyParam;
  K230PandaState unhealthy = ready;
  unhealthy.comms_healthy = 0;

  PandaHealthGate gate;
  PandaGateOutput out = gate.update(ready, t0, false);
  // 준비된 panda는 홀드 없이 게이트를 통과한다
  ASSERT_TRUE(out.state_fresh);
  ASSERT_TRUE(out.ready_raw);
  ASSERT_TRUE(out.ready);
  ASSERT_TRUE(out.controls_allowed);
  ASSERT_FALSE(out.hold_applied);
  out = gate.update(unhealthy, t0 + 50000000ULL, false);
  // 50 ms 상태 공백은 마지막 판정을 유지한다
  ASSERT_FALSE(out.ready_raw);
  ASSERT_TRUE(out.hold_applied);
  ASSERT_TRUE(out.ready);
  ASSERT_TRUE(out.controls_allowed);
  out = gate.update(unhealthy, t0 + 100000000ULL, false);
  ASSERT_TRUE(out.hold_applied) << "홀드는 정확히 100 ms까지 덮는다";
  out = gate.update(unhealthy, t0 + 100000001ULL, false);
  // 홀드는 100 ms 뒤 끝난다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_FALSE(out.ready);
  ASSERT_FALSE(out.controls_allowed);
  out = gate.update(ready, t0 + 1200000000ULL, false);
  // 1.2 s 지난 스냅샷은 필드가 준비돼 보여도 낡은 것이다
  ASSERT_FALSE(out.state_fresh);
  ASSERT_FALSE(out.ready_raw);
  ASSERT_FALSE(out.ready);

  PandaHealthGate explicit_off;
  explicit_off.update(ready, t0, false);
  K230PandaState off = ready;
  off.controls_allowed = 0;
  off.timestamp_ns = t0 + 10000000ULL;
  out = explicit_off.update(off, t0 + 10000000ULL, false);
  // 신선하고 전송이 준비된 controls_allowed=0은 홀드하지 않는다
  ASSERT_TRUE(out.ready_raw);
  ASSERT_TRUE(out.controls_off_explicit);
  ASSERT_FALSE(out.hold_applied);
  ASSERT_TRUE(out.ready);
  ASSERT_FALSE(out.controls_allowed);
  K230PandaState gap = off;
  gap.comms_healthy = 0;
  out = explicit_off.update(gap, t0 + 60000000ULL, false);
  // 명시적 off 뒤 상태 공백은 controls를 끈 채로 둔다
  ASSERT_TRUE(out.hold_applied);
  ASSERT_TRUE(out.ready);
  ASSERT_FALSE(out.controls_allowed);

  PandaHealthGate cold;
  out = cold.update(unhealthy, t0, false);
  // 첫 준비 스냅샷 전에는 홀드가 없다
  ASSERT_FALSE(out.ready);
  ASSERT_FALSE(out.hold_applied);
  out = cold.update(unhealthy, t0, true);
  // force_engaged는 panda 게이트를 건너뛴다
  ASSERT_TRUE(out.ready);
  ASSERT_TRUE(out.controls_allowed);
  ASSERT_FALSE(out.ready_raw);
}

/* 문서화된 안전 홀드 2: 잘못된 plan 프레임은 150 ms까지 마지막 유효 경로로
 * 덮고, 모델 freshness 타임아웃은 그대로 하드 게이트다. */
TEST(ControlReplay, PathInvalidHold) {
  const uint64_t timeout_ns = 250000000ULL;
  const K230ModelState good = usable_model_state();

  PathHoldGate gate;
  PathHoldOutput out = gate.update(good, 1100000000ULL, timeout_ns);
  // 쓸 수 있는 plan은 홀드를 그대로 지난다
  ASSERT_TRUE(out.path.usable_for_steering);
  ASSERT_FALSE(out.hold_applied);
  K230ModelState collapsed = good;
  collapsed.model_timestamp_ns = 1050000000ULL;
  collapse_plan(&collapsed);
  out = gate.update(collapsed, 1100000000ULL, timeout_ns);
  // 무너진 프레임 하나는 마지막 쓸 수 있는 경로로 덮는다
  ASSERT_EQ(out.raw.invalid_reason, "path_invalid");
  ASSERT_TRUE(out.hold_applied);
  ASSERT_TRUE(out.path.usable_for_steering);
  ASSERT_TRUE(out.path.invalid_reason.empty());
  collapsed.model_timestamp_ns = 1120000000ULL;
  out = gate.update(collapsed, 1150000000ULL, timeout_ns);
  ASSERT_TRUE(out.hold_applied) << "홀드는 정확히 150 ms까지 덮는다";
  out = gate.update(collapsed, 1150000001ULL, timeout_ns);
  // 홀드는 마지막 쓸 수 있는 프레임 150 ms 뒤 끝난다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_FALSE(out.path.usable_for_steering);
  ASSERT_EQ(out.path.invalid_reason, "path_invalid");

  PathHoldGate stale_gate;
  stale_gate.update(good, 1100000000ULL, timeout_ns);
  K230ModelState stale = collapsed;
  stale.model_timestamp_ns = 1000000000ULL;
  out = stale_gate.update(stale, 1400000000ULL, timeout_ns);
  // 낡은 모델은 바로 막고 홀드하지 않는다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_EQ(out.path.invalid_reason, "model_stale");

  PathHoldGate invalid_gate;
  invalid_gate.update(good, 1100000000ULL, timeout_ns);
  K230ModelState invalid = good;
  invalid.valid = 0;
  invalid.model_timestamp_ns = 1120000000ULL;
  out = invalid_gate.update(invalid, 1130000000ULL, timeout_ns);
  // 무효 모델은 홀드하지 않는다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_EQ(out.path.invalid_reason, "model_invalid");
}

// ---------------------------------------------------------------- paramsd·torqued 소비

LiveLateralParams odd_live_params() {
  LiveLateralParams live;
  live.use_vehicle = true;
  live.steer_ratio = 13.1f;
  live.stiffness_factor = 0.7f;
  live.angle_offset_deg = 2.5f;
  live.roll_rad = 0.04f;
  live.use_torque = true;
  live.lat_accel_factor = 3.0f;
  live.lat_accel_offset = 0.2f;
  live.friction = 0.05f;
  return live;
}

/* 스위치를 끄면 학습값을 넣어도 모든 틱이 비트 단위로 같아야 한다. 무효·캘리브 완료도
 * 끈 쪽에서는 차단하지 않는다. */
TEST(ControlReplay, LiveParamsSwitchOffIsIdentical) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  LateralController plain(config), fed(config);
  double t = 1.0;
  for (int tick = 0; tick < 600; ++tick, t += 0.01) {
    VehicleCanState vehicle = ready_vehicle(t);
    vehicle.steering_angle_deg = 12.0f * std::sin(0.02f * tick);
    vehicle.yaw_rate_valid = true;
    vehicle.yaw_rate_rad_s = 0.05f * std::sin(0.02f * tick);
    vehicle.lat_accel_valid = true;
    vehicle.lat_accel_mps2 = -0.3f;
    vehicle.driver_torque = tick % 97 == 0 ? 200 : 10;
    LateralTarget target = replay_target();
    for (int i = 0; i < kLateralControlN; ++i) target.curvatures[i] = 0.002f * std::cos(0.01f * tick);
    fed.set_live_params(odd_live_params(), false, true);
    const LateralControlResult a = plain.update(replay_path(), target, vehicle, t, tick);
    const LateralControlResult b = fed.update(replay_path(), target, vehicle, t, tick);
    // 학습기를 끄면 막는 일이 없다
    ASSERT_EQ(a.active_block, b.active_block);
    ASSERT_EQ(a.active, b.active);
    // 학습기를 끄면 모든 틱이 비트까지 같다
    ASSERT_EQ(std::memcmp(&a.desired_curvature, &b.desired_curvature, sizeof(float)), 0);
    ASSERT_EQ(std::memcmp(&a.actual_curvature, &b.actual_curvature, sizeof(float)), 0);
    ASSERT_EQ(std::memcmp(&a.normalized_output, &b.normalized_output, sizeof(float)), 0);
    ASSERT_EQ(std::memcmp(&a.feedforward, &b.feedforward, sizeof(float)), 0);
    ASSERT_EQ(a.desired_torque, b.desired_torque);
    ASSERT_EQ(a.apply_torque, b.apply_torque);
  }
}

/* 켜면 실제 곡률은 opendbc VehicleModel(update_params(x, sr) → calc_curvature(sa, u, roll))을
 * 부호 반전한 값이다. double 독립 전사본과 비교한다. */
double upstream_measured_curvature(const SteeringParams &p, const LiveLateralParams &live,
                                   double angle_deg, double u) {
  const double civic_m = 1326.0 + 136.0, civic_l = 2.70, civic_af = civic_l * 0.4;
  const double civic_ar = civic_l - civic_af;
  const double m = p.mass_kg, l = p.wheelbase_m, af = p.center_to_front_m(), ar = l - af;
  const double tsf = p.tire_stiffness_factor, x = std::max<double>(live.stiffness_factor, 0.1);
  const double cf = 192150.0 * tsf * m / civic_m * (ar / l) / (civic_ar / civic_l) * x;
  const double cr = 202500.0 * tsf * m / civic_m * (af / l) / (civic_af / civic_l) * x;
  const double sf = m * (cf * af - cr * ar) / (l * l * cf * cr);
  const double factor = (1.0 - p.steer_ratio_rear) / (1.0 - sf * u * u) / l;
  const double sa = (angle_deg - live.angle_offset_deg) * 3.14159265358979323846 / 180.0;
  const double roll = std::fabs(sf) < 1e-6 ? 0.0 : 9.81 * live.roll_rad / ((1.0 / sf) - u * u);
  return -(factor * sa / std::max<double>(live.steer_ratio, 0.1) + roll);
}

TEST(ControlReplay, LiveVehicleParamsFollowVehicleModel) {
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  LiveLateralParams live = odd_live_params();
  live.use_torque = false;
  for (float u : {3.0f, 12.0f, 27.0f}) {
    for (float angle : {-30.0f, 0.0f, 4.0f}) {
      TorqueController torque;
      const float got = torque.estimate_actual_curvature(u, angle, params, 0.0f, false, live);
      const double want = upstream_measured_curvature(params, live, angle, u);
      ASSERT_NEAR(got, want, 2e-6 * std::fabs(want) + 1e-9)
          << "실시간 SR·강성·오프셋·롤이 opendbc calc_curvature를 따른다";
    }
  }
  // 롤은 FF에서 roll·g를 빼고 편경사 추정은 쓰지 않는다(마찰은 포화 구간이라 같다)
  TorqueController with_roll, without_roll;
  LiveLateralParams flat = live;
  flat.roll_rad = 0.0f;
  params.live_bank_compensation = true;
  for (int i = 0; i < 150; ++i) {
    with_roll.update(true, 20.0f, 0.004f, 1.0f, false, false, params, 0.0f, false, -0.5f, live);
    without_roll.update(true, 20.0f, 0.004f, 1.0f, false, false, params, 0.0f, false, -0.5f, flat);
  }
  ASSERT_NEAR(with_roll.feedforward() - without_roll.feedforward(), -live.roll_rad * 9.81f, 1e-4f)
      << "실시간 롤은 feedforward에서 roll*g를 빼고 뱅크 추정을 대신한다";
}

/* 상류는 PID를 횡가속 공간에서 돌리고 끝에서 latAccelFactor로 나눈다. 그러면 마찰이 없을 때
 * 출력 × latAccelFactor가 배율과 무관하다(사전값 경로 포함). 마찰은 토크 공간에 그대로,
 * 절편은 −offset/latAccelFactor로 더해진다. */
TEST(ControlReplay, LiveTorqueParamsMatchUpstreamStructure) {
  SteeringParams params;
  params.enabled = true;
  params.torque_use_angle = true;
  params.torque_friction = 0.0f;
  params.live_bank_compensation = false;
  const float prior = params.torque_lat_accel_factor;
  auto run = [&](bool use, float factor, float offset, float friction, std::vector<float> *out) {
    TorqueController torque;
    LiveLateralParams live;
    live.use_torque = use;
    live.lat_accel_factor = factor;
    live.lat_accel_offset = offset;
    live.friction = friction;
    for (int i = 0; i < 300; ++i) {
      const float desired = 0.0015f * std::sin(0.03f * i);
      const float angle = 1.5f * std::sin(0.03f * i - 0.4f);
      torque.update(true, 20.0f, desired, angle, false, false, params, 0.0f, false, 0.0f, live);
      out->push_back(torque.normalized_output());
    }
  };
  std::vector<float> base, low, high, offset, friction;
  run(false, 0.0f, 0.0f, 0.0f, &base);
  run(true, 3.0f, 0.0f, 0.0f, &low);
  run(true, 5.5f, 0.0f, 0.0f, &high);
  run(true, 3.0f, 0.1f, 0.0f, &offset);
  run(true, 3.0f, 0.0f, 0.05f, &friction);
  float worst_scale = 0.0f, worst_offset = 0.0f, worst_friction = 0.0f;
  for (size_t i = 0; i < base.size(); ++i) {
    // 출력이 포화하지 않는다
    ASSERT_LT(std::fabs(base[i]), 0.9f);
    ASSERT_LT(std::fabs(low[i]), 0.9f);
    const float ref = base[i] * prior;
    worst_scale = std::max({worst_scale, std::fabs(low[i] * 3.0f - ref), std::fabs(high[i] * 5.5f - ref)});
    const float sign = params.torque_output_sign >= 0 ? 1.0f : -1.0f;
    worst_offset = std::max(worst_offset, std::fabs((offset[i] - low[i]) * 3.0f + sign * 0.1f));
    // 마찰은 |오차| < 0.2에서 선형이라 차이가 토크 공간 0.05 이하, 부호는 오차를 따른다
    worst_friction = std::max(worst_friction, std::fabs(friction[i] - low[i]) - 0.05f);
  }
  ASSERT_LT(worst_scale, 2e-5f) << "출력×latAccelFactor는 배율과 무관하다";
  ASSERT_LT(worst_offset, 2e-5f) << "학습한 오프셋은 -offset/latAccelFactor로 들어간다";
  ASSERT_LT(worst_friction, 1e-6f) << "마찰은 토크 공간의 항이고 계수로 제한된다";
}

TEST(ControlReplay, ParamsdInvalidBlocksOnlyWhenUsed) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.driving_params.vehicle_state_timeout_ms = 2000;
  config.steering_params.use_live_vehicle_params = true;
  LateralController controller(config);
  const VehicleCanState vehicle = ready_vehicle(1.0);
  controller.set_live_params(odd_live_params(), true, true);
  ASSERT_TRUE(controller.update(replay_path(), replay_target(), vehicle, 1.0, 0).active)
      << "유효한 학습값은 제어를 활성으로 둔다";
  controller.set_live_params(odd_live_params(), false, false);
  ASSERT_TRUE(controller.update(replay_path(), replay_target(), vehicle, 1.01, 1).active)
      << "보정 전 무효 학습값은 막지 않는다(상류 cal_status 검사)";
  controller.set_live_params(odd_live_params(), false, true);
  ASSERT_EQ(controller.update(replay_path(), replay_target(), vehicle, 1.02, 2).active_block,
            BlockReason::ParamsdInvalid)
      << "보정 뒤 무효 학습값은 막는다";
  LiveLateralParams unseen = odd_live_params();
  unseen.use_vehicle = false;
  controller.set_live_params(unseen, false, true);
  ASSERT_NE(controller.update(replay_path(), replay_target(), vehicle, 1.03, 3).active_block,
            BlockReason::ParamsdInvalid)
      << "paramsd가 발행하기 전에는 막지 않는다(상류 sm.seen)";
}

TEST(ControlReplay, CurvatureLimitFollowsRoll) {
  LateralTarget target = replay_target();
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = 0.05f;
    target.psis[i] = 0.05f * 20.0f * model_t_idx(i);
  }
  const float v = 20.0f, roll = 0.03f;
  const float up = lag_adjusted_desired_curvature(target, v, 0.0f, 0.34f, 0.05f, roll);
  ASSERT_NEAR(up, (kMaxLateralAccel + roll * 9.81f) / (v * v), 1e-7f)
      << "롤만큼 횡가속도 상한이 roll*g 움직인다";
  for (int i = 0; i < kLateralControlN; ++i) {
    target.curvatures[i] = -0.05f;
    target.psis[i] = -target.psis[i];
  }
  const float down = lag_adjusted_desired_curvature(target, v, 0.0f, 0.34f, -0.05f, roll);
  ASSERT_NEAR(down, (-kMaxLateralAccel + roll * 9.81f) / (v * v), 1e-7f)
      << "하한도 같은 roll*g만큼 움직인다";
}

// 2026-09-24 실차: 663 ms 멈춤 뒤 NaN 속도가 좌측 최대 곡률을 심어 재활성 때 32° 조향했다.
TEST(ControlReplay, StaleSpeedKeepsCurvature) {
  LateralControllerConfig config;
  config.force_engaged = true;
  LateralController controller(config);
  const LateralTarget target = replay_target();
  float before = 0.0f;
  for (int i = 0; i < 20; ++i) {
    const double t = 1.0 + 0.01 * i;
    before = controller.update(replay_path(), target, ready_vehicle(t), t, i).desired_curvature;
  }
  const LateralControlResult stale =
      controller.update(replay_path(), target, ready_vehicle(1.19), 1.9, 20);
  ASSERT_EQ(stale.active_block, BlockReason::VehicleStateStale)
      << "휠 속도가 시한보다 오래됐다";
  ASSERT_EQ(stale.desired_curvature, before) << "속도가 낡으면 마지막 목표 곡률을 유지한다";
  const LateralControlResult back =
      controller.update(replay_path(), target, ready_vehicle(1.91), 1.91, 21);
  const float step = kMaxLateralJerk / (60.0f / 3.6f * 60.0f / 3.6f) * kCurvatureRateWindowS;
  ASSERT_NEAR(back.desired_curvature, before, step * 1.001f)
      << "회복하면 멈추기 전 곡률에서 이어간다";
}

// 보드는 JSON을 읽고 재생·테스트는 기본값을 쓴다. 둘이 갈리면 재생 대조가 보드를 대변하지 못한다.
TEST(ControlReplay, SteeringJsonMatchesDefaults) {
  SteeringParams json, defaults;
  std::string error;
  ASSERT_TRUE(load_steering_params_json("params/steering.json", &json, &error))
      << "steering.json 읽기";
  // steering.json 토크 이득이 코드 기본값과 같다
  ASSERT_EQ(json.torque_lat_accel_factor, defaults.torque_lat_accel_factor);
  ASSERT_EQ(json.torque_kp, defaults.torque_kp);
  ASSERT_EQ(json.torque_ki, defaults.torque_ki);
  ASSERT_EQ(json.torque_friction, defaults.torque_friction);
  const std::string path = "/tmp/gtest_control_replay_raw_keys.json";
  std::FILE *f = std::fopen(path.c_str(), "w");
  ASSERT_NE(f, nullptr) << "raw 키 픽스처 쓰기";
  std::fputs("{\"torque_kf_raw\": 20}\n", f);
  std::fclose(f);
  SteeringParams rejected;
  const bool loaded = load_steering_params_json(path, &rejected, &error);
  std::remove(path.c_str());
  // 2026-09-24 이전 raw 이득 키는 기본값으로 넘어가지 않고 거부된다
  ASSERT_FALSE(loaded);
  ASSERT_NE(error.find("torque_kf_raw"), std::string::npos);
}

/* 상류 controlsd: 비활성 중 목표 곡률은 실제 곡률을 따라가고, 재활성 때 거기서 한계 안으로 출발한다. */
TEST(ControlReplay, InactiveDesiredTracksActual) {
  LateralControllerConfig config;
  config.force_engaged = true;
  config.steering_params.angle_offset_deg = 0.0f;
  LateralController controller(config);
  const float v = 60.0f / 3.6f;
  const float step = kMaxLateralJerk / (v * v) * kCurvatureRateWindowS;
  LateralPath blocked = replay_path();
  blocked.usable_for_steering = false;
  LateralControlResult r;
  int tick = 0;
  const auto step_once = [&](const LateralPath &path) {
    const double t = 1.0 + 0.01 * tick;
    VehicleCanState vehicle = ready_vehicle(t);
    vehicle.steering_angle_deg = 10.0f;
    r = controller.update(path, replay_target(), vehicle, t, tick++);
  };
  for (int i = 0; i < 100; ++i) step_once(blocked);
  // 비활성 동안 목표 곡률은 실제 곡률에 자리 잡는다
  ASSERT_FALSE(r.active);
  ASSERT_NEAR(r.desired_curvature, r.actual_curvature, 1e-6f);
  float previous = r.desired_curvature;
  while (!r.active && tick < 300) {
    previous = r.desired_curvature;
    step_once(replay_path());
  }
  ASSERT_TRUE(r.active) << "경로 디바운스가 풀린다";
  // 다시 활성이 되면 plan이 아니라 실제 곡률에서 출발한다
  ASSERT_NEAR(r.desired_curvature, previous, step * 1.001f);
  ASSERT_GT(std::fabs(r.desired_curvature - replay_target().curvatures[0]), 10.0f * step);
}

// 명령줄로 받은 CAN 픽스처(없으면 CanFixture를 건너뛴다)
const char *g_fixture_path = nullptr;

struct TimedCanFrame {
  uint64_t timestamp_us = 0;
  CanFrame frame;
};

uint32_t read_u32_le(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8u) |
         (static_cast<uint32_t>(data[2]) << 16u) | (static_cast<uint32_t>(data[3]) << 24u);
}

uint64_t read_u64_le(const uint8_t *data) {
  return static_cast<uint64_t>(read_u32_le(data)) |
         (static_cast<uint64_t>(read_u32_le(data + 4)) << 32u);
}

/* K230CAN1: "K230CAN1", u32 version 1, u32 record_size 24, u64 count, 이어서 레코드
 * (u64 timestamp_us, u32 address, u8 bus, u8 length, data[8], 2 B 패딩). 시간순이어야 한다.
 * 형식이 틀리면 실패를 남기고 빈 목록을 돌려준다. */
std::vector<TimedCanFrame> read_can_fixture(const std::string &path) {
  constexpr size_t kHeaderSize = 24;
  constexpr uint32_t kRecordSize = 24;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    ADD_FAILURE() << "열 수 없음 " << path;
    return {};
  }
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                                   std::istreambuf_iterator<char>());
  if (bytes.size() < kHeaderSize || std::memcmp(bytes.data(), "K230CAN1", 8) != 0) {
    ADD_FAILURE() << "CAN 픽스처 magic이 틀림";
    return {};
  }
  if (read_u32_le(bytes.data() + 8) != 1 || read_u32_le(bytes.data() + 12) != kRecordSize) {
    ADD_FAILURE() << "지원하지 않는 CAN 픽스처 버전";
    return {};
  }
  const uint64_t count = read_u64_le(bytes.data() + 16);
  if (count > (std::numeric_limits<size_t>::max() - kHeaderSize) / kRecordSize ||
      bytes.size() != kHeaderSize + static_cast<size_t>(count) * kRecordSize) {
    ADD_FAILURE() << "CAN 픽스처 크기가 틀림";
    return {};
  }
  std::vector<TimedCanFrame> records;
  records.reserve(static_cast<size_t>(count));
  for (size_t index = 0; index < static_cast<size_t>(count); ++index) {
    const uint8_t *record = bytes.data() + kHeaderSize + index * kRecordSize;
    TimedCanFrame timed;
    timed.timestamp_us = read_u64_le(record);
    timed.frame.address = read_u32_le(record + 8);
    timed.frame.bus = record[12];
    timed.frame.length = record[13];
    std::copy_n(record + 14, timed.frame.data.size(), timed.frame.data.begin());
    if ((index > 0 && timed.timestamp_us < records.back().timestamp_us) ||
        timed.frame.address > 0x1fffffffU || timed.frame.bus > 7u ||
        timed.frame.length > timed.frame.data.size()) {
      ADD_FAILURE() << "CAN 픽스처 레코드가 틀림, 인덱스 " << index;
      return {};
    }
    records.push_back(timed);
  }
  return records;
}

/* 픽스처는 60초 연속 주행 구간이어야 한다(active > 5900틱, 토크 > 0). 정차
 * 구간은 이 전제에 걸려 실패한다. tools/control/export_can_fixture.py가 녹화
 * events/NNN.bin 하나를 이 형식으로 내보낸다. */
TEST(ControlReplay, CanFixture) {
  if (g_fixture_path == nullptr)
    GTEST_SKIP() << "gtest_control_replay <fixture.k230can>으로 준 경우만 돈다";
  const std::vector<TimedCanFrame> records = read_can_fixture(g_fixture_path);
  ASSERT_FALSE(records.empty()) << "CAN 픽스처에 프레임이 없다";
  LateralControllerConfig config;
  config.force_engaged = true;
  std::string error;
  ASSERT_TRUE(load_steering_params_json("params/steering.json", &config.steering_params, &error))
      << "조향 파라미터 읽기";
  ASSERT_TRUE(load_driving_params_json("params/driving.json", &config.driving_params, &error))
      << "주행 파라미터 읽기";
  ASSERT_NEAR(config.driving_params.mdps_speed_spoof_kph, 60.0f, 1e-6f)
      << "주행 파라미터의 MDPS 속도";
  LateralController controller(config);
  VehicleCanState vehicle;
  const LateralPath path = replay_path();
  const LateralTarget target = replay_target();
  size_t next_record = 0;
  size_t active_ticks = 0;
  size_t invalid_frames = 0;
  size_t lkas0 = 0;
  size_t lkas1 = 0;
  size_t clu1 = 0;
  size_t mdps2 = 0;
  int max_torque = 0;
  float max_curvature_error = 0.0f;
  float ref_prev_curvature = 0.0f;

  const double duration_s = records.back().timestamp_us / 1000000.0;
  const int ticks = static_cast<int>(std::ceil(duration_s * 100.0)) + 2;
  for (int tick = 0; tick < ticks; ++tick) {
    const double now_s = static_cast<double>(tick) * 0.01;
    const uint64_t now_us = static_cast<uint64_t>(now_s * 1000000.0);
    // 이 틱까지 도착한 프레임을 먹인다.
    for (; next_record < records.size() && records[next_record].timestamp_us <= now_us;
         ++next_record) {
      const CanFrame &frame = records[next_record].frame;
      update_vehicle_can_state(&vehicle, frame.address, frame.data,
                               frame.length, frame.bus, now_s);
    }
    const auto result = controller.update(path, target, vehicle, now_s, tick);
    /* 컨트롤러는 휠 속도 평균으로 곡률을 낸다. 클러스터 속도를 먹이면
     * 참조식이 다른 입력을 보게 되어 비교가 성립하지 않는다. 첫 WHL_SPD11
     * 전에는 속도가 NaN이고 컨트롤러가 어차피 비활성이라 비교하지 않는다. */
    if (std::isfinite(result.control_speed_kph)) {
      // 컨트롤러처럼 비활성이면 plan 대신 실제 곡률을 클립한다.
      const float speed = std::max(0.0f, result.control_speed_kph / 3.6f);
      const float requested =
          result.active
              ? reference_plan_curvature(target, speed, config.steering_params.steer_actuator_delay)
              : result.actual_curvature;
      const float expected_curvature =
          reference_clip_curvature(speed, ref_prev_curvature, requested);
      ref_prev_curvature = expected_curvature;
      max_curvature_error = std::max(
          max_curvature_error, std::fabs(result.desired_curvature - expected_curvature));
    }
    if (result.active) ++active_ticks;
    max_torque = std::max(max_torque, std::abs(result.apply_torque));
    for (const CanFrame &frame : result.frames) {
      const bool valid_length =
          (frame.address == kHyundaiClu11Address && frame.length == 4) ||
          (frame.address != kHyundaiClu11Address && frame.length == 8);
      if (!valid_length) ++invalid_frames;
      if (frame.address == kHyundaiLkas11Address && frame.bus == 0) ++lkas0;
      if (frame.address == kHyundaiLkas11Address && frame.bus == 1) ++lkas1;
      if (frame.address == kHyundaiClu11Address && frame.bus == 1) ++clu1;
      if (frame.address == kHyundaiMdps12Address && frame.bus == 2) ++mdps2;
    }
  }
  ASSERT_EQ(next_record, records.size()) << "픽스처 프레임을 모두 먹였다";
  ASSERT_GT(active_ticks, 5900) << "주행 구간 내내 컨트롤러가 활성이다";
  ASSERT_EQ(invalid_frames, 0) << "생성한 CAN 프레임의 길이가 맞다";
  // LKAS/MDPS는 100 Hz로 나간다
  ASSERT_GT(lkas0, 5900);
  ASSERT_EQ(lkas0, lkas1);
  ASSERT_EQ(lkas0, mdps2);
  // CLU11은 50 Hz로 나간다
  ASSERT_GT(clu1, 2900);
  ASSERT_GE(clu1 * 2, lkas0 - 1);
  ASSERT_LE(clu1 * 2, lkas0 + 1);
  // 조향 토크 범위
  ASSERT_GT(max_torque, 0);
  ASSERT_LE(max_torque, 384);
  ASSERT_LT(max_curvature_error, 1e-6f)
      << "지연 보정 곡률이 openpilot 참조식과 같다";
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc > 2) {
    std::fprintf(stderr, "usage: %s [fixture.k230can]\n", argv[0]);
    return 2;
  }
  if (argc == 2) g_fixture_path = argv[1];
  return RUN_ALL_TESTS();
}

