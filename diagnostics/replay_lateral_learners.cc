/* 녹화를 paramsd·torqued 학습기로 재생한다. CAN은 런타임과 같은 vehicle_can으로 풀고
 * controlsd와 같은 LateralLearners를 부른다(활성·보낸 토크는 ControlState, 운전자 개입은
 * 기록된 운전자 토크를 컨트롤러와 같게 디바운스). 참조 구현 대조용으로 입력·출력을 남길 수 있다.
 * 사용: replay_lateral_learners [--upstream-schedule] [--fit-all] [--inputs in.bin] [--outputs out.csv]
 *       [--torque-inputs tin.bin] [--torque-outputs tout.csv] [--torque-cache c.bin]
 *       [--steering route/params/steering.json] <events...>
 * --upstream-schedule: 조향각·속도를 상류처럼 20 Hz로만 관측(기본은 런타임과 같은 매 틱).
 * --fit-all: torqued 적합에 점 전부(기본은 상류처럼 2000점 무작위).
 * --torque-cache: 있으면 복원하고 저장 틱마다 덮어쓴다(상류 LiveTorqueParameters).
 * --steering: 녹화 당시 파라미터(사전값·지연·토크 튜닝). 없으면 코드 기본값이라 보드와 다를 수 있다. */
#include "control_params.h"
#include "ipc_messages.h"
#include "lateral_learners.h"
#include "recording_format.h"
#include "vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

struct InputRow {  // 참조 구현이 읽는 고정 레이아웃
  double t, angle_deg, speed, yaw, lat;
  int32_t fresh, gear, yaw_valid, lat_valid;
};
struct TorqueInputRow {
  double t, steer, speed, yaw, roll;
  int32_t fresh, lat_active, steer_override, pose_valid;
};

constexpr int kSteeringPressedMinCount = 5;  // lateral_controller.cc와 같다

}  // namespace

int main(int argc, char **argv) {
  std::string inputs_path, outputs_path, torque_inputs_path, torque_outputs_path, torque_cache_path;
  std::string steering_path;
  bool fit_all = false;
  VehicleParamsOptions options;
  std::vector<std::string> events;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--upstream-schedule") options.car_state_every_tick = false;
    else if (arg == "--inputs" && i + 1 < argc) inputs_path = argv[++i];
    else if (arg == "--outputs" && i + 1 < argc) outputs_path = argv[++i];
    else if (arg == "--fit-all") fit_all = true;
    else if (arg == "--torque-inputs" && i + 1 < argc) torque_inputs_path = argv[++i];
    else if (arg == "--torque-outputs" && i + 1 < argc) torque_outputs_path = argv[++i];
    else if (arg == "--torque-cache" && i + 1 < argc) torque_cache_path = argv[++i];
    else if (arg == "--steering" && i + 1 < argc) steering_path = argv[++i];
    else events.push_back(arg);
  }
  if (events.empty()) {
    std::fprintf(stderr, "usage: %s [--upstream-schedule] [--fit-all] [--inputs in.bin] [--outputs out.csv] "
                         "[--torque-inputs tin.bin] [--torque-outputs tout.csv] [--torque-cache c.bin] "
                         "[--steering steering.json] <events.bin...>\n", argv[0]);
    return 1;
  }

  SteeringParams sp;
  if (!steering_path.empty()) {
    std::string error;
    if (!load_steering_params_json(steering_path, &sp, &error)) {
      std::fprintf(stderr, "%s: %s\n", steering_path.c_str(), error.c_str());
      return 1;
    }
  }
  const DrivingParams dp;
  const double timeout_s = dp.vehicle_state_timeout_ms / 1000.0;
  std::string cache;
  if (!torque_cache_path.empty()) {
    std::ifstream f(torque_cache_path, std::ios::binary);
    cache.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  }
  LateralLearners learners(sp, std::string(), cache, 1, options);
  if (!cache.empty())
    std::printf("torqued 캐시 %zu B: %s\n", cache.size(),
                learners.torque_restore_status() == TorqueRestore::Restored      ? "복원"
                : learners.torque_restore_status() == TorqueRestore::KeyMismatch ? "키 불일치"
                                                                                 : "손상");
  learners.torque_estimator().set_fit_all_points(fit_all);
  int pressed_counter = 0;

  std::FILE *in_file = inputs_path.empty() ? nullptr : std::fopen(inputs_path.c_str(), "wb");
  std::FILE *out_file = outputs_path.empty() ? nullptr : std::fopen(outputs_path.c_str(), "w");
  if (out_file)
    std::fprintf(out_file, "t,inputs_ok,valid,sensor_valid,steer_ratio,stiffness,roll_deg,"
                           "offset_avg_deg,offset_deg,sr_std,sf_std,yaw_bias\n");
  std::FILE *tin_file = torque_inputs_path.empty() ? nullptr : std::fopen(torque_inputs_path.c_str(), "wb");
  std::FILE *tout_file = torque_outputs_path.empty() ? nullptr : std::fopen(torque_outputs_path.c_str(), "w");
  if (tout_file)
    std::fprintf(tout_file, "t,inputs_ok,valid,factor_raw,offset_raw,friction_raw,factor,offset,"
                            "friction,points,cal_perc,decay\n");

  VehicleCanState vehicle{};
  std::vector<char> buf;
  /* ControlState 레코드 시각은 controlsd가 만든 시각이라 앞에 기록된 CAN 배치보다 이를 수
   * 있다. 실시간이면 now가 항상 수신 시각 뒤이므로, 재생에서도 본 CAN 중 최신을 하한으로 둔다. */
  double latest_can_s = 0.0;
  long publishes = 0, active_ticks = 0, ticks = 0;
  double first_t = -1.0, last_t = 0.0;
  for (const std::string &path : events) {
    std::ifstream file(path, std::ios::binary);
    K230EventFileHeader hdr{};
    file.read(reinterpret_cast<char *>(&hdr), sizeof(hdr));
    if (!file || std::memcmp(hdr.magic, "K230LOG1", 8) != 0) continue;
    file.seekg(hdr.header_size);
    K230EventRecordHeader rh{};
    while (file.read(reinterpret_cast<char *>(&rh), sizeof(rh))) {
      if (rh.type < 1 || rh.type > static_cast<uint16_t>(K230RecordType::LearnerState) ||
          rh.payload_size > (1U << 20))
        break;
      buf.resize(rh.payload_size);
      if (!file.read(buf.data(), rh.payload_size)) break;
      const double record_s = static_cast<double>(rh.timestamp_ns) * 1e-9;
      const double now_s = std::max(record_s, latest_can_s);
      if (rh.type == static_cast<uint16_t>(K230RecordType::CanRx)) {
        latest_can_s = std::max(latest_can_s, record_s);
        K230RecordedCanBatchHeader batch{};
        if (rh.payload_size < sizeof(batch)) continue;
        std::memcpy(&batch, buf.data(), sizeof(batch));
        size_t offset = sizeof(batch);
        for (uint32_t i = 0; i < batch.count; ++i) {
          if (offset + sizeof(K230RecordedCanFrame) > rh.payload_size) break;
          K230RecordedCanFrame frame{};
          std::memcpy(&frame, buf.data() + offset, sizeof(frame));
          offset += sizeof(frame);
          if (frame.data_len > 8) continue;
          std::array<uint8_t, 8> data{};
          std::memcpy(data.data(), frame.data, frame.data_len);
          update_vehicle_can_state(&vehicle, frame.address, data,
                                   static_cast<uint8_t>(frame.data_len),
                                   static_cast<uint8_t>(frame.src), now_s);
        }
        continue;
      }
      // 제어 틱(ControlState, 100 Hz)마다 한 번 넣는다. controlsd가 부를 자리와 같다.
      if (rh.type != static_cast<uint16_t>(K230RecordType::ControlState)) continue;
      K230ControlState cs{};
      std::memcpy(&cs, buf.data(), std::min(sizeof(cs), buf.size()));
      const bool pressed = std::abs(cs.driver_torque) > sp.steering_pressed_threshold;
      pressed_counter = std::clamp(pressed_counter + (pressed ? 1 : -1), 0,
                                   kSteeringPressedMinCount * 2 + 1);
      learners.update(vehicle, now_s, timeout_s, cs.active != 0, cs.apply_torque,
                      pressed_counter > kSteeringPressedMinCount);

      const VehicleParamsInput &in = learners.last_vehicle_input();
      if (in_file) {
        const InputRow row{in.t_s, in.steering_angle_deg, in.speed_mps, in.yaw_rate_rad_s,
                           in.lat_accel_mps2, in.inputs_fresh ? 1 : 0, in.gear,
                           in.yaw_rate_valid ? 1 : 0, in.lat_accel_valid ? 1 : 0};
        std::fwrite(&row, sizeof(row), 1, in_file);
      }
      if (first_t < 0.0) first_t = now_s;
      last_t = now_s;
      ++ticks;
      if (in.inputs_fresh && in.speed_mps > 1.0 && std::fabs(in.steering_angle_deg) < 45.0 &&
          in.gear != 7)
        ++active_ticks;
      if (learners.vehicle_published()) {
        ++publishes;
        const VehicleParams &p = learners.vehicle_params();
        if (out_file)
          std::fprintf(out_file, "%.4f,%d,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                       now_s - first_t, p.inputs_ok, p.valid, p.sensor_valid, p.steer_ratio,
                       p.stiffness_factor, p.roll_rad * 180.0 / 3.14159265358979323846,
                       p.angle_offset_average_deg, p.angle_offset_deg, p.steer_ratio_std,
                       p.stiffness_factor_std, learners.yaw_bias_rad_s());
      }

      const TorqueEstimatorInput &tin = learners.last_torque_input();
      if (tin_file) {
        const TorqueInputRow row{tin.t_s, tin.steer_torque, tin.speed_mps, tin.yaw_rate_rad_s,
                                 tin.roll_rad, tin.inputs_fresh ? 1 : 0, tin.lat_active ? 1 : 0,
                                 tin.steer_override ? 1 : 0, tin.pose_valid ? 1 : 0};
        std::fwrite(&row, sizeof(row), 1, tin_file);
      }
      if (learners.torque_persist_due() && !torque_cache_path.empty()) {
        std::ofstream f(torque_cache_path, std::ios::binary | std::ios::trunc);
        f.write(learners.torque_cache().data(),
                static_cast<std::streamsize>(learners.torque_cache().size()));
      }
      if (learners.torque_published() && tout_file) {
        const TorqueParams &q = learners.torque_params();
        std::fprintf(tout_file, "%.4f,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g\n",
                     now_s - first_t, q.inputs_ok, q.valid, q.lat_accel_factor_raw,
                     q.lat_accel_offset_raw, q.friction_raw, q.lat_accel_factor,
                     q.lat_accel_offset, q.friction, q.total_bucket_points, q.cal_perc, q.decay);
      }
    }
  }
  if (in_file) std::fclose(in_file);
  if (out_file) std::fclose(out_file);
  if (tin_file) std::fclose(tin_file);
  if (tout_file) std::fclose(tout_file);
  const VehicleParams &p = learners.vehicle_params();
  std::printf("%.0fs, 틱 %ld, 활성 %.0fs, 출력 %ld | SR %.3f (±%.3f) 강성 %.3f 영점 %+.3f도 "
              "(합계 %+.3f) 롤 %+.3f도 | valid %d | 자이로 바이어스 %+.4f deg/s\n",
              last_t - first_t, ticks, active_ticks / 100.0, publishes, p.steer_ratio,
              p.steer_ratio_std, p.stiffness_factor, p.angle_offset_average_deg,
              p.angle_offset_deg, p.roll_rad * 180.0 / 3.14159265358979323846, p.valid,
              learners.yaw_bias_rad_s() * 180.0 / 3.14159265358979323846);
  const TorqueParams &q = learners.torque_params();
  std::printf("torqued: 점 %d (진행 %d%%) valid %d | 원시 배율 %.3f 절편 %+.3f 마찰 %.3f | "
              "필터 배율 %.3f 절편 %+.3f 마찰 %.3f (사전 %.3f/%.3f) decay %.1f\n",
              q.total_bucket_points, q.cal_perc, q.valid, q.lat_accel_factor_raw,
              q.lat_accel_offset_raw, q.friction_raw, q.lat_accel_factor, q.lat_accel_offset,
              q.friction, sp.torque_lat_accel_factor, sp.torque_friction, q.decay);
  for (int b = 0; b < TorqueEstimator::kBuckets; ++b)
    std::printf("%s%d", b ? " " : "  버킷 ", learners.torque_estimator().bucket_size(b));
  std::printf("\n");
  return 0;
}
