// 녹화된 ModelState/ControlState로 OpenpilotLateralPlanner를 재실행한다.
// 녹화된 인지 결과에 대해 플래너가 무엇을 요구했는지 오프라인으로 재현한다.
// 사용: planner_replay <out.csv> <events.bin...>
#include "k230_ipc.h"
#include "openpilot_lateral_planner.h"
#include "recording_format.h"
#include "steering_params.h"
#include "driving_params.h"
#include "vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace {
constexpr float kMaxLateralJerk = 5.0f;
constexpr float kMaxLateralAccel = 3.3f;
constexpr float kMaxCurvature = 0.3f;
constexpr float kWindowS = 0.05f;

float interp_lateral(float x, const float *values) {
  if (x <= 0.0f) return values[0];
  for (int i = 1; i < kLateralControlN; ++i) {
    const float hx = model_t_idx(i);
    if (x <= hx) {
      const float lx = model_t_idx(i - 1);
      return values[i - 1] + (x - lx) / (hx - lx) * (values[i] - values[i - 1]);
    }
  }
  return values[kLateralControlN - 1];
}

// lateral_controller.cc의 lag_adjusted_desired_curvature 복제 (roll=0)
float lag_adjusted(const LateralTarget &t, float v, float delay_s, float age_s) {
  if (!t.valid) return 0.0f;
  const float delay = std::max(0.01f, delay_s) + std::clamp(age_s, 0.0f, 0.25f);
  const float cur = t.curvatures[0];
  const float psi = interp_lateral(delay, t.psis);
  const float speed = std::max(v, 0.1f);
  float des = cur + 2.0f * (psi / (speed * delay) - cur);
  const float rate = kMaxLateralJerk / (speed * speed);
  des = std::clamp(des, cur - rate * kWindowS, cur + rate * kWindowS);
  const float ls = std::max(speed, 1.0f);
  des = std::clamp(des, -kMaxLateralAccel / (ls * ls), kMaxLateralAccel / (ls * ls));
  return std::clamp(des, -kMaxCurvature, kMaxCurvature);
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: %s <out.csv> <events...>\n", argv[0]); return 1; }

  SteeringParams steering;
  DrivingParams driving;
  OpenpilotLateralPlanner planner(steering, driving);

  VehicleCanState vehicle{};   // 블링커/개입 없음
  std::FILE *out = std::fopen(argv[1], "w");
  std::fprintf(out, "t,v_kph,measured,des_rec,des_replay,target_curv,target_y,"
                    "lane_l,lane_r,prob_l,prob_r,d_prob,laneless,lane_w,mpc_valid,heading0,heading_target\n");

  float v_kph = 0.0f, measured = 0.0f, des_rec = 0.0f;
  bool have_cs = false;
  for (int a = 2; a < argc; ++a) {
    std::ifstream f(argv[a], std::ios::binary);
    K230EventFileHeader hdr{};
    f.read(reinterpret_cast<char *>(&hdr), sizeof(hdr));
    if (std::memcmp(hdr.magic, "K230LOG1", 8) != 0) continue;
    f.seekg(hdr.header_size);
    K230EventRecordHeader rh{};
    std::vector<char> buf;
    while (f.read(reinterpret_cast<char *>(&rh), sizeof(rh))) {
      buf.resize(rh.payload_size);
      if (!f.read(buf.data(), rh.payload_size)) break;
      if (rh.type == static_cast<uint16_t>(K230RecordType::ControlState) &&
          rh.payload_size >= sizeof(K230ControlState)) {
        K230ControlState cs{};
        std::memcpy(&cs, buf.data(), sizeof(cs));
        v_kph = cs.ego_speed_kph > 0.0f ? cs.ego_speed_kph : cs.cluster_speed_kph;
        measured = cs.actual_curvature;
        des_rec = cs.desired_curvature;
        have_cs = true;
      } else if (rh.type == static_cast<uint16_t>(K230RecordType::ModelState) &&
                 rh.payload_size >= sizeof(K230ModelState)) {
        if (!have_cs) continue;
        K230ModelState ms{};
        std::memcpy(&ms, buf.data(), sizeof(ms));
        const float v = v_kph / 3.6f;
        LateralTarget t = planner.update(ms, vehicle, v, measured, true, 0.0f);
        const float des = lag_adjusted(t, v, steering.steer_actuator_delay, 0.05f);
        std::fprintf(out, "%.3f,%.1f,%.6f,%.6f,%.6f,%.6f,%.3f,"
                          "%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.2f,%d,%.4f,%.4f\n",
                     rh.timestamp_ns * 1e-9, v_kph, measured, des_rec, des,
                     t.curvature, t.target_y_m,
                     t.lane_left_y_m, t.lane_right_y_m,
                     t.lane_left_prob, t.lane_right_prob, t.lane_d_prob,
                     t.laneless_mode ? 1 : 0, t.lane_width_m,
                     t.mpc_solution_valid ? 1 : 0, t.heading_rad, t.psis[kLateralControlN - 1]);
      }
    }
  }
  std::fclose(out);
  return 0;
}
