// paramsd·torqued 이식 검사: 필터 수학, 합성 주행 수렴, 게이트, 출력 제한, 저장/복원.
#include "check_harness.h"
#include "control_fixtures.h"
#include "lateral_learners.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }

VehicleModelConstants k7() {
  VehicleModelConstants c;
  c.mass_kg = 1816.0;
  c.wheelbase_m = 2.855;
  c.center_to_front_m = 2.855 * 0.4;
  c.tire_stiffness_factor = 1.0;
  c.steer_ratio = 16.8;
  return c;
}

void verify_jacobian() {
  const CarKalman::Globals g = CarKalman::globals_from(k7());
  const CarKalman::Vec x = {1.05, 15.5, 0.01, 0.002, 15.0, 0.1, 0.05, 0.03, 0.01};
  CarKalman::Vec xn;
  CarKalman::Mat f;
  CarKalman::step(g, x, 0.01, &xn, &f);
  double worst = 0.0;
  for (int k = 0; k < CarKalman::kN; ++k) {
    const double h = 1e-6 * std::fmax(1.0, std::fabs(x[k]));
    CarKalman::Vec xp = x, xm = x, fp, fm;
    CarKalman::Mat dummy;
    xp[k] += h;
    xm[k] -= h;
    CarKalman::step(g, xp, 0.01, &fp, &dummy);
    CarKalman::step(g, xm, 0.01, &fm, &dummy);
    for (int i = 0; i < CarKalman::kN; ++i)
      worst = std::fmax(worst, std::fabs((fp[i] - fm[i]) / (2 * h) - f[i][k]));
  }
  require(worst < 1e-7, "analytic Jacobian matches central differences");
}

bool positive_definite(const CarKalman::Mat &p) {
  double l[CarKalman::kN][CarKalman::kN] = {};
  for (int i = 0; i < CarKalman::kN; ++i) {
    for (int j = 0; j <= i; ++j) {
      double s = p[i][j];
      for (int k = 0; k < j; ++k) s -= l[i][k] * l[j][k];
      if (i == j) {
        if (s <= 0.0) return false;
        l[i][i] = std::sqrt(s);
      } else {
        l[i][j] = s / l[j][j];
      }
    }
  }
  return true;
}

/* 같은 모델을 참값으로 1 ms 적분해 조향각·속도·요레이트·횡가속을 만든다. */
struct Truth {
  double sr = 15.2, stiffness = 0.9, offset = rad(-1.6), roll = rad(0.5);
};

struct SynthSample {
  double t, angle_deg, speed, yaw, lat;
};

template <class F>
void synthesize(const Truth &truth, double seconds, F &&emit) {
  CarKalman::Globals g = CarKalman::globals_from(k7());
  CarKalman::Vec x = CarKalman::initial_x();
  x[CarKalman::kStiffness] = truth.stiffness;
  x[CarKalman::kSteerRatio] = truth.sr;
  x[CarKalman::kAngleOffset] = truth.offset;
  x[CarKalman::kAngleOffsetFast] = 0.0;
  x[CarKalman::kRoadRoll] = truth.roll;
  const double fine = 0.001;
  const int per_tick = 10;
  const int ticks = static_cast<int>(seconds * 100.0);
  for (int n = 0; n < ticks; ++n) {
    const double t = n * 0.01;
    // 5~25 m/s를 천천히 오가고, 직진 구간과 굽은 구간을 섞는다.
    const double speed = 15.0 + 10.0 * std::sin(2 * kPi * t / 240.0);
    const double weave = std::sin(2 * kPi * t / 90.0) > 0.0 ? 1.0 : 0.15;
    const double effective = weave * (rad(18.0) * std::sin(2 * kPi * t / 7.0) +
                                      rad(9.0) * std::sin(2 * kPi * t / 2.3));
    // 측정은 구간 시작의 상태로 찍는다. 적분 뒤 값을 붙이면 요레이트가 조향보다 10 ms
    // 앞서 보여 필터가 강성을 올려 읽는다.
    x[CarKalman::kSpeedX] = speed;
    x[CarKalman::kSteerAngle] = effective + truth.offset;
    CarKalman::Vec xn;
    CarKalman::Mat f;
    CarKalman::step(g, x, fine, &xn, &f);
    const double vdot = (xn[CarKalman::kSpeedY] - x[CarKalman::kSpeedY]) / fine;
    const double lat = vdot + speed * x[CarKalman::kYawRate] + 9.81 * truth.roll;
    emit(SynthSample{t, deg(effective + truth.offset), speed, x[CarKalman::kYawRate], lat});
    for (int k = 0; k < per_tick; ++k) {
      CarKalman::step(g, x, fine, &xn, &f);
      x = xn;
    }
  }
}

VehicleParamsInput input_from(const SynthSample &s) {
  VehicleParamsInput in;
  in.t_s = s.t;
  in.inputs_fresh = true;
  in.steering_angle_deg = s.angle_deg;
  in.speed_mps = s.speed;
  in.gear = 5;
  in.yaw_rate_valid = true;
  in.yaw_rate_rad_s = s.yaw;
  in.lat_accel_valid = true;
  in.lat_accel_mps2 = s.lat;
  return in;
}

CarKalman::Mat open_p0() {
  CarKalman::Mat p0 = CarKalman::q();
  p0[CarKalman::kSteerRatio][CarKalman::kSteerRatio] = 3.0 * 3.0;
  p0[CarKalman::kStiffness][CarKalman::kStiffness] = 0.3 * 0.3;
  p0[CarKalman::kAngleOffset][CarKalman::kAngleOffset] = rad(2.0) * rad(2.0);
  return p0;
}

/* 매 틱 관측이면 필터 수학만 남는다. 빠른 영점이 조향의 일부를 흡수해 SR은 약 1% 낮다. */
void verify_converges_and_stays_positive_definite() {
  const CarKalman::Mat p0 = open_p0();
  VehicleParamsOptions options;
  options.p_initial = &p0;
  options.car_state_every_tick = true;
  VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0, options);
  const Truth truth;
  bool always_pd = true, always_sensor_valid = true;
  int publishes = 0;
  synthesize(truth, 900.0, [&](const SynthSample &s) {
    if (learner.update(input_from(s))) {
      ++publishes;
      always_pd = always_pd && positive_definite(learner.kf().p());
      if (s.t > 60.0) always_sensor_valid = always_sensor_valid && learner.params().sensor_valid;
    }
  });
  const VehicleParams &p = learner.params();
  require(publishes > 17900 && publishes < 18100, "publishes at 20 Hz");
  require(always_pd, "Joseph update keeps the covariance positive definite");
  require(std::fabs(p.steer_ratio - truth.sr) < 0.3, "steer ratio converges to the truth");
  require(std::fabs(p.stiffness_factor - truth.stiffness) < 0.1, "stiffness converges");
  require(std::fabs(p.angle_offset_average_deg - deg(truth.offset)) < 0.15,
          "angle offset converges");
  require(std::fabs(p.roll_rad - truth.roll) < rad(0.3), "roll follows the lateral accelerometer");
  require(p.valid && p.inputs_ok, "converged estimate is valid");
  require(always_sensor_valid, "model and observed yaw rate agree");
}

/* 상류 스케줄(20 Hz 관측)은 조향이 50 ms씩 멈춰 들어가 SR을 낮게 읽는다. 영점·롤은 그대로다.
 * 이 성질이 바뀌면(스케줄이 달라지면) 여기서 걸린다. */
void verify_upstream_schedule_reads_steer_ratio_low() {
  const CarKalman::Mat p0 = open_p0();
  VehicleParamsOptions options;
  options.p_initial = &p0;
  options.car_state_every_tick = false;
  VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0, options);
  const Truth truth;
  synthesize(truth, 900.0, [&](const SynthSample &s) { learner.update(input_from(s)); });
  const VehicleParams &p = learner.params();
  require(p.steer_ratio < truth.sr - 0.3 && p.steer_ratio > truth.sr - 1.5,
          "upstream 20 Hz schedule reads the steer ratio low");
  require(std::fabs(p.angle_offset_average_deg - deg(truth.offset)) < 0.15,
          "upstream schedule still finds the angle offset");
  require(std::fabs(p.roll_rad - truth.roll) < rad(0.3), "upstream schedule still finds the roll");
}

void verify_gates_hold_state() {
  VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0);
  const CarKalman::Vec before = learner.kf().x();
  auto feed = [&](double speed, double angle, int gear, bool fresh) {
    for (int n = 0; n < 500; ++n) {
      VehicleParamsInput in;
      in.t_s = n * 0.01;
      in.inputs_fresh = fresh;
      in.steering_angle_deg = angle;
      in.speed_mps = speed;
      in.gear = gear;
      in.yaw_rate_valid = true;
      in.yaw_rate_rad_s = 0.2;
      learner.update(in);
    }
  };
  feed(0.9, 5.0, 5, true);
  feed(10.0, 50.0, 5, true);
  feed(10.0, 5.0, 7, true);
  feed(10.0, 5.0, 5, false);
  require(learner.kf().x() == before, "slow, large-angle, reverse and stale inputs leave the state alone");
}

void verify_output_limits_and_hysteresis() {
  VehicleParamsLearner learner(k7(), 16.8, 1.0, 0.0);
  double t = 0.0;
  auto publish = [&]() {
    for (;;) {
      VehicleParamsInput in;
      in.t_s = t;
      t += 0.01;
      in.inputs_fresh = true;
      in.speed_mps = 0.0;  // 비활성: 상태를 우리가 정한 값에 둔다
      if (learner.update(in)) return learner.params();
    }
  };
  publish();
  learner.kf().mutable_x()[CarKalman::kAngleOffset] = rad(12.0);
  double last = publish().angle_offset_average_deg;
  require(near(static_cast<float>(last), 1.0f, 1e-9f), "offset output moves at most 1 deg per publish");
  VehicleParams p{};
  for (int i = 0; i < 20; ++i) p = publish();
  require(near(static_cast<float>(p.angle_offset_average_deg), 12.0f, 1e-9f), "offset output reaches the state");
  require(!p.angle_offset_average_valid && !p.valid, "offset above 10 deg is invalid");
  learner.kf().mutable_x()[CarKalman::kAngleOffset] = rad(9.0);
  for (int i = 0; i < 20; ++i) p = publish();
  require(!p.angle_offset_average_valid, "recovery needs the lowered 8 deg threshold");
  learner.kf().mutable_x()[CarKalman::kAngleOffset] = rad(7.5);
  for (int i = 0; i < 20; ++i) p = publish();
  require(p.angle_offset_average_valid, "valid again below 8 deg");
  learner.kf().mutable_x()[CarKalman::kSteerRatio] = 34.0;
  p = publish();
  require(!p.steer_ratio_valid && !p.valid, "steer ratio above twice the prior is invalid");
}

void verify_persistence() {
  const VehicleModelConstants c = k7();
  VehicleParamsLearner learner(c, 16.8, 1.0, 0.0);
  VehicleParamsInput in;
  in.inputs_fresh = true;
  int due = 0, published = 0;
  for (int n = 0; n < 1201 * 5 + 2; ++n) {
    in.t_s = n * 0.01;
    if (learner.update(in)) {
      if (learner.persist_due()) {
        require(published % 1200 == 0, "persists on publish 0, 1200, ...");
        ++due;
      }
      ++published;
    }
  }
  require(due == 2, "persists once a minute including the first publish");

  VehicleParams p;
  p.steer_ratio = 15.1;
  p.stiffness_factor = 0.8;
  p.angle_offset_average_deg = -1.62;
  const std::string saved = persist_vehicle_params(p, c, -0.0017);
  const VehicleParamsInit ok = restore_vehicle_params(saved, c);
  require(ok.restored && near(static_cast<float>(ok.steer_ratio), 15.1f, 1e-4f) &&
              near(static_cast<float>(ok.angle_offset_deg), -1.62f, 1e-4f) &&
              near(static_cast<float>(ok.yaw_bias_rad_s), -0.0017f, 1e-7f),
          "restores steer ratio, average offset and yaw bias");
  require(ok.stiffness_factor == 1.0, "stiffness is reset every drive");

  VehicleModelConstants other = c;
  other.mass_kg += 50.0;
  require(!restore_vehicle_params(saved, other).restored, "a different car is not restored");
  VehicleModelConstants prior = c;
  prior.steer_ratio = 7.0;  // 15.1 > 2×7
  require(!restore_vehicle_params(saved, prior).restored, "an insane steer ratio is not restored");
  const VehicleParamsInit bad = restore_vehicle_params("{ nope", c);
  require(!bad.restored && bad.steer_ratio == c.steer_ratio && bad.angle_offset_deg == 0.0,
          "a broken file falls back to the prior");
}

void verify_yaw_bias() {
  YawBiasEstimator est;
  double t = 0.0;
  for (int i = 0; i < 150; ++i, t += 0.01) est.update(t, 0.0, true, -0.0019);
  require(est.bias() == 0.0, "bias waits for two seconds of standstill");
  for (int i = 0; i < 2000; ++i, t += 0.01) est.update(t, 0.0, true, -0.0019);
  require(near(static_cast<float>(est.bias()), -0.0019f, 1e-6f), "bias follows the standstill yaw rate");
  const double before = est.bias();
  est.update(t, 12.0, true, 0.3);
  require(est.bias() == before, "bias does not move while driving");
  require(near(static_cast<float>(est.update(t, 12.0, true, 0.3)), static_cast<float>(0.3 + 0.0019), 1e-6f),
          "corrected yaw rate subtracts the bias");
  // 휠속도 0으로 기어가며 도는 동안(>0.5°/s)은 흡수하지 않는다
  for (int i = 0; i < 1000; ++i, t += 0.01) est.update(t, 0.0, true, 0.05);
  require(est.bias() == before, "a creeping turn at zero wheel speed is not taken as bias");
}

// ---------------------------------------------------------------- torqued

constexpr double kGravity = 9.81;
constexpr double kTorqueLag = 0.34;

/* 조향 토크를 20초에 −0.5→0.5로 쓸어 되돌린다(꼭짓점은 20 Hz 틱 위). 지연 뒤 횡가속은
 * 직선에서 법선 방향으로 half_width만큼, 올릴 때 아래·내릴 때 위로 벗어난다(마찰 평행사변형).
 * 요레이트는 v·r − g·sinθ가 그 횡가속이 되게 만든다(우측 양수). 폭이 넓을수록 적합 기울기가 커진다(상류 성질: 버킷 밀도가
 * 토크 축 고정 경계에서 계단이라 두 가지가 다른 자리에서 잘린다. 폭 0.02 +0.4%, 0.05 +2.8%). */
struct TorqueTruth {
  double factor = 1.6;
  double offset = 0.05;
  double half_width = 0.02;
  double roll_rad = 1.0 * kPi / 180.0;
  double speed = 20.0;
};

double sweep(double t) {
  const double phase = std::fmod(t + 400.0, 40.0);
  return phase < 20.0 ? -0.5 + phase / 20.0 : 0.5 - (phase - 20.0) / 20.0;
}
double rising(double t) { return std::fmod(t + 400.0, 40.0) < 20.0 ? 1.0 : -1.0; }

TorqueEstimatorInput torque_input(const TorqueTruth &tr, double t) {
  const double phi = std::atan(tr.factor);
  const double src = t - kTorqueLag;
  const double y = tr.factor * sweep(src) + tr.offset - rising(src) * tr.half_width * std::cos(phi);
  TorqueEstimatorInput in;
  in.t_s = t;
  in.inputs_fresh = true;
  in.lat_active = true;
  in.steer_torque = sweep(t) + rising(t) * tr.half_width * std::sin(phi);
  in.speed_mps = tr.speed;
  in.pose_valid = true;
  in.roll_rad = tr.roll_rad;
  in.yaw_rate_rad_s = (y + std::sin(tr.roll_rad) * kGravity) / tr.speed;
  return in;
}

/* 100 Hz 틱 [tick0, tick0+ticks). edit이 입력을 바꿀 수 있다. */
template <class F>
void drive_torque(TorqueEstimator *est, const TorqueTruth &tr, long tick0, long ticks, F &&edit) {
  for (long i = tick0; i < tick0 + ticks; ++i) {
    TorqueEstimatorInput in = torque_input(tr, i * 0.01);
    edit(i, &in);
    est->update(in);
  }
}
void drive_torque(TorqueEstimator *est, const TorqueTruth &tr, long tick0, long ticks) {
  drive_torque(est, tr, tick0, ticks, [](long, TorqueEstimatorInput *) {});
}

TorqueTuning prior(double factor = 1.4) {
  TorqueTuning t;
  t.lat_accel_factor = factor;
  t.friction = 0.1;
  return t;
}

/* 독립 참조: [x,1,y]ᵀ[x,1,y]의 최소 고유값(3차 특성방정식 폐형해)과 그 고유벡터(두 행의 외적). */
void reference_tls(const std::vector<std::array<double, 2>> &pts, double *slope, double *offset,
                   double *friction) {
  double a[3][3] = {};
  for (const auto &p : pts) {
    const double r[3] = {p[0], 1.0, p[1]};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) a[i][j] += r[i] * r[j];
  }
  const double q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
  const double p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
  const double p2 = std::pow(a[0][0] - q, 2) + std::pow(a[1][1] - q, 2) + std::pow(a[2][2] - q, 2) + 2 * p1;
  const double pp = std::sqrt(p2 / 6.0);
  double b[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) b[i][j] = (a[i][j] - (i == j ? q : 0.0)) / pp;
  const double det = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) -
                     b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) +
                     b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
  const double phi = std::acos(std::fmax(-1.0, std::fmin(1.0, det / 2.0))) / 3.0;
  const double lambda = q + 2 * pp * std::cos(phi + 2.0 * kPi / 3.0);
  double m[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) m[i][j] = a[i][j] - (i == j ? lambda : 0.0);
  double best[3] = {}, best_norm = -1.0;
  for (int r0 = 0; r0 < 3; ++r0) {
    const int r1 = (r0 + 1) % 3;
    const double c[3] = {m[r0][1] * m[r1][2] - m[r0][2] * m[r1][1],
                         m[r0][2] * m[r1][0] - m[r0][0] * m[r1][2],
                         m[r0][0] * m[r1][1] - m[r0][1] * m[r1][0]};
    const double n = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    if (n > best_norm) {
      best_norm = n;
      for (int k = 0; k < 3; ++k) best[k] = c[k];
    }
  }
  *slope = -best[0] / best[2];
  *offset = -best[1] / best[2];
  const double sn = std::fabs(*slope) / std::sqrt(*slope * *slope + 1.0);
  const double cs = 1.0 / std::sqrt(*slope * *slope + 1.0);
  double mean = 0.0, var = 0.0;
  for (const auto &p : pts) mean += -p[0] * sn + p[1] * cs;
  mean /= static_cast<double>(pts.size());
  for (const auto &p : pts) var += std::pow(-p[0] * sn + p[1] * cs - mean, 2);
  *friction = std::sqrt(var / static_cast<double>(pts.size())) * 1.5;
}

double clipd(double v, double lo, double hi) { return std::fmin(std::fmax(v, lo), hi); }

/* 한 번의 필터 갱신(상류 update_params). */
void expect_filter_step(const TorqueParams &before_filters, double decay_before, const TorqueParams &raw,
                        const TorqueTuning &tuning, TorqueParams *out) {
  const double a = 0.05 / (decay_before + 0.05);
  out->lat_accel_factor = (1.0 - a) * before_filters.lat_accel_factor +
                          a * clipd(raw.lat_accel_factor_raw, 0.7 * tuning.lat_accel_factor,
                                    1.3 * tuning.lat_accel_factor);
  out->lat_accel_offset = (1.0 - a) * before_filters.lat_accel_offset + a * raw.lat_accel_offset_raw;
  out->friction = (1.0 - a) * before_filters.friction +
                  a * clipd(raw.friction_raw, 0.5 * tuning.friction, 1.5 * tuning.friction);
  out->decay = std::fmin(decay_before + 0.05, 250.0);
}

bool same_filters(const TorqueParams &a, const TorqueParams &b, double tol) {
  return std::fabs(a.lat_accel_factor - b.lat_accel_factor) <= tol &&
         std::fabs(a.lat_accel_offset - b.lat_accel_offset) <= tol &&
         std::fabs(a.friction - b.friction) <= tol && std::fabs(a.decay - b.decay) <= tol;
}

void verify_torque_exact_line() {
  TorqueTruth tr;
  tr.half_width = 0.0;
  TorqueEstimator est(prior(), kTorqueLag, 1);
  drive_torque(&est, tr, 0, 40000);
  const TorqueParams &p = est.params();
  require(std::fabs(p.lat_accel_factor_raw - tr.factor) < 1e-9 &&
              std::fabs(p.lat_accel_offset_raw - tr.offset) < 1e-9 && p.friction_raw < 1e-9,
          "points exactly on a line give that line and zero friction");
}

/* 긴 합성 주행: 유효 시점, 원시 추정, 필터 첫 걸음, 저장 틱의 이중 갱신, decay 상한. */
void verify_torque_learns_parallelogram() {
  TorqueTruth tr;
  const TorqueTuning tuning = prior();
  TorqueEstimator est(tuning, kTorqueLag, 7);
  est.set_fit_all_points(true);
  TorqueParams prev{};
  long frame = -1, first_valid_frame = -1;
  bool checked_first_step = false, checked_double = false;
  double prev_decay = 0.0;
  bool published = false;
  for (long i = 0; i < 150000 || !published; ++i) {  // 발행 틱에서 끝낸다
    published = est.update(torque_input(tr, i * 0.01));
    if (i % 5 == 0) ++frame;
    if (!published) continue;
    const TorqueParams &p = est.params();
    require((frame % 5) == 0, "torque params publish every fifth 20 Hz frame");
    require(p.inputs_ok && p.use_params && p.max_resets == 1.0, "envelope, useParams, resets");
    if (p.valid && first_valid_frame < 0) {
      first_valid_frame = frame;
      require(p.cal_perc == 100 && p.total_bucket_points >= 4000, "valid means every bucket is full");
      require(prev.cal_perc < 100 && !prev.valid, "the previous message was still calibrating");
      require(prev.lat_accel_factor == tuning.lat_accel_factor && prev.friction == tuning.friction &&
                  prev.lat_accel_offset == 0.0 && prev.decay == 50.0,
              "filters hold the prior until the buckets are valid");
      TorqueParams want;
      expect_filter_step(prev, prev.decay, p, tuning, &want);
      require(same_filters(p, want, 1e-12), "first valid message is one filter step from the prior");
      checked_first_step = true;
    } else if (p.valid && first_valid_frame >= 0 && prev.valid && p.decay < 250.0) {
      // 직전 발행 틱이 저장 틱이면 그 사이 필터가 두 번 돌았다
      const double step = ((frame - 5) % 240 == 0) ? 0.10 : 0.05;
      require(std::fabs(p.decay - prev_decay - step) < 1e-9, "decay grows 0.05 per update, twice on a cache frame");
      if (step > 0.07) checked_double = true;
    }
    prev = p;
    prev_decay = p.decay;
  }
  require(checked_first_step && checked_double, "the long drive reached both filter checks");
  const TorqueParams &p = est.params();
  require(p.valid && p.decay == 250.0, "decay saturates at 250");
  require(first_valid_frame * 0.05 > 250.0 && first_valid_frame * 0.05 < 400.0,
          "calibration takes the inner buckets' 500 points");
  // 마지막 발행이 쓴 점들로 독립 참조와 대조
  double slope, offset, friction;
  reference_tls(est.points(), &slope, &offset, &friction);
  require(std::fabs(p.lat_accel_factor_raw - slope) < 1e-9 && std::fabs(p.lat_accel_offset_raw - offset) < 1e-9 &&
              std::fabs(p.friction_raw - friction) < 1e-9,
          "total least squares matches an independent closed-form solution");
  require(std::fabs(p.lat_accel_factor_raw - tr.factor) < 0.005 * tr.factor &&
              std::fabs(p.lat_accel_offset_raw - tr.offset) < 0.002 &&
              std::fabs(p.friction_raw - 1.5 * tr.half_width) < 0.02 * 1.5 * tr.half_width,
          "parallelogram gives its slope, offset and 1.5x half width");
  require(p.lat_accel_factor > tuning.lat_accel_factor + 0.5 * (tr.factor - tuning.lat_accel_factor) &&
              p.lat_accel_factor < tr.factor,
          "filtered factor moves most of the way to the estimate");

  // 무작위 2000점 추출도 같은 답 근처
  TorqueEstimator sampled(tuning, kTorqueLag, 7);
  drive_torque(&sampled, tr, 0, 150000);
  require(std::fabs(sampled.params().lat_accel_factor_raw - p.lat_accel_factor_raw) < 0.01 &&
              std::fabs(sampled.params().friction_raw - p.friction_raw) < 0.005,
          "a 2000 point random subset fits close to the full set");
}

void verify_torque_clips_to_prior_band() {
  TorqueTruth tr;
  tr.factor = 1.9;        // 사전 1.4의 +30%(1.82) 밖
  tr.half_width = 0.12;   // 마찰 0.18, 사전 0.1의 +50%(0.15) 밖
  const TorqueTuning tuning = prior();
  TorqueEstimator est(tuning, kTorqueLag, 3);
  est.set_fit_all_points(true);
  TorqueParams prev{};
  for (long i = 0; i < 100000; ++i) {
    if (!est.update(torque_input(tr, i * 0.01))) continue;
    const TorqueParams &p = est.params();
    if (p.valid) {
      require(p.lat_accel_factor_raw > 1.82 && p.friction_raw > 0.15, "raw estimates are outside the band");
      TorqueParams want;
      expect_filter_step(prev, prev.decay, p, tuning, &want);
      require(same_filters(p, want, 1e-12), "filters step toward the clipped values");
      return;
    }
    prev = p;
  }
  require(false, "clip drive never became valid");
}

long count_points_added(TorqueEstimator *est, const TorqueTruth &tr, long tick0, long ticks,
                        const std::function<void(long, TorqueEstimatorInput *)> &edit) {
  const int before = est->total_points();
  drive_torque(est, tr, tick0, ticks, edit);
  return est->total_points() - before;
}

void verify_torque_gates() {
  TorqueTruth tr;
  const auto none = [](long, TorqueEstimatorInput *) {};
  {
    TorqueEstimator est(prior(), kTorqueLag, 1);
    drive_torque(&est, tr, 0, 495);
    require(est.total_points() == 0, "no point before 100 history samples");
    drive_torque(&est, tr, 495, 1);
    require(est.total_points() == 1, "the 100th sample adds the first point");
  }
  struct Gate {
    const char *what;
    std::function<void(long, TorqueEstimatorInput *)> edit;
  };
  const Gate gates[] = {
      {"speed at 15 m/s is not above MIN_VEL", [](long, TorqueEstimatorInput *in) { in->speed_mps = 15.0; }},
      {"invalid pose", [](long, TorqueEstimatorInput *in) { in->pose_valid = false; }},
      {"torque at the 0.02 threshold", [](long, TorqueEstimatorInput *in) { in->steer_torque = 0.02; }},
      {"lateral accel above 1", [](long, TorqueEstimatorInput *in) {
         in->yaw_rate_rad_s = (1.001 + std::sin(in->roll_rad) * kGravity) / in->speed_mps; }},
      {"lateral control inactive", [](long, TorqueEstimatorInput *in) { in->lat_active = false; }},
      {"driver override", [](long, TorqueEstimatorInput *in) { in->steer_override = true; }},
      {"stale inputs", [](long, TorqueEstimatorInput *in) { in->inputs_fresh = false; }},
  };
  for (const Gate &g : gates) {
    TorqueEstimator est(prior(), kTorqueLag, 1);
    require(count_points_added(&est, tr, 0, 2000, none) > 200, "baseline adds points");
    drive_torque(&est, tr, 2000, 40, g.edit);  // 속도·토크는 지연만큼 지난 이력에서 읽는다
    if (count_points_added(&est, tr, 2040, 1000, g.edit) != 0) throw std::runtime_error(g.what);
  }
  // 0.5초 개입·해제는 2초 창 + 지연이 지날 때까지 막는다. 낡은 입력은 이력을 지우지 않는다.
  for (int which = 0; which < 3; ++which) {
    TorqueEstimator est(prior(), kTorqueLag, 1);
    drive_torque(&est, tr, 0, 2000);
    const auto edit = [which](long, TorqueEstimatorInput *in) {
      if (which == 0) in->steer_override = true;
      else if (which == 1) in->lat_active = false;
      else in->inputs_fresh = false;
    };
    // 마지막 해제 표본 20.45초 → 창 [t−2−lag, t)가 그 뒤 20.5초를 벗어나는 22.84초까지 막힘
    count_points_added(&est, tr, 2000, 50, edit);
    const long soon = count_points_added(&est, tr, 2050, which == 2 ? 5 : 230, none);
    const long later = count_points_added(&est, tr, which == 2 ? 2055 : 2290, 100, none);
    if (which < 2) require(soon == 0 && later > 10, "a short disengagement blocks for the 2 s window plus lag");
    else require(soon == 1 && later > 10, "points resume on the first fresh sample after stale inputs");
  }
  // 상류 성질: 활성은 보간 후 0이 아니면 참이라 한 표본짜리 해제는 걸러지지 않고, 두 표본이면 걸린다
  for (int samples = 1; samples <= 2; ++samples) {
    TorqueEstimator est(prior(), kTorqueLag, 1);
    drive_torque(&est, tr, 0, 2500);
    drive_torque(&est, tr, 2500, 10, [samples](long i, TorqueEstimatorInput *in) {
      if (i < 2500 + 5 * samples) in->lat_active = false;
    });
    const long added = count_points_added(&est, tr, 2510, 220, none);  // 두 표본이면 막히는 구간
    if (samples == 1) require(added > 40, "a single inactive sample interpolates to nonzero and passes");
    else require(added == 0, "two inactive samples interpolate to zero between them and block");
  }
}

void verify_torque_schedule_and_cache() {
  TorqueTruth tr;
  const TorqueTuning tuning = prior();
  TorqueEstimator est(tuning, kTorqueLag, 5);
  est.set_fit_all_points(true);
  long frame = -1;
  std::string calibrating_cache, valid_cache;
  std::vector<std::array<double, 2>> valid_points;
  TorqueParams at_valid_persist{};
  for (long i = 0; i < 90000; ++i) {
    const bool published = est.update(torque_input(tr, i * 0.01));
    if (i % 5 != 0) continue;
    ++frame;
    require(est.persist_due() == (frame % 240 == 0), "cache every 240 frames, starting at the first");
    require(published == (frame % 5 == 0), "publish every 5 frames");
    if (!est.persist_due()) continue;
    if (frame == 2400) calibrating_cache = est.cache();  // 120초: 계산 가능, 아직 무효
    if (est.params().valid && valid_cache.empty()) {
      valid_cache = est.cache();
      valid_points = est.points();
      at_valid_persist = est.params();
    }
  }
  require(!calibrating_cache.empty() && !valid_cache.empty(), "both caches were taken");

  // 유효 캐시: 저장 get_msg는 발행값에서 필터를 한 번 더 돌린 값이다
  TorqueParams cached;
  expect_filter_step(at_valid_persist, at_valid_persist.decay, at_valid_persist, tuning, &cached);
  TorqueEstimator restored(tuning, kTorqueLag, 5, valid_cache);
  restored.set_fit_all_points(true);
  require(restored.restore_status() == TorqueRestore::Restored, "valid cache restores");
  const std::vector<std::array<double, 2>> pts = restored.points();
  require(pts.size() == valid_points.size(), "all cached points come back");
  for (size_t k = 0; k < pts.size(); ++k)
    require(pts[k][0] == static_cast<float>(valid_points[k][0]) &&
                pts[k][1] == static_cast<float>(valid_points[k][1]),
            "points round-trip at float precision in order");
  TorqueEstimatorInput stale = torque_input(tr, 0.0);
  stale.inputs_fresh = false;
  require(restored.update(stale), "restored estimator publishes on its first frame");
  const TorqueParams &r = restored.params();
  require(!r.inputs_ok && r.valid && r.max_resets == 1.0, "restored buckets are valid, envelope follows inputs");
  TorqueParams start;
  start.lat_accel_factor = static_cast<float>(cached.lat_accel_factor);
  start.lat_accel_offset = static_cast<float>(cached.lat_accel_offset);
  start.friction = static_cast<float>(cached.friction);
  TorqueParams want;
  expect_filter_step(start, static_cast<float>(cached.decay), r, tuning, &want);
  require(same_filters(r, want, 1e-12), "restored filters and decay continue from the cached message");

  // 무효 캐시: 점과 decay만, 필터는 사전값
  TorqueEstimator partial(tuning, kTorqueLag, 5, calibrating_cache);
  require(partial.restore_status() == TorqueRestore::Restored && partial.total_points() > 0,
          "a calibrating cache restores its points");
  partial.update(stale);
  require(!partial.params().valid && partial.params().lat_accel_factor == tuning.lat_accel_factor &&
              partial.params().friction == tuning.friction && partial.params().decay == 50.0,
          "a calibrating cache keeps the prior filters");

  // 튜닝이 바뀌면 무시(지우지 않음), 깨졌으면 지움
  TorqueEstimator other(prior(1.5), kTorqueLag, 5, valid_cache);
  require(other.restore_status() == TorqueRestore::KeyMismatch && other.total_points() == 0,
          "a cache from another tuning is ignored");
  other.update(stale);
  require(other.params().lat_accel_factor == 1.5 && other.params().decay == 50.0, "ignored cache leaves the prior");
  const std::string truncated = valid_cache.substr(0, valid_cache.size() - 1);
  std::string bad_magic = valid_cache;
  bad_magic[0] = 'X';
  require(TorqueEstimator(tuning, kTorqueLag, 5, truncated).restore_status() == TorqueRestore::Corrupt &&
              TorqueEstimator(tuning, kTorqueLag, 5, bad_magic).restore_status() == TorqueRestore::Corrupt &&
              TorqueEstimator(tuning, kTorqueLag, 5, "").restore_status() == TorqueRestore::None,
          "corrupt caches are reported for removal");
}

// ---------------------------------------------------------------- controlsd 연결

void verify_learners_glue() {
  const SteeringParams sp;
  VehicleCanState vehicle = ready_vehicle(1.0);
  vehicle.steering_angle_deg = 3.0f;
  vehicle.yaw_rate_valid = true;
  vehicle.yaw_rate_rad_s = 0.02f;
  vehicle.lat_accel_valid = true;
  vehicle.lat_accel_mps2 = -0.4f;  // 반전 저장: 좌측 비력 +0.4
  {
    LateralLearners l(sp, "", "", 1);
    require(!l.vehicle_restored() && !l.vehicle_restore_rejected() &&
                l.torque_restore_status() == TorqueRestore::None && !l.live().use_vehicle &&
                !l.live().use_torque,
            "fresh learners have nothing to offer yet");
    l.update(vehicle, 1.0, 0.5, true, 100, false);
    require(l.last_vehicle_input().inputs_fresh &&
                std::fabs(l.last_vehicle_input().lat_accel_mps2 - 0.4) < 1e-6 &&
                l.last_torque_input().lat_active,
            "paramsd inputs map sign and scale as the replay tool does");
    // 좌회전(보낸 토크 +, ESP12 요레이트 +)은 torqued에서 둘 다 음수다(우측 양수 관례)
    require(std::fabs(l.last_torque_input().steer_torque + 100.0 / sp.steer_max) < 1e-12 &&
                l.last_torque_input().yaw_rate_rad_s == -l.last_vehicle_input().yaw_rate_rad_s &&
                l.last_vehicle_input().yaw_rate_rad_s > 0.0,
            "torqued runs in the controller frame, so its offset matches torque_lat_accel_offset");
    require(l.vehicle_published() && l.torque_published() && l.vehicle_persist_due() &&
                l.torque_persist_due(),
            "the first tick publishes and persists both (upstream frame 0)");
    const LiveLateralParams live = l.live();
    require(live.use_vehicle && live.steer_ratio == sp.steer_ratio && live.use_torque &&
                live.lat_accel_factor == static_cast<float>(static_cast<float>(1.0 / sp.torque_kf())) &&
                live.friction == sp.torque_friction() && live.lat_accel_offset == 0.0f,
            "first messages carry the priors");
    // 봉투가 무효인 torqued 메시지는 토크 값을 바꾸지 않는다
    VehicleCanState stale = vehicle;
    for (long i = 1; i <= 40; ++i) l.update(stale, 1.0 + 0.01 * i + 5.0, 0.5, true, 100, false);
    require(!l.torque_params().inputs_ok && l.live().lat_accel_factor == live.lat_accel_factor,
            "stale torqued messages leave the applied values");

    // 저장 → 복원
    const std::string json = l.vehicle_persist_json();
    LateralLearners restored(sp, json, l.torque_cache(), 2);
    require(restored.vehicle_restored() && restored.torque_restore_status() == TorqueRestore::Restored,
            "both learners restore their own saves");
  }
  LateralLearners bad(sp, "{ not json", std::string("junk"), 3);
  require(bad.vehicle_restore_rejected() && !bad.vehicle_restored() &&
              bad.torque_restore_status() == TorqueRestore::Corrupt,
          "rejected saves are reported so controlsd removes them");
}

}  // namespace

int main() {
  return run_checks("lateral learners: ok", [] {
    verify_jacobian();
    verify_converges_and_stays_positive_definite();
    verify_upstream_schedule_reads_steer_ratio_low();
    verify_gates_hold_state();
    verify_output_limits_and_hysteresis();
    verify_persistence();
    verify_yaw_bias();
    verify_torque_exact_line();
    verify_torque_learns_parallelogram();
    verify_torque_clips_to_prior_band();
    verify_torque_gates();
    verify_torque_schedule_and_cache();
    verify_learners_glue();
  });
}
