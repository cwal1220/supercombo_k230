#include "lateral_learners.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "utils_json.h"
#include "utils_time.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;
constexpr double kDtMdl = 0.05;
double rad(double deg) { return deg * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }

// opendbc VehicleDynamicsParams(Civic 기준 스케일)
constexpr double kStdCargoKg = 136.0;
constexpr double kCivicMass = 1326.0 + kStdCargoKg;
constexpr double kCivicWheelbase = 2.70;
constexpr double kCivicCenterToFront = kCivicWheelbase * 0.4;
constexpr double kCivicCenterToRear = kCivicWheelbase - kCivicCenterToFront;
constexpr double kCivicInertia = 2500.0;
constexpr double kCivicStiffnessFront = 192150.0;
constexpr double kCivicStiffnessRear = 202500.0;

// paramsd.py
constexpr double kMaxAngleOffsetDelta = 20.0 * kDtMdl;  // deg/출력
constexpr double kRollMaxDelta = 20.0 * kPi / 180.0 * kDtMdl;
constexpr double kRollMin = -10.0 * kPi / 180.0;
constexpr double kRollMax = 10.0 * kPi / 180.0;
constexpr double kRollLoweredMax = 8.0 * kPi / 180.0;
constexpr double kRollStdMax = 1.5 * kPi / 180.0;
constexpr double kLateralAccSensorThreshold = 4.0;
constexpr double kOffsetMax = 10.0;
constexpr double kOffsetLoweredMax = 8.0;
constexpr double kMinActiveSpeed = 1.0;
constexpr double kLowActiveSpeed = 10.0;
constexpr int kPersistEveryFrames = 1200;
constexpr int kGearReverse = 7;  // ELECT_GEAR: P0 D5 N6 R7 S8

/* 입력 대체. 요레이트 std는 ESP12 고주파 잡음(0.06~0.08°/s)에 여유를 둔 값,
 * 롤은 횡가속 유도값이라 거의 정상상태(|u·r| 작음)에서만 관측한다. */
constexpr double kYawRateStd = 0.1 * kPi / 180.0;
constexpr double kRollStd = 1.0 * kPi / 180.0;
constexpr double kRollMinSpeed = 5.0;
constexpr double kRollMaxCentripetal = 0.5;
constexpr double kPredictStepS = 0.001;
constexpr int kPersistVersion = 1;

bool check_valid_with_hysteresis(bool current_valid, double val, double threshold,
                                 double lowered_threshold) {
  return std::fabs(val) < (current_valid ? threshold : lowered_threshold);
}

double clip(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

bool near_constant(double a, double b) { return std::fabs(a - b) <= 1e-3 * std::max(1.0, std::fabs(b)); }

}  // namespace

// ---------------------------------------------------------------- CarKalman

CarKalman::Vec CarKalman::initial_x() {
  return {1.0, 15.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0};
}

CarKalman::Mat CarKalman::q() {
  const double d[kN] = {std::pow(0.05 / 100.0, 2), std::pow(0.01, 2), std::pow(rad(0.02), 2),
                        std::pow(rad(0.25), 2), std::pow(0.1, 2), std::pow(0.01, 2),
                        std::pow(rad(0.1), 2), std::pow(rad(0.1), 2), std::pow(rad(1.0), 2)};
  Mat m{};
  for (int i = 0; i < kN; ++i) m[i][i] = d[i];
  return m;
}

CarKalman::Globals CarKalman::globals_from(const VehicleModelConstants &c) {
  Globals g;
  const double l = c.wheelbase_m;
  g.mass = c.mass_kg;
  g.center_to_front = c.center_to_front_m;
  g.center_to_rear = l - c.center_to_front_m;
  g.inertia = kCivicInertia * c.mass_kg * l * l / (kCivicMass * kCivicWheelbase * kCivicWheelbase);
  g.stiffness_front = kCivicStiffnessFront * c.tire_stiffness_factor * c.mass_kg / kCivicMass *
                      (g.center_to_rear / l) / (kCivicCenterToRear / kCivicWheelbase);
  g.stiffness_rear = kCivicStiffnessRear * c.tire_stiffness_factor * c.mass_kg / kCivicMass *
                     (g.center_to_front / l) / (kCivicCenterToFront / kCivicWheelbase);
  return g;
}

/* Guiggiani 7.211-7.213 선형 단일트랙. 횡속도·요레이트만 움직이고 나머지는 랜덤워크. */
void CarKalman::step(const Globals &g, const Vec &x, double h, Vec *x_next, Mat *f) {
  const double sf = x[kStiffness], sr = x[kSteerRatio], u = x[kSpeedX];
  const double v = x[kSpeedY], r = x[kYawRate], th = x[kRoadRoll];
  const double e = x[kSteerAngle] - x[kAngleOffset] - x[kAngleOffsetFast];
  const double m = g.mass, j = g.inertia, af = g.center_to_front, ar = g.center_to_rear;
  const double cf0 = g.stiffness_front, cr0 = g.stiffness_rear;
  const double csum = cf0 + cr0, d1 = cf0 * af - cr0 * ar, d2 = cf0 * af * af + cr0 * ar * ar;

  const double a00 = -sf * csum / (m * u), a01 = -sf * d1 / (m * u) - u;
  const double a10 = -sf * d1 / (j * u), a11 = -sf * d2 / (j * u);
  const double b0 = sf * cf0 / (m * sr), b1 = sf * cf0 * af / (j * sr);

  *x_next = x;
  (*x_next)[kSpeedY] += h * (a00 * v + a01 * r + b0 * e - kGravity * th);
  (*x_next)[kYawRate] += h * (a10 * v + a11 * r + b1 * e);

  Mat jac{};
  jac[kSpeedY][kStiffness] = (-csum / (m * u)) * v + (-d1 / (m * u)) * r + (cf0 / (m * sr)) * e;
  jac[kSpeedY][kSteerRatio] = -b0 * e / sr;
  jac[kSpeedY][kAngleOffset] = -b0;
  jac[kSpeedY][kAngleOffsetFast] = -b0;
  jac[kSpeedY][kSpeedX] = (sf * csum / (m * u * u)) * v + (sf * d1 / (m * u * u) - 1.0) * r;
  jac[kSpeedY][kSpeedY] = a00;
  jac[kSpeedY][kYawRate] = a01;
  jac[kSpeedY][kSteerAngle] = b0;
  jac[kSpeedY][kRoadRoll] = -kGravity;
  jac[kYawRate][kStiffness] = (-d1 / (j * u)) * v + (-d2 / (j * u)) * r + (cf0 * af / (j * sr)) * e;
  jac[kYawRate][kSteerRatio] = -b1 * e / sr;
  jac[kYawRate][kAngleOffset] = -b1;
  jac[kYawRate][kAngleOffsetFast] = -b1;
  jac[kYawRate][kSpeedX] = (sf * d1 / (j * u * u)) * v + (sf * d2 / (j * u * u)) * r;
  jac[kYawRate][kSpeedY] = a10;
  jac[kYawRate][kYawRate] = a11;
  jac[kYawRate][kSteerAngle] = b1;
  for (int i = 0; i < kN; ++i)
    for (int k = 0; k < kN; ++k) (*f)[i][k] = (i == k ? 1.0 : 0.0) + h * jac[i][k];
}

void CarKalman::init(const Vec &x, const Mat &p, bool has_time, double t) {
  x_ = x;
  p_ = p;
  has_time_ = has_time;
  t_ = t;
}

void CarKalman::predict(double dt) {
  static const Mat qm = q();
  const int steps = std::max(1, static_cast<int>(std::ceil(dt / kPredictStepS - 1e-9)));
  const double h = dt / steps;
  for (int s = 0; s < steps; ++s) {
    Vec xn;
    Mat f;
    step(g_, x_, h, &xn, &f);
    Mat fp{};
    for (int i = 0; i < kN; ++i)
      for (int k = 0; k < kN; ++k) {
        double acc = 0.0;
        for (int l = 0; l < kN; ++l) acc += f[i][l] * p_[l][k];
        fp[i][k] = acc;
      }
    for (int i = 0; i < kN; ++i)
      for (int k = 0; k < kN; ++k) {
        double acc = 0.0;
        for (int l = 0; l < kN; ++l) acc += fp[i][l] * f[k][l];
        p_[i][k] = acc + h * qm[i][k];
      }
    x_ = xn;
  }
}

/* Joseph 형식(rednose): P = (I−KH)P(I−KH)ᵀ + K R Kᵀ, H = e_state. */
void CarKalman::predict_and_observe(double t, int state, double z, double r) {
  if (!has_time_) {
    set_time(t);
  } else if (t > t_) {
    predict(t - t_);
    t_ = t;
  }
  const double s = p_[state][state] + r;
  Vec k;
  for (int i = 0; i < kN; ++i) k[i] = p_[i][state] / s;
  const double y = z - x_[state];
  for (int i = 0; i < kN; ++i) x_[i] += k[i] * y;
  Mat a;  // (I − K e_sᵀ) P
  for (int i = 0; i < kN; ++i)
    for (int c = 0; c < kN; ++c) a[i][c] = p_[i][c] - k[i] * p_[state][c];
  for (int i = 0; i < kN; ++i)
    for (int c = 0; c < kN; ++c) p_[i][c] = a[i][c] - a[i][state] * k[c] + r * k[i] * k[c];
}

// ---------------------------------------------------------------- VehicleParamsLearner

VehicleParamsLearner::VehicleParamsLearner(const VehicleModelConstants &c, double steer_ratio,
                                           double stiffness_factor, double angle_offset_rad,
                                           const VehicleParamsOptions &options)
    : c_(c), car_state_every_tick_(options.car_state_every_tick),
      min_sr_(0.5 * c.steer_ratio), max_sr_(2.0 * c.steer_ratio),
      kf_(CarKalman::globals_from(c)) {
  x_initial_ = CarKalman::initial_x();
  x_initial_[CarKalman::kSteerRatio] = steer_ratio;
  x_initial_[CarKalman::kStiffness] = stiffness_factor;
  x_initial_[CarKalman::kAngleOffset] = angle_offset_rad;
  p_initial_ = options.p_initial ? *options.p_initial : CarKalman::q();
  reset(false, 0.0);
}

void VehicleParamsLearner::reset(bool has_time, double t) {
  kf_.init(x_initial_, p_initial_, has_time, t);
  angle_offset_deg_ = deg(x_initial_[CarKalman::kAngleOffset]);
  roll_ = 0.0;
  active_ = false;
  avg_angle_offset_deg_ = angle_offset_deg_;
}

void VehicleParamsLearner::handle_car_state(const VehicleParamsInput &in) {
  const bool in_linear_region = std::fabs(in.steering_angle_deg) < 45.0;
  observed_speed_ = in.speed_mps;
  active_ = observed_speed_ > kMinActiveSpeed && in_linear_region && in.gear != kGearReverse;
  if (active_) {
    kf_.predict_and_observe(in.t_s, CarKalman::kSteerAngle, rad(in.steering_angle_deg),
                            std::pow(rad(0.05), 2));
    kf_.predict_and_observe(in.t_s, CarKalman::kSpeedX, observed_speed_, std::pow(0.1, 2));
  }
}

void VehicleParamsLearner::handle_device_motion(const VehicleParamsInput &in) {
  double yaw_rate = in.yaw_rate_rad_s, yaw_rate_std = kYawRateStd;
  const bool yaw_rate_valid = in.yaw_rate_valid && yaw_rate_std > 0.0 && yaw_rate_std < 10.0 &&
                              std::fabs(yaw_rate) < 1.0;
  if (!yaw_rate_valid) {
    yaw_rate = 0.0;
    yaw_rate_std = rad(10.0);
  }
  observed_yaw_rate_ = yaw_rate;

  const double centripetal = in.speed_mps * yaw_rate;
  double roll = 0.0, roll_std = rad(10.0);
  if (in.lat_accel_valid && in.speed_mps > kRollMinSpeed &&
      std::fabs(centripetal) < kRollMaxCentripetal) {
    const double candidate = std::asin(clip((in.lat_accel_mps2 - centripetal) / kGravity, -1.0, 1.0));
    if (kRollStd < kRollStdMax && candidate > kRollMin && candidate < kRollMax) {
      roll = candidate;
      roll_std = kRollStd;
    }
  }
  observed_roll_ = clip(roll, observed_roll_ - kRollMaxDelta, observed_roll_ + kRollMaxDelta);

  if (active_) {
    // 상류는 −자세 요레이트를 관측한다. ESP12는 이미 좌측 양수라 그대로.
    kf_.predict_and_observe(in.t_s, CarKalman::kYawRate, observed_yaw_rate_,
                            yaw_rate_std * yaw_rate_std);
    kf_.predict_and_observe(in.t_s, CarKalman::kRoadRoll, observed_roll_, roll_std * roll_std);
    kf_.predict_and_observe(in.t_s, CarKalman::kAngleOffsetFast, 0.0, std::pow(rad(10.0), 2));
    // 자기관측: 값을 당기지 않고 긴 직선에서 공분산이 무한히 커지는 것만 막는다.
    const double stiffness = kf_.x()[CarKalman::kStiffness];
    const double steer_ratio = kf_.x()[CarKalman::kSteerRatio];
    kf_.predict_and_observe(in.t_s, CarKalman::kStiffness, stiffness, 0.5 * 0.5);
    kf_.predict_and_observe(in.t_s, CarKalman::kSteerRatio, steer_ratio, 5.0 * 5.0);
  }
}

bool VehicleParamsLearner::update(const VehicleParamsInput &in) {
  persist_due_ = false;
  /* 요레이트·롤 관측과 출력은 deviceMotion처럼 20 Hz. 조향각·속도는 기본 매 틱이고,
   * 상류 스케줄이면 그 20 Hz 틱의 최신값 하나만 본다(conflate). */
  const bool motion_due = !has_motion_t_ || in.t_s - last_motion_t_ >= kDtMdl - 1e-3;
  if (!motion_due && !car_state_every_tick_) return false;
  if (in.inputs_fresh) {
    handle_car_state(in);
    if (!active_) kf_.set_time(in.t_s);
    if (motion_due) {
      handle_device_motion(in);
      if (!active_) kf_.set_time(in.t_s);
    }
  } else {
    // 상류는 낡은 입력을 버리고 시각도 멈춘다. 재개 시 긴 dt 예측이 발산하므로 비활성처럼 둔다.
    kf_.set_time(in.t_s);
  }
  if (!motion_due) return false;
  has_motion_t_ = true;
  last_motion_t_ = in.t_s;
  ++motion_frame_;
  last_ = get_msg(in.inputs_fresh);
  persist_due_ = motion_frame_ % kPersistEveryFrames == 0;
  return true;
}

VehicleParams VehicleParamsLearner::get_msg(bool inputs_ok) {
  bool finite = true;
  for (double v : kf_.x()) finite = finite && std::isfinite(v);
  if (!finite) {
    std::fprintf(stderr, "vehicle params: NaN in estimate, resetting\n");
    reset(true, kf_.time());
  }
  const CarKalman::Vec &x = kf_.x();
  const CarKalman::Mat &p = kf_.p();

  avg_angle_offset_deg_ = clip(deg(x[CarKalman::kAngleOffset]),
                               avg_angle_offset_deg_ - kMaxAngleOffsetDelta,
                               avg_angle_offset_deg_ + kMaxAngleOffsetDelta);
  angle_offset_deg_ = clip(deg(x[CarKalman::kAngleOffset] + x[CarKalman::kAngleOffsetFast]),
                           angle_offset_deg_ - kMaxAngleOffsetDelta,
                           angle_offset_deg_ + kMaxAngleOffsetDelta);
  roll_ = clip(x[CarKalman::kRoadRoll], roll_ - kRollMaxDelta, roll_ + kRollMaxDelta);
  const double roll_std = std::sqrt(p[CarKalman::kRoadRoll][CarKalman::kRoadRoll]);

  bool sensors_valid = true;
  if (active_ && observed_speed_ > kLowActiveSpeed) {
    // 상류는 두 요레이트의 부호가 반대라 더한다. 여기서는 같은 부호라 뺀다.
    sensors_valid = std::fabs(observed_speed_ * (x[CarKalman::kYawRate] - observed_yaw_rate_)) <
                    kLateralAccSensorThreshold;
  }
  avg_offset_valid_ = check_valid_with_hysteresis(avg_offset_valid_, avg_angle_offset_deg_,
                                                  kOffsetMax, kOffsetLoweredMax);
  total_offset_valid_ = check_valid_with_hysteresis(total_offset_valid_, angle_offset_deg_,
                                                    kOffsetMax, kOffsetLoweredMax);
  roll_valid_ = check_valid_with_hysteresis(roll_valid_, roll_, kRollMax, kRollLoweredMax);

  VehicleParams out;
  out.inputs_ok = inputs_ok;
  out.sensor_valid = sensors_valid;
  out.steer_ratio = x[CarKalman::kSteerRatio];
  out.stiffness_factor = x[CarKalman::kStiffness];
  out.roll_rad = roll_;
  out.angle_offset_average_deg = avg_angle_offset_deg_;
  out.angle_offset_deg = angle_offset_deg_;
  out.steer_ratio_valid = min_sr_ <= out.steer_ratio && out.steer_ratio <= max_sr_;
  out.stiffness_factor_valid = 0.2 <= out.stiffness_factor && out.stiffness_factor <= 5.0;
  out.angle_offset_average_valid = avg_offset_valid_;
  out.angle_offset_valid = total_offset_valid_;
  out.valid = out.angle_offset_average_valid && out.angle_offset_valid && roll_valid_ &&
              roll_std < kRollStdMax && out.stiffness_factor_valid && out.steer_ratio_valid;
  out.steer_ratio_std = std::sqrt(p[CarKalman::kSteerRatio][CarKalman::kSteerRatio]);
  out.stiffness_factor_std = std::sqrt(p[CarKalman::kStiffness][CarKalman::kStiffness]);
  out.angle_offset_average_std = std::sqrt(p[CarKalman::kAngleOffset][CarKalman::kAngleOffset]);
  out.angle_offset_fast_std =
      std::sqrt(p[CarKalman::kAngleOffsetFast][CarKalman::kAngleOffsetFast]);
  return out;
}

// ---------------------------------------------------------------- 저장/복원

std::string persist_vehicle_params(const VehicleParams &p, const VehicleModelConstants &c,
                                   double yaw_bias_rad_s) {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "{\n  \"version\": %d,\n  \"mass_kg\": %.6g,\n  \"wheelbase_m\": %.6g,\n"
                "  \"center_to_front_m\": %.6g,\n  \"tire_stiffness_factor\": %.6g,\n"
                "  \"steer_ratio\": %.8g,\n  \"stiffness_factor\": %.8g,\n"
                "  \"angle_offset_average_deg\": %.8g,\n  \"yaw_bias_rad_s\": %.8g\n}\n",
                kPersistVersion, c.mass_kg, c.wheelbase_m, c.center_to_front_m,
                c.tire_stiffness_factor, p.steer_ratio, p.stiffness_factor,
                p.angle_offset_average_deg, yaw_bias_rad_s);
  return buf;
}

VehicleParamsInit restore_vehicle_params(const std::string &json, const VehicleModelConstants &c) {
  VehicleParamsInit init;
  init.steer_ratio = c.steer_ratio;
  float version = 0, mass = 0, wheelbase = 0, cf = 0, tsf = 0, sr = 0, sf = 0, offset = 0, bias = 0;
  const bool parsed = parse_json_float_value(json, "version", &version) &&
                      parse_json_float_value(json, "mass_kg", &mass) &&
                      parse_json_float_value(json, "wheelbase_m", &wheelbase) &&
                      parse_json_float_value(json, "center_to_front_m", &cf) &&
                      parse_json_float_value(json, "tire_stiffness_factor", &tsf) &&
                      parse_json_float_value(json, "steer_ratio", &sr) &&
                      parse_json_float_value(json, "stiffness_factor", &sf) &&
                      parse_json_float_value(json, "angle_offset_average_deg", &offset) &&
                      parse_json_float_value(json, "yaw_bias_rad_s", &bias);
  // 상류 지문 대조 자리. 단일 차종이라 차량 상수가 같으면 같은 차로 본다.
  const bool same_car = parsed && static_cast<int>(version) == kPersistVersion &&
                        near_constant(mass, c.mass_kg) && near_constant(wheelbase, c.wheelbase_m) &&
                        near_constant(cf, c.center_to_front_m) &&
                        near_constant(tsf, c.tire_stiffness_factor);
  const bool sane = same_car && std::isfinite(sr) && 0.5 * c.steer_ratio <= sr &&
                    sr <= 2.0 * c.steer_ratio && std::isfinite(offset) && std::isfinite(bias);
  if (!sane) return init;
  init.restored = true;
  init.steer_ratio = sr;
  init.angle_offset_deg = offset;
  init.yaw_bias_rad_s = bias;
  init.stiffness_factor = 1.0;
  return init;
}

// ---------------------------------------------------------------- YawBiasEstimator

double YawBiasEstimator::update(double t_s, double speed_mps, bool yaw_valid,
                                double yaw_rate_rad_s) {
  constexpr double kStandstillMps = 0.05;
  constexpr double kSettleS = 2.0;
  constexpr double kMaxStandstillYaw = 0.5 * kPi / 180.0;
  constexpr double kAlpha = 0.01 / (2.0 + 0.01);  // 100 Hz, 2초
  const bool standing = std::fabs(speed_mps) < kStandstillMps;
  if (standing && !standing_) standing_since_s_ = t_s;
  standing_ = standing;
  if (standing && t_s - standing_since_s_ >= kSettleS && yaw_valid &&
      std::isfinite(yaw_rate_rad_s) && std::fabs(yaw_rate_rad_s - bias_) < kMaxStandstillYaw)
    bias_ += kAlpha * (yaw_rate_rad_s - bias_);
  return yaw_rate_rad_s - bias_;
}

// ---------------------------------------------------------------- TorqueEstimator

namespace {

// torqued.py
constexpr int kMinPointsTotal = 4000;
constexpr int kFitPointsTotal = 2000;
constexpr double kMinVel = 15.0;
constexpr double kFrictionFactor = 1.5;
constexpr double kFactorSanity = 0.3;
constexpr double kFrictionSanity = 0.5;
constexpr double kSteerMinThreshold = 0.02;
constexpr double kMinFilterDecay = 50.0;
constexpr double kMaxFilterDecay = 250.0;
constexpr double kLatAccThreshold = 1.0;
constexpr double kMinEngageBuffer = 2.0;
constexpr int kTorqueVersion = 1;
constexpr double kSteerBucketBounds[TorqueEstimator::kBuckets][2] = {
    {-0.5, -0.3}, {-0.3, -0.2}, {-0.2, -0.1}, {-0.1, 0.0},
    {0.0, 0.1},   {0.1, 0.2},   {0.2, 0.3},   {0.3, 0.5}};
constexpr int kMinBucketPoints[TorqueEstimator::kBuckets] = {100, 300, 500, 500,
                                                             500, 500, 300, 100};
constexpr char kTorqueCacheMagic[8] = {'K', '2', '3', '0', 'T', 'Q', 'C', '1'};

uint64_t splitmix64(uint64_t *state) {
  uint64_t z = (*state += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

/* 대칭 3×3의 최소 고유값 고유벡터(순환 야코비). [x,1,y]의 최소 특이벡터와 같다. */
std::array<double, 3> smallest_eigenvector(std::array<std::array<double, 3>, 3> a) {
  std::array<std::array<double, 3>, 3> v{};
  for (int i = 0; i < 3; ++i) v[i][i] = 1.0;
  for (int sweep = 0; sweep < 50; ++sweep) {
    if (a[0][1] == 0.0 && a[0][2] == 0.0 && a[1][2] == 0.0) break;
    for (int p = 0; p < 2; ++p) {
      for (int q = p + 1; q < 3; ++q) {
        if (a[p][q] == 0.0) continue;
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = (theta >= 0.0 ? 1.0 : -1.0) /
                         (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        for (int k = 0; k < 3; ++k) {
          const double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
          const double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        a[p][q] = a[q][p] = 0.0;
        for (int k = 0; k < 3; ++k) {
          const double vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  int m = 0;
  for (int i = 1; i < 3; ++i)
    if (a[i][i] < a[m][m]) m = i;
  return {v[0][m], v[1][m], v[2][m]};
}

template <class T> void put(std::string *out, const T &v) {
  out->append(reinterpret_cast<const char *>(&v), sizeof(v));
}
template <class T> bool take(const std::string &in, size_t *pos, T *v) {
  if (*pos + sizeof(T) > in.size()) return false;
  std::memcpy(v, in.data() + *pos, sizeof(T));
  *pos += sizeof(T);
  return true;
}

}  // namespace

TorqueEstimator::TorqueEstimator(const TorqueTuning &offline, double lag_s, uint64_t seed,
                                 const std::string &cache)
    : offline_(offline), lag_s_(lag_s), rng_(seed) {
  fit_points_.reserve(kBuckets * kPointsPerBucket);
  reset();
  decay_ = kMinFilterDecay;
  min_factor_ = (1.0 - kFactorSanity) * offline_.lat_accel_factor;
  max_factor_ = (1.0 + kFactorSanity) * offline_.lat_accel_factor;
  min_friction_ = (1.0 - kFrictionSanity) * offline_.friction;
  max_friction_ = (1.0 + kFrictionSanity) * offline_.friction;
  double factor = offline_.lat_accel_factor, offset = 0.0, friction = offline_.friction;
  if (!cache.empty()) restore_ = restore(cache, &factor, &offset, &friction);
  const double alpha = kDtMdl / (decay_ + kDtMdl);
  factor_f_ = {factor, alpha};
  offset_f_ = {offset, alpha};
  friction_f_ = {friction, alpha};
}

void TorqueEstimator::reset() {
  resets_ += 1.0;
  decay_ = kMinFilterDecay;
  raw_head_ = raw_count_ = 0;
  for (Bucket &b : buckets_) b.head = b.count = 0;
}

void TorqueEstimator::add_point(double x, double y) {
  for (int i = 0; i < kBuckets; ++i) {
    if (x >= kSteerBucketBounds[i][0] && x < kSteerBucketBounds[i][1]) {
      Bucket &b = buckets_[i];
      if (b.count < kPointsPerBucket) {
        b.points[(b.head + b.count) % kPointsPerBucket] = {x, y};
        ++b.count;
      } else {
        b.points[b.head] = {x, y};
        b.head = (b.head + 1) % kPointsPerBucket;
      }
      break;
    }
  }
}

int TorqueEstimator::total_points() const {
  int n = 0;
  for (const Bucket &b : buckets_) n += b.count;
  return n;
}

std::vector<std::array<double, 2>> TorqueEstimator::points() const {
  std::vector<std::array<double, 2>> out;
  out.reserve(total_points());
  for (const Bucket &b : buckets_)
    for (int i = 0; i < b.count; ++i) out.push_back(b.at(i));
  return out;
}

bool TorqueEstimator::is_calculable() const {
  for (const Bucket &b : buckets_)
    if (b.count == 0) return false;
  return true;
}

bool TorqueEstimator::is_valid() const {
  for (int i = 0; i < kBuckets; ++i)
    if (buckets_[i].count < kMinBucketPoints[i]) return false;
  return total_points() >= kMinPointsTotal;
}

int TorqueEstimator::valid_percent() const {
  const double total = std::min(static_cast<double>(total_points()) / kMinPointsTotal * 100.0, 100.0);
  double individual = 1e300;
  for (int i = 0; i < kBuckets; ++i)
    individual = std::min(individual,
                          static_cast<double>(buckets_[i].count) / kMinBucketPoints[i] * 100.0);
  individual = std::min(individual, 100.0);
  return static_cast<int>((total + individual) / 2.0);
}

void TorqueEstimator::estimate_params(double *slope, double *offset, double *friction) {
  fit_points_.clear();  // 예약해 둔 버퍼를 재사용한다
  for (const Bucket &b : buckets_)
    for (int i = 0; i < b.count; ++i) fit_points_.push_back(b.at(i));
  const int total = static_cast<int>(fit_points_.size());
  int n = total;
  if (!fit_all_points_ && total > kFitPointsTotal) {  // rng.choice(replace=False): 부분 셔플
    n = kFitPointsTotal;
    for (int i = 0; i < n; ++i) {
      const int j = i + static_cast<int>(splitmix64(&rng_) % static_cast<uint64_t>(total - i));
      std::swap(fit_points_[i], fit_points_[j]);
    }
  }
  // 총최소제곱: [x, 1, y]의 최소 특이벡터 n에서 y = −(n0·x + n1)/n2
  std::array<std::array<double, 3>, 3> ata{};
  for (int i = 0; i < n; ++i) {
    const double r[3] = {fit_points_[i][0], 1.0, fit_points_[i][1]};
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b) ata[a][b] += r[a] * r[b];
  }
  const std::array<double, 3> v = smallest_eigenvector(ata);
  *slope = -v[0] / v[2];
  *offset = -v[1] / v[2];
  // slope2rot은 |기울기|로 돌린다(상류 그대로). 흩어짐 = 직선에 수직인 성분
  const double sin = std::sqrt(*slope * *slope / (*slope * *slope + 1.0));
  const double cos = std::sqrt(1.0 / (*slope * *slope + 1.0));
  double mean = 0.0;
  for (int i = 0; i < n; ++i) mean += -fit_points_[i][0] * sin + fit_points_[i][1] * cos;
  mean /= n;
  double var = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = -fit_points_[i][0] * sin + fit_points_[i][1] * cos - mean;
    var += d * d;
  }
  *friction = std::sqrt(var / n) * kFrictionFactor;
}

void TorqueEstimator::update_params(double factor, double offset, double friction) {
  decay_ = std::min(decay_ + kDtMdl, kMaxFilterDecay);
  const double alpha = kDtMdl / (decay_ + kDtMdl);
  Filter *filters[3] = {&factor_f_, &offset_f_, &friction_f_};
  const double values[3] = {factor, offset, friction};
  for (int i = 0; i < 3; ++i) {
    filters[i]->x = (1.0 - filters[i]->alpha) * filters[i]->x + filters[i]->alpha * values[i];
    filters[i]->alpha = alpha;
  }
}

double TorqueEstimator::interp_raw(double x, double RawSample::*field) const {  // np.interp
  auto at = [&](int i) -> const RawSample & { return raw_[(raw_head_ + i) % kHistLen]; };
  const int n = raw_count_;
  if (x < at(0).t) return at(0).*field;
  if (x >= at(n - 1).t) return at(n - 1).*field;
  int lo = 0, hi = n - 1;  // at(lo).t <= x < at(hi).t
  while (hi - lo > 1) {
    const int mid = (lo + hi) / 2;
    if (at(mid).t <= x) lo = mid;
    else hi = mid;
  }
  const RawSample &a = at(lo), &b = at(hi);
  if (a.t == x) return a.*field;
  const double slope = (b.*field - a.*field) / (b.t - a.t);
  return slope * (x - a.t) + a.*field;
}

void TorqueEstimator::handle_device_motion(const TorqueEstimatorInput &in) {
  if (raw_count_ < kHistLen || !in.pose_valid) return;
  const double t = in.t_s;
  // np.arange(t − MIN_ENGAGE_BUFFER, t + lag, DT_MDL): 이력은 지연만큼 밀려 있으니 지금까지의 활성
  const double start = t - kMinEngageBuffer;
  const long n = static_cast<long>(std::ceil((t + lag_s_ - start) / kDtMdl));
  const double delta = (start + kDtMdl) - start;
  bool all_active = true, any_override = false;
  for (long i = 0; i < n; ++i) {
    const double ti = i == 0 ? start : i == 1 ? start + kDtMdl : start + i * delta;
    all_active = all_active && interp_raw(ti, &RawSample::lat_active) != 0.0;
    any_override = any_override || interp_raw(ti, &RawSample::steer_override) != 0.0;
  }
  const double vego = interp_raw(t, &RawSample::vego);
  const double steer = interp_raw(t, &RawSample::steer);
  const double lateral_acc = vego * in.yaw_rate_rad_s - std::sin(in.roll_rad) * kGravity;
  if (all_active && !any_override && vego > kMinVel && std::fabs(steer) > kSteerMinThreshold &&
      std::fabs(lateral_acc) <= kLatAccThreshold)
    add_point(steer, lateral_acc);
}

bool TorqueEstimator::update(const TorqueEstimatorInput &in) {
  persist_due_ = false;
  const bool motion_due = !has_motion_t_ || in.t_s - last_motion_t_ >= kDtMdl - 1e-3;
  if (!motion_due) return false;
  has_motion_t_ = true;
  last_motion_t_ = in.t_s;
  ++frame_;
  if (in.inputs_fresh) {
    // 상류는 conflate된 carControl·carOutput·carState 최신값을 deviceMotion보다 먼저 넣는다
    RawSample &s = raw_[(raw_head_ + raw_count_) % kHistLen];
    s = {in.t_s + lag_s_, in.lat_active ? 1.0 : 0.0, in.steer_torque, in.speed_mps,
         in.steer_override ? 1.0 : 0.0};
    if (raw_count_ < kHistLen) ++raw_count_;
    else raw_head_ = (raw_head_ + 1) % kHistLen;
    handle_device_motion(in);
  }
  const bool publish = frame_ % 5 == 0;
  if (publish) last_ = get_msg(in.inputs_fresh);
  if (frame_ % 240 == 0) {
    cache_ = serialize(get_msg(in.inputs_fresh));
    persist_due_ = true;
  }
  return publish;
}

TorqueParams TorqueEstimator::get_msg(bool inputs_ok) {
  TorqueParams m;
  m.inputs_ok = inputs_ok;
  m.use_params = true;  // 현대 + 토크 제어
  if (is_calculable()) {
    double factor = 0.0, offset = 0.0, friction = 0.0;
    estimate_params(&factor, &offset, &friction);
    m.lat_accel_factor_raw = factor;
    m.lat_accel_offset_raw = offset;
    m.friction_raw = friction;
    if (is_valid()) {
      if (std::isnan(factor) || std::isnan(offset) || std::isnan(friction)) {
        std::fprintf(stderr, "torque params: NaN in estimate, resetting\n");
        m.valid = false;
        reset();
      } else {
        m.valid = true;
        update_params(clip(factor, min_factor_, max_factor_), offset,
                      clip(friction, min_friction_, max_friction_));
      }
    }
  }
  m.lat_accel_factor = factor_f_.x;
  m.lat_accel_offset = offset_f_.x;
  m.friction = friction_f_.x;
  m.total_bucket_points = total_points();
  m.cal_perc = valid_percent();
  m.decay = decay_;
  m.max_resets = resets_;
  return m;
}

/* 상류 LiveTorqueParameters(capnp Float32)와 같은 정밀도로 남긴다. 키는 상류 get_restore_key의
 * 튜닝값·버전(지문과 제어 종류는 단일 차종·토크 제어라 고정). */
std::string TorqueEstimator::serialize(const TorqueParams &p) const {
  std::string out(kTorqueCacheMagic, sizeof(kTorqueCacheMagic));
  put(&out, static_cast<int32_t>(kTorqueVersion));
  put(&out, static_cast<float>(offline_.friction));
  put(&out, static_cast<float>(offline_.lat_accel_factor));
  put(&out, static_cast<uint8_t>(p.valid ? 1 : 0));
  put(&out, static_cast<float>(p.lat_accel_factor));
  put(&out, static_cast<float>(p.lat_accel_offset));
  put(&out, static_cast<float>(p.friction));
  put(&out, static_cast<float>(p.decay));
  put(&out, static_cast<uint32_t>(total_points()));
  for (const Bucket &b : buckets_) {
    for (int i = 0; i < b.count; ++i) {
      put(&out, static_cast<float>(b.at(i)[0]));
      put(&out, static_cast<float>(b.at(i)[1]));
    }
  }
  return out;
}

TorqueRestore TorqueEstimator::restore(const std::string &cache, double *factor, double *offset,
                                       double *friction) {
  size_t pos = sizeof(kTorqueCacheMagic);
  int32_t version = 0;
  float key_friction = 0, key_factor = 0, f_factor = 0, f_offset = 0, f_friction = 0, decay = 0;
  uint8_t valid = 0;
  uint32_t n = 0;
  if (cache.size() < pos || std::memcmp(cache.data(), kTorqueCacheMagic, pos) != 0 ||
      !take(cache, &pos, &version) || !take(cache, &pos, &key_friction) ||
      !take(cache, &pos, &key_factor) || !take(cache, &pos, &valid) ||
      !take(cache, &pos, &f_factor) || !take(cache, &pos, &f_offset) ||
      !take(cache, &pos, &f_friction) || !take(cache, &pos, &decay) || !take(cache, &pos, &n) ||
      cache.size() != pos + static_cast<size_t>(n) * 2 * sizeof(float))
    return TorqueRestore::Corrupt;
  if (version != kTorqueVersion || key_friction != static_cast<float>(offline_.friction) ||
      key_factor != static_cast<float>(offline_.lat_accel_factor))
    return TorqueRestore::KeyMismatch;
  if (valid) {
    *factor = f_factor;
    *offset = f_offset;
    *friction = f_friction;
  }
  for (uint32_t i = 0; i < n; ++i) {
    float x = 0, y = 0;
    take(cache, &pos, &x);
    take(cache, &pos, &y);
    add_point(x, y);
  }
  decay_ = decay;
  return TorqueRestore::Restored;
}

// ---------------------------------------------------------------- LateralLearners

namespace {

VehicleParamsInit restore_or_prior(const std::string &json, const VehicleModelConstants &c) {
  if (!json.empty()) return restore_vehicle_params(json, c);
  VehicleParamsInit init;
  init.steer_ratio = c.steer_ratio;
  return init;
}

TorqueTuning torque_tuning(const SteeringParams &params) {
  TorqueTuning t;  // CP 값은 Float32
  t.lat_accel_factor = static_cast<float>(1.0 / params.torque_kf());
  t.friction = params.torque_friction();
  return t;
}

}  // namespace

VehicleModelConstants LateralLearners::constants(const SteeringParams &params) {
  VehicleModelConstants c;
  c.mass_kg = params.mass_kg;
  c.wheelbase_m = params.wheelbase_m;
  c.center_to_front_m = params.center_to_front_m();
  c.tire_stiffness_factor = params.tire_stiffness_factor;
  c.steer_ratio = params.steer_ratio;
  return c;
}

LateralLearners::LateralLearners(const SteeringParams &params, const std::string &vehicle_json,
                                 const std::string &torque_cache, uint64_t seed,
                                 const VehicleParamsOptions &options)
    : constants_(constants(params)),
      init_(restore_or_prior(vehicle_json, constants_)),
      vehicle_restored_(init_.restored),
      vehicle_restore_rejected_(!vehicle_json.empty() && !init_.restored),
      bias_(init_.yaw_bias_rad_s),
      vehicle_(constants_, init_.steer_ratio, init_.stiffness_factor, rad(init_.angle_offset_deg),
               options),
      torque_(torque_tuning(params), params.steer_actuator_delay, seed, torque_cache),
      steer_max_(std::max(1, params.steer_max)),
      output_sign_(params.torque_output_sign >= 0 ? 1 : -1) {}

void LateralLearners::update(const VehicleCanState &vehicle, double now_s, double timeout_s,
                             bool lat_active, int apply_torque, bool steering_pressed) {
  const float speed_kph = vehicle_speed_kph(vehicle, now_s, timeout_s);
  const bool esp_fresh = signal_time_fresh(vehicle.esp12_time_s, now_s, timeout_s);
  VehicleParamsInput in;
  in.t_s = now_s;
  in.inputs_fresh = vehicle_state_fresh(vehicle, now_s, timeout_s) && esp_fresh &&
                    std::isfinite(speed_kph);
  in.steering_angle_deg = vehicle.steering_angle_deg;
  in.speed_mps = std::isfinite(speed_kph) ? speed_kph / 3.6 : 0.0;
  in.gear = vehicle.gear;
  in.yaw_rate_valid = esp_fresh && vehicle.yaw_rate_valid;
  in.yaw_rate_rad_s = bias_.update(now_s, in.speed_mps, in.yaw_rate_valid, vehicle.yaw_rate_rad_s);
  in.lat_accel_valid = esp_fresh && vehicle.lat_accel_valid;
  in.lat_accel_mps2 = -vehicle.lat_accel_mps2;  // 반전 저장돼 있다
  last_vehicle_input_ = in;
  vehicle_published_ = vehicle_.update(in);
  const VehicleParams &vp = vehicle_.params();
  if (vehicle_published_) {
    live_.use_vehicle = true;
    live_.steer_ratio = static_cast<float>(vp.steer_ratio);
    live_.stiffness_factor = static_cast<float>(vp.stiffness_factor);
    live_.angle_offset_deg = static_cast<float>(vp.angle_offset_deg);
    live_.roll_rad = static_cast<float>(vp.roll_rad);
  }

  /* torqued는 컨트롤러 관례(우측 양수)로 돌린다. 보낸 토크는 출력 부호를 되돌리고
   * ESP12 요레이트(좌측 양수)는 뒤집는다. 그래야 학습 절편이 torque_lat_accel_offset과
   * 같은 부호다. */
  TorqueEstimatorInput tin;
  tin.t_s = now_s;
  tin.inputs_fresh = in.inputs_fresh;
  tin.lat_active = lat_active;
  tin.steer_torque = static_cast<double>(output_sign_ * apply_torque) / steer_max_;
  tin.speed_mps = in.speed_mps;
  tin.steer_override = steering_pressed;
  tin.pose_valid = in.yaw_rate_valid;
  tin.yaw_rate_rad_s = -in.yaw_rate_rad_s;
  tin.roll_rad = vp.roll_rad;
  last_torque_input_ = tin;
  torque_published_ = torque_.update(tin);
  const TorqueParams &tp = torque_.params();
  if (torque_published_ && tp.inputs_ok && tp.use_params) {
    live_.use_torque = true;
    live_.lat_accel_factor = static_cast<float>(tp.lat_accel_factor);
    live_.lat_accel_offset = static_cast<float>(tp.lat_accel_offset);
    live_.friction = static_cast<float>(tp.friction);
  }
}

std::string LateralLearners::vehicle_persist_json() const {
  return persist_vehicle_params(vehicle_.params(), constants_, bias_.bias());
}
