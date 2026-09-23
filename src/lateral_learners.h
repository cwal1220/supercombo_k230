#pragma once

/* openpilot paramsd(VehicleParamsLearner·CarKalman)와 torqued(TorqueEstimator) 이식.
 * 상류와 다른 것: 요레이트·롤은 ESP12(locationd 자세 대신), 지연은 고정값(lagd 대신),
 * 예측은 1 ms로 분할(상류의 0.05 s 오일러 한 걸음은 7.5 m/s 아래에서 발산),
 * 조향각·속도는 매 틱 관측(상류는 20 Hz, 아래 VehicleParamsOptions), 입력이 낡으면 비활성과 같이 처리. */

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "control_params.h"
#include "lateral_torque.h"
#include "vehicle_can.h"

// ---------------------------------------------------------------- paramsd

struct VehicleModelConstants {
  double mass_kg = 0.0;
  double wheelbase_m = 0.0;
  double center_to_front_m = 0.0;
  double tire_stiffness_factor = 1.0;
  double steer_ratio = 0.0;  // 사전값. 유효 범위 [0.5, 2]배의 기준
};

/* CarKalman(car_kf.py): 선형 단일트랙 동역학 위에서 SR·강성·영점·롤을 푸는 EKF. */
class CarKalman {
public:
  static constexpr int kN = 9;
  enum State : int { kStiffness, kSteerRatio, kAngleOffset, kAngleOffsetFast,
                     kSpeedX, kSpeedY, kYawRate, kSteerAngle, kRoadRoll };
  using Vec = std::array<double, kN>;
  using Mat = std::array<std::array<double, kN>, kN>;

  struct Globals {
    double mass = 0.0, inertia = 0.0, center_to_front = 0.0, center_to_rear = 0.0;
    double stiffness_front = 0.0, stiffness_rear = 0.0;
  };

  static Vec initial_x();
  static Mat q();
  static Globals globals_from(const VehicleModelConstants &c);
  /* 1차 오일러 한 걸음과 그 야코비안. 검사가 수치 미분과 대조한다. */
  static void step(const Globals &g, const Vec &x, double h, Vec *x_next, Mat *f);

  explicit CarKalman(const Globals &g) : g_(g) {}
  void init(const Vec &x, const Mat &p, bool has_time, double t);
  void set_time(double t) { t_ = t; has_time_ = true; }
  bool has_time() const { return has_time_; }
  double time() const { return t_; }
  /* rednose predict_and_observe. 한 상태를 직접 보는 관측만 있다(H가 단위행). */
  void predict_and_observe(double t, int state, double z, double r);

  const Vec &x() const { return x_; }
  const Mat &p() const { return p_; }
  Vec &mutable_x() { return x_; }  // 검사용

private:
  void predict(double dt);

  Globals g_;
  Vec x_{};
  Mat p_{};
  double t_ = 0.0;
  bool has_time_ = false;
};

/* 한 제어 틱(100 Hz)의 입력. 요레이트·횡가속은 좌측 양수. */
struct VehicleParamsInput {
  double t_s = 0.0;
  bool inputs_fresh = false;  // 상류 sm.all_checks()
  double steering_angle_deg = 0.0;
  double speed_mps = 0.0;
  int gear = 0;
  bool yaw_rate_valid = false;
  double yaw_rate_rad_s = 0.0;  // 바이어스 제거 후
  bool lat_accel_valid = false;
  double lat_accel_mps2 = 0.0;  // 비력(가속도계), 좌측 양수
};

/* vehicleParameters 메시지. */
struct VehicleParams {
  bool inputs_ok = false;  // 봉투 valid
  bool valid = false;      // 필드 valid
  bool sensor_valid = true;
  double steer_ratio = 0.0;
  double stiffness_factor = 0.0;
  double roll_rad = 0.0;
  double angle_offset_average_deg = 0.0;
  double angle_offset_deg = 0.0;
  bool steer_ratio_valid = false;
  bool stiffness_factor_valid = false;
  bool angle_offset_average_valid = false;
  bool angle_offset_valid = false;
  double steer_ratio_std = 0.0;
  double stiffness_factor_std = 0.0;
  double angle_offset_average_std = 0.0;
  double angle_offset_fast_std = 0.0;
};

struct VehicleParamsOptions {
  const CarKalman::Mat *p_initial = nullptr;  // nullptr면 상류처럼 P0 = Q
  /* 매 제어 틱 관측한다. false면 상류처럼 20 Hz마다 최신 조향각·속도만 보는데, 조향 입력이
   * 50 ms씩 멈춰 들어가 SR을 낮게 배우고(합성 참값 15.2 → 14.3~14.6) 빠른 영점이 커브마다
   * 그 몫을 메워 진입·탈출 곡률 오차가 6~9% 크다(2026-09-21·22 세 주행). */
  bool car_state_every_tick = true;
};

class VehicleParamsLearner {
public:
  VehicleParamsLearner(const VehicleModelConstants &c, double steer_ratio,
                       double stiffness_factor, double angle_offset_rad,
                       const VehicleParamsOptions &options = {});

  /* 제어 틱마다 부른다. 20 Hz 틱이면 true이고 params()가 갱신된다. */
  bool update(const VehicleParamsInput &in);
  const VehicleParams &params() const { return last_; }
  /* 출력 1200번(1분)마다. 첫 출력에서도 참이다(상류 sm.frame % 1200 == 0). */
  bool persist_due() const { return persist_due_; }
  CarKalman &kf() { return kf_; }

private:
  void reset(bool has_time, double t);
  void handle_car_state(const VehicleParamsInput &in);
  void handle_device_motion(const VehicleParamsInput &in);
  VehicleParams get_msg(bool inputs_ok);

  VehicleModelConstants c_;
  bool car_state_every_tick_ = true;
  double min_sr_ = 0.0;
  double max_sr_ = 0.0;
  CarKalman kf_;
  CarKalman::Vec x_initial_{};
  CarKalman::Mat p_initial_{};
  double observed_speed_ = 0.0;
  double observed_yaw_rate_ = 0.0;
  double observed_roll_ = 0.0;
  bool avg_offset_valid_ = true;
  bool total_offset_valid_ = true;
  bool roll_valid_ = true;
  double angle_offset_deg_ = 0.0;
  double avg_angle_offset_deg_ = 0.0;
  double roll_ = 0.0;
  bool active_ = false;
  bool has_motion_t_ = false;
  double last_motion_t_ = 0.0;
  long motion_frame_ = -1;
  bool persist_due_ = false;
  VehicleParams last_{};
};

/* 상류 retrieve_initial_vehicle_params. 거부되면 restored=false로 사전값을 돌려주고,
 * 호출자는 저장 파일을 지운다. 강성은 복원하지 않는다(젖은 노면 값이 넘어오지 않게). */
struct VehicleParamsInit {
  bool restored = false;
  double steer_ratio = 0.0;
  double stiffness_factor = 1.0;
  double angle_offset_deg = 0.0;
  double yaw_bias_rad_s = 0.0;
};
VehicleParamsInit restore_vehicle_params(const std::string &json,
                                         const VehicleModelConstants &c);
std::string persist_vehicle_params(const VehicleParams &p, const VehicleModelConstants &c,
                                   double yaw_bias_rad_s);

/* ESP12 자이로 바이어스. 상류는 locationd가 빼 주지만 원시값에는 남아 있다
 * (2026-09-22 정차 중앙값 −0.06~−0.11°/s). 휠속도는 1~2 km/h 밑에서 0을 내서 핸들을
 * 꺾고 기어가는 동안도 정차로 보이므로, 2초 연속 정차이고 거의 돌지 않을 때만 따라간다. */
class YawBiasEstimator {
public:
  explicit YawBiasEstimator(double initial_bias = 0.0) : bias_(initial_bias) {}
  /* 보정된 요레이트를 돌려준다. */
  double update(double t_s, double speed_mps, bool yaw_valid, double yaw_rate_rad_s);
  double bias() const { return bias_; }

private:
  double bias_ = 0.0;
  bool standing_ = false;
  double standing_since_s_ = 0.0;
};

// ---------------------------------------------------------------- torqued

/* CP.lateralTuning.torque. 사전값이자 허용 폭(배율 ±30%, 마찰 ±50%)과 캐시 키의 기준이다. */
struct TorqueTuning {
  double lat_accel_factor = 0.0;  // 정규화 토크 1.0이 내는 횡가속도 [m/s²]
  double friction = 0.0;
};

/* 한 제어 틱의 입력. 토크·요레이트는 상류 torqued처럼 우측 양수다(컨트롤러 곡률과 같은
 * 관례라 학습 절편을 FF에 그대로 뺄 수 있다). 롤은 paramsd 관례. */
struct TorqueEstimatorInput {
  double t_s = 0.0;
  bool inputs_fresh = false;  // 상류 sm.all_checks()
  bool lat_active = false;
  double steer_torque = 0.0;  // 실제 보낸 토크 / steer_max, 우측 양수
  double speed_mps = 0.0;
  bool steer_override = false;
  bool pose_valid = false;      // 요레이트·롤 출처가 유효
  double yaw_rate_rad_s = 0.0;  // 바이어스 제거 후, 우측 양수
  double roll_rad = 0.0;        // 양수 = 오른쪽이 낮다(paramsd·상류 NED 관례)
};

/* lateralTorqueParameters. 제어는 필터값을 쓴다. */
struct TorqueParams {
  bool inputs_ok = false;  // 봉투 valid
  bool valid = false;      // 필드 valid
  bool use_params = true;
  double lat_accel_factor_raw = 0.0;
  double lat_accel_offset_raw = 0.0;
  double friction_raw = 0.0;
  double lat_accel_factor = 0.0;
  double lat_accel_offset = 0.0;
  double friction = 0.0;
  int total_bucket_points = 0;
  int cal_perc = 0;
  double decay = 0.0;
  double max_resets = 0.0;
};

enum class TorqueRestore { None, Restored, KeyMismatch, Corrupt };

class TorqueEstimator {
public:
  static constexpr int kBuckets = 8;
  static constexpr int kPointsPerBucket = 1500;
  static constexpr int kHistLen = 100;  // 5초 × 20 Hz

  /* lag_s는 상류 lateralDelay 자리. cache가 비어 있지 않으면 상류처럼 먼저 복원한다.
   * Corrupt면 호출자가 캐시를 지운다(상류 params.remove). 키 불일치는 두기만 한다. */
  TorqueEstimator(const TorqueTuning &offline, double lag_s, uint64_t seed,
                  const std::string &cache = std::string());

  /* 제어 틱마다 부른다. 4 Hz 출력 틱이면 true이고 params()가 갱신된다. */
  bool update(const TorqueEstimatorInput &in);
  const TorqueParams &params() const { return last_; }
  /* 20 Hz 240번(12초)마다, 첫 틱 포함. 상류처럼 get_msg를 한 번 더 돌린 결과라
   * 그 틱에는 필터가 두 번 갱신된다. */
  bool persist_due() const { return persist_due_; }
  const std::string &cache() const { return cache_; }
  TorqueRestore restore_status() const { return restore_; }
  const TorqueTuning &tuning() const { return offline_; }

  /* 적합에 점을 전부 쓴다(상류는 2000점 무작위 추출). 참조 구현 대조용. */
  void set_fit_all_points(bool all) { fit_all_points_ = all; }
  int bucket_size(int i) const { return buckets_[i].count; }
  int total_points() const;
  /* 상류 get_points()[:, [0, 2]]: 버킷 순서, 버킷 안은 오래된 것부터. */
  std::vector<std::array<double, 2>> points() const;

private:
  struct RawSample {
    double t = 0.0;  // 지연을 더한 시각
    double lat_active = 0.0;
    double steer = 0.0;
    double vego = 0.0;
    double steer_override = 0.0;
  };
  struct Bucket {  // NPQueue: 가득 차면 가장 오래된 점을 민다
    std::vector<std::array<double, 2>> points =
        std::vector<std::array<double, 2>>(kPointsPerBucket);
    int head = 0;
    int count = 0;
    const std::array<double, 2> &at(int i) const {
      return points[(head + i) % kPointsPerBucket];
    }
  };
  struct Filter {  // FirstOrderFilter(dt = DT_MDL)
    double x = 0.0;
    double alpha = 0.0;
  };

  void reset();
  void add_point(double x, double y);
  bool is_calculable() const;
  bool is_valid() const;
  int valid_percent() const;
  void estimate_params(double *slope, double *offset, double *friction);
  void update_params(double factor, double offset, double friction);
  void handle_device_motion(const TorqueEstimatorInput &in);
  double interp_raw(double t, double RawSample::*field) const;
  TorqueParams get_msg(bool inputs_ok);
  std::string serialize(const TorqueParams &p) const;
  TorqueRestore restore(const std::string &cache, double *factor, double *offset,
                        double *friction);

  TorqueTuning offline_;
  double lag_s_ = 0.0;
  uint64_t rng_ = 0;
  bool fit_all_points_ = false;
  double min_factor_ = 0.0, max_factor_ = 0.0, min_friction_ = 0.0, max_friction_ = 0.0;
  double resets_ = 0.0;
  double decay_ = 0.0;
  std::array<RawSample, kHistLen> raw_{};
  int raw_head_ = 0;  // 가장 오래된 표본
  int raw_count_ = 0;
  std::array<Bucket, kBuckets> buckets_{};
  std::vector<std::array<double, 2>> fit_points_;
  Filter factor_f_, offset_f_, friction_f_;
  long frame_ = -1;
  bool has_motion_t_ = false;
  double last_motion_t_ = 0.0;
  bool persist_due_ = false;
  TorqueRestore restore_ = TorqueRestore::None;
  std::string cache_;
  TorqueParams last_{};
};

// ---------------------------------------------------------------- controlsd 연결

/* k230_controlsd 안의 paramsd·torqued. 제어 틱 끝에 이번 틱 값(보낸 토크 포함)으로 갱신하고
 * 컨트롤러는 다음 틱에 live()를 쓴다. 상류 controlsd가 직전 메시지를 쓰는 것과 같다.
 * 사전값·지연은 생성 시 파라미터로 고정한다(상류 CarParams처럼 주행 중 바뀌지 않는다). */
class LateralLearners {
public:
  /* vehicle_json·torque_cache는 저장 파일 내용이다(없으면 빈 문자열). */
  LateralLearners(const SteeringParams &params, const std::string &vehicle_json,
                  const std::string &torque_cache, uint64_t seed,
                  const VehicleParamsOptions &options = {});

  void update(const VehicleCanState &vehicle, double now_s, double timeout_s, bool lat_active,
              int apply_torque, bool steering_pressed);

  /* use_vehicle = paramsd가 한 번이라도 냈다(상류 sm.seen). use_torque = 봉투가 유효한
   * torqued 메시지를 받은 적이 있다. 토크 값은 그 뒤 봉투가 유효한 메시지로만 바뀐다
   * (상류 sm.all_checks(['lateralTorqueParameters'])). */
  LiveLateralParams live() const { return live_; }
  bool vehicle_valid() const { return vehicle_.params().valid; }
  const VehicleParams &vehicle_params() const { return vehicle_.params(); }
  const TorqueParams &torque_params() const { return torque_.params(); }
  double yaw_bias_rad_s() const { return bias_.bias(); }
  double prior_steer_ratio() const { return constants_.steer_ratio; }
  bool vehicle_published() const { return vehicle_published_; }
  bool torque_published() const { return torque_published_; }

  // 저장: 이번 틱에 새 내용이 생겼으면 참. 쓰기는 호출자가 제어 루프 밖에서 한다.
  bool vehicle_persist_due() const { return vehicle_.persist_due(); }
  std::string vehicle_persist_json() const;
  bool torque_persist_due() const { return torque_.persist_due(); }
  const std::string &torque_cache() const { return torque_.cache(); }
  // 상류는 복원이 거부된 paramsd 저장과 깨진 torqued 캐시를 지운다
  bool vehicle_restore_rejected() const { return vehicle_restore_rejected_; }
  TorqueRestore torque_restore_status() const { return torque_.restore_status(); }
  bool vehicle_restored() const { return vehicle_restored_; }

  // 대조 도구용: 이번 틱에 넣은 입력과 추정기
  const VehicleParamsInput &last_vehicle_input() const { return last_vehicle_input_; }
  const TorqueEstimatorInput &last_torque_input() const { return last_torque_input_; }
  TorqueEstimator &torque_estimator() { return torque_; }
  const TorqueEstimator &torque_estimator() const { return torque_; }

private:
  static VehicleModelConstants constants(const SteeringParams &params);

  VehicleModelConstants constants_;
  VehicleParamsInit init_;
  bool vehicle_restored_ = false;
  bool vehicle_restore_rejected_ = false;
  YawBiasEstimator bias_;
  VehicleParamsLearner vehicle_;
  TorqueEstimator torque_;
  int steer_max_ = 1;
  int output_sign_ = -1;  // torque_output_sign: 보낸 토크 = 부호 × 우측 양수 제어 출력
  LiveLateralParams live_{};
  bool vehicle_published_ = false;
  bool torque_published_ = false;
  VehicleParamsInput last_vehicle_input_{};
  TorqueEstimatorInput last_torque_input_{};
};
