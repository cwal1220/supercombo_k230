#include "openpilot_lateral_planner.h"

#include "k230_ipc.h"
#include "steering_params.h"
#include "vehicle_can.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_lat.h"
}

namespace {

constexpr int kMpcN = LAT_N;
constexpr int kMpcNodes = LAT_N + 1;
constexpr double kDtModel = 0.05;

double interp(double x, const double *xp, const double *fp, size_t count) {
  if (count == 0) return 0.0;
  if (x <= xp[0]) return fp[0];
  for (size_t i = 1; i < count; ++i) {
    if (x <= xp[i]) {
      const double span = xp[i] - xp[i - 1];
      if (span <= 0.0) return fp[i];
      const double p = (x - xp[i - 1]) / span;
      return fp[i - 1] + p * (fp[i] - fp[i - 1]);
    }
  }
  return fp[count - 1];
}

class FirstOrderFilter {
public:
  FirstOrderFilter(double value, double rc)
      : value_(value), alpha_(kDtModel / (rc + kDtModel)) {}

  double update(double value) {
    value_ = (1.0 - alpha_) * value_ + alpha_ * value;
    return value_;
  }

  double value() const { return value_; }

private:
  double value_;
  double alpha_;
};

class LanePlanner {
public:
  LanePlanner(double camera_offset_m, double path_offset_m)
      : camera_offset_m_(camera_offset_m), path_offset_m_(path_offset_m) {}

  void parse(const K230ModelState &model) {
    for (int i = 0; i < kTrajectorySize; ++i) {
      lane_t_[i] = model.lane_t[i];
      lane_x_[i] = model.lanes[1][i].x;
      left_y_[i] = model.lanes[1][i].y + camera_offset_m_;
      right_y_[i] = model.lanes[2][i].y + camera_offset_m_;
    }
    left_prob_ = model.lane_probabilities[1];
    right_prob_ = model.lane_probabilities[2];
    left_std_ = model.lane_stds[1];
    right_std_ = model.lane_stds[2];
  }

  std::array<std::array<double, 3>, kTrajectorySize> lane_path(
      float v_ego, const std::array<double, kTrajectorySize> &path_t,
      std::array<std::array<double, 3>, kTrajectorySize> path) {
    for (auto &point : path) point[1] += path_offset_m_;

    std::array<double, kTrajectorySize> width{};
    for (int i = 0; i < kTrajectorySize; ++i)
      width[i] = right_y_[i] - left_y_[i];

    double left_prob = left_prob_;
    double right_prob = right_prob_;
    double width_mod = 1.0;
    for (double t : {0.0, 1.5, 3.0}) {
      const double lane_width = interp(t * (v_ego + 7.0), lane_x_.data(), width.data(), width.size());
      const double candidate = lane_width <= 4.0 ? 1.0
          : lane_width >= 5.0 ? 0.0 : 5.0 - lane_width;
      width_mod = std::min(width_mod, candidate);
    }
    left_prob *= width_mod;
    right_prob *= width_mod;
    const auto std_mod = [](double value) {
      return value <= 0.15 ? 1.0 : value >= 0.3 ? 0.0 : (0.3 - value) / 0.15;
    };
    left_prob *= std_mod(left_std_);
    right_prob *= std_mod(right_std_);

    lane_width_certainty_.update(left_prob * right_prob);
    lane_width_estimate_.update(std::fabs(right_y_[0] - left_y_[0]));
    const double speed_width = interp(v_ego, kLaneWidthSpeed.data(), kLaneWidth.data(),
                                      kLaneWidthSpeed.size());
    lane_width_ = lane_width_certainty_.value() * lane_width_estimate_.value() +
                  (1.0 - lane_width_certainty_.value()) * speed_width;
    const double half_width = std::min(4.0, lane_width_) * 0.5;
    const double denominator = left_prob + right_prob + 0.0001;
    d_prob_ = left_prob + right_prob - left_prob * right_prob;

    std::array<double, kTrajectorySize> lane_path_y{};
    for (int i = 0; i < kTrajectorySize; ++i) {
      const double from_left = left_y_[i] + half_width;
      const double from_right = right_y_[i] - half_width;
      lane_path_y[i] = (left_prob * from_left + right_prob * from_right) / denominator;
    }

    std::array<double, kTrajectorySize> valid_t{};
    std::array<double, kTrajectorySize> valid_y{};
    size_t valid_count = 0;
    for (int i = 0; i < kTrajectorySize; ++i) {
      if (std::isfinite(lane_t_[i])) {
        valid_t[valid_count] = lane_t_[i];
        valid_y[valid_count] = lane_path_y[i];
        ++valid_count;
      }
    }
    if (valid_count > 0) {
      for (int i = 0; i < kTrajectorySize; ++i) {
        const double lane_y = interp(path_t[i], valid_t.data(), valid_y.data(), valid_count);
        path[i][1] = d_prob_ * lane_y + (1.0 - d_prob_) * path[i][1];
      }
    }
    return path;
  }

  double mean_near_probability() const { return (left_prob_ + right_prob_) * 0.5; }
  void scale_near_probability(double scale) {
    left_prob_ *= scale;
    right_prob_ *= scale;
  }

private:
  static constexpr std::array<double, 2> kLaneWidthSpeed = {0.0, 31.0};
  static constexpr std::array<double, 2> kLaneWidth = {2.8, 3.5};
  std::array<double, kTrajectorySize> lane_t_{};
  std::array<double, kTrajectorySize> lane_x_{};
  std::array<double, kTrajectorySize> left_y_{};
  std::array<double, kTrajectorySize> right_y_{};
  FirstOrderFilter lane_width_estimate_{3.7, 9.95};
  FirstOrderFilter lane_width_certainty_{1.0, 0.95};
  double left_prob_ = 0.0;
  double right_prob_ = 0.0;
  double left_std_ = 0.0;
  double right_std_ = 0.0;
  double lane_width_ = 3.7;
  double d_prob_ = 0.0;
  double camera_offset_m_ = 0.0;
  double path_offset_m_ = 0.0;
};

class AcadosLateralMpc {
public:
  AcadosLateralMpc() : capsule_(lat_acados_create_capsule()) {
    lat_acados_create(capsule_);
    reset({0.0, 0.0, 0.0, 0.0});
  }

  ~AcadosLateralMpc() {
    lat_acados_free(capsule_);
    lat_acados_free_capsule(capsule_);
  }

  void reset(const std::array<double, LAT_NX> &x0) {
    x_sol_ = {};
    u_sol_ = {};
    const std::array<double, LAT_NY> yref{};
    const std::array<double, LAT_NP> params{};
    for (int i = 0; i < kMpcN; ++i)
      ocp_nlp_cost_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                             capsule_->nlp_in, i, "yref",
                             const_cast<double *>(yref.data()));
    const std::array<double, 2> terminal_yref{};
    ocp_nlp_cost_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                           capsule_->nlp_in, kMpcN, "yref",
                           const_cast<double *>(terminal_yref.data()));
    for (int i = 0; i <= kMpcN; ++i) {
      std::array<double, LAT_NX> zero{};
      ocp_nlp_out_set(capsule_->nlp_config, capsule_->nlp_dims, capsule_->nlp_out,
                      i, "x", zero.data());
      lat_acados_update_params(capsule_, i, const_cast<double *>(params.data()), LAT_NP);
    }
    set_initial_state(x0);
    lat_acados_solve(capsule_);
  }

  void run(const std::array<double, LAT_NX> &x0,
           const std::array<double, LAT_NP> &params,
           const std::array<double, kMpcNodes> &y,
           const std::array<double, kMpcNodes> &heading,
           double heading_weight) {
    const double weights[9] = {
        1.0, 0.0, 0.0,
        0.0, heading_weight, 0.0,
        0.0, 0.0, 1.0,
    };
    const double terminal_weights[4] = {0.15, 0.0, 0.0, 0.15 * heading_weight};
    for (int i = 0; i < kMpcN; ++i)
      ocp_nlp_cost_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                             capsule_->nlp_in, i, "W", const_cast<double *>(weights));
    ocp_nlp_cost_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                           capsule_->nlp_in, kMpcN, "W",
                           const_cast<double *>(terminal_weights));

    set_initial_state(x0);
    for (int i = 0; i <= kMpcN; ++i) {
      const double velocity_cost = params[0] + 5.0;
      if (i < kMpcN) {
        std::array<double, LAT_NY> yref = {y[i], heading[i] * velocity_cost, 0.0};
        ocp_nlp_cost_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                               capsule_->nlp_in, i, "yref", yref.data());
      } else {
        std::array<double, 2> yref = {y[i], heading[i] * velocity_cost};
        ocp_nlp_cost_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                               capsule_->nlp_in, i, "yref", yref.data());
      }
      lat_acados_update_params(capsule_, i, const_cast<double *>(params.data()), LAT_NP);
    }

    status_ = lat_acados_solve(capsule_);
    for (int i = 0; i <= kMpcN; ++i)
      ocp_nlp_out_get(capsule_->nlp_config, capsule_->nlp_dims, capsule_->nlp_out,
                      i, "x", x_sol_[i].data());
    for (int i = 0; i < kMpcN; ++i)
      ocp_nlp_out_get(capsule_->nlp_config, capsule_->nlp_dims, capsule_->nlp_out,
                      i, "u", u_sol_[i].data());
    ocp_nlp_eval_cost(capsule_->nlp_solver, capsule_->nlp_in, capsule_->nlp_out);
    ocp_nlp_get(capsule_->nlp_config, capsule_->nlp_solver, "cost_value", &cost_);
  }

  const auto &states() const { return x_sol_; }
  const auto &controls() const { return u_sol_; }
  int status() const { return status_; }
  double cost() const { return cost_; }

private:
  void set_initial_state(const std::array<double, LAT_NX> &x0) {
    ocp_nlp_constraints_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                                  capsule_->nlp_in, 0, "lbx",
                                  const_cast<double *>(x0.data()));
    ocp_nlp_constraints_model_set(capsule_->nlp_config, capsule_->nlp_dims,
                                  capsule_->nlp_in, 0, "ubx",
                                  const_cast<double *>(x0.data()));
  }

  lat_solver_capsule *capsule_;
  std::array<std::array<double, LAT_NX>, kMpcNodes> x_sol_{};
  std::array<std::array<double, LAT_NU>, kMpcN> u_sol_{};
  int status_ = 0;
  double cost_ = 0.0;
};

}  // namespace

struct OpenpilotLateralPlanner::Impl {
  explicit Impl(const K7SteeringParams &steering)
      : lane_planner(steering.camera_offset_m, steering.path_offset_m) {
    const double center_to_front = steering.center_to_front_m();
    constexpr double civic_mass = 1326.0 + 136.0;
    constexpr double civic_wheelbase = 2.70;
    constexpr double civic_center_to_front = civic_wheelbase * 0.4;
    double tire_rear = 202500.0 * steering.tire_stiffness_factor *
                       steering.mass_kg / civic_mass;
    tire_rear *= (center_to_front / steering.wheelbase_m) /
                 (civic_center_to_front / civic_wheelbase);
    factor1 = steering.wheelbase_m - center_to_front;
    factor2 = center_to_front * steering.mass_kg /
              (steering.wheelbase_m * tire_rear);
  }

  LateralTarget update(const K230ModelState &model,
                       const K7VehicleCanState &vehicle, float v_ego,
                       float measured_curvature, bool active,
                       float output_scale) {
    LateralTarget target;
    if (!model.valid) return target;

    lane_planner.parse(model);
    for (size_t i = 0; i < model_lane_probs.size(); ++i)
      model_lane_probs[i] = model.lane_probabilities[i];
    for (size_t i = 0; i < model_road_edge_stds.size(); ++i)
      model_road_edge_stds[i] = model.road_edge_stds[i];
    const double lane_change_prob = model.desire_state[3] + model.desire_state[4];
    update_lane_change(vehicle, v_ego, measured_curvature, active,
                       output_scale, lane_change_prob);
    if (desire == 3 || desire == 4)
      lane_planner.scale_near_probability(lane_change_lane_prob);
    std::array<std::array<double, 3>, kTrajectorySize> path{};
    std::array<double, kTrajectorySize> path_t{};
    std::array<double, kTrajectorySize> yaw{};
    for (int i = 0; i < kTrajectorySize; ++i) {
      path[i] = {model.plan[i].x, model.plan[i].y, model.plan[i].z};
      path_t[i] = model.model_t[i];
      yaw[i] = model.plan_orientations[i].z;
    }
    const auto model_path = path;

    const double lane_probability = lane_planner.mean_near_probability();
    bool use_model_path = false;
    const bool lane_change_off = lane_change_state == 0;
    if (lane_probability < 0.3 && lane_change_off) {
      use_model_path = true;
      laneless_buffer = true;
    } else if (lane_probability > 0.5 && laneless_buffer && lane_change_off) {
      laneless_buffer = false;
    } else if (laneless_buffer && lane_change_off) {
      use_model_path = true;
    } else if (!lane_change_off) {
      laneless_buffer = false;
    }
    if (!use_model_path)
      path = lane_planner.lane_path(v_ego, path_t, path);

    std::array<double, kTrajectorySize> distance{};
    std::array<double, kTrajectorySize> model_distance{};
    std::array<double, kTrajectorySize> path_y{};
    for (int i = 0; i < kTrajectorySize; ++i)
      distance[i] = std::sqrt(path[i][0] * path[i][0] + path[i][1] * path[i][1] +
                              path[i][2] * path[i][2]);
    for (int i = 0; i < kTrajectorySize; ++i) {
      model_distance[i] = std::sqrt(model_path[i][0] * model_path[i][0] +
                                    model_path[i][1] * model_path[i][1] +
                                    model_path[i][2] * model_path[i][2]);
      path_y[i] = path[i][1];
    }

    std::array<double, kMpcNodes> y_pts{};
    std::array<double, kMpcNodes> heading_pts{};
    for (int i = 0; i < kMpcNodes; ++i) {
      const double query = std::max(0.0f, v_ego) * path_t[i];
      y_pts[i] = interp(query, distance.data(), path_y.data(), distance.size());
      heading_pts[i] = interp(query, model_distance.data(), yaw.data(), model_distance.size());
    }

    const double lateral_factor = std::max(0.0, factor1 - factor2 * v_ego * v_ego);
    const double heading_weight = use_model_path
        ? (v_ego <= 5.0f ? 1.0 : v_ego >= 10.0f ? 0.15
                                                : 1.0 - (v_ego - 5.0) * 0.17)
        : 1.0;
    mpc.run(x0, {std::max(0.0f, v_ego), lateral_factor}, y_pts, heading_pts,
            heading_weight);
    bool has_nan = false;
    for (const auto &state : mpc.states()) has_nan = has_nan || !std::isfinite(state[3]);
    if (has_nan || mpc.status() != 0) {
      mpc.reset({0.0, 0.0, 0.0, 0.0});
      x0 = {0.0, 0.0, 0.0, measured_curvature};
    } else {
      std::array<double, kMpcNodes> curvatures{};
      for (int i = 0; i < kMpcNodes; ++i) curvatures[i] = mpc.states()[i][3];
      x0[3] = interp(kDtModel, path_t.data(), curvatures.data(), curvatures.size());
    }
    invalid_count = (mpc.cost() > 20000.0 || has_nan) ? invalid_count + 1 : 0;

    target.valid = true;
    target.mpc_solution_valid = invalid_count < 2;
    target.lookahead_x = static_cast<float>(std::max(0.0f, v_ego) * path_t[1]);
    target.target_y = static_cast<float>(y_pts[1]);
    target.heading = static_cast<float>(mpc.states()[0][2]);
    target.curvature = static_cast<float>(mpc.states()[0][3]);
    target.desire = desire;
    for (int i = 0; i < kLateralControlN; ++i) {
      target.d_path_points[i] = static_cast<float>(y_pts[i]);
      target.psis[i] = static_cast<float>(mpc.states()[i][2]);
      target.curvatures[i] = static_cast<float>(mpc.states()[i][3]);
      target.curvature_rates[i] = i < kMpcN
          ? static_cast<float>(mpc.controls()[i][0]) : 0.0f;
    }
    return target;
  }

  void update_lane_change(const K7VehicleCanState &vehicle, float v_ego,
                          float measured_curvature, bool active,
                          float output_scale, double lane_change_prob) {
    const bool one_blinker = vehicle.left_blinker != vehicle.right_blinker;
    const bool below_speed = v_ego < 30.0f / 3.6f;
    int direction_now = direction;
    if (vehicle.left_blinker) direction_now = -1;
    if (vehicle.right_blinker) direction_now = 1;

    const double left_edge_prob = std::clamp(1.0 - model_road_edge_stds[0], 0.0, 1.0);
    const double right_edge_prob = std::clamp(1.0 - model_road_edge_stds[1], 0.0, 1.0);
    const double left_nearside_prob = model_lane_probs[0];
    const double right_nearside_prob = model_lane_probs[3];
    const int road_edge = right_edge_prob > 0.35 && right_nearside_prob < 0.2 &&
                                  left_nearside_prob >= right_nearside_prob
        ? 1
        : left_edge_prob > 0.35 && left_nearside_prob < 0.2 &&
                                  right_nearside_prob >= left_nearside_prob
            ? -1 : 0;
    const int lane_direction = vehicle.left_blinker ? -1 : vehicle.right_blinker ? 1 : 2;
    const bool road_edge_blocked = lane_change_state == 0 && road_edge == lane_direction;

    if (road_edge_blocked) {
      direction = 0;
    } else if (!active || lane_change_timer > 10.0 ||
        (std::fabs(output_scale) >= 0.8f && lane_change_timer > 0.5)) {
      lane_change_state = 0;
      direction = 0;
    } else {
      const bool steering_pressed = std::abs(vehicle.driver_torque) > 150;
      const bool torque_applied = steering_pressed &&
          ((vehicle.driver_torque > 0 && direction == -1) ||
           (vehicle.driver_torque < 0 && direction == 1));
      const bool blindspot_detected =
          (vehicle.left_blindspot && direction == -1) ||
          (vehicle.right_blindspot && direction == 1);
      if (lane_change_state == 0 && one_blinker && !previous_one_blinker &&
          !below_speed) {
        lane_change_state = 1;
        direction = direction_now;
        lane_change_lane_prob = 1.0;
        const double speed_points[4] = {30.0 / 3.6, 60.0 / 3.6,
                                        80.0 / 3.6, 110.0 / 3.6};
        const double timing[4] = {0.1, 0.4, 0.6, 0.8};
        lane_change_adjust = interp(v_ego, speed_points, timing, 4);
        if ((measured_curvature > 0.0005f && direction == -1) ||
            (measured_curvature < -0.0005f && direction == 1))
          lane_change_adjust = std::min(2.0, lane_change_adjust * 1.5);
      } else if (lane_change_state == 1) {
        if (!one_blinker || below_speed) {
          lane_change_state = 0;
        } else if (!blindspot_detected && torque_applied) {
          lane_change_state = 2;
        }
      } else if (lane_change_state == 2) {
        lane_change_lane_prob = std::max(0.0, lane_change_lane_prob -
                                               lane_change_adjust * kDtModel);
        if (lane_change_prob < 0.02 && lane_change_lane_prob < 0.01)
          lane_change_state = 3;
      } else if (lane_change_state == 3) {
        lane_change_lane_prob = std::min(1.0, lane_change_lane_prob + kDtModel);
        if (lane_change_lane_prob > 0.99) {
          lane_change_state = one_blinker ? 1 : 0;
          if (!one_blinker) direction = 0;
        }
      }
    }

    lane_change_timer = lane_change_state < 2 ? 0.0 : lane_change_timer + kDtModel;
    previous_one_blinker = road_edge_blocked ? false : one_blinker;
    desire = lane_change_state >= 2 && direction == -1 ? 3
        : lane_change_state >= 2 && direction == 1 ? 4 : 0;
  }

  LanePlanner lane_planner;
  AcadosLateralMpc mpc;
  std::array<double, LAT_NX> x0{};
  double factor1 = 0.0;
  double factor2 = 0.0;
  bool laneless_buffer = false;
  int invalid_count = 0;
  int lane_change_state = 0;
  int direction = 0;
  int desire = 0;
  bool previous_one_blinker = false;
  double lane_change_lane_prob = 1.0;
  double lane_change_adjust = 2.0;
  double lane_change_timer = 0.0;
  std::array<double, 4> model_lane_probs{};
  std::array<double, 2> model_road_edge_stds{};
};

OpenpilotLateralPlanner::OpenpilotLateralPlanner(const K7SteeringParams &params)
    : impl_(std::make_unique<Impl>(params)) {}

OpenpilotLateralPlanner::~OpenpilotLateralPlanner() = default;

LateralTarget OpenpilotLateralPlanner::update(const K230ModelState &model,
                                              const K7VehicleCanState &vehicle,
                                              float v_ego,
                                              float measured_curvature,
                                              bool active,
                                              float output_scale) {
  return impl_->update(model, vehicle, v_ego, measured_curvature, active,
                       output_scale);
}
