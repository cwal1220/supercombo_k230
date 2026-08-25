#pragma once

#include "lateral_target.h"

#include <memory>

struct K230ModelState;
struct DrivingParams;
struct SteeringParams;
struct VehicleCanState;

class OpenpilotLateralPlanner {
public:
  OpenpilotLateralPlanner(const SteeringParams &params,
                          const DrivingParams &driving);
  ~OpenpilotLateralPlanner();

  OpenpilotLateralPlanner(const OpenpilotLateralPlanner &) = delete;
  OpenpilotLateralPlanner &operator=(const OpenpilotLateralPlanner &) = delete;

  void update_params(const SteeringParams &params,
                     const DrivingParams &driving);

  LateralTarget update(const K230ModelState &model,
                       const VehicleCanState &vehicle, float v_ego,
                       float measured_curvature, bool active,
                       float output_scale);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
