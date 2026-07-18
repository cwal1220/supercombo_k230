#pragma once

#include "lateral_control.h"

#include <memory>

struct K230ModelState;
struct K7SteeringParams;
struct K7VehicleCanState;

class OpenpilotLateralPlanner {
public:
  explicit OpenpilotLateralPlanner(const K7SteeringParams &params);
  ~OpenpilotLateralPlanner();

  OpenpilotLateralPlanner(const OpenpilotLateralPlanner &) = delete;
  OpenpilotLateralPlanner &operator=(const OpenpilotLateralPlanner &) = delete;

  LateralTarget update(const K230ModelState &model,
                       const K7VehicleCanState &vehicle, float v_ego,
                       float measured_curvature, bool active,
                       float output_scale);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
