#pragma once

#include "lateral_target.h"

#include <memory>

struct K230ModelState;
struct DrivingParams;
struct SteeringParams;
struct VehicleCanState;

class LateralPlanner {
public:
  LateralPlanner(const SteeringParams &params,
                          const DrivingParams &driving);
  ~LateralPlanner();

  LateralPlanner(const LateralPlanner &) = delete;
  LateralPlanner &operator=(const LateralPlanner &) = delete;

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
