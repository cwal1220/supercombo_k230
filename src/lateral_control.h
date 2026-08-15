#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

/* OpenpilotLateralPlanner(acados MPC)가 만들고 K7LateralController가 소비하는
 * 횡방향 계획. 생산자는 controlsd의 MPC 하나뿐이다. */

constexpr int kLateralControlN = 17;

struct LateralTarget {
    bool valid = false;
    bool mpc_solution_valid = false;
    bool laneless_mode = false;
    float lookahead_x = 0.0f;
    float target_y = 0.0f;
    float heading = 0.0f;
    float curvature = 0.0f;
    float output_scale = 0.0f;
    int desire = 0;
    float psis[kLateralControlN] = {};
    float curvatures[kLateralControlN] = {};
    float curvature_rates[kLateralControlN] = {};
    float d_path_points[kLateralControlN] = {};
};

#endif
