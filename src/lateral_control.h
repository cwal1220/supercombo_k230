#ifndef LATERAL_CONTROL_H
#define LATERAL_CONTROL_H

/* OpenpilotLateralPlanner(acados MPC)가 만들고 K7LateralController가 소비하는
 * 횡방향 계획. 생산자는 controlsd의 MPC 하나뿐이다. */

#include <cstdint>

constexpr int kLateralControlN = 17;

struct LateralTarget {
    bool valid = false;
    /* 이 계획의 근거가 된 카메라 프레임 캡처 시각(k230_now_ns 기준).
     * 컨트롤러가 lag 보상과 staleness gate에 쓴다. 0이면 미상(테스트 등)으로
     * 취급해 나이 보정을 건너뛴다. */
    uint64_t capture_timestamp_ns = 0;
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
    /* 차선 관측값. 제어에 쓰지 않고 로그/분석 전용이다. 차가 차선 중앙에서
     * 얼마나 벗어났는지는 pathY로 알 수 없어 이 값이 필요하다. 모델 좌표계
     * (+y = 오른쪽), camera_offset 적용 후. */
    bool lane_valid = false;
    float lane_left_y = 0.0f;
    float lane_right_y = 0.0f;
    float lane_width = 0.0f;
    float lane_left_prob = 0.0f;
    float lane_right_prob = 0.0f;
    float lane_left_std = 0.0f;
    float lane_right_std = 0.0f;
    float lane_d_prob = 0.0f;
};

#endif
