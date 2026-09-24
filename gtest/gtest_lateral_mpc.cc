/* LateralMpc가 openpilot 0.8.16 lat_mpc의 OCP를 실제로 최적화하는지 검증한다.
 * 동역학/코스트를 여기서 독립으로 다시 구현해, 수렴점에서 참 목적함수의
 * 기울기가 0인지 유한차분으로 본다. 감도 A/B가 틀리면 이 검사가 걸린다. */
#include "lateral_mpc.h"
#include "model_output.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace {

struct Scenario {
    const char *name;
    double v_ego;
    double rotation_radius;
    double curvature0;
    // y_ref = amplitude * (1 - cos(pi * t / horizon)), 차선 변경 비슷한 목표.
    double amplitude;
    LateralMpcWeights weights;
};

struct Reference {
    std::array<double, kLatMpcNodes> y{};
    std::array<double, kLatMpcNodes> heading{};
};

Reference build_reference(const Scenario &s) {
    Reference ref;
    const double horizon = model_t_idx_double(kLatMpcN);
    for (int i = 0; i < kLatMpcNodes; ++i) {
        const double p = horizon > 0.0 ? model_t_idx_double(i) / horizon : 0.0;
        ref.y[i] = s.amplitude * (1.0 - std::cos(3.14159265358979 * p));
    }
    for (int i = 0; i < kLatMpcNodes; ++i) {
        const int prev = i > 0 ? i - 1 : 0;
        const int next = i < kLatMpcN ? i + 1 : kLatMpcN;
        const double dt = model_t_idx_double(next) - model_t_idx_double(prev);
        const double dx = std::max(0.1, s.v_ego) * dt;
        ref.heading[i] = dt > 0.0 ? (ref.y[next] - ref.y[prev]) / dx : 0.0;
    }
    return ref;
}

// 솔버와 독립인 RK4 적분. 상태는 (y, psi, curvature).
void simulate_step(const double *z, double u, double v, double rr, double dt,
                   double *out) {
    auto f = [&](const double *s, double *d) {
        d[0] = v * std::sin(s[1]) + rr * std::cos(s[1]) * (v * s[2]);
        d[1] = v * s[2];
        d[2] = u;
    };
    double k1[3], k2[3], k3[3], k4[3], tmp[3];
    f(z, k1);
    for (int r = 0; r < 3; ++r) tmp[r] = z[r] + 0.5 * dt * k1[r];
    f(tmp, k2);
    for (int r = 0; r < 3; ++r) tmp[r] = z[r] + 0.5 * dt * k2[r];
    f(tmp, k3);
    for (int r = 0; r < 3; ++r) tmp[r] = z[r] + dt * k3[r];
    f(tmp, k4);
    for (int r = 0; r < 3; ++r)
        out[r] = z[r] + dt / 6.0 * (k1[r] + 2.0 * k2[r] + 2.0 * k3[r] + k4[r]);
}

/* 입력열만으로 결정되는 참 목적함수. 상태를 전방 적분으로 소거했으므로
 * 결손이 0인 수렴점에서는 솔버가 푸는 NLP와 같은 값이다. */
double objective(const std::array<double, kLatMpcN> &u, const Scenario &s,
                 const Reference &ref, const LateralMpcWeights &w) {
    const double v = s.v_ego;
    const double speed = v + 5.0;
    double z[3] = {0.0, 0.0, s.curvature0};
    double total = 0.0;
    for (int i = 0; i < kLatMpcNodes; ++i) {
        const bool terminal = i == kLatMpcN;
        const double scale = terminal ? w.terminal : LateralMpc::time_step(i);
        const double e_y = z[0] - ref.y[i];
        const double e_psi = speed * (z[1] - ref.heading[i]);
        double stage = w.path * e_y * e_y + w.heading * e_psi * e_psi;
        if (!terminal) {
            const double rate = u[i];
            stage += w.rate * (4.0 * speed * rate) * (4.0 * speed * rate) +
                     w.steering_rate * rate * rate;
        }
        total += 0.5 * scale * stage;
        if (!terminal) {
            double next[3];
            simulate_step(z, u[i], v, s.rotation_radius, LateralMpc::time_step(i),
                          next);
            for (int r = 0; r < 3; ++r) z[r] = next[r];
        }
    }
    return total;
}

void check_scenario(const Scenario &s) {
    const Reference ref = build_reference(s);

    LateralMpc mpc;
    // 20 Hz 워밍스타트를 흉내내 같은 입력으로 반복 수렴시킨다.
    for (int iter = 0; iter < 60; ++iter)
        mpc.run(s.curvature0, s.v_ego, s.rotation_radius, ref.y, ref.heading,
                s.weights);
    EXPECT_EQ(mpc.status(), 0);

    // 초기 상태 등식 제약.
    const auto &nodes = mpc.nodes();
    double init_err = std::fabs(nodes[0].y) + std::fabs(nodes[0].psi) +
                      std::fabs(nodes[0].curvature - s.curvature0);
    EXPECT_LT(init_err, 1e-12) << "초기 상태 제약 오차";

    // 다중슈팅 결손: 수렴점에서 궤적이 동역학적으로 타당해야 한다.
    double max_defect = 0.0;
    for (int i = 0; i < kLatMpcN; ++i) {
        const double z[3] = {nodes[i].y, nodes[i].psi, nodes[i].curvature};
        double next[3];
        simulate_step(z, mpc.rates()[i], s.v_ego, s.rotation_radius,
                      LateralMpc::time_step(i), next);
        max_defect = std::max(max_defect, std::fabs(next[0] - nodes[i + 1].y));
        max_defect = std::max(max_defect, std::fabs(next[1] - nodes[i + 1].psi));
        max_defect = std::max(max_defect, std::fabs(next[2] - nodes[i + 1].curvature));
    }
    EXPECT_LT(max_defect, 1e-10) << "결손 최대값";

    /* 참 목적함수의 기울기. 스케일이 커서 절대값 대신 코스트에 대한
     * 상대 기울기를 본다(입력 1e-6 변화당 코스트 변화 비율). */
    auto u = mpc.rates();
    const double base = objective(u, s, ref, s.weights);
    double max_grad = 0.0;
    for (int i = 0; i < kLatMpcN; ++i) {
        const double step = 1e-6;
        auto plus = u, minus = u;
        plus[i] += step;
        minus[i] -= step;
        const double grad = (objective(plus, s, ref, s.weights) -
                             objective(minus, s, ref, s.weights)) /
                            (2.0 * step);
        max_grad = std::max(max_grad, std::fabs(grad));
    }
    const double relative = max_grad / std::max(1.0, base);
    EXPECT_LT(relative, 1e-6) << "상대 기울기 최대값";
    // 솔버가 보고하는 코스트가 독립 계산과 같아야 한다.
    const double cost_err = std::fabs(mpc.cost() - base) / std::max(1.0, base);
    EXPECT_LT(cost_err, 1e-12) << "코스트 상대 오차";
}

TEST(LateralMpc, OptimizesTheOcp) {
    // 저속은 heading 1.0, 10 m/s 이상은 0.15. 플래너 스케줄과 같다.
    LateralMpcWeights slow;
    LateralMpcWeights fast;
    fast.heading = 0.15;
    // steering_rate를 끈 설정. 0.8.16 원본 코스트와 같다.
    LateralMpcWeights bare = fast;
    bare.steering_rate = 0.0;
    LateralMpcWeights bare_slow = slow;
    bare_slow.steering_rate = 0.0;

    const Scenario scenarios[] = {
        {"정지", 0.0, 0.0, 0.0, 0.0, slow},
        {"저속 직선", 3.0, 0.4, 0.0, 0.2, slow},
        {"시내 커브", 12.0, 0.5, 0.02, 1.0, fast},
        {"고속 차선변경", 27.0, 0.6, -0.005, 3.5, fast},
        {"급곡률 초기값", 20.0, 0.5, 0.25, 0.0, fast},
        {"steering_rate 없음(저속)", 3.0, 0.4, 0.0, 0.2, bare_slow},
        {"steering_rate 없음(고속)", 20.0, 0.5, 0.0, 2.0, bare},
        {"steering_rate 2000", 3.0, 0.4, 0.0, 0.3, [&] {
            LateralMpcWeights w = slow;
            w.steering_rate = 2000.0;
            return w;
        }()},
    };
    for (const auto &s : scenarios) {
        SCOPED_TRACE(s.name);
        check_scenario(s);
    }
}

}  // namespace
