#ifndef LATERAL_MPC_H
#define LATERAL_MPC_H

/* openpilot 0.8.16 lateral_mpc_lib와 같은 OCP를 푸는 축소 SQP 솔버.
 * 상태 (y, psi, curvature), 입력 curvature_rate, 노드 17개(T_IDXS 격자).
 * acados가 풀던 x_ego 축은 코스트/제약/다른 상태에 전혀 안 쓰여 빼냈다.
 * 조향률 코스트 항은 0.9.4 lat_mpc에서 가져왔다. */

#include <array>

constexpr int kLatMpcN = 16;
constexpr int kLatMpcNodes = kLatMpcN + 1;

struct LateralMpcNode {
    double y = 0.0;
    double psi = 0.0;
    double curvature = 0.0;
};

/* openpilot set_weights와 같은 항. a = v_ego + 5로 스케일한 잔차에 걸린다.
 *   path          y
 *   heading       a*psi         (목표 a*heading)
 *   rate          4a*curv_rate  속도에 비례하는 0.8.16의 항
 *   steering_rate curv_rate     속도와 무관, 0.9.4의 STEERING_RATE_COST
 * 종단 노드는 (path, heading)에만 terminal 배율로 걸린다.
 * steering_rate는 0.9.4의 psi_accel/(v+0.1)과 같은 양이라 가중치가 그대로
 * 옮겨온다. 700은 실차 3세그먼트 재생에서 지령 저크 -25%(20 kph 이하 -27%),
 * 편차는 20 kph 이하에 4.1e-3로 국한되고 50 kph 이상은 7e-5다. */
struct LateralMpcWeights {
    double path = 1.0;
    double heading = 1.0;
    double rate = 1.0;
    double terminal = 0.15;
    double steering_rate = 700.0;
};

class LateralMpc {
public:
    LateralMpc() { reset(); }

    // 궤적과 워밍스타트를 버린다. 다음 run의 초기 curvature는 인자로 들어온다.
    void reset();

    /* 이전 해에서 선형화해 LQ 부문제를 Riccati로 정확히 풀고 전체 스텝을
     * 적용한다(acados SQP_RTI와 같은 구조). 잔차가 상태/입력에 선형이라 1회로
     * 수렴하며, 반복을 늘려도 얻는 게 없다(verification.md 참고).
     * y/psi 초기 상태는 자차 기준 0으로 고정이다. */
    void run(double curvature, double v_ego, double rotation_radius,
             const std::array<double, kLatMpcNodes> &y_ref,
             const std::array<double, kLatMpcNodes> &heading_ref,
             const LateralMpcWeights &weights);

    const std::array<LateralMpcNode, kLatMpcNodes> &nodes() const { return x_; }
    const std::array<double, kLatMpcN> &rates() const { return u_; }
    double cost() const { return cost_; }
    // 0이 아니면 해를 버려야 한다. 1 = 비유한 값, 2 = 헤시안 비양정.
    int status() const { return status_; }

    // 노드 i의 적분 구간 길이. T_IDXS 간격이다.
    static double time_step(int i);

private:
    std::array<LateralMpcNode, kLatMpcNodes> x_{};
    std::array<double, kLatMpcN> u_{};
    double cost_ = 0.0;
    int status_ = 0;
};

#endif
