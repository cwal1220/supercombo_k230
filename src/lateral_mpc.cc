#include "lateral_mpc.h"

#include "model_output.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kNx = 3;  // y, psi, curvature
constexpr int kY = 0, kPsi = 1, kCurv = 2;

// openpilot idxbx=[2,3]의 radians(90)/radians(50). 실주행에서 활성되지 않는다.
constexpr double kPsiLimit = 1.5707963267948966;
constexpr double kCurvatureLimit = 0.8726646259971648;

// 잔차 3번 항의 (v_ego + 5) 배율. openpilot의 hacky weight 그대로다.
constexpr double kSpeedOffset = 5.0;
constexpr double kRateScale = 4.0;

using Mat = double[kNx][kNx];

void dynamics(const double *z, double u, double v, double rr, double *out) {
    const double sin_psi = std::sin(z[kPsi]);
    const double cos_psi = std::cos(z[kPsi]);
    const double psi_rate = v * z[kCurv];
    out[kY] = v * sin_psi + rr * cos_psi * psi_rate;
    out[kPsi] = psi_rate;
    out[kCurv] = u;
}

// df/dz. 0이 아닌 항만 있다.
void jacobian(const double *z, double v, double rr, Mat jac) {
    const double sin_psi = std::sin(z[kPsi]);
    const double cos_psi = std::cos(z[kPsi]);
    for (int r = 0; r < kNx; ++r)
        for (int c = 0; c < kNx; ++c) jac[r][c] = 0.0;
    jac[kY][kPsi] = v * cos_psi - rr * sin_psi * (v * z[kCurv]);
    jac[kY][kCurv] = rr * cos_psi * v;
    jac[kPsi][kCurv] = v;
}

/* acados ERK(4단 1스텝) 및 forward VDE와 같은 적분. z_next와 함께
 * A = dz_next/dz, B = dz_next/du를 같은 RK4 스테이지에서 전파한다. */
void rk4_step(const double *z, double u, double v, double rr, double dt,
              double *z_next, Mat a_out, double *b_out) {
    // du/dz 시드는 (0,0,1): curvature_rate가 곧 curvature의 미분이다.
    static constexpr double kFu[kNx] = {0.0, 0.0, 1.0};
    const double half = 0.5 * dt;

    double arg[kNx];
    double k[4][kNx];
    Mat jac;
    // 스테이지 인자의 감도. sx = d(arg)/dz, su = d(arg)/du.
    Mat sx[4];
    double su[4][kNx];

    for (int stage = 0; stage < 4; ++stage) {
        const double scale = stage == 0 ? 0.0 : (stage == 3 ? dt : half);
        for (int r = 0; r < kNx; ++r)
            arg[r] = stage == 0 ? z[r] : z[r] + scale * k[stage - 1][r];
        dynamics(arg, u, v, rr, k[stage]);
        jacobian(arg, v, rr, jac);

        // d(arg)/d(z,u)를 만든 뒤 야코비안을 곱해 스테이지 감도를 얻는다.
        Mat dargdz;
        double dargdu[kNx];
        for (int r = 0; r < kNx; ++r) {
            for (int c = 0; c < kNx; ++c)
                dargdz[r][c] = (r == c ? 1.0 : 0.0) +
                               (stage == 0 ? 0.0 : scale * sx[stage - 1][r][c]);
            dargdu[r] = stage == 0 ? 0.0 : scale * su[stage - 1][r];
        }
        for (int r = 0; r < kNx; ++r) {
            for (int c = 0; c < kNx; ++c) {
                double acc = 0.0;
                for (int m = 0; m < kNx; ++m) acc += jac[r][m] * dargdz[m][c];
                sx[stage][r][c] = acc;
            }
            double acc = kFu[r];
            for (int m = 0; m < kNx; ++m) acc += jac[r][m] * dargdu[m];
            su[stage][r] = acc;
        }
    }

    const double sixth = dt / 6.0;
    for (int r = 0; r < kNx; ++r) {
        z_next[r] = z[r] + sixth * (k[0][r] + 2.0 * k[1][r] + 2.0 * k[2][r] + k[3][r]);
        b_out[r] = sixth * (su[0][r] + 2.0 * su[1][r] + 2.0 * su[2][r] + su[3][r]);
        for (int c = 0; c < kNx; ++c)
            a_out[r][c] = (r == c ? 1.0 : 0.0) +
                          sixth * (sx[0][r][c] + 2.0 * sx[1][r][c] +
                                   2.0 * sx[2][r][c] + sx[3][r][c]);
    }
}

}  // namespace

double LateralMpc::time_step(int i) {
    return model_t_idx_double(i + 1) - model_t_idx_double(i);
}

void LateralMpc::reset() {
    x_ = {};
    u_ = {};
    cost_ = 0.0;
    status_ = 0;
}

void LateralMpc::run(double curvature, double v_ego, double rotation_radius,
                     const std::array<double, kLatMpcNodes> &y_ref,
                     const std::array<double, kLatMpcNodes> &heading_ref,
                     const LateralMpcWeights &weights) {
    const double speed = v_ego + kSpeedOffset;
    // 잔차 스케일을 흡수한 상태/입력 가중치.
    const double w_y = weights.path;
    const double w_psi = weights.heading * speed * speed;
    const double w_u = weights.rate * (kRateScale * speed) * (kRateScale * speed);

    Mat a[kLatMpcN];
    double b[kLatMpcN][kNx];
    double defect[kLatMpcN][kNx];
    Mat q[kLatMpcNodes];
    double grad[kLatMpcNodes][kNx];
    double r_uu[kLatMpcN];
    double r_u[kLatMpcN];

    status_ = 0;
    // 선형화: 이전 해를 따라 적분해 감도와 다중슈팅 결손을 모은다.
    for (int i = 0; i < kLatMpcN; ++i) {
        const double z[kNx] = {x_[i].y, x_[i].psi, x_[i].curvature};
        double next[kNx];
        rk4_step(z, u_[i], v_ego, rotation_radius, time_step(i), next, a[i], b[i]);
        defect[i][kY] = next[kY] - x_[i + 1].y;
        defect[i][kPsi] = next[kPsi] - x_[i + 1].psi;
        defect[i][kCurv] = next[kCurv] - x_[i + 1].curvature;
    }

    // 코스트: 스테이지는 dt로, 종단은 terminal 배율로 스케일한다(acados scaling).
    for (int i = 0; i < kLatMpcNodes; ++i) {
        const bool terminal = i == kLatMpcN;
        const double scale = terminal ? weights.terminal : time_step(i);
        for (int r = 0; r < kNx; ++r) {
            grad[i][r] = 0.0;
            for (int c = 0; c < kNx; ++c) q[i][r][c] = 0.0;
        }
        q[i][kY][kY] = scale * w_y;
        q[i][kPsi][kPsi] = scale * w_psi;
        grad[i][kY] = scale * w_y * (x_[i].y - y_ref[i]);
        grad[i][kPsi] = scale * w_psi * (x_[i].psi - heading_ref[i]);
        if (!terminal) {
            r_uu[i] = scale * w_u;
            r_u[i] = scale * w_u * u_[i];
        }
    }

    // 후방 Riccati. 값함수 0.5*dz'P dz + p'dz.
    Mat p_mat;
    double p_vec[kNx];
    double gain_k[kLatMpcN];
    double gain_K[kLatMpcN][kNx];
    for (int r = 0; r < kNx; ++r) {
        p_vec[r] = grad[kLatMpcN][r];
        for (int c = 0; c < kNx; ++c) p_mat[r][c] = q[kLatMpcN][r][c];
    }
    for (int i = kLatMpcN - 1; i >= 0; --i) {
        // pb = P*(A dz + B du + d) 전개에 쓰는 항들.
        double pd[kNx];  // P*d + p
        for (int r = 0; r < kNx; ++r) {
            double acc = p_vec[r];
            for (int c = 0; c < kNx; ++c) acc += p_mat[r][c] * defect[i][c];
            pd[r] = acc;
        }
        double pb[kNx];  // P*B
        for (int r = 0; r < kNx; ++r) {
            double acc = 0.0;
            for (int c = 0; c < kNx; ++c) acc += p_mat[r][c] * b[i][c];
            pb[r] = acc;
        }
        double h_uu = r_uu[i];
        for (int r = 0; r < kNx; ++r) h_uu += b[i][r] * pb[r];
        if (!(h_uu > 0.0)) {
            reset();
            status_ = 2;
            return;
        }
        double h_uz[kNx];  // B'*P*A
        for (int c = 0; c < kNx; ++c) {
            double acc = 0.0;
            for (int r = 0; r < kNx; ++r) acc += pb[r] * a[i][r][c];
            h_uz[c] = acc;
        }
        double h_u = r_u[i];
        for (int r = 0; r < kNx; ++r) h_u += b[i][r] * pd[r];

        gain_k[i] = -h_u / h_uu;
        for (int c = 0; c < kNx; ++c) gain_K[i][c] = -h_uz[c] / h_uu;

        // P <- Q + A'PA + h_uz' K, p <- q + A'(P(B k + d) + p)
        Mat pa;
        for (int r = 0; r < kNx; ++r)
            for (int c = 0; c < kNx; ++c) {
                double acc = 0.0;
                for (int m = 0; m < kNx; ++m) acc += p_mat[r][m] * a[i][m][c];
                pa[r][c] = acc;
            }
        Mat next_p;
        for (int r = 0; r < kNx; ++r)
            for (int c = 0; c < kNx; ++c) {
                double acc = q[i][r][c] + h_uz[r] * gain_K[i][c];
                for (int m = 0; m < kNx; ++m) acc += a[i][m][r] * pa[m][c];
                next_p[r][c] = acc;
            }
        double next_p_vec[kNx];
        for (int r = 0; r < kNx; ++r) {
            double acc = grad[i][r];
            for (int m = 0; m < kNx; ++m)
                acc += a[i][m][r] * (pd[m] + pb[m] * gain_k[i]);
            next_p_vec[r] = acc;
        }
        for (int r = 0; r < kNx; ++r) {
            p_vec[r] = next_p_vec[r];
            // 수치 대칭성을 유지한다.
            for (int c = 0; c <= r; ++c) {
                const double sym = 0.5 * (next_p[r][c] + next_p[c][r]);
                p_mat[r][c] = sym;
                p_mat[c][r] = sym;
            }
        }
    }

    // 전방 스텝. 초기 상태는 등식 제약이라 증분이 목표와의 차이로 고정된다.
    double dz[kNx] = {-x_[0].y, -x_[0].psi, curvature - x_[0].curvature};
    for (int i = 0; i < kLatMpcN; ++i) {
        double du = gain_k[i];
        for (int c = 0; c < kNx; ++c) du += gain_K[i][c] * dz[c];
        double next_dz[kNx];
        for (int r = 0; r < kNx; ++r) {
            double acc = b[i][r] * du + defect[i][r];
            for (int c = 0; c < kNx; ++c) acc += a[i][r][c] * dz[c];
            next_dz[r] = acc;
        }
        x_[i].y += dz[kY];
        x_[i].psi += dz[kPsi];
        x_[i].curvature += dz[kCurv];
        u_[i] += du;
        for (int r = 0; r < kNx; ++r) dz[r] = next_dz[r];
    }
    x_[kLatMpcN].y += dz[kY];
    x_[kLatMpcN].psi += dz[kPsi];
    x_[kLatMpcN].curvature += dz[kCurv];

    for (int i = 0; i < kLatMpcNodes; ++i) {
        if (!std::isfinite(x_[i].y) || !std::isfinite(x_[i].psi) ||
            !std::isfinite(x_[i].curvature) ||
            (i < kLatMpcN && !std::isfinite(u_[i]))) {
            reset();
            status_ = 1;
            return;
        }
        x_[i].psi = std::clamp(x_[i].psi, -kPsiLimit, kPsiLimit);
        x_[i].curvature = std::clamp(x_[i].curvature, -kCurvatureLimit, kCurvatureLimit);
    }

    // acados의 cost_value와 같은 정의. 새 반복점에서 평가한다.
    cost_ = 0.0;
    for (int i = 0; i < kLatMpcNodes; ++i) {
        const bool terminal = i == kLatMpcN;
        const double scale = terminal ? weights.terminal : time_step(i);
        const double e_y = x_[i].y - y_ref[i];
        const double e_psi = x_[i].psi - heading_ref[i];
        double stage = w_y * e_y * e_y + w_psi * e_psi * e_psi;
        if (!terminal) stage += w_u * u_[i] * u_[i];
        cost_ += 0.5 * scale * stage;
    }
}
