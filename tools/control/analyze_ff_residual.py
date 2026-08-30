#!/usr/bin/env python3
"""현행 선형 FF의 잔차에 NNFF가 잡을 구조가 남아 있는지 검사한다.

extract_lateral_dataset가 만든 CSV를 받아, 실제 적용 토크를 다섯 모델로
예측하고 블록 홀드아웃 RMSE를 비교한다. 모두 "측정된 상태에서 필요한
토크"를 맞히는 plant inverse 문제이며, 이는 NNFF 학습 목표와 같다.

  M0  현행 FF     steering.json 그대로. 적합 없음.
  M1  선형 재적합  기울기/절편만 데이터에 맞춘다.
  M2  선형+마찰    M1에 저크 부호 기반 마찰항을 더한다. 현행 구조의 상한.
  M3  선형 18입력  NNFF와 같은 입력을 주되 선형으로만 푼다.
  M4  NNFF급 MLP  sunnypilot NNLC와 같은 18-7-13-1 sigmoid 망.
  M5  구간 평균    M3 잔차를 (속도, 횡가속, 저크) 격자에서 평균해 더한다.
                  매끄러운 함수가 뽑아낼 수 있는 구조의 비모수 상한.

M2 -> M3은 "입력을 늘려서" 얻는 이득이고, M3 -> M4/M5가 "비선형이라서"
얻는 이득이다. NNFF 고유의 기여는 뒤쪽이다.

사용:
  python3 analyze_ff_residual.py <lateral_dataset.csv...>
"""

import argparse
import sys

import numpy as np

GRAVITY = 9.8
# NNFF 입력 시간축 (openpilot NNLC past_times / future_times)
PAST_TIMES = [-0.3, -0.2, -0.1]
FUTURE_TIMES = [0.3, 0.6, 1.0, 1.5]
# 시간 이동 표본이 이 이상 어긋나면 그 행을 버린다
SHIFT_TOLERANCE_S = 0.05
# 저크 평활 창(초)과 마찰 포화 문턱(openpilot FRICTION_THRESHOLD)
JERK_WINDOW_S = 0.10
FRICTION_THRESHOLD = 0.2
BLOCK_SECONDS = 60.0
FOLDS = 5


def load(paths):
    frames = []
    for path in paths:
        d = np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                          encoding="utf-8")
        frames.append(d)
        print(f"  {path}: {len(d)}행")
    return frames


def shift_series(t, values, offset, tolerance=SHIFT_TOLERANCE_S):
    """t+offset 시점의 값과 그 표본이 실제로 존재하는지 여부를 돌려준다."""
    idx = np.clip(np.searchsorted(t, t + offset), 0, len(t) - 1)
    ok = np.abs(t[idx] - (t + offset)) <= tolerance
    return values[idx], ok


def build_features(d, args):
    """한 route CSV에서 NNFF 입력 18개와 목표 토크를 만든다."""
    t = d["t"]
    v = d["v"]
    lat = d["lat"]
    roll = d["roll"]

    # 저크: 평활 후 중심 차분. 원신호는 300 Hz ESP12를 62 Hz로 뽑은 것이라
    # 그대로 미분하면 양자화 잡음이 지배한다.
    window = max(1, int(round(JERK_WINDOW_S * len(t) / max(t[-1] - t[0], 1e-6))))
    kernel = np.ones(window) / window
    lat_smooth = np.convolve(lat, kernel, mode="same")
    jerk = np.gradient(lat_smooth, t)

    columns = [v, lat, jerk, roll]
    valid = np.ones(len(t), dtype=bool)
    for offset in PAST_TIMES:
        shifted, ok = shift_series(t, lat, offset)
        columns.append(shifted)
        valid &= ok
    if args.future_source == "plan":
        # 추론 시점과 같은 조건: 미래값을 녹화된 모델 플랜에서 가져온다.
        for name in ("la_p03", "la_p06", "la_p10", "la_p15"):
            columns.append(d[name])
        valid &= d["model_age"] >= 0.0
        valid &= d["model_age"] < 0.2
    else:
        for offset in FUTURE_TIMES:
            shifted, ok = shift_series(t, lat, offset)
            columns.append(shifted)
            valid &= ok
    for offset in PAST_TIMES:
        shifted, ok = shift_series(t, roll, offset)
        columns.append(shifted)
        valid &= ok
    if args.future_source == "plan":
        for name in ("roll_p03", "roll_p06", "roll_p10", "roll_p15"):
            columns.append(d[name])
    else:
        for offset in FUTURE_TIMES:
            shifted, ok = shift_series(t, roll, offset)
            columns.append(shifted)
            valid &= ok

    x = np.column_stack(columns)
    y = d["tq_norm"]

    valid &= d["active"] == 1
    valid &= d["desire"] == 0
    valid &= d["lat_valid"] == 1
    valid &= np.abs(d["driver_tq"]) < args.driver_max
    valid &= v >= args.min_speed
    valid &= np.abs(y) < 0.98          # 포화 구간 제외
    valid &= np.isfinite(x).all(axis=1)
    valid &= np.isfinite(y)

    block = np.floor((t - t[0]) / BLOCK_SECONDS).astype(int)
    return x[valid], y[valid], block[valid], d["bank"][valid]


def fold_indices(blocks, folds=FOLDS):
    """연속 구간 단위로 나눈다. 62 Hz 인접 표본은 서로 거의 같은 정보라
    무작위 분할을 쓰면 비선형 모델이 자기 자신을 훔쳐본다."""
    unique = np.unique(blocks)
    for fold in range(folds):
        held = unique[fold::folds]
        test = np.isin(blocks, held)
        yield ~test, test


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def fit_linear(x, y):
    a = np.column_stack([x, np.ones(len(x))])
    coef, *_ = np.linalg.lstsq(a, y, rcond=None)
    return coef


def apply_linear(coef, x):
    return np.column_stack([x, np.ones(len(x))]) @ coef


class Mlp:
    """NNLC 모델 파일과 같은 구조: 18-7(σ)-13(σ)-1(identity).

    NNLC는 뒤쪽을 13-3-1 두 층으로 나누지만 둘 다 identity라 하나의 선형
    사상과 같다. 파라미터 수도 283개로 동일하다."""

    def __init__(self, n_in, seed=0):
        rng = np.random.default_rng(seed)
        sizes = [(n_in, 7), (7, 13), (13, 1)]
        self.w = [rng.normal(0, np.sqrt(2.0 / a), (a, b)) for a, b in sizes]
        self.b = [np.zeros(b) for _, b in sizes]

    def forward(self, x):
        h1 = 1.0 / (1.0 + np.exp(-(x @ self.w[0] + self.b[0])))
        h2 = 1.0 / (1.0 + np.exp(-(h1 @ self.w[1] + self.b[1])))
        return h1, h2, (h2 @ self.w[2] + self.b[2]).ravel()

    def predict(self, x):
        return self.forward(x)[2]

    def fit(self, x, y, epochs=120, batch=4096, lr=0.01, seed=0):
        rng = np.random.default_rng(seed)
        m = [np.zeros_like(p) for p in self.w + self.b]
        vel = [np.zeros_like(p) for p in self.w + self.b]
        step = 0
        for _ in range(epochs):
            order = rng.permutation(len(x))
            for start in range(0, len(x), batch):
                idx = order[start:start + batch]
                xb, yb = x[idx], y[idx]
                h1, h2, out = self.forward(xb)
                n = len(idx)
                d_out = (2.0 / n) * (out - yb)[:, None]
                grad_w3 = h2.T @ d_out
                grad_b3 = d_out.sum(axis=0)
                d_h2 = (d_out @ self.w[2].T) * h2 * (1 - h2)
                grad_w2 = h1.T @ d_h2
                grad_b2 = d_h2.sum(axis=0)
                d_h1 = (d_h2 @ self.w[1].T) * h1 * (1 - h1)
                grad_w1 = xb.T @ d_h1
                grad_b1 = d_h1.sum(axis=0)
                grads = [grad_w1, grad_w2, grad_w3, grad_b1, grad_b2, grad_b3]
                params = self.w + self.b
                step += 1
                for i, (p, g) in enumerate(zip(params, grads)):
                    m[i] = 0.9 * m[i] + 0.1 * g
                    vel[i] = 0.999 * vel[i] + 0.001 * g * g
                    m_hat = m[i] / (1 - 0.9 ** step)
                    v_hat = vel[i] / (1 - 0.999 ** step)
                    p -= lr * m_hat / (np.sqrt(v_hat) + 1e-8)
        return self


def binned_mean(x_train, y_train, x_test, edges):
    """(속도, 횡가속, 저크) 격자의 조건부 평균. 빈 칸은 전체 평균으로 채운다."""
    def digitize(x):
        return tuple(np.clip(np.digitize(x[:, col], e), 0, len(e))
                     for col, e in edges)

    keys_train = digitize(x_train)
    keys_test = digitize(x_test)
    shape = tuple(len(e) + 1 for _, e in edges)
    flat_train = np.ravel_multi_index(keys_train, shape)
    flat_test = np.ravel_multi_index(keys_test, shape)
    total = np.bincount(flat_train, weights=y_train, minlength=np.prod(shape))
    count = np.bincount(flat_train, minlength=np.prod(shape))
    fallback = 0.0  # 표본이 적은 칸은 보정하지 않는다
    means = np.where(count >= 20, total / np.maximum(count, 1), fallback)
    return means[flat_test]


def gain_table(x, y, bank):
    """속도 x 토크 구간별 실효 latAccelFactor. fit 도구와 같은 토크 버킷을
    쓴다. 횡가속으로 나누면 0 근처 칸에서 비가 발산한다."""
    v, lat = x[:, 0], x[:, 1]
    compensated = lat - bank
    speed_bins = [(5, 15), (15, 22), (22, 28), (28, 99)]
    torque_bins = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1),
                   (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
    print("\n실효 latAccelFactor (m/s^2 per unit torque, n>=200인 칸만)")
    print("  속도\\토크    " + "".join(f"{lo:+.1f}~{hi:+.1f}".rjust(13)
                                      for lo, hi in torque_bins))
    for slo, shi in speed_bins:
        row = f"  {slo*3.6:3.0f}-{shi*3.6:3.0f} kph ".ljust(14)
        for tlo, thi in torque_bins:
            m = (v >= slo) & (v < shi) & (y >= tlo) & (y < thi)
            if m.sum() < 200:
                row += "-".rjust(13)
                continue
            row += f"{compensated[m].mean() / y[m].mean():+.2f}({m.sum()})".rjust(13)
        print(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+")
    parser.add_argument("--max-lat-accel", type=float, default=3.5,
                        help="torque_max_lat_accel_raw / 10")
    parser.add_argument("--kf-raw", type=float, default=8.0)
    parser.add_argument("--friction", type=float, default=0.1,
                        help="torque_friction_raw / 1000")
    parser.add_argument("--min-speed", type=float, default=5.0)
    parser.add_argument("--driver-max", type=float, default=100.0)
    parser.add_argument("--steer-max", type=float, default=384.0)
    parser.add_argument("--future-source", choices=("measured", "plan"),
                        default="measured",
                        help="미래 횡가속을 실측 시계열에서 당겨올지, 녹화된 "
                             "모델 플랜에서 뽑을지. plan이 실제 추론 조건이다")
    parser.add_argument("--curve-threshold", type=float, default=0.5,
                        help="이 값을 넘는 |편경사 보정 횡가속|을 커브로 본다")
    args = parser.parse_args()

    print("입력:")
    frames = load(args.csv)
    parts = [build_features(d, args) for d in frames]
    offset = 0
    xs, ys, blocks, banks = [], [], [], []
    for x, y, block, bank in parts:
        xs.append(x)
        ys.append(y)
        blocks.append(block + offset)
        banks.append(bank)
        offset += block.max() + 1
    x = np.vstack(xs)
    y = np.concatenate(ys)
    block = np.concatenate(blocks)
    bank = np.concatenate(banks)
    if len(x) < 1000:
        print("표본이 부족하다.")
        return 1
    print(f"\n유효 표본 {len(x)}개 / {len(x) / 62.0 / 60.0:.1f}분, "
          f"블록 {len(np.unique(block))}개")

    gain_table(x, y, bank)

    kf = args.kf_raw * 0.1 / args.max_lat_accel
    names = ("M0", "M1", "M2", "M3", "M4", "M5")
    scores = {name: [] for name in names}
    curve_scores = {name: [] for name in names}
    curve_rows = 0
    for train, test in fold_indices(block):
        x_tr, y_tr = x[train], y[train]
        x_te, y_te = x[test], y[test]
        bank_te = bank[test]
        # 커브 구간만 따로 본다. 직선이 표본의 대부분이라 전체 RMSE는
        # 커브에서의 차이를 희석한다.
        curve = np.abs(x_te[:, 1] - bank_te) > args.curve_threshold
        curve_rows += int(curve.sum())

        def record(name, prediction):
            scores[name].append(rmse(y_te, prediction))
            if curve.sum() >= 50:
                curve_scores[name].append(rmse(y_te[curve], prediction[curve]))

        # M0: 현행 FF. 측정 횡가속에서 편경사를 빼고 kf를 곱한다.
        record("M0", kf * (x_te[:, 1] - bank_te))

        # M1: 같은 입력 하나로 기울기/절편만 재적합
        feat_tr = (x_tr[:, 1] - bank[train])[:, None]
        feat_te = (x_te[:, 1] - bank_te)[:, None]
        coef = fit_linear(feat_tr, y_tr)
        record("M1", apply_linear(coef, feat_te))

        # M2: 마찰항 추가. openpilot과 같이 저크를 문턱에서 포화시킨다.
        def friction_feature(xx):
            return np.clip(xx[:, 2] / FRICTION_THRESHOLD, -1.0, 1.0)[:, None]
        feat2_tr = np.hstack([feat_tr, friction_feature(x_tr)])
        feat2_te = np.hstack([feat_te, friction_feature(x_te)])
        coef2 = fit_linear(feat2_tr, y_tr)
        record("M2", apply_linear(coef2, feat2_te))

        # M3: NNFF와 같은 18입력을 선형으로만 푼다
        coef3 = fit_linear(x_tr, y_tr)
        pred3_tr = apply_linear(coef3, x_tr)
        pred3_te = apply_linear(coef3, x_te)
        record("M3", pred3_te)

        # M4: NNFF급 MLP. NNLC와 같이 입력을 표준화한다.
        mean, std = x_tr.mean(axis=0), x_tr.std(axis=0) + 1e-6
        net = Mlp(x.shape[1]).fit((x_tr - mean) / std, y_tr)
        record("M4", net.predict((x_te - mean) / std))

        # M5: M3 잔차를 격자에서 평균해 보정 (비모수 상한)
        edges = [(0, np.array([15, 20, 25, 30])),
                 (1, np.array([-1.5, -1.0, -0.6, -0.3, -0.1, 0.1, 0.3, 0.6, 1.0, 1.5])),
                 (2, np.array([-0.5, -0.15, 0.15, 0.5]))]
        correction = binned_mean(x_tr, y_tr - pred3_tr, x_te, edges)
        record("M5", pred3_te + correction)

    labels = {
        "M0": "현행 FF (steering.json)",
        "M1": "선형 재적합",
        "M2": "선형 + 마찰",
        "M3": "선형 18입력",
        "M4": "NNFF급 MLP (18-7-13-1)",
        "M5": "구간 평균 (비모수 상한)",
    }
    print(f"\n블록 홀드아웃 RMSE ({FOLDS} fold, {BLOCK_SECONDS:.0f}s 블록)")
    print("  모델                       정규화토크  fold편차   raw(±384)  환산 m/s^2")
    for name, label in labels.items():
        value = float(np.mean(scores[name]))
        spread = float(np.std(scores[name]))
        print(f"  {label:25s} {value:9.4f} {spread:9.4f} {value * args.steer_max:10.1f} "
              f"{value / kf:11.3f}")
    def summarize(table, title):
        linear = float(np.mean(table["M3"]))
        best = min(float(np.mean(table["M4"])), float(np.mean(table["M5"])))
        current = float(np.mean(table["M2"]))
        print(f"\n  {title}")
        print(f"    입력 확장 이득 (M2 -> M3):  {(current - linear) / current * 100:+5.1f}%"
              f"  {(current - linear) * args.steer_max:+6.1f} raw")
        print(f"    비선형 이득   (M3 -> M4/M5): {(linear - best) / linear * 100:+5.1f}%"
              f"  {(linear - best) * args.steer_max:+6.1f} raw   <- NNFF 고유 기여")

    summarize(scores, "전체")
    if curve_scores["M3"]:
        share = 100.0 * curve_rows / len(x)
        print(f"\n커브 구간만 (|보정 횡가속| > {args.curve_threshold}, 표본의 {share:.0f}%)")
        print("  모델                       정규화토크             raw(±384)")
        for name, label in labels.items():
            value = float(np.mean(curve_scores[name]))
            print(f"  {label:25s} {value:9.4f}          {value * args.steer_max:10.1f}")
        summarize(curve_scores, "커브")
    return 0


if __name__ == "__main__":
    sys.exit(main())
