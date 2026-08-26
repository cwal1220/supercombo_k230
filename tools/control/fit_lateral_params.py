#!/usr/bin/env python3
"""주행 데이터에서 횡제어 파라미터를 실측한다.

openpilot v0.11의 torqued(라이브 토크 회귀)와 lagd(라이브 지연 추정)를
보드 데이터에 대해 오프라인으로 돌리는 도구다. 파라미터를 감으로 움직이는
대신 측정값을 얻는다.

  fit  controlsd 텍스트 로그(1 Hz hz= 라인)에서 토크-횡가속 관계를 회귀한다.
       -> torque_max_lat_accel_raw, torque_friction_raw, torque_lat_accel_offset

  lag  v3 녹화의 ControlState 스트림에서 요청 곡률과 실측 곡률을
       상호상관해 steer_actuator_delay를 추정한다. 텍스트 로그는 1 Hz라
       0.3~0.4 s 지연을 분해할 수 없어 녹화가 필요하다.
       한 번에 한 route만 넣는다. 타임스탬프가 겹치는 다른 route를 섞으면
       정렬 병합으로 신호가 뒤섞인다.

사용 예:
  python3 fit_lateral_params.py fit /tmp/supercombo_k230.log
  python3 fit_lateral_params.py lag  <route_dir>/events
"""

import argparse
import glob
import os
import re
import struct
import sys

import numpy as np

# ---------------------------------------------------------------- fit ----

# openpilot torqued 상수
MIN_SPEED_KPH = 54.0          # MIN_VEL 15 m/s
STEER_MIN_THRESHOLD = 0.02
FRICTION_FACTOR = 1.5         # 잔차 표준편차의 ~85% 커버
STEER_BUCKET_BOUNDS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0.0),
                       (0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
MIN_BUCKET_POINTS = [10, 30, 50, 50, 50, 50, 30, 10]  # qlog 스케일(/10)
STEER_MAX = 384.0

HZ_RE = re.compile(r"k230_controlsd: hz=")


def _field(line, key, cast=float):
    m = re.search(re.escape(key) + r"=(-?[\d.]+)", line)
    return cast(m.group(1)) if m else None


def parse_log_samples(paths, min_speed_kph, driver_max, sign, steer_max):
    """hz= 라인에서 (정규화 토크, 실측 횡가속) 표본을 뽑는다."""
    samples = []
    cluster_fallback = [False]
    yaw_fallback = [False]
    sign_probe = [0.0]
    skipped = {"inactive": 0, "driver": 0, "desire": 0, "slow": 0,
               "small": 0, "saturated": 0, "no_yaw": 0}
    for path in paths:
        file_samples_start = len(samples)
        sign_probe[0] = 0.0
        with open(path, "r", errors="replace") as f:
            for raw in f:
                for line in raw.replace("\r", "\n").split("\n"):
                    if not HZ_RE.search(line):
                        continue
                    active = _field(line, "active", int)
                    if active != 1:
                        skipped["inactive"] += 1
                        continue
                    driver = _field(line, "driver", int)
                    if driver is None or abs(driver) >= driver_max:
                        skipped["driver"] += 1
                        continue
                    desire = _field(line, "desire", int)
                    if desire not in (0, None):
                        skipped["desire"] += 1
                        continue
                    # wheel=은 제어가 쓰는 휠 속도(신 로그), speed=는 계기판
                    # 속도(구 로그, 고속에서 ~5% 큼 -> lat_accel ~10% 과대).
                    speed = _field(line, "wheel")
                    if speed is None:
                        speed = _field(line, "speed")
                        cluster_fallback[0] = True
                    if speed is None or speed < min_speed_kph:
                        skipped["slow"] += 1
                        continue
                    m = re.search(r"torque=(-?\d+)/(-?\d+)", line)
                    curve_yaw = _field(line, "curveYaw")
                    if m is None or curve_yaw is None:
                        skipped["no_yaw"] += 1
                        continue
                    # 랙에 실제로 걸린 값은 적용 토크다. 부호는
                    # torque_output_sign을 반영해 곡률 관례로 되돌린다.
                    torque_norm = sign * int(m.group(2)) / steer_max
                    if abs(torque_norm) < STEER_MIN_THRESHOLD:
                        skipped["small"] += 1
                        continue
                    if abs(torque_norm) >= 1.0:
                        skipped["saturated"] += 1
                        continue
                    # ESP12 실측 횡가속(lat=)이 있으면 그것을 쓴다. 요레이트
                    # 간접값과 달리 openpilot torqued의 offset 정의와 일치한다.
                    lat_accel = _field(line, "lat")
                    v = speed / 3.6
                    kin = curve_yaw * v * v
                    if lat_accel is None:
                        lat_accel = kin
                        yaw_fallback[0] = True
                    else:
                        # 부호 감지용: 반전 수정(2026-08-25) 이전 로그 호환
                        sign_probe[0] += lat_accel * kin
                    samples.append((torque_norm, lat_accel))
        if sign_probe[0] < 0:
            print(f"공지: {path}: lat= 부호가 반전 이전 로그다. 파일 단위로 뒤집는다.")
            for i in range(file_samples_start, len(samples)):
                t, l = samples[i]
                samples[i] = (t, -l)
    if yaw_fallback[0]:
        print("경고: lat= 필드가 없어 요레이트 간접값을 썼다. offset은 "
              "openpilot 정의와 달라 FF 보정에 그대로 넣으면 안 된다.")
    if cluster_fallback[0]:
        print("경고: wheel= 필드가 없어 계기판 속도(speed=)를 썼다. "
              "고속에서 latAccelFactor가 ~10% 과대평가될 수 있다.")
    return np.array(samples), skipped


def bucket_and_fit(samples):
    """torqued의 total-least-squares 회귀. 버킷으로 좌우 균형을 강제한다."""
    buckets = {b: [] for b in STEER_BUCKET_BOUNDS}
    for x, y in samples:
        for lo, hi in STEER_BUCKET_BOUNDS:
            if lo <= x < hi:
                buckets[(lo, hi)].append((x, y))
                break
    counts = {b: len(v) for b, v in buckets.items()}
    lacking = [(b, c, m) for (b, c, m) in
               ((b, counts[b], m) for b, m in zip(STEER_BUCKET_BOUNDS, MIN_BUCKET_POINTS))
               if c < m]

    pts = np.array([p for v in buckets.values() for p in v])
    if len(pts) < 100:
        return None, counts, lacking
    # torqued estimate_params와 동일: [x, 1, y] SVD
    mat = np.column_stack([pts[:, 0], np.ones(len(pts)), pts[:, 1]])
    _, _, v = np.linalg.svd(mat, full_matrices=False)
    if abs(v.T[2, 2]) < 1e-9:
        return None, counts, lacking
    slope, offset = -v.T[0:2, 2] / v.T[2, 2]
    # torqued slope2rot에서 부호를 유지한다. |slope|로 만들면 slope<0에서
    # 잔차에 기울기 성분이 남아 friction이 부풀려진다.
    sin = slope / np.sqrt(slope**2 + 1)
    cos = 1.0 / np.sqrt(slope**2 + 1)
    rot = np.array([[cos, -sin], [sin, cos]])
    spread = (np.column_stack([pts[:, 0], pts[:, 1]]) @ rot)[:, 1]
    friction = float(np.std(spread) * FRICTION_FACTOR)
    return (float(slope), float(offset), friction), counts, lacking


def cmd_fit(args):
    samples, skipped = parse_log_samples(args.logs, args.min_speed,
                                         args.driver_max, args.sign,
                                         args.steer_max)
    print(f"표본 {len(samples)}개  (제외: " +
          " ".join(f"{k}={v}" for k, v in skipped.items() if v) + ")")
    if len(samples) < 100:
        print("표본이 부족하다. engage 상태의 고속 주행 로그가 더 필요하다.")
        return 1
    result, counts, lacking = bucket_and_fit(samples)
    print("버킷 분포:")
    for (lo, hi), c in counts.items():
        print(f"  [{lo:+.1f},{hi:+.1f})  {c}")
    if lacking:
        print("경고: 다음 버킷이 부족해 결과가 한쪽 데이터에 치우칠 수 있다:")
        for (lo, hi), c, m in lacking:
            print(f"  [{lo:+.1f},{hi:+.1f})  {c} < {m}")
    if result is None:
        print("회귀 불가 — 유효 표본 부족.")
        return 1
    slope, offset, friction = result
    print()
    print(f"latAccelFactor        {slope:+.3f} m/s^2 per unit torque")
    print(f"latAccelOffset        {offset:+.4f} m/s^2")
    print(f"frictionCoefficient   {friction:.4f} m/s^2")
    print()
    print("파라미터 환산 (steering.json):")
    print(f"  torque_max_lat_accel_raw   {slope * 10:.0f}")
    print(f"  torque_lat_accel_offset    {offset:+.3f}")
    print(f"  torque_friction_raw        {friction * 1000:.0f}")
    return 0


# ---------------------------------------------------------------- lag ----

# recording_format.h와 일치해야 한다.
EVENT_FILE_MAGIC = b"K230LOG1"
RECORD_HEADER = struct.Struct("<QHHI")           # timestamp_ns, type, flags, size
TYPE_CONTROL_STATE = 4
CONTROL_STATE_SIZE_V3 = 240
# K230ControlState 내부 오프셋 (v3, 240 B)
OFF_ACTIVE = 16
OFF_SPEED_KPH = 56
OFF_DESIRED_CURV = 72
OFF_ACTUAL_CURV = 76

MIN_LAG_S = 0.10
MAX_LAG_S = 0.65
MIN_CORR = 0.95   # lagd MIN_NCC
GRID_DT = 0.01
WINDOW_S = 60.0
MIN_SPEED_LAG_KPH = 54.0
MIN_LAT_ACCEL_RANGE = 0.5   # 창 안에 이만큼의 횡가속 변화가 있어야 상관이 유의미


def read_control_states(paths):
    rows = []
    for path in paths:
        with open(path, "rb") as f:
            head = f.read(24)
            if len(head) < 24 or head[:8] != EVENT_FILE_MAGIC:
                print(f"건너뜀(헤더 아님): {path}", file=sys.stderr)
                continue
            version, header_size = struct.unpack_from("<II", head, 8)
            if version < 3:
                print(f"건너뜀(v{version}, ControlState 없음): {path}", file=sys.stderr)
                continue
            f.seek(header_size)
            while True:
                rh = f.read(RECORD_HEADER.size)
                if len(rh) < RECORD_HEADER.size:
                    break
                ts, rtype, _flags, size = RECORD_HEADER.unpack(rh)
                payload = f.read(size)
                if len(payload) < size:
                    break
                if rtype != TYPE_CONTROL_STATE or size < CONTROL_STATE_SIZE_V3:
                    continue
                active = struct.unpack_from("<I", payload, OFF_ACTIVE)[0]
                speed = struct.unpack_from("<f", payload, OFF_SPEED_KPH)[0]
                desired = struct.unpack_from("<f", payload, OFF_DESIRED_CURV)[0]
                actual = struct.unpack_from("<f", payload, OFF_ACTUAL_CURV)[0]
                rows.append((ts * 1e-9, active, speed, desired, actual))
    return np.array(rows)


def estimate_lag(rows):
    """창 단위 정규화 상호상관으로 desired->actual 지연을 추정한다."""
    if len(rows) < 100:
        return None
    rows = rows[np.argsort(rows[:, 0])]
    # 레코드 간격이 벌어진 곳(청크 유실, 재시작, 다른 route)을 보간하면
    # 직선 다리가 생겨 아무 지연에서나 corr~1이 나온다. 세그먼트로 쪼갠다.
    gaps = np.where(np.diff(rows[:, 0]) > 0.5)[0]
    win = int(WINDOW_S / GRID_DT)
    lags = []
    for seg in np.split(rows, gaps + 1):
        lags.extend(_lags_in_segment(seg, win))
    return lags


def _lags_in_segment(rows, win):
    if len(rows) < 100:
        return []
    t = rows[:, 0]
    grid = np.arange(t[0], t[-1], GRID_DT)
    if len(grid) < win:
        return []
    desired = np.interp(grid, t, rows[:, 3])
    actual = np.interp(grid, t, rows[:, 4])
    ok = (np.interp(grid, t, rows[:, 1]) > 0.5) & \
         (np.interp(grid, t, rows[:, 2]) >= MIN_SPEED_LAG_KPH)
    speed_mps = np.interp(grid, t, rows[:, 2]) / 3.6
    max_shift = int(MAX_LAG_S / GRID_DT)
    min_shift = int(MIN_LAG_S / GRID_DT)
    lags = []
    for start in range(0, len(grid) - win + 1, win // 2):
        sl = slice(start, start + win)
        if ok[sl].mean() < 0.9:
            continue
        la = desired[sl] * speed_mps[sl] ** 2
        if la.max() - la.min() < MIN_LAT_ACCEL_RANGE:
            continue
        # lagd와 같이 원신호 NCC를 쓴다. 주기 신호의 피크 모호성은
        # corr 문턱(0.95)과 여러 창의 중앙값으로 걸러진다.
        d = desired[sl] - desired[sl].mean()
        a = actual[sl] - actual[sl].mean()
        dn = np.linalg.norm(d)
        best_corr, best_shift = -1.0, None
        for shift in range(min_shift, max_shift + 1):
            av = a[shift:]
            dv = d[:len(av)]
            denom = dn * np.linalg.norm(av)
            if denom < 1e-9:
                continue
            corr = float(np.dot(dv, av) / denom)
            if corr > best_corr:
                best_corr, best_shift = corr, shift
        if best_shift is not None and best_corr >= MIN_CORR:
            lags.append((best_shift * GRID_DT, best_corr))
    return lags


def cmd_lag(args):
    paths = []
    for p in args.events:
        if os.path.isdir(p):
            paths.extend(sorted(glob.glob(os.path.join(p, "*.bin"))))
        else:
            paths.append(p)
    rows = read_control_states(paths)
    print(f"ControlState 레코드 {len(rows)}개")
    lags = estimate_lag(rows)
    if not lags:
        print("유효한 창이 없다. 54 km/h 이상에서 곡률 변화가 있는")
        print("engage 주행의 v3 녹화가 필요하다.")
        return 1
    vals = np.array([l for l, _ in lags])
    print(f"창 {len(vals)}개  지연 중앙값 {np.median(vals):.3f} s"
          f"  평균 {vals.mean():.3f}  표준편차 {vals.std():.3f}")
    for lag_s, corr in lags:
        print(f"  lag={lag_s:.3f}s corr={corr:.3f}")
    print()
    print(f"  steer_actuator_delay 후보: {np.median(vals):.2f}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit", help="토크-횡가속 회귀 (controlsd 텍스트 로그)")
    f.add_argument("logs", nargs="+")
    f.add_argument("--min-speed", type=float, default=MIN_SPEED_KPH,
                   help="km/h, 기본 54 (openpilot MIN_VEL)")
    f.add_argument("--driver-max", type=int, default=50)
    f.add_argument("--sign", type=int, default=-1, choices=(-1, 1),
                   help="steering.json torque_output_sign")
    f.add_argument("--steer-max", type=float, default=STEER_MAX,
                   help="steering.json steer_max")
    f.set_defaults(func=cmd_fit)
    l = sub.add_parser("lag", help="actuator delay 추정 (v3 녹화 events)")
    l.add_argument("events", nargs="+", help="events 디렉토리 또는 .bin 파일")
    l.set_defaults(func=cmd_lag)
    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
