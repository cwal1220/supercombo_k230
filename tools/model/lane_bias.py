"""Measure the lateral bias of a recorded drive and say where it comes from.

The lateral planner steers toward the midpoint of laneLines[1] (left) and
laneLines[2] (right) in a frame where +y is to the RIGHT, so

    offset(x) = (left_y(x) + right_y(x)) / 2

is where the model thinks the lane centre sits relative to the camera at
distance x ahead. Positive means "the lane centre is to my right", i.e. the car
is sitting LEFT of centre.

Fitting offset(x) = a + b*x separates the two things that produce it:

  a  translation, in metres. A camera that is not on the vehicle centreline,
     or a car that is genuinely off-centre. Camera intrinsics CANNOT produce
     this: a principal-point or focal-length error acts about the camera, so
     its lateral effect is exactly zero at x=0.
  b  rotation, in rad/m. This is the calibration-shaped term -- a yaw error
     from the camera matrix or the online extrinsics. A wrong cx of dcx pixels
     shows up here as roughly dcx/fx.

So: a large `a` with a near-zero `b` means the bias is not a camera
calibration problem, and re-deriving the camera matrix will not fix it.

Straightness is judged by the model's own 50 m path, NOT by the steering angle.
Filtering on steering angle looks tempting and is wrong: when the car needs a
non-zero angle to go straight, an |angle| < k filter keeps only one side of the
curve distribution and manufactures a rotation term that is not there.

Usage:
    python tools/model/lane_bias.py <route_dir> [<route_dir> ...]
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np

from recording_reader import (RECORD_CONTROL_STATE, RECORD_MODEL_STATE,
                              TRAJECTORY_SIZE, iter_event_records,
                              route_event_files)

# openpilot X_IDXS: the distance each trajectory point sits at.
X_IDXS = np.array([192.0 * (i / (TRAJECTORY_SIZE - 1.0)) ** 2
                   for i in range(TRAJECTORY_SIZE)])
FIT_POINTS = 20          # 0 .. 75 m; beyond that the lane lines get noisy
STRAIGHT_PLAN_Y_M = 0.5  # |plan y| at 48 m below this counts as straight
MIN_SPEED_KPH = 60.0
MIN_LANE_PROB = 0.6
MAX_DRIVER_TORQUE = 50   # ignore frames where the driver was nudging


class RouteLateral:
    """Per-frame lateral quantities, model records joined to control records."""

    def __init__(self):
        self.lane_offset = []    # [N, TRAJECTORY_SIZE] perceived lane centre
        self.plan_y = []         # [N, TRAJECTORY_SIZE] model path
        self.left_prob = []
        self.right_prob = []
        self.model_ts = []
        self.control = {k: [] for k in
                        ("ts", "active", "speed_kph", "steer_deg",
                         "desired_curv", "actual_curv", "driver_torque")}

    def finish(self):
        for key in ("lane_offset", "plan_y", "left_prob", "right_prob"):
            setattr(self, key, np.array(getattr(self, key)))
        self.model_ts = np.array(self.model_ts, np.uint64)
        self.control = {k: np.array(v) for k, v in self.control.items()}
        return self


def _floats(data: memoryview, offset: int, count: int) -> np.ndarray:
    return np.frombuffer(data, "<f4", count, offset)


def read_route_lateral(route: Path) -> RouteLateral:
    out = RouteLateral()
    for path in route_event_files(route):
        for rec in iter_event_records(path):
            if rec.type == RECORD_MODEL_STATE:
                layout = rec.model_layout()
                lanes = layout["lanes"]
                stride = TRAJECTORY_SIZE * 12
                left = _floats(rec.payload, lanes + stride, TRAJECTORY_SIZE * 3)[1::3]
                right = _floats(rec.payload, lanes + 2 * stride, TRAJECTORY_SIZE * 3)[1::3]
                plan = _floats(rec.payload, layout["plan"], TRAJECTORY_SIZE * 3)
                probs = _floats(rec.payload, layout["lane_probabilities"], 4)
                out.lane_offset.append((left + right) / 2.0)
                out.plan_y.append(plan[1::3])
                out.left_prob.append(probs[1])
                out.right_prob.append(probs[2])
                out.model_ts.append(
                    struct.unpack_from("<Q", rec.payload,
                                       layout["capture_timestamp_ns"])[0])
            elif rec.type == RECORD_CONTROL_STATE:
                state = rec.control_state()
                out.control["ts"].append(rec.timestamp_ns)
                out.control["active"].append(int(state["active"]))
                out.control["speed_kph"].append(float(state["speed_kph"]))
                out.control["steer_deg"].append(float(state["steering_angle_deg"]))
                out.control["desired_curv"].append(float(state["desired_curvature"]))
                out.control["actual_curv"].append(float(state["actual_curvature"]))
                out.control["driver_torque"].append(int(state["driver_torque"]))
    return out.finish()


def route_offsets(route: Path) -> dict[str, float]:
    """The lateral tuning that was active on this drive, from its snapshot."""
    path = route / "params" / "steering.json"
    if not path.exists():
        return {}
    params = json.loads(path.read_text())
    return {k: params[k] for k in
            ("camera_offset_m", "path_offset_m", "angle_offset_deg")
            if k in params}


def analyse(route: Path) -> None:
    route = Path(route)
    data = read_route_lateral(route)
    if not len(data.model_ts) or not len(data.control["ts"]):
        print(f"{route.name}: no paired model/control records")
        return

    idx = np.clip(np.searchsorted(data.control["ts"], data.model_ts),
                  0, len(data.control["ts"]) - 1)
    ctl = {k: v[idx] for k, v in data.control.items() if k != "ts"}

    straight = np.abs(data.plan_y[:, 16]) < STRAIGHT_PLAN_Y_M
    keep = (straight
            & (data.left_prob > MIN_LANE_PROB) & (data.right_prob > MIN_LANE_PROB)
            & (ctl["speed_kph"] > MIN_SPEED_KPH) & (ctl["active"] == 1)
            & (np.abs(ctl["driver_torque"]) < MAX_DRIVER_TORQUE)
            & np.isfinite(data.lane_offset).all(1))

    tuning = route_offsets(route)
    print(f"\n=== {route.name} ===")
    if tuning:
        print("  active tuning: " + "  ".join(f"{k}={v}" for k, v in tuning.items()))
    print(f"  {int(keep.sum())} straight engaged samples "
          f"(model 48 m path within {STRAIGHT_PLAN_Y_M} m, >{MIN_SPEED_KPH:.0f} km/h, "
          f"both lanes >{MIN_LANE_PROB}, no driver torque)")
    if keep.sum() < 200:
        print("  too few samples to fit")
        return

    profile = data.lane_offset[keep].mean(0)
    slope, intercept = np.polyfit(X_IDXS[:FIT_POINTS], profile[:FIT_POINTS], 1)
    print("  perceived lane-centre offset (+ = centre is to my right = I am left)")
    for j in (0, 4, 8, 12, 16, 20):
        print(f"    x={X_IDXS[j]:6.1f} m   {profile[j]:+.3f} m")
    print(f"  fit over 0-{X_IDXS[FIT_POINTS - 1]:.0f} m:")
    print(f"    translation {intercept * 100:+6.1f} cm    "
          f"<- camera intrinsics cannot cause this")
    print(f"    rotation    {slope * 1000:+6.2f} mrad  "
          f"({np.degrees(slope):+.3f} deg)  <- the calibration-shaped term")

    steer = ctl["steer_deg"][keep]
    print(f"  straight-line steering angle  mean {steer.mean():+.3f} deg  "
          f"sd {steer.std():.3f}")
    print(f"  curvature command  desired {ctl['desired_curv'][keep].mean():+.6f} 1/m"
          f"   actual {ctl['actual_curv'][keep].mean():+.6f} 1/m")
    if tuning.get("path_offset_m"):
        raw = intercept + tuning["path_offset_m"]
        print(f"  path_offset_m={tuning['path_offset_m']} is already applied, so the "
              f"uncompensated translation is about {raw * 100:+.1f} cm")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="+", type=Path)
    for route in parser.parse_args().routes:
        analyse(route)


if __name__ == "__main__":
    main()
