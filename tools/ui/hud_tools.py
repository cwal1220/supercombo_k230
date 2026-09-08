#!/usr/bin/env python3
"""HUD snapshot helpers.

  hud_tools.py inputs <route_dir> <out_dir> [--time SECONDS]
      Extract hud_snapshot inputs from a recordd route: model.bin (raw
      K230ModelState, 3256 bytes), control.bin (raw K230ControlState padded to
      240 bytes) and camera.png (the matching road frame). Without --time the
      moment is chosen automatically: controller active, 40-95 km/h, all lane
      lines confident, a lead if any.

  hud_tools.py compose <prefix> [camera.png]
      Turn hud_snapshot K230ARGB frames (<prefix>_<scenario>.argb, native
      480x800 portrait or 800x480 landscape) into PNGs: <name>_overlay.png
      (800x480 logical view, transparent) and, with a camera frame,
      <name>_composite.png.
"""
from __future__ import annotations

import argparse
import glob
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import k230_route as kr  # noqa: E402

MODEL_STATE_SIZE = 3256
CONTROL_STATE_SIZE = 240
FRAME_MAGIC = b"K230ARGB"
LOGICAL = (800, 480)


# ---- inputs ----

def scan(route: Path):
    layout = kr.model_state_layout(5)
    assert layout["__size__"] == MODEL_STATE_SIZE
    controls, models = [], []
    for path in kr.route_event_files(route):
        data = path.read_bytes()
        _, _, header_size = struct.unpack_from("<8sII", data, 0)
        offset, end = header_size, len(data)
        while offset + 16 <= end:
            ts, rtype, _, payload = struct.unpack_from("<QHHI", data, offset)
            offset += 16
            if offset + payload > end:
                break
            if rtype == kr.RECORD_CONTROL_STATE:
                controls.append((ts, np.frombuffer(data, kr.CONTROL_STATE, count=1, offset=offset)[0]))
            elif rtype == kr.RECORD_MODEL_STATE and payload == MODEL_STATE_SIZE:
                frame_id, = struct.unpack_from("<Q", data, offset)
                lanes = struct.unpack_from("<4f", data, offset + layout["lane_probabilities"])
                lead_valid, = struct.unpack_from("<I", data, offset + layout["lead"])
                models.append((ts, path, offset, frame_id, lanes, lead_valid))
            offset += payload
    return controls, models


def score(control, model) -> float:
    if control["active"] != 1:
        return -1.0
    speed = float(control["speed_kph"])
    if not 40.0 <= speed <= 95.0:
        return -1.0
    lanes = model[4]
    value = 1.0 + 2.0 * min(lanes[1], lanes[2]) + 0.5 * (lanes[0] + lanes[3])
    value += 1.5 if control["radar_lead_valid"] else 0.0
    value += 1.0 if model[5] else 0.0
    value += 0.5 if control["tpms_valid"] else 0.0
    return value


def cmd_inputs(args: argparse.Namespace) -> int:
    controls, models = scan(args.route)
    if not controls or not models:
        sys.exit("route has no control/model records")
    control_ts = np.array([c[0] for c in controls], dtype=np.uint64)
    model_ts = np.array([m[0] for m in models], dtype=np.uint64)

    if args.time is not None:
        target = int(model_ts[0]) + int(args.time * 1e9)
        mi = int(np.clip(np.searchsorted(model_ts, target), 0, len(models) - 1))
    else:
        best = (-2.0, 0)
        for i in range(0, len(models), 3):
            j = int(np.searchsorted(control_ts, models[i][0]))
            if j >= len(controls):
                break
            value = score(controls[j][1], models[i])
            if value > best[0]:
                best = (value, i)
        mi = best[1]
    model = models[mi]
    cj = min(int(np.searchsorted(control_ts, model[0])), len(controls) - 1)
    control = controls[cj][1]

    args.out.mkdir(parents=True, exist_ok=True)
    data = model[1].read_bytes()
    (args.out / "model.bin").write_bytes(data[model[2]:model[2] + MODEL_STATE_SIZE])
    raw = control.tobytes()
    (args.out / "control.bin").write_bytes(raw + b"\0" * (CONTROL_STATE_SIZE - len(raw)))
    seconds = (int(model[0]) - int(model_ts[0])) / 1e9
    print(f"moment t={seconds:.1f}s frame_id={model[3]} speed={control['speed_kph']:.1f} "
          f"active={control['active']} lanes={np.round(model[4], 2)} lead={model[5]}")

    for segment in kr.route_segments(args.route):
        frames = segment.frames
        hits = np.nonzero(frames["frame_id"] == model[3])[0]
        if not len(hits):
            continue
        index = int(frames["encode_index"][hits[0]]) - int(frames["encode_index"][0])
        camera = args.out / "camera.png"
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(segment.path / "road.hevc"),
                        "-vf", f"select=eq(n\\,{index})", "-frames:v", "1", str(camera)], check=True)
        print(f"camera frame: {segment.path.name} index {index} -> {camera}")
        break
    else:
        print("camera frame not found in frames.bin", file=sys.stderr)
    return 0


# ---- compose ----

def load_frame(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:8] != FRAME_MAGIC:
        raise ValueError(f"{path}: not a K230ARGB file")
    width, height = struct.unpack_from("<II", data, 8)
    pixels = np.frombuffer(data, np.uint8, count=width * height * 4, offset=16)
    native = pixels.reshape(height, width, 4)  # B, G, R, A
    if (width, height) == (LOGICAL[1], LOGICAL[0]):
        logical = np.rot90(native, 1)  # logical (lx, ly) -> native (row lx, col 479 - ly)
    elif (width, height) == LOGICAL:
        logical = native
    else:
        raise ValueError(f"{path}: unexpected size {width}x{height}")
    return logical[..., [2, 1, 0, 3]].copy()


def cmd_compose(args: argparse.Namespace) -> int:
    camera = None
    if args.camera is not None:
        camera = Image.open(args.camera).convert("RGBA").resize(LOGICAL, Image.BILINEAR)
    frames = sorted(glob.glob(f"{args.prefix}_*.argb"))
    if not frames:
        print(f"no frames matching {args.prefix}_*.argb", file=sys.stderr)
        return 1
    for frame in frames:
        path = Path(frame)
        rgba = load_frame(path)
        overlay = Image.fromarray(rgba, "RGBA")
        overlay.save(path.with_name(path.stem + "_overlay.png"))
        coverage = (rgba[..., 3] > 0).mean() * 100.0
        line = f"{path.stem}: overlay coverage {coverage:.1f}%"
        if camera is not None:
            composite = camera.copy()
            composite.alpha_composite(overlay)
            composite.convert("RGB").save(path.with_name(path.stem + "_composite.png"))
            line += " (+composite)"
        print(line)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    inputs = commands.add_parser("inputs")
    inputs.add_argument("route", type=Path)
    inputs.add_argument("out", type=Path)
    inputs.add_argument("--time", type=float, help="seconds from route start instead of auto-pick")
    inputs.set_defaults(run=cmd_inputs)
    compose = commands.add_parser("compose")
    compose.add_argument("prefix")
    compose.add_argument("camera", nargs="?", type=Path)
    compose.set_defaults(run=cmd_compose)
    args = parser.parse_args()
    return args.run(args)


if __name__ == "__main__":
    sys.exit(main())
