#!/usr/bin/env python3
"""Build an SCNV12R1 replay plus host reference outputs for board verification.

The same recorded frames then go through the board runtime (RVV warp, temporal
plumbing, kmodel) and through the host (numpy warp port, ONNX), so any
mismatch points at the runtime rather than at the model. See
docs/diagnostics.md for the board side.

Usage:
  python make_replay.py --route ROUTE --out DIR --frames 100 --skip 400 \
      --model models/onnx/supercombo_uint8.onnx --rpy r,p,y
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np

from op094_runner import Op094Runner
from route_frames import iter_model_inputs, route_calibration

MAGIC = b"SCNV12R1"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--skip", type=int, default=400)
    parser.add_argument("--model", default=None,
                        help="ONNX for the host reference outputs")
    parser.add_argument("--rpy", default=None,
                        help="roll,pitch,yaw in radians. Use the board's "
                             "params/calibration.json so both warps match; the "
                             "calibration service overrides the "
                             "SUPERCOMBO_INPUT_WARP_* variables every frame.")
    args = parser.parse_args()

    route = Path(args.route)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    rpy = (np.array([float(x) for x in args.rpy.split(",")], np.float32)
           if args.rpy else route_calibration(route))
    print("warp rpy:", np.round(rpy, 6))

    runner = Op094Runner(args.model) if args.model else None
    frames, outputs = [], []
    width = height = 0
    for _, img, big, nv12 in iter_model_inputs(route, rpy=rpy, skip=args.skip,
                                               limit=args.frames):
        frames.append(nv12)
        if runner is not None:
            outputs.append(runner.run(img, big))
    if len(frames) < args.frames:
        raise SystemExit(f"route yielded only {len(frames)} frames")

    from k230_route import route_segments
    segments = route_segments(route)
    width, height = segments[0].width, segments[0].height
    replay = out_dir / "replay.scnv12"
    with open(replay, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<III", width, height, len(frames)))
        for nv12 in frames:
            f.write(nv12.tobytes())
    print(f"replay: {len(frames)} frames {width}x{height} -> {replay} "
          f"({replay.stat().st_size / 1e6:.0f} MB)")

    if runner is not None:
        np.save(out_dir / "host_ref.npy", np.stack(outputs))
        print(f"host reference: {len(outputs)} x {len(outputs[0])} floats")
    (out_dir / "replay_meta.json").write_text(json.dumps({
        "route": str(route), "skip": args.skip, "frames": len(frames),
        "rpy_rad": [float(v) for v in rpy], "model": args.model,
    }, indent=2))


if __name__ == "__main__":
    main()
