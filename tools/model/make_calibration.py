#!/usr/bin/env python3
"""Build the PTQ calibration NPZ for the v0.9.4 kmodel from recorded drives.

Samples must carry the model's own temporal inputs, so each route is replayed
in order through the fp32/uint8-input ONNX and the input tensors are captured
every --stride frames after a warm-up. Mixing routes (city, highway, dawn,
night) keeps the activation ranges representative of what the car sees.

Usage:
  python make_calibration.py --routes ROUTE [ROUTE ...] \
      --model models/onnx/supercombo_uint8.onnx --out models/ptq/supercombo_calib.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from op094_runner import INPUT_ORDER, Op094Runner
from route_frames import iter_model_inputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--routes", nargs="+", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--per-route", type=int, default=15)
    parser.add_argument("--warmup", type=int, default=150,
                        help="frames to run before sampling, so the feature "
                             "buffer is full")
    parser.add_argument("--stride", type=int, default=40)
    args = parser.parse_args()

    runner = Op094Runner(args.model)
    samples: dict[str, list[np.ndarray]] = {k: [] for k in INPUT_ORDER}

    for route_str in args.routes:
        route = Path(route_str)
        runner.reset()
        taken = 0
        limit = args.warmup + args.stride * args.per_route + 1
        for index, img, big, _ in iter_model_inputs(route, limit=limit):
            inputs = runner.build_inputs(img, big)
            runner.step(inputs)
            if index >= args.warmup and (index - args.warmup) % args.stride == 0:
                if taken >= args.per_route:
                    break
                for key in INPUT_ORDER:
                    samples[key].append(inputs[key][0].copy())
                taken += 1
        print(f"{route.name}: {taken} samples", flush=True)

    stacked = {k: np.stack(v) for k, v in samples.items()}
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, **stacked)
    print(f"wrote {out} with {len(stacked['input_imgs'])} samples")
    for key, value in stacked.items():
        print(f"  {key:20s} {value.shape} {value.dtype}")


if __name__ == "__main__":
    main()
