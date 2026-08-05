#!/usr/bin/env python3
"""Apply the guarded lane-center correction to saved modern model outputs."""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np


X_IDXS = np.array([192.0 * (i / 32.0) ** 2 for i in range(33)], dtype=np.float32)


def read_outputs(path: Path) -> tuple[np.ndarray, bool]:
  if path.suffix == ".npy":
    outputs = np.load(path).astype(np.float32, copy=True)
    return outputs, False
  with path.open("rb") as stream:
    if stream.read(8) != b"SCODMP1\0":
      raise ValueError("bad SCODMP1 output magic")
    output_floats, frames = struct.unpack("<II", stream.read(8))
    outputs = np.fromfile(stream, dtype=np.float32).reshape(frames, output_floats).copy()
  return outputs, True


def fuse(output: np.ndarray) -> bool:
  lanes = output[117:381].reshape(4, 33, 2)
  lane_stds = np.exp(np.minimum(output[381:645].reshape(4, 33, 2), 11.0))
  logits = output[645:653].reshape(4, 2)[:, 1]
  probabilities = 1.0 / (1.0 + np.exp(-np.clip(logits, -50.0, 50.0)))
  probability_weight = np.clip((min(probabilities[1], probabilities[2]) - 0.3) / 0.4, 0.0, 1.0)
  std_weight = np.clip((0.3 - max(lane_stds[1, 0, 0], lane_stds[2, 0, 0])) / 0.15, 0.0, 1.0)
  weight = float(probability_weight * std_weight)
  widths = lanes[2, :, 0] - lanes[1, :, 0]
  if weight <= 0.0 or np.count_nonzero((widths >= 2.0) & (widths <= 5.0)) < 24:
    return False

  center_y = 0.5 * (lanes[1, :, 0] + lanes[2, :, 0])
  plan = output[1576:2071].reshape(33, 15)
  valid = np.isfinite(plan[:, 0]) & (plan[:, 0] >= 0.0) & (plan[:, 0] <= 192.0)
  x = plan[valid, 0]
  lane_y = np.interp(x, X_IDXS, center_y)
  plan[valid, 1] = plan[valid, 1] * (1.0 - weight) + lane_y * weight
  x0 = np.maximum(0.0, x - 1.0)
  x1 = np.minimum(192.0, x + 1.0)
  yaw = np.arctan2(np.interp(x1, X_IDXS, center_y) -
                   np.interp(x0, X_IDXS, center_y), x1 - x0)
  plan[valid, 11] = plan[valid, 11] * (1.0 - weight) + yaw * weight
  return True


def plan_error(candidate: np.ndarray, reference: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  candidate_plan = candidate[1576:2071].reshape(33, 15)
  reference_plan = reference[1576:2071].reshape(33, 15)
  x = candidate_plan[:, 0]
  reference_x = reference_plan[:, 0]
  valid_reference = np.isfinite(reference_x) & np.isfinite(reference_plan[:, 1])
  order = np.argsort(reference_x[valid_reference])
  reference_x = reference_x[valid_reference][order]
  reference_y = reference_plan[valid_reference, 1][order]
  valid = (np.isfinite(x) & np.isfinite(candidate_plan[:, 1]) &
           np.isfinite(candidate_plan[:, 11]) & (x >= 0.0) & (x <= 120.0) &
           (x >= reference_x[0]) & (x <= reference_x[-1]))
  expected_y = np.interp(x[valid], reference_x, reference_y)
  x0 = np.maximum(reference_x[0], x[valid] - 1.0)
  x1 = np.minimum(reference_x[-1], x[valid] + 1.0)
  expected_yaw = np.arctan2(np.interp(x1, reference_x, reference_y) -
                            np.interp(x0, reference_x, reference_y), x1 - x0)
  return (np.abs(candidate_plan[valid, 1] - expected_y),
          np.abs(candidate_plan[valid, 11] - expected_yaw))


def summarize(errors: list[np.ndarray]) -> dict[str, float]:
  values = np.concatenate(errors)
  return {
      "mae": float(np.mean(values)),
      "p95": float(np.percentile(values, 95)),
      "max": float(np.max(values)),
      "samples": int(values.size),
  }


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--inputs", required=True, type=Path)
  parser.add_argument("--out", required=True, type=Path)
  parser.add_argument("--reference", type=Path,
                      help="optional source-ONNX outputs used for a quality report")
  parser.add_argument("--reference-start", type=int, default=0,
                      help="input frame corresponding to reference[0]")
  parser.add_argument("--report", type=Path)
  args = parser.parse_args()
  outputs, had_header = read_outputs(args.inputs)
  if outputs.ndim != 2 or outputs.shape[1] != 2576:
    raise ValueError(f"expected (frames,2576), got {outputs.shape}")
  original = outputs.copy() if args.reference else None
  applied = sum(fuse(output) for output in outputs)
  args.out.parent.mkdir(parents=True, exist_ok=True)
  if args.out.suffix == ".npy":
    np.save(args.out, outputs)
  else:
    with args.out.open("wb") as stream:
      stream.write(b"SCODMP1\0")
      stream.write(struct.pack("<II", outputs.shape[1], outputs.shape[0]))
      outputs.astype("<f4", copy=False).tofile(stream)
  if args.reference:
    if not args.report:
      raise ValueError("--reference requires --report")
    reference, _ = read_outputs(args.reference)
    frame_count = min(len(reference), len(outputs) - args.reference_start)
    if frame_count <= 0:
      raise ValueError("reference range does not overlap input outputs")
    before_y: list[np.ndarray] = []
    before_yaw: list[np.ndarray] = []
    after_y: list[np.ndarray] = []
    after_yaw: list[np.ndarray] = []
    for index in range(frame_count):
      output_index = args.reference_start + index
      y, yaw = plan_error(original[output_index], reference[index])
      before_y.append(y)
      before_yaw.append(yaw)
      y, yaw = plan_error(outputs[output_index], reference[index])
      after_y.append(y)
      after_yaw.append(yaw)
    report = {
        "frames": frame_count,
        "reference_start": args.reference_start,
        "fused_frames_total": applied,
        "comparison_range_m": 120.0,
        "lateral_y_m": {"before": summarize(before_y), "after": summarize(after_y)},
        "yaw_rad": {"before": summarize(before_yaw), "after": summarize(after_yaw)},
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
  print(f"wrote {args.out} frames={len(outputs)} fused={applied} source_header={had_header}")


if __name__ == "__main__":
  main()
