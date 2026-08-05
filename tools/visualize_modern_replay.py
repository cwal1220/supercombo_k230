#!/usr/bin/env python3
"""Render a K230 modern-supercombo SCODMP1 replay without openpilot dependencies."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import cv2
import numpy as np


X_IDXS = np.array([192.0 * (i / 32.0) ** 2 for i in range(33)], dtype=np.float32)
CAMERA_HEIGHT = 1.22

CAMERA_SIZE = (1920.0, 1080.0)
CAMERA_INTRINSICS = (1625.7416788144435, 1585.9830269782024,
                     946.13450988811394, 537.34063862123787)
CAMERA_DISTORTION = (0.124801017, -0.930510234, 0.000743543609,
                     0.000532516864, 1.30742029)


def read_replay(path: Path) -> tuple[int, int, np.ndarray]:
  with path.open("rb") as src:
    if src.read(8) != b"SCNV12R1":
      raise ValueError("bad SCNV12R1 replay magic")
    width, height, frames = struct.unpack("<III", src.read(12))
    data = np.fromfile(src, dtype=np.uint8)
  frame_bytes = width * height * 3 // 2
  if data.size != frames * frame_bytes:
    raise ValueError("replay frame count/size mismatch")
  return width, height, data.reshape(frames, height * 3 // 2, width)


def read_outputs(path: Path) -> np.ndarray:
  if path.suffix == ".npy":
    data = np.load(path)
    if data.ndim != 2 or data.shape[1] != 2576:
      raise ValueError(f"expected (frames,2576) npy outputs, got {data.shape}")
    return data.astype(np.float32, copy=False)
  with path.open("rb") as src:
    if src.read(8) != b"SCODMP1\0":
      raise ValueError("bad SCODMP1 output magic")
    output_floats, frames = struct.unpack("<II", src.read(8))
    if output_floats != 2576:
      raise ValueError(f"expected 2576 outputs, got {output_floats}")
    data = np.fromfile(src, dtype=np.float32)
  if data.size != frames * output_floats:
    raise ValueError("output frame count/size mismatch")
  return data.reshape(frames, output_floats)


def rotation_from_rpy(rpy: tuple[float, float, float]) -> np.ndarray:
  roll, pitch, yaw = rpy
  cr, sr = np.cos(roll), np.sin(roll)
  cp, sp = np.cos(pitch), np.sin(pitch)
  cy, sy = np.cos(yaw), np.sin(yaw)
  return np.array([
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
  ], dtype=np.float64)


def project(points: np.ndarray, width: int, height: int,
            rpy: tuple[float, float, float]) -> np.ndarray:
  ref_width, ref_height = CAMERA_SIZE
  ref_fx, ref_fy, ref_cx, ref_cy = CAMERA_INTRINSICS
  fx, fy = ref_fx * width / ref_width, ref_fy * height / ref_height
  cx, cy = ref_cx * width / ref_width, ref_cy * height / ref_height
  view_from_device = np.array([[0.0, 1.0, 0.0],
                               [0.0, 0.0, 1.0],
                               [1.0, 0.0, 0.0]], dtype=np.float64)
  view = (view_from_device @ rotation_from_rpy(rpy) @ points.T).T
  depth = view[:, 2]
  valid = np.isfinite(view).all(axis=1) & (depth > 0.1)
  result = np.full((points.shape[0], 2), np.nan, dtype=np.float32)
  nx = view[valid, 0] / depth[valid]
  ny = view[valid, 1] / depth[valid]
  k1, k2, p1, p2, k3 = CAMERA_DISTORTION
  r2 = nx * nx + ny * ny
  radial = 1.0 + k1 * r2 + k2 * r2**2 + k3 * r2**3
  dx = nx * radial + 2.0 * p1 * nx * ny + p2 * (r2 + 2.0 * nx * nx)
  dy = ny * radial + p1 * (r2 + 2.0 * ny * ny) + 2.0 * p2 * nx * ny
  result[valid, 0] = fx * dx + cx
  result[valid, 1] = fy * dy + cy
  return result


def draw_line(frame: np.ndarray, points: np.ndarray, color: tuple[int, int, int],
              thickness: int, alpha: float, rpy: tuple[float, float, float],
              z_offset: float = 0.0) -> None:
  camera_points = points.copy()
  camera_points[:, 2] += z_offset
  pixels = project(camera_points, frame.shape[1], frame.shape[0], rpy)
  valid = np.isfinite(pixels).all(axis=1)
  valid &= (pixels[:, 0] >= -100) & (pixels[:, 0] < frame.shape[1] + 100)
  valid &= (pixels[:, 1] >= -100) & (pixels[:, 1] < frame.shape[0] + 100)
  pixels = np.rint(pixels[valid]).astype(np.int32)
  if len(pixels) < 2:
    return
  overlay = frame.copy()
  cv2.polylines(overlay, [pixels], False, color, thickness, cv2.LINE_AA)
  cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, frame)


def sigmoid(value: float) -> float:
  return float(1.0 / (1.0 + np.exp(-np.clip(value, -50.0, 50.0))))


def draw_output(frame: np.ndarray, output: np.ndarray,
                rpy: tuple[float, float, float], reference: bool = False) -> None:
  lane_means = output[117:381].reshape(4, 33, 2)
  lane_logits = output[645:653]
  edge_means = output[653:785].reshape(2, 33, 2)
  edge_log_stds = output[785:917].reshape(2, 33, 2)
  plan = output[1576:2071].reshape(33, 15)[:, :3]

  for edge in range(2):
    points = np.column_stack((X_IDXS, edge_means[edge]))
    confidence = float(np.clip(1.0 - np.exp(min(float(edge_log_stds[edge, 0, 0]), 11.0)), 0.2, 0.75))
    draw_line(frame, points, (220, 80, 220) if reference else (60, 70, 255),
              1 if reference else 2, confidence, rpy)
  for lane in range(4):
    probability = sigmoid(float(lane_logits[lane * 2 + 1]))
    if probability < 0.15:
      continue
    points = np.column_stack((X_IDXS, lane_means[lane]))
    draw_line(frame, points, (255, 90, 255) if reference else (30, 230, 255),
              1 if reference else 2, float(np.clip(probability, 0.2, 0.9)),
              rpy)
  draw_line(frame, plan, (255, 80, 255) if reference else (70, 255, 100),
            2 if reference else 4, 0.7, rpy, CAMERA_HEIGHT)


def render_frame(nv12: np.ndarray, output: np.ndarray, index: int,
                 rpy: tuple[float, float, float],
                 reference_output: np.ndarray | None = None) -> np.ndarray:
  frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
  draw_output(frame, output, rpy)
  if reference_output is not None:
    draw_output(frame, reference_output, rpy, reference=True)
  plan = output[1576:2071].reshape(33, 15)[:, :3]

  cv2.rectangle(frame, (0, frame.shape[0] - 42), (frame.shape[1], frame.shape[0]), (0, 0, 0), -1)
  text = (f"K230 modern supercombo  frame {index:03d}  k230_ov5647  "
          f"plan {plan[-1, 0]:.1f}m")
  cv2.putText(frame, text, (12, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
              0.52, (235, 245, 245), 1, cv2.LINE_AA)
  return frame


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--replay", required=True, type=Path)
  parser.add_argument("--outputs", required=True, type=Path)
  parser.add_argument("--out", required=True, type=Path)
  parser.add_argument("--contact-sheet", type=Path)
  parser.add_argument("--fps", type=float, default=20.0)
  parser.add_argument("--rpy-deg", type=float, nargs=3, metavar=("ROLL", "PITCH", "YAW"),
                      default=(0.0, 0.0, 0.0))
  parser.add_argument("--warmup-frames", type=int, default=0)
  parser.add_argument("--output-start-frame", type=int, default=0,
                      help="replay frame corresponding to outputs[0]")
  parser.add_argument("--reference-outputs", type=Path,
                      help="optional .npy host outputs beginning after warm-up")
  args = parser.parse_args()

  width, height, replay = read_replay(args.replay)
  outputs = read_outputs(args.outputs)
  first_frame = max(args.warmup_frames, args.output_start_frame)
  end_frame = min(len(replay), args.output_start_frame + len(outputs))
  if end_frame <= first_frame:
    raise ValueError("no frames to render")
  reference_outputs = np.load(args.reference_outputs) if args.reference_outputs else None
  rpy = tuple(np.deg2rad(args.rpy_deg).tolist())
  args.out.parent.mkdir(parents=True, exist_ok=True)
  writer = cv2.VideoWriter(str(args.out), cv2.VideoWriter_fourcc(*"mp4v"),
                           args.fps, (width, height))
  if not writer.isOpened():
    raise RuntimeError(f"cannot create {args.out}")

  rendered: dict[int, np.ndarray] = {}
  render_indices = range(first_frame, end_frame)
  sheet_indices = set(np.linspace(first_frame, end_frame - 1, 4, dtype=int).tolist())
  for index in render_indices:
    output_index = index - args.output_start_frame
    ref_index = index - args.warmup_frames
    reference = (reference_outputs[ref_index] if reference_outputs is not None and
                 ref_index < len(reference_outputs) else None)
    frame = render_frame(replay[index], outputs[output_index], index, rpy, reference)
    writer.write(frame)
    if index in sheet_indices:
      rendered[index] = frame.copy()
  writer.release()

  if args.contact_sheet:
    args.contact_sheet.parent.mkdir(parents=True, exist_ok=True)
    sheet = np.hstack([rendered[index] for index in sorted(rendered)])
    if not cv2.imwrite(str(args.contact_sheet), sheet):
      raise RuntimeError(f"cannot create {args.contact_sheet}")
  print(f"wrote {args.out} frames={end_frame - first_frame} size={width}x{height} "
        f"camera=k230_ov5647 warmup={args.warmup_frames}")


if __name__ == "__main__":
  main()
