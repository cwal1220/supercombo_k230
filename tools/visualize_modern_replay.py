#!/usr/bin/env python3
"""Render a K230 modern-supercombo SCODMP1 replay without openpilot dependencies."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import cv2
import numpy as np


X_IDXS = np.array([192.0 * (i / 32.0) ** 2 for i in range(33)], dtype=np.float32)


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


def project(points: np.ndarray, width: int, height: int, camera_height: float = 0.0) -> np.ndarray:
  fx = 910.0 * width / 1164.0
  fy = 910.0 * height / 874.0
  forward = points[:, 0]
  valid = np.isfinite(points).all(axis=1) & (forward > 0.5) & (forward < 200.0)
  result = np.full((points.shape[0], 2), np.nan, dtype=np.float32)
  result[valid, 0] = width * 0.5 + fx * points[valid, 1] / forward[valid]
  result[valid, 1] = height * 0.5 + fy * (points[valid, 2] + camera_height) / forward[valid]
  return result


def draw_line(frame: np.ndarray, points: np.ndarray, color: tuple[int, int, int],
              thickness: int, alpha: float, camera_height: float = 0.0) -> None:
  pixels = project(points, frame.shape[1], frame.shape[0], camera_height)
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


def render_frame(nv12: np.ndarray, output: np.ndarray, index: int) -> np.ndarray:
  frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
  lane_means = output[117:381].reshape(4, 33, 2)
  lane_logits = output[645:653]
  edge_means = output[653:785].reshape(2, 33, 2)
  edge_log_stds = output[785:917].reshape(2, 33, 2)
  plan = output[1576:2071].reshape(33, 15)[:, :3]

  for edge in range(2):
    points = np.column_stack((X_IDXS, edge_means[edge]))
    confidence = float(np.clip(1.0 - np.exp(min(float(edge_log_stds[edge, 0, 0]), 11.0)), 0.2, 0.75))
    draw_line(frame, points, (60, 70, 255), 2, confidence)
  for lane in range(4):
    probability = sigmoid(float(lane_logits[lane * 2 + 1]))
    if probability < 0.15:
      continue
    points = np.column_stack((X_IDXS, lane_means[lane]))
    draw_line(frame, points, (30, 230, 255), 2, float(np.clip(probability, 0.2, 0.9)))
  draw_line(frame, plan, (70, 255, 100), 4, 0.7, camera_height=1.22)

  cv2.rectangle(frame, (0, frame.shape[0] - 42), (frame.shape[1], frame.shape[0]), (0, 0, 0), -1)
  text = f"K230 modern supercombo  frame {index:03d}  plan {plan[-1, 0]:.1f}m"
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
  args = parser.parse_args()

  width, height, replay = read_replay(args.replay)
  outputs = read_outputs(args.outputs)
  frames = min(len(replay), len(outputs))
  if frames == 0:
    raise ValueError("no frames to render")
  args.out.parent.mkdir(parents=True, exist_ok=True)
  writer = cv2.VideoWriter(str(args.out), cv2.VideoWriter_fourcc(*"mp4v"),
                           args.fps, (width, height))
  if not writer.isOpened():
    raise RuntimeError(f"cannot create {args.out}")

  rendered: dict[int, np.ndarray] = {}
  sheet_indices = set(np.linspace(0, frames - 1, 4, dtype=int).tolist())
  for index in range(frames):
    frame = render_frame(replay[index], outputs[index], index)
    writer.write(frame)
    if index in sheet_indices:
      rendered[index] = frame.copy()
  writer.release()

  if args.contact_sheet:
    args.contact_sheet.parent.mkdir(parents=True, exist_ok=True)
    sheet = np.hstack([rendered[index] for index in sorted(rendered)])
    if not cv2.imwrite(str(args.contact_sheet), sheet):
      raise RuntimeError(f"cannot create {args.contact_sheet}")
  print(f"wrote {args.out} frames={frames} size={width}x{height}")


if __name__ == "__main__":
  main()
