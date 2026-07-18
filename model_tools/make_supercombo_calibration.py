#!/usr/bin/env python3
import argparse
import bz2
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from cereal import log as capnp_log
from model_viz_experiment.visualize_model import (
  MODEL_PATH,
  HALF_H,
  HALF_W,
  RECURRENT_SIZE,
  intrinsics_for_frame,
  model_input_from_yuv,
  model_warp_matrix,
  run_model,
  warped_yuv_from_frame,
)

K230_MODEL_W = 512
K230_MODEL_H = 256
K230_FX = 910.0
K230_FY = 910.0
K230_CX = 256.0
K230_CY = 47.6
K230_HEIGHT = 1.22


def segment_index(path: Path) -> int:
  try:
    return int(path.name.rsplit("--", 1)[1])
  except Exception:
    return 0


def find_segments(data_root: Path) -> list[Path]:
  segments = []
  for qlog in data_root.rglob("qlog.bz2"):
    seg = qlog.parent
    if (seg / "fcamera.hevc").is_file():
      segments.append(seg)
  return sorted(segments, key=lambda p: (str(p.parent), segment_index(p)))


def read_segment_list(path: Path) -> list[Path]:
  segments = []
  for line in path.read_text().splitlines():
    line = line.strip()
    if not line or line.startswith("#"):
      continue
    seg = Path(line.split()[0]).expanduser()
    segments.append(seg)
  return segments


def read_live_calibration(qlog_path: Path):
  events = capnp_log.Event.read_multiple_bytes(bz2.decompress(qlog_path.read_bytes()))
  for event in events:
    if event.which() == "liveCalibration":
      live_calib = event.liveCalibration
      extrinsic = np.array(live_calib.extrinsicMatrix, dtype=np.float32).reshape(3, 4)
      return extrinsic
  return None


def extract_frames(video_path: Path, out_dir: Path, count: int) -> list[Path]:
  out_dir.mkdir(parents=True, exist_ok=True)
  pattern = out_dir / "frame_%04d.png"
  cmd = [
    "ffmpeg",
    "-hide_banner",
    "-loglevel", "error",
    "-i", str(video_path),
    "-frames:v", str(count),
    str(pattern),
  ]
  subprocess.run(cmd, check=True)
  return sorted(out_dir.glob("frame_*.png"))[:count]


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
  cr, sr = np.cos(roll), np.sin(roll)
  cp, sp = np.cos(pitch), np.sin(pitch)
  cy, sy = np.cos(yaw), np.sin(yaw)
  rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float32)
  ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float32)
  rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
  return rz @ ry @ rx


def transform_scale_buffer(mat: np.ndarray, scale: float) -> np.ndarray:
  transform_out = np.array(
    [[1.0 / scale, 0.0, 0.5], [0.0, 1.0 / scale, 0.5], [0.0, 0.0, 1.0]],
    dtype=np.float32,
  )
  transform_in = np.array(
    [[scale, 0.0, -0.5 * scale], [0.0, scale, -0.5 * scale], [0.0, 0.0, 1.0]],
    dtype=np.float32,
  )
  return transform_in @ mat @ transform_out


def k230_projection_matrix() -> np.ndarray:
  ground_from_medmodel_frame = np.array(
    [
      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00],
      [-1.09890110e-03, 0.00000000e+00, 2.81318681e-01],
      [-1.84808520e-20, 9.00738606e-04, -4.28751576e-02],
    ],
    dtype=np.float32,
  )
  intrinsics = np.array(
    [[K230_FX, 0.0, K230_CX], [0.0, K230_FY, K230_CY], [0.0, 0.0, 1.0]],
    dtype=np.float32,
  )
  rot = rotation_from_rpy(0.0, 0.0, 0.0)
  device_from_road = rot.copy()
  device_from_road[:, 1] *= -1.0
  device_from_road[:, 2] *= -1.0
  view_from_road = np.stack([device_from_road[1], device_from_road[2], device_from_road[0]], axis=0)
  extrinsic = np.array(
    [
      [view_from_road[0, 0], view_from_road[0, 1], view_from_road[0, 2], 0.0],
      [view_from_road[1, 0], view_from_road[1, 1], view_from_road[1, 2], K230_HEIGHT],
      [view_from_road[2, 0], view_from_road[2, 1], view_from_road[2, 2], 0.0],
    ],
    dtype=np.float32,
  )
  camera_frame_from_road = intrinsics @ extrinsic
  camera_frame_from_ground = camera_frame_from_road[:, [0, 1, 3]]
  return camera_frame_from_ground @ ground_from_medmodel_frame


def remap_plane(src: np.ndarray, projection: np.ndarray, dst_w: int, dst_h: int) -> np.ndarray:
  xs, ys = np.meshgrid(np.arange(dst_w, dtype=np.float32), np.arange(dst_h, dtype=np.float32))
  denom = projection[2, 0] * xs + projection[2, 1] * ys + projection[2, 2]
  valid = np.abs(denom) > 1e-6
  map_x = np.full((dst_h, dst_w), -1.0, dtype=np.float32)
  map_y = np.full((dst_h, dst_w), -1.0, dtype=np.float32)
  map_x[valid] = (projection[0, 0] * xs[valid] + projection[0, 1] * ys[valid] + projection[0, 2]) / denom[valid]
  map_y[valid] = (projection[1, 0] * xs[valid] + projection[1, 1] * ys[valid] + projection[1, 2]) / denom[valid]
  return cv2.remap(src, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def k230_warped_yuv_from_frame(frame: np.ndarray) -> np.ndarray:
  resized = cv2.resize(frame, (K230_MODEL_W, K230_MODEL_H), interpolation=cv2.INTER_AREA)
  yuv = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV_I420).reshape(-1)
  y_size = K230_MODEL_W * K230_MODEL_H
  uv_size = (K230_MODEL_W // 2) * (K230_MODEL_H // 2)
  y_plane = yuv[:y_size].reshape(K230_MODEL_H, K230_MODEL_W)
  u_plane = yuv[y_size:y_size + uv_size].reshape(K230_MODEL_H // 2, K230_MODEL_W // 2)
  v_plane = yuv[y_size + uv_size:y_size + 2 * uv_size].reshape(K230_MODEL_H // 2, K230_MODEL_W // 2)

  projection_y = k230_projection_matrix()
  projection_uv = transform_scale_buffer(projection_y, 0.5)
  warped_y = remap_plane(y_plane, projection_y, K230_MODEL_W, K230_MODEL_H).astype(np.float32)
  warped_u = remap_plane(u_plane, projection_uv, HALF_W, HALF_H).astype(np.float32)
  warped_v = remap_plane(v_plane, projection_uv, HALF_W, HALF_H).astype(np.float32)

  return np.stack(
    [
      warped_y[0::2, 0::2],
      warped_y[1::2, 0::2],
      warped_y[0::2, 1::2],
      warped_y[1::2, 1::2],
      warped_u,
      warped_v,
    ],
    axis=0,
  )


def append_samples_from_segment(session, seg: Path, wanted: int, warmup: int, warp_mode: str):
  extrinsic = read_live_calibration(seg / "qlog.bz2")
  if warp_mode == "openpilot" and extrinsic is None:
    return []

  samples = []
  recurrent_state = np.zeros((1, RECURRENT_SIZE), dtype=np.float32)
  prev_yuv = np.zeros((6, HALF_H, HALF_W), dtype=np.float32)
  prev_big_yuv = np.zeros((6, HALF_H, HALF_W), dtype=np.float32)

  with tempfile.TemporaryDirectory(prefix="k230_calib_frames_") as td:
    frame_paths = extract_frames(seg / "fcamera.hevc", Path(td), warmup + wanted)
    for idx, frame_path in enumerate(frame_paths):
      frame = cv2.imread(str(frame_path))
      if frame is None:
        continue

      if warp_mode == "k230":
        current_yuv = k230_warped_yuv_from_frame(frame)
        current_big_yuv = np.zeros_like(current_yuv)
        input_imgs = model_input_from_yuv(prev_yuv, current_yuv)
        big_input_imgs = np.zeros((1, 12, HALF_H, HALF_W), dtype=np.float32)
      else:
        intrinsics = intrinsics_for_frame(frame.shape[1], frame.shape[0])
        warp = model_warp_matrix(intrinsics, extrinsic, bigmodel_frame=False)
        big_warp = model_warp_matrix(intrinsics, extrinsic, bigmodel_frame=True)

        current_yuv, _ = warped_yuv_from_frame(frame, warp)
        current_big_yuv, _ = warped_yuv_from_frame(frame, big_warp)
        input_imgs = model_input_from_yuv(prev_yuv, current_yuv)
        big_input_imgs = model_input_from_yuv(prev_big_yuv, current_big_yuv)

      desire = np.zeros((1, 8), dtype=np.float32)
      traffic_convention = np.array([[1.0, 0.0]], dtype=np.float32)
      raw, next_state = run_model(session, input_imgs, big_input_imgs, recurrent_state)

      if idx >= warmup:
        samples.append({
          "input_imgs": input_imgs[0].copy(),
          "big_input_imgs": big_input_imgs[0].copy(),
          "desire": desire[0].copy(),
          "traffic_convention": traffic_convention[0].copy(),
          "initial_state": recurrent_state[0].copy(),
        })

      recurrent_state = next_state
      prev_yuv = current_yuv
      prev_big_yuv = current_big_yuv
  return samples


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--data-root", required=True, type=Path)
  parser.add_argument("--out", required=True, type=Path)
  parser.add_argument("--model", default=MODEL_PATH, type=Path)
  parser.add_argument("--samples", default=20, type=int)
  parser.add_argument("--warmup", default=5, type=int)
  parser.add_argument("--per-segment", default=2, type=int)
  parser.add_argument("--max-segments", default=50, type=int)
  parser.add_argument("--warp-mode", choices=["openpilot", "k230"], default="openpilot")
  parser.add_argument("--segment-list", type=Path,
                      help="optional newline-delimited list of segment directories to use in order")
  parser.add_argument("--metadata-out", type=Path,
                      help="optional JSON manifest describing the collected samples")
  args = parser.parse_args()

  session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
  all_samples = []
  segment_source = read_segment_list(args.segment_list) if args.segment_list else find_segments(args.data_root)
  collected_meta = []
  for seg in segment_source[:args.max_segments]:
    if len(all_samples) >= args.samples:
      break
    try:
      new_samples = append_samples_from_segment(session, seg, args.per_segment, args.warmup, args.warp_mode)
    except Exception as e:
      print(f"skip {seg}: {e}")
      continue
    for local_idx, sample in enumerate(new_samples):
      sample["segment"] = str(seg)
      sample["segment_sample_index"] = local_idx
    all_samples.extend(new_samples)
    print(f"{seg}: +{len(new_samples)} samples, total={len(all_samples)}")

  if len(all_samples) < args.samples:
    raise RuntimeError(f"only collected {len(all_samples)} samples")

  all_samples = all_samples[:args.samples]
  collected_meta = [
    {
      "sample_index": idx,
      "segment": s.get("segment"),
      "segment_sample_index": s.get("segment_sample_index"),
    }
    for idx, s in enumerate(all_samples)
  ]
  args.out.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(
    args.out,
    input_imgs=np.stack([s["input_imgs"] for s in all_samples]).astype(np.float32),
    big_input_imgs=np.stack([s["big_input_imgs"] for s in all_samples]).astype(np.float32),
    desire=np.stack([s["desire"] for s in all_samples]).astype(np.float32),
    traffic_convention=np.stack([s["traffic_convention"] for s in all_samples]).astype(np.float32),
    initial_state=np.stack([s["initial_state"] for s in all_samples]).astype(np.float32),
  )
  if args.metadata_out:
    args.metadata_out.parent.mkdir(parents=True, exist_ok=True)
    args.metadata_out.write_text(json.dumps(collected_meta, indent=2))
  print(args.out)
  print(f"samples={len(all_samples)}")


if __name__ == "__main__":
  main()
