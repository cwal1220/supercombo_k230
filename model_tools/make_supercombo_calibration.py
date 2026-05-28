#!/usr/bin/env python3
import argparse
import bz2
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


def append_samples_from_segment(session, seg: Path, wanted: int, warmup: int):
  extrinsic = read_live_calibration(seg / "qlog.bz2")
  if extrinsic is None:
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
  args = parser.parse_args()

  session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
  all_samples = []
  for seg in find_segments(args.data_root)[:args.max_segments]:
    if len(all_samples) >= args.samples:
      break
    try:
      new_samples = append_samples_from_segment(session, seg, args.per_segment, args.warmup)
    except Exception as e:
      print(f"skip {seg}: {e}")
      continue
    all_samples.extend(new_samples)
    print(f"{seg}: +{len(new_samples)} samples, total={len(all_samples)}")

  if len(all_samples) < args.samples:
    raise RuntimeError(f"only collected {len(all_samples)} samples")

  all_samples = all_samples[:args.samples]
  args.out.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(
    args.out,
    input_imgs=np.stack([s["input_imgs"] for s in all_samples]).astype(np.float32),
    big_input_imgs=np.stack([s["big_input_imgs"] for s in all_samples]).astype(np.float32),
    desire=np.stack([s["desire"] for s in all_samples]).astype(np.float32),
    traffic_convention=np.stack([s["traffic_convention"] for s in all_samples]).astype(np.float32),
    initial_state=np.stack([s["initial_state"] for s in all_samples]).astype(np.float32),
  )
  print(args.out)
  print(f"samples={len(all_samples)}")


if __name__ == "__main__":
  main()
