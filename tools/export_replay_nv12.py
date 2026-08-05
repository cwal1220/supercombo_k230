#!/usr/bin/env python3
"""Export a deterministic SCNV12R1 replay without changing camera geometry."""

from __future__ import annotations

import argparse
import struct
import subprocess
from pathlib import Path


def probe_size(video: Path) -> tuple[int, int]:
  result = subprocess.run([
      "ffprobe", "-v", "error", "-select_streams", "v:0",
      "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", str(video),
  ], check=True, capture_output=True, text=True)
  width, height = result.stdout.strip().split("x")
  return int(width), int(height)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--video", required=True, type=Path)
  parser.add_argument("--out", required=True, type=Path)
  parser.add_argument("--start", type=float, default=0.0)
  parser.add_argument("--frames", type=int, required=True)
  parser.add_argument("--fps", type=float, default=20.0)
  parser.add_argument("--source-fps", type=float, default=25.0,
                      help="used for frame-index seeking on timestamp-less raw HEVC")
  args = parser.parse_args()

  if args.frames <= 0 or args.fps <= 0.0 or args.source_fps <= 0.0:
    raise ValueError("frames and frame rates must be positive")
  width, height = probe_size(args.video)
  if width % 2 or height % 2:
    raise ValueError("NV12 requires even dimensions")
  skip_frames = int(round(args.start * args.source_fps))
  vf = f"select='gte(n,{skip_frames})',fps={args.fps}"
  command = [
      "ffmpeg", "-hide_banner", "-loglevel", "error", "-i", str(args.video),
      "-vf", vf, "-frames:v", str(args.frames), "-pix_fmt", "nv12",
      "-f", "rawvideo", "pipe:1",
  ]
  args.out.parent.mkdir(parents=True, exist_ok=True)
  frame_bytes = width * height * 3 // 2
  with args.out.open("wb") as dst:
    dst.write(b"SCNV12R1")
    dst.write(struct.pack("<III", width, height, args.frames))
    process = subprocess.Popen(command, stdout=subprocess.PIPE)
    assert process.stdout is not None
    written = 0
    while True:
      block = process.stdout.read(1024 * 1024)
      if not block:
        break
      dst.write(block)
      written += len(block)
    return_code = process.wait()
  if return_code != 0:
    raise RuntimeError(f"ffmpeg failed with exit code {return_code}")
  expected = args.frames * frame_bytes
  if written != expected:
    args.out.unlink(missing_ok=True)
    raise RuntimeError(f"expected {expected} raw bytes, got {written}")
  print(f"wrote {args.out} frames={args.frames} size={width}x{height} fps={args.fps:g} start={args.start:g}s")


if __name__ == "__main__":
  main()
