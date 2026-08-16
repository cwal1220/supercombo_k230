#!/usr/bin/env python3
"""PTQ NPZ의 실주행 입력으로 sequence_runner용 SCSEQ1 파일을 만든다.

같은 데이터를 float(mode 0)와 uint8(mode 2)로 내보내 신구 kmodel의 출력을
동일 조건에서 비교하는 데 쓴다. NPZ의 이미지 값은 0..255 정수(float 표현)라
uint8 변환은 무손실이다.
"""
import argparse
import struct
from pathlib import Path

import numpy as np

MODE_FLOAT_YUV6 = 0
MODE_UINT8_YUV6 = 2


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--npz", type=Path, required=True)
  parser.add_argument("--out", type=Path, required=True)
  parser.add_argument("--mode", choices=("float", "uint8"), required=True)
  parser.add_argument("--frames", type=int, default=20)
  args = parser.parse_args()

  data = np.load(str(args.npz))
  imgs = data["input_imgs"]
  big = data["big_input_imgs"]
  desire = data["desire"]
  traffic = data["traffic_convention"]
  frames = min(args.frames, imgs.shape[0])

  for name, arr in (("input_imgs", imgs), ("big_input_imgs", big)):
    if not np.array_equal(arr, np.round(arr)) or arr.min() < 0 or arr.max() > 255:
      raise RuntimeError(f"{name} is not exact 0..255 integers; uint8 export would be lossy")

  mode = MODE_UINT8_YUV6 if args.mode == "uint8" else MODE_FLOAT_YUV6
  with args.out.open("wb") as out:
    out.write(b"SCSEQ1\x00\x00")
    out.write(struct.pack("<II", mode, frames))
    for i in range(frames):
      for arr in (imgs[i], big[i]):
        if mode == MODE_UINT8_YUV6:
          out.write(arr.astype(np.uint8).tobytes())
        else:
          out.write(arr.astype("<f4").tobytes())
      out.write(desire[i].astype("<f4").tobytes())
      out.write(traffic[i].astype("<f4").tobytes())
  print(f"wrote {args.out} mode={args.mode} frames={frames} "
        f"bytes={args.out.stat().st_size}")


if __name__ == "__main__":
  main()
