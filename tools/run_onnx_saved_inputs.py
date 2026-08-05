#!/usr/bin/env python3
"""Run an ONNX driving model on saved inputs, optionally rebuilding context."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--model", required=True, type=Path)
  parser.add_argument("--metadata", required=True, type=Path)
  parser.add_argument("--out", required=True, type=Path)
  parser.add_argument("--autoregressive-features", action="store_true",
                      help="load image streams only and rebuild feature history from ONNX outputs")
  parser.add_argument("--save-inputs-npz", type=Path,
                      help="save the exact per-frame ONNX feeds for PTQ/replay")
  args = parser.parse_args()

  metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
  inputs: dict[str, np.ndarray] = {}
  metadata_dir = args.metadata.parent
  stream_names = ("img", "big_img") if args.autoregressive_features else metadata["streams"].keys()
  for name in stream_names:
    stream = metadata["streams"][name]
    stream_path = Path(stream)
    if not stream_path.is_absolute():
      candidates = [metadata_dir / stream_path.name, Path.cwd() / stream_path]
      stream_path = next((path for path in candidates if path.is_file()), candidates[0])
    shape = tuple(metadata["input_shapes"][name])
    dtype = np.dtype(metadata["input_dtypes"][name])
    inputs[name] = np.fromfile(stream_path, dtype=dtype).reshape(shape)

  session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
  frames = min(array.shape[0] for array in inputs.values())
  outputs = []
  collected_inputs: dict[str, list[np.ndarray]] = {}
  feature_queue = np.zeros((96, 1, 512), dtype=np.float32)
  previous_hidden = np.zeros((1, 512), dtype=np.float32)
  node_dtypes = {
      "tensor(uint8)": np.uint8,
      "tensor(float16)": np.float16,
      "tensor(float)": np.float32,
  }
  for frame in range(frames):
    if args.autoregressive_features:
      feature_queue = np.concatenate((feature_queue[1:], previous_hidden.reshape(1, 1, 512)), axis=0)
      values = {
          "img": inputs["img"][frame],
          "big_img": inputs["big_img"][frame],
          "features_buffer": feature_queue[::4].reshape(1, 24, 512),
          "desire_pulse": np.zeros((1, 25, 8), dtype=np.float32),
          "traffic_convention": np.array([[1.0, 0.0]], dtype=np.float32),
          "action_t": np.array([[0.2, 0.5]], dtype=np.float32),
      }
      feed = {
          node.name: np.ascontiguousarray(values[node.name], dtype=node_dtypes[node.type])
          for node in session.get_inputs()
      }
    else:
      feed = {node.name: np.ascontiguousarray(inputs[node.name][frame]) for node in session.get_inputs()}
    output = np.asarray(session.run(None, feed)[0]).reshape(-1)
    if args.save_inputs_npz:
      for name, value in feed.items():
        collected_inputs.setdefault(name, []).append(value.copy())
    outputs.append(output)
    if args.autoregressive_features:
      previous_hidden = output[1064:1576].reshape(1, 512).astype(np.float32)

  result = np.stack(outputs).astype(np.float32)
  args.out.parent.mkdir(parents=True, exist_ok=True)
  np.save(args.out, result)
  if args.save_inputs_npz:
    args.save_inputs_npz.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.save_inputs_npz,
                        **{name: np.stack(values) for name, values in collected_inputs.items()})
    print(f"wrote {args.save_inputs_npz} inputs={list(collected_inputs)}")
  print(f"wrote {args.out} shape={result.shape}")


if __name__ == "__main__":
  main()
