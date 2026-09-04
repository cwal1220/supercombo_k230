#!/usr/bin/env python3
"""Pre-quantize Conv/Gemm weights on nncase's per-channel uint8 grid and correct biases.

Rounding the weights in the ONNX graph makes the compiler's own weight quantization
lossless, so the per-channel mean shift each layer's quantized weights introduce can
be measured with ONNX Runtime on calibration inputs and cancelled in the bias
(sequentially, with upstream corrections applied). Activation-side quantization is
untouched. The grid comes from an nncase QuantScheme.json exported with
export_weight_range_by_channel (exact match); without it, per-channel min/max is used.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
from onnx import numpy_helper

INPUTS = ("input_imgs", "big_input_imgs", "desire", "traffic_convention", "initial_state")


def load_scheme_ranges(path):
  if path is None:
    return {}
  ranges = {}
  for entry in json.loads(Path(path).read_text())["Outputs"]:
    if entry["DataRangeMode"] == "by_channel":
      ranges[entry["Name"]] = (np.array([r["Min"] for r in entry["DataRange"]]),
                               np.array([r["Max"] for r in entry["DataRange"]]))
  return ranges


def quantize_grid(w, lo, hi):
  flat = w.reshape(w.shape[0], -1)
  scale = np.maximum((hi - lo) / 255.0, 1e-12)
  zero = np.round(-lo / scale)
  q = np.clip(np.round(flat / scale[:, None]) + zero[:, None], 0, 255)
  return ((q - zero[:, None]) * scale[:, None]).reshape(w.shape)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--in-model", type=Path, required=True)
  parser.add_argument("--out-model", type=Path, required=True)
  parser.add_argument("--calib-npz", type=Path, required=True)
  parser.add_argument("--samples", type=int, default=128, help="calibration samples used for the bias means")
  parser.add_argument("--scheme", type=Path, help="nncase QuantScheme.json with by_channel weight ranges")
  parser.add_argument("--no-bias-correction", action="store_true")
  parser.add_argument("--threads", type=int, default=4)
  args = parser.parse_args()

  model = onnx.load(str(args.in_model))
  inits = {t.name: t for t in model.graph.initializer}
  scheme = load_scheme_ranges(args.scheme)
  layers = [n for n in model.graph.node if n.op_type in ("Conv", "Gemm") and n.input[1] in inits]
  with np.load(args.calib_npz, allow_pickle=False) as calib:
    count = len(calib[INPUTS[0]])
    idx = np.linspace(0, count - 1, min(args.samples, count)).round().astype(int)
    feeds = [{k: calib[k][i:i + 1].astype(np.float32) for k in INPUTS} for i in idx]
  options = ort.SessionOptions()
  options.intra_op_num_threads = args.threads
  options.log_severity_level = 3

  def output_means(graph, outputs):
    probe = onnx.ModelProto()
    probe.CopyFrom(graph)
    del probe.graph.output[:]
    probe.graph.output.extend(onnx.ValueInfoProto(name=o) for o in outputs)
    session = ort.InferenceSession(probe.SerializeToString(), options, providers=["CPUExecutionProvider"])
    acc = {o: 0.0 for o in outputs}
    for feed in feeds:
      for name, value in zip(outputs, session.run(outputs, feed)):
        acc[name] = acc[name] + (value.mean(axis=(0, 2, 3)) if value.ndim == 4 else value.reshape(-1)) / len(feeds)
    return acc

  start = time.monotonic()
  reference = None if args.no_bias_correction else output_means(model, [n.output[0] for n in layers])
  from_scheme = 0
  for index, node in enumerate(layers):
    tensor = inits[node.input[1]]
    w = numpy_helper.to_array(tensor).astype(np.float64)
    channels = w.shape[0]
    if node.input[1] in scheme and len(scheme[node.input[1]][0]) == channels:
      lo, hi = scheme[node.input[1]]
      from_scheme += 1
    else:
      flat = w.reshape(channels, -1)
      lo, hi = np.minimum(flat.min(1), 0.0), np.maximum(flat.max(1), 0.0)
    tensor.CopyFrom(numpy_helper.from_array(quantize_grid(w, lo, hi).astype(np.float32), tensor.name))
    if reference is not None and len(node.input) > 2 and node.input[2] in inits:
      shifted = output_means(model, [node.output[0]])[node.output[0]]
      bias = numpy_helper.to_array(inits[node.input[2]]).astype(np.float64)
      inits[node.input[2]].CopyFrom(numpy_helper.from_array(
        (bias + reference[node.output[0]] - shifted).astype(np.float32), node.input[2]))
    if index % 25 == 0:
      print(f"{index + 1}/{len(layers)} {node.name} ({time.monotonic() - start:.0f}s)", flush=True)
  onnx.checker.check_model(model)
  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(model, str(args.out_model))
  print(f"{args.out_model}: {len(layers)} layers pre-quantized ({from_scheme} on scheme ranges), "
        f"bias correction={'off' if args.no_bias_correction else f'{len(feeds)} samples'}")


if __name__ == "__main__":
  main()
