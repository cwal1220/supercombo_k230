#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def add_initializer(graph: onnx.GraphProto, name: str, arr: np.ndarray) -> None:
  graph.initializer.append(numpy_helper.from_array(np.ascontiguousarray(arr), name))


def get_attr(node: onnx.NodeProto, name: str, default=None):
  for attr in node.attribute:
    if attr.name == name:
      return helper.get_attribute_value(attr)
  return default


def assert_front_pattern(model: onnx.ModelProto) -> tuple[str, str, str, str]:
  nodes = list(model.graph.node)
  if len(nodes) < 5:
    raise RuntimeError("model has too few nodes")

  cast_img, cast_big, concat, sub, div = nodes[:5]
  if cast_img.op_type != "Cast" or cast_img.input[0] != "img" or get_attr(cast_img, "to") != TensorProto.FLOAT:
    raise RuntimeError(f"unexpected img Cast node: {cast_img}")
  if cast_big.op_type != "Cast" or cast_big.input[0] != "big_img" or get_attr(cast_big, "to") != TensorProto.FLOAT:
    raise RuntimeError(f"unexpected big_img Cast node: {cast_big}")
  if concat.op_type != "Concat" or list(concat.input) != [cast_img.output[0], cast_big.output[0]] or get_attr(concat, "axis") != 1:
    raise RuntimeError(f"unexpected front Concat node: {concat}")
  if sub.op_type != "Sub" or sub.input[0] != concat.output[0]:
    raise RuntimeError(f"unexpected front Sub node: {sub}")
  if div.op_type != "Div" or div.input[0] != sub.output[0]:
    raise RuntimeError(f"unexpected front Div node: {div}")

  init = {tensor.name: numpy_helper.to_array(tensor) for tensor in model.graph.initializer}
  mean = init.get(sub.input[1])
  std = init.get(div.input[1])
  if mean is None or std is None:
    raise RuntimeError("front mean/std initializers not found")
  if mean.shape != (1, 24, 1, 1) or std.shape != (1, 24, 1, 1):
    raise RuntimeError(f"unexpected mean/std shapes: {mean.shape}, {std.shape}")
  if not np.allclose(mean, 127.5) or not np.allclose(std, 63.75):
    raise RuntimeError(f"unexpected mean/std values: mean {mean.min()}..{mean.max()}, std {std.min()}..{std.max()}")

  return cast_img.output[0], cast_big.output[0], concat.output[0], div.output[0]


def rewrite_front(model: onnx.ModelProto) -> None:
  _, _, _, normalized_output = assert_front_pattern(model)
  graph = model.graph
  tail_nodes = list(graph.node[5:])

  scale = np.array([1.0 / 63.75], dtype=np.float32)
  zero_point = np.array([128], dtype=np.uint8)
  half_pixel_offset = np.array([0.5 / 63.75], dtype=np.float32)
  add_initializer(graph, "k230_front_dq_scale", scale)
  add_initializer(graph, "k230_front_dq_zero_point", zero_point)
  add_initializer(graph, "k230_front_half_pixel_offset", half_pixel_offset)

  new_front = [
    helper.make_node(
      "DequantizeLinear",
      ["img", "k230_front_dq_scale", "k230_front_dq_zero_point"],
      ["k230_front_img_norm_minus_half"],
      name="k230_front_img_dequant_norm",
    ),
    helper.make_node(
      "Add",
      ["k230_front_img_norm_minus_half", "k230_front_half_pixel_offset"],
      ["k230_front_img_norm"],
      name="k230_front_img_add_half_pixel",
    ),
    helper.make_node(
      "DequantizeLinear",
      ["big_img", "k230_front_dq_scale", "k230_front_dq_zero_point"],
      ["k230_front_big_img_norm_minus_half"],
      name="k230_front_big_img_dequant_norm",
    ),
    helper.make_node(
      "Add",
      ["k230_front_big_img_norm_minus_half", "k230_front_half_pixel_offset"],
      ["k230_front_big_img_norm"],
      name="k230_front_big_img_add_half_pixel",
    ),
    helper.make_node(
      "Concat",
      ["k230_front_img_norm", "k230_front_big_img_norm"],
      [normalized_output],
      name="k230_front_concat_normalized_imgs",
      axis=1,
    ),
  ]

  del graph.node[:]
  graph.node.extend(new_front + tail_nodes)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--in-model", required=True, type=Path)
  parser.add_argument("--out-model", required=True, type=Path)
  parser.add_argument("--check", action="store_true")
  args = parser.parse_args()

  model = onnx.load(args.in_model)
  old_nodes = len(model.graph.node)
  rewrite_front(model)
  if args.check:
    onnx.checker.check_model(model)
    model = onnx.shape_inference.infer_shapes(model)
  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(model, args.out_model)
  print(f"wrote {args.out_model}")
  print(f"old_nodes={old_nodes}")
  print(f"new_nodes={len(model.graph.node)}")
  print(f"size={args.out_model.stat().st_size}")
  print(f"sha256={sha256_file(args.out_model)}")


if __name__ == "__main__":
  main()
