#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper, shape_inference


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def add_initializer(graph: onnx.GraphProto, name: str, value: np.ndarray) -> str:
  graph.initializer.append(numpy_helper.from_array(np.ascontiguousarray(value), name))
  return name


def static_shapes(model: onnx.ModelProto) -> dict[str, tuple[int, ...]]:
  inferred = shape_inference.infer_shapes(model)
  shapes: dict[str, tuple[int, ...]] = {}
  for value in list(inferred.graph.input) + list(inferred.graph.value_info) + list(inferred.graph.output):
    dims = tuple(int(dim.dim_value) for dim in value.type.tensor_type.shape.dim)
    if dims and all(dims):
      shapes[value.name] = dims
  return shapes


def initializer_values(model: onnx.ModelProto) -> dict[str, np.ndarray]:
  return {tensor.name: numpy_helper.to_array(tensor) for tensor in model.graph.initializer}


def rewrite_fixed_ops(model: onnx.ModelProto) -> dict[str, int]:
  graph = model.graph
  shapes = static_shapes(model)
  values = initializer_values(model)
  stats = {
    "reduce_mean_to_depthwise_conv": 0,
    "reduce_l2_to_primitives": 0,
    "hidden_slice_to_gather": 0,
    "singleton_transpose_to_reshape": 0,
    "redundant_expand_removed": 0,
  }
  new_nodes: list[onnx.NodeProto] = []
  expand_replacements: dict[str, str] = {}
  for node in graph.node:
    if node.op_type != "Expand":
      continue
    input_shape = shapes.get(node.input[0])
    output_shape = shapes.get(node.output[0])
    if input_shape == (1, 1) and output_shape == (1, 512):
      expand_replacements[node.output[0]] = node.input[0]

  for node in graph.node:
    for index, input_name in enumerate(node.input):
      if input_name in expand_replacements:
        node.input[index] = expand_replacements[input_name]

  for node in graph.node:
    if node.op_type == "Expand" and node.output[0] in expand_replacements:
      stats["redundant_expand_removed"] += 1
      continue

    if node.op_type == "ReduceMean":
      input_shape = shapes.get(node.input[0])
      axes = values.get(node.input[1]) if len(node.input) > 1 else None
      if input_shape is None or len(input_shape) != 4 or axes is None or tuple(axes.tolist()) not in ((2, 3), (-1, -2)):
        raise RuntimeError(f"unsupported ReduceMean for K230 rewrite: {node.name}")
      channels, height, width = input_shape[1:]
      weight_name = add_initializer(
        graph,
        f"{node.output[0]}_mean_dwconv_w",
        np.full((channels, 1, height, width), 1.0 / (height * width), dtype=np.float32),
      )
      new_nodes.append(helper.make_node(
        "Conv",
        [node.input[0], weight_name],
        list(node.output),
        name=f"{node.name}_mean_dwconv",
        dilations=[1, 1],
        group=channels,
        kernel_shape=[height, width],
        pads=[0, 0, 0, 0],
        strides=[1, 1],
      ))
      stats["reduce_mean_to_depthwise_conv"] += 1
      continue

    if node.op_type == "ReduceL2":
      input_shape = shapes.get(node.input[0])
      axes = values.get(node.input[1]) if len(node.input) > 1 else None
      if input_shape is None or len(input_shape) != 2 or axes is None or tuple(axes.tolist()) not in ((1,), (-1,)):
        raise RuntimeError(f"unsupported ReduceL2 for K230 rewrite: {node.name}")
      width = input_shape[-1]
      square = f"{node.output[0]}_square"
      summed = f"{node.output[0]}_sum"
      ones_name = add_initializer(
        graph,
        f"{node.output[0]}_ones_{width}x1",
        np.ones((width, 1), dtype=np.float32),
      )
      new_nodes.extend([
        helper.make_node("Mul", [node.input[0], node.input[0]], [square], name=f"{node.name}_square"),
        helper.make_node("MatMul", [square, ones_name], [summed], name=f"{node.name}_sum_matmul"),
        helper.make_node("Sqrt", [summed], list(node.output), name=f"{node.name}_sqrt"),
      ])
      stats["reduce_l2_to_primitives"] += 1
      continue

    if node.name == "Slice_263" and node.op_type == "Slice":
      starts = values.get(node.input[1])
      ends = values.get(node.input[2])
      axes = values.get(node.input[3])
      if starts is None or ends is None or axes is None:
        raise RuntimeError("hidden-state Slice parameters must be constant")
      start, end, axis = int(starts[0]), int(ends[0]), int(axes[0])
      if (start, end, axis) != (1064, 1576, 1):
        raise RuntimeError(f"unexpected hidden-state Slice: start={start}, end={end}, axis={axis}")
      index_name = add_initializer(
        graph,
        "hidden_gather_indices_1064_1576",
        np.arange(start, end, dtype=np.int64),
      )
      new_nodes.append(helper.make_node(
        "Gather", [node.input[0], index_name], list(node.output), name="Slice_263_gather", axis=axis,
      ))
      stats["hidden_slice_to_gather"] += 1
      continue

    if node.name in ("p_node_Transpose_0", "p_node_index") and node.op_type == "Transpose":
      output_shape = shapes.get(node.output[0])
      if output_shape is None:
        raise RuntimeError(f"static output shape not found for {node.name}")
      shape_name = add_initializer(
        graph,
        f"{node.output[0]}_reshape_shape",
        np.asarray(output_shape, dtype=np.int64),
      )
      new_nodes.append(helper.make_node(
        "Reshape", [node.input[0], shape_name], list(node.output), name=f"{node.name}_reshape",
      ))
      stats["singleton_transpose_to_reshape"] += 1
      continue

    new_nodes.append(node)

  del graph.node[:]
  graph.node.extend(new_nodes)
  return stats


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--in-model", required=True, type=Path)
  parser.add_argument("--out-model", required=True, type=Path)
  parser.add_argument("--check", action="store_true")
  args = parser.parse_args()

  model = onnx.load(args.in_model)
  stats = rewrite_fixed_ops(model)
  if args.check:
    onnx.checker.check_model(model)
    model = shape_inference.infer_shapes(model)
  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(model, args.out_model)

  print(f"wrote {args.out_model}")
  for key, value in stats.items():
    print(f"{key}={value}")
  print(f"size={args.out_model.stat().st_size}")
  print(f"sha256={sha256_file(args.out_model)}")


if __name__ == "__main__":
  main()
