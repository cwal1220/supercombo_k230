#!/usr/bin/env python3
import argparse
import tempfile
from collections import Counter
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
from onnx import TensorProto, helper, numpy_helper, shape_inference


BIG_FEATURE = "input.476"
MAIN_FEATURE = "input.276"
PATTERNS = {
  "Gemm_227": "Concat_226",
  "Gemm_274": "Concat_273",
}


def attrs(node: onnx.NodeProto) -> dict:
  return {attr.name: helper.get_attribute_value(attr) for attr in node.attribute}


def input_shape(value: onnx.ValueInfoProto) -> tuple[int, ...]:
  return tuple(int(dim.dim_value) for dim in value.type.tensor_type.shape.dim)


def zero_feed(model: onnx.ModelProto) -> dict[str, np.ndarray]:
  initializer_names = {init.name for init in model.graph.initializer}
  feed = {}
  for value in model.graph.input:
    if value.name in initializer_names:
      continue
    dtype = np.float32
    elem_type = value.type.tensor_type.elem_type
    if elem_type == TensorProto.UINT8:
      dtype = np.uint8
    elif elem_type == TensorProto.INT8:
      dtype = np.int8
    feed[value.name] = np.zeros(input_shape(value), dtype=dtype)
  return feed


def run_big_zero_feature(model_path: Path) -> np.ndarray:
  model = onnx.load(model_path)
  if not any(value.name == BIG_FEATURE for value in model.graph.value_info):
    model = shape_inference.infer_shapes(model)
  if not any(value.name == BIG_FEATURE for value in model.graph.output):
    model.graph.output.append(helper.make_tensor_value_info(BIG_FEATURE, TensorProto.FLOAT, [1, 1024]))

  with tempfile.NamedTemporaryFile(suffix=".onnx") as f:
    onnx.save(model, f.name)
    session = ort.InferenceSession(f.name, providers=["CPUExecutionProvider"])
    outputs = session.run([BIG_FEATURE], zero_feed(model))
  return outputs[0].reshape(1, 1024).astype(np.float32)


def rewrite_model(model: onnx.ModelProto, big_feature: np.ndarray, fold_zero_big: bool,
                  keep_big_input: bool) -> onnx.ModelProto:
  initializers = {init.name: init for init in model.graph.initializer}
  new_model = onnx.ModelProto()
  new_model.CopyFrom(model)
  graph = new_model.graph

  new_initializers = []
  new_nodes = []
  for node in graph.node:
    if node.name in PATTERNS.values():
      continue
    if node.name in PATTERNS:
      if node.op_type != "Gemm":
        raise RuntimeError(f"{node.name} is not Gemm")
      node_attrs = attrs(node)
      if int(node_attrs.get("transB", 0)) != 1:
        raise RuntimeError(f"{node.name} is expected to use transB=1")
      weight = numpy_helper.to_array(initializers[node.input[1]]).astype(np.float32)
      bias = numpy_helper.to_array(initializers[node.input[2]]).astype(np.float32)
      if weight.shape != (512, 2048) or bias.shape != (512,):
        raise RuntimeError(f"unexpected {node.name} weight/bias shape: {weight.shape}, {bias.shape}")

      main_weight = weight[:, :1024].copy()
      big_weight = weight[:, 1024:].copy()
      new_bias = bias.copy()
      if fold_zero_big:
        new_bias += (big_feature @ big_weight.T).reshape(-1)

      w_name = f"{node.input[1]}__no_big_main"
      b_name = f"{node.input[2]}__no_big_bias"
      new_initializers.append(numpy_helper.from_array(main_weight.astype(np.float32), w_name))
      new_initializers.append(numpy_helper.from_array(new_bias.astype(np.float32), b_name))
      new_nodes.append(helper.make_node(
        "Gemm",
        [MAIN_FEATURE, w_name, b_name],
        list(node.output),
        name=f"{node.name}_no_big",
        alpha=1.0,
        beta=1.0,
        transB=1,
      ))
      continue

    new_nodes.append(node)

  del graph.node[:]
  graph.node.extend(new_nodes)
  graph.initializer.extend(new_initializers)
  keep_inputs = {"big_input_imgs"} if keep_big_input else None
  prune_unreachable(new_model, keep_inputs=keep_inputs)
  return new_model


def prune_unreachable(model: onnx.ModelProto, keep_inputs: set[str] | None = None) -> None:
  if keep_inputs is None:
    keep_inputs = set()
  graph = model.graph
  producers = {out: node for node in graph.node for out in node.output}
  required_values = {out.name for out in graph.output}
  required_nodes = set()

  changed = True
  while changed:
    changed = False
    for value in list(required_values):
      node = producers.get(value)
      if node is None or node.name in required_nodes:
        continue
      required_nodes.add(node.name)
      for inp in node.input:
        if inp and inp not in required_values:
          required_values.add(inp)
          changed = True

  kept_nodes = [node for node in graph.node if node.name in required_nodes]
  del graph.node[:]
  graph.node.extend(kept_nodes)

  initializer_names = {init.name for init in graph.initializer}
  used_initializers = {name for name in required_values if name in initializer_names}
  kept_initializers = [init for init in graph.initializer if init.name in used_initializers]
  del graph.initializer[:]
  graph.initializer.extend(kept_initializers)

  kept_inputs = [
    value for value in graph.input
    if (value.name in required_values or value.name in keep_inputs) and value.name not in used_initializers
  ]
  del graph.input[:]
  graph.input.extend(kept_inputs)

  produced = {out for node in kept_nodes for out in node.output}
  available = required_values | produced | {value.name for value in graph.output}
  kept_value_info = [value for value in graph.value_info if value.name in available]
  del graph.value_info[:]
  graph.value_info.extend(kept_value_info)


def elem_count(shape: list[int] | tuple[int, ...] | None) -> int:
  if not shape or any(dim <= 0 for dim in shape):
    return 0
  out = 1
  for dim in shape:
    out *= int(dim)
  return out


def shapes_for(model: onnx.ModelProto) -> dict[str, list[int]]:
  inferred = shape_inference.infer_shapes(model)
  shapes = {}
  for value in list(inferred.graph.input) + list(inferred.graph.value_info) + list(inferred.graph.output):
    tensor_type = value.type.tensor_type
    dims = [int(dim.dim_value) for dim in tensor_type.shape.dim]
    if dims and all(dim > 0 for dim in dims):
      shapes[value.name] = dims
  for init in inferred.graph.initializer:
    shapes[init.name] = list(init.dims)
  return shapes


def model_stats(model: onnx.ModelProto) -> dict[str, float | int | dict[str, int]]:
  shapes = shapes_for(model)
  initializers = {init.name: init for init in model.graph.initializer}
  macs = 0
  elem_ops = 0
  conv_macs = 0
  gemm_macs = 0

  for node in model.graph.node:
    if node.op_type == "Conv":
      x_shape = shapes.get(node.input[0])
      w_shape = shapes.get(node.input[1])
      y_shape = shapes.get(node.output[0])
      if x_shape and w_shape and y_shape and len(x_shape) == 4 and len(w_shape) == 4 and len(y_shape) == 4:
        groups = int(attrs(node).get("group", 1))
        node_macs = int(y_shape[0] * y_shape[1] * y_shape[2] * y_shape[3] * (w_shape[1]) * w_shape[2] * w_shape[3])
        # ONNX Conv weight shape stores C/group, so no extra division by group is needed.
        conv_macs += node_macs
        macs += node_macs
    elif node.op_type == "Gemm":
      x_shape = shapes.get(node.input[0])
      w_shape = shapes.get(node.input[1])
      if x_shape and w_shape and len(x_shape) == 2 and len(w_shape) == 2:
        trans_b = int(attrs(node).get("transB", 0))
        out_features = w_shape[0] if trans_b else w_shape[1]
        k = w_shape[1] if trans_b else w_shape[0]
        node_macs = int(x_shape[0] * out_features * k)
        gemm_macs += node_macs
        macs += node_macs
    elif node.op_type in {"Elu", "Relu", "Add", "Sub", "Mul", "Sigmoid", "Tanh"}:
      elem_ops += elem_count(shapes.get(node.output[0]))

  return {
    "nodes": len(model.graph.node),
    "inputs": len(model.graph.input),
    "initializers": len(model.graph.initializer),
    "op_counts": dict(Counter(node.op_type for node in model.graph.node)),
    "conv_macs": conv_macs,
    "gemm_macs": gemm_macs,
    "macs": macs,
    "approx_flops": 2 * macs + elem_ops,
    "elem_ops": elem_ops,
  }


def print_stats(label: str, stats: dict[str, float | int | dict[str, int]]) -> None:
  print(f"\n{label}")
  print(f"nodes={stats['nodes']} inputs={stats['inputs']} initializers={stats['initializers']}")
  print(f"op_counts={stats['op_counts']}")
  print(f"conv_macs={stats['conv_macs']:,}")
  print(f"gemm_macs={stats['gemm_macs']:,}")
  print(f"total_macs={stats['macs']:,}")
  print(f"approx_flops={stats['approx_flops']:,}")


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--input", required=True, type=Path)
  parser.add_argument("--output", required=True, type=Path)
  parser.add_argument("--mode", choices=["fold-zero", "drop"], default="fold-zero")
  parser.add_argument("--keep-big-input", action="store_true",
                      help="preserve big_input_imgs as an unused graph input for 5-input runtime compatibility")
  args = parser.parse_args()

  original = onnx.load(args.input)
  big_feature = run_big_zero_feature(args.input)
  rewritten = rewrite_model(
    original,
    big_feature,
    fold_zero_big=args.mode == "fold-zero",
    keep_big_input=args.keep_big_input,
  )
  onnx.checker.check_model(rewritten)

  args.output.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(rewritten, args.output)
  print(args.output)
  print(f"zero_big_feature mean={float(big_feature.mean()):.9f} std={float(big_feature.std()):.9f}")

  original_stats = model_stats(original)
  rewritten_stats = model_stats(rewritten)
  print_stats("original", original_stats)
  print_stats(f"no_big_{args.mode}", rewritten_stats)
  saved_macs = int(original_stats["macs"]) - int(rewritten_stats["macs"])
  saved_flops = int(original_stats["approx_flops"]) - int(rewritten_stats["approx_flops"])
  print(f"\nreduction_macs={saved_macs:,} ({saved_macs / int(original_stats['macs']) * 100.0:.2f}%)")
  print(f"reduction_approx_flops={saved_flops:,} ({saved_flops / int(original_stats['approx_flops']) * 100.0:.2f}%)")


if __name__ == "__main__":
  main()
