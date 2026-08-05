#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper


ATTENTION_BLOCK_NODE_NAMES = {
  "p_node_view_1",
  "p_node_permute",
  "p_node_unbind__0",
  "p_node_unbind__1",
  "p_node_unbind__2",
  "p_node_transpose",
  "p_node_matmul",
  "p_node_mul",
  "p_node_masked_fill",
  "p_node_softmax",
  "p_node_matmul_1",
  "p_node_transpose_1",
  "p_node_view_2",
}

ATTENTION_SLICE_NODE_NAME_VARIANTS = (
  {
    "p_node_Slice_38",
    "p_node_Slice_42",
    "p_node_Slice_46",
  },
  {
    "p_node_Slice_38_gather",
    "p_node_Slice_42_gather",
    "p_node_Slice_46_gather",
  },
)


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def add_initializer(graph: onnx.GraphProto, name: str, arr: np.ndarray) -> None:
  graph.initializer.append(numpy_helper.from_array(np.ascontiguousarray(arr), name))


def add_i64(graph: onnx.GraphProto, name: str, values: list[int] | np.ndarray) -> str:
  add_initializer(graph, name, np.asarray(values, dtype=np.int64))
  return name


def lower_gelu_sigmoid(model: onnx.ModelProto) -> int:
  lowered = 0
  new_nodes: list[onnx.NodeProto] = []
  for node in model.graph.node:
    if node.op_type != "Gelu":
      new_nodes.append(node)
      continue
    base = node.name or f"k230_full_gelu_{lowered}"
    x = node.input[0]
    y = node.output[0]
    scale_name = f"{base}_sigmoid_scale"
    scaled = f"{base}_scaled"
    sig = f"{base}_sigmoid"
    add_initializer(model.graph, scale_name, np.array([1.702], dtype=np.float32))
    new_nodes.extend([
      helper.make_node("Mul", [x, scale_name], [scaled], name=f"{base}_lower_mul_sigmoid_scale"),
      helper.make_node("Sigmoid", [scaled], [sig], name=f"{base}_lower_sigmoid"),
      helper.make_node("Mul", [x, sig], [y], name=f"{base}_lower_mul_out"),
    ])
    lowered += 1
  del model.graph.node[:]
  model.graph.node.extend(new_nodes)
  return lowered


def lower_layer_norm(model: onnx.ModelProto) -> int:
  lowered = 0
  new_nodes: list[onnx.NodeProto] = []
  for node in model.graph.node:
    if node.op_type != "LayerNormalization":
      new_nodes.append(node)
      continue
    base = node.name or f"k230_full_layer_norm_{lowered}"
    x, scale, bias = node.input[:3]
    y = node.output[0]
    epsilon = 1.0e-5
    for attr in node.attribute:
      if attr.name == "epsilon":
        epsilon = float(helper.get_attribute_value(attr))

    shape_2d = f"{base}_shape_2d"
    shape_3d = f"{base}_shape_3d"
    mean_weight = f"{base}_mean_weight"
    eps_name = f"{base}_epsilon"
    x2 = f"{base}_x2"
    mean = f"{base}_mean"
    centered = f"{base}_centered"
    square = f"{base}_square"
    variance = f"{base}_variance"
    variance_eps = f"{base}_variance_eps"
    std = f"{base}_std"
    norm = f"{base}_norm"
    scaled = f"{base}_scaled"
    y2 = f"{base}_y2"

    add_i64(model.graph, shape_2d, [9, 512])
    add_i64(model.graph, shape_3d, [1, 9, 512])
    add_initializer(model.graph, mean_weight, np.full((512, 1), 1.0 / 512.0, dtype=np.float32))
    add_initializer(model.graph, eps_name, np.array([epsilon], dtype=np.float32))

    new_nodes.extend([
      helper.make_node("Reshape", [x, shape_2d], [x2], name=f"{base}_Reshape2D"),
      helper.make_node("MatMul", [x2, mean_weight], [mean], name=f"{base}_MeanMatMul"),
      helper.make_node("Sub", [x2, mean], [centered], name=f"{base}_Center"),
      helper.make_node("Mul", [centered, centered], [square], name=f"{base}_Square"),
      helper.make_node("MatMul", [square, mean_weight], [variance], name=f"{base}_VarMatMul"),
      helper.make_node("Add", [variance, eps_name], [variance_eps], name=f"{base}_AddEps"),
      helper.make_node("Sqrt", [variance_eps], [std], name=f"{base}_Sqrt"),
      helper.make_node("Div", [centered, std], [norm], name=f"{base}_Div"),
      helper.make_node("Mul", [norm, scale], [scaled], name=f"{base}_Scale"),
      helper.make_node("Add", [scaled, bias], [y2], name=f"{base}_Bias"),
      helper.make_node("Reshape", [y2, shape_3d], [y], name=f"{base}_Reshape3D"),
    ])
    lowered += 1
  del model.graph.node[:]
  model.graph.node.extend(new_nodes)
  return lowered


def rewrite_attention(model: onnx.ModelProto) -> int:
  graph = model.graph
  node_by_name = {node.name: node for node in graph.node}
  attention_node_names = set(ATTENTION_BLOCK_NODE_NAMES)
  for slice_names in ATTENTION_SLICE_NODE_NAME_VARIANTS:
    if slice_names.issubset(node_by_name):
      attention_node_names.update(slice_names)
      break
  else:
    expected = sorted(set().union(*ATTENTION_SLICE_NODE_NAME_VARIANTS))
    raise RuntimeError(f"attention Slice/Gather nodes not found; expected one variant of: {expected}")

  if not attention_node_names.issubset(node_by_name):
    missing = sorted(attention_node_names - set(node_by_name))
    raise RuntimeError(f"attention block node names not found: {missing}")

  new_nodes: list[onnx.NodeProto] = []
  inserted = False
  for node in graph.node:
    if node.name == "p_node_view_1":
      base = "k230_full_attn"
      context_parts: list[str] = []
      scale = "p_scalar_tensor_default"
      mask = "p_bitwise_not"
      fill_value = "p_val_51"

      for head in range(8):
        q_idx = add_i64(graph, f"{base}_h{head}_q_idx", np.arange(head * 64, (head + 1) * 64))
        k_idx = add_i64(graph, f"{base}_h{head}_k_idx", np.arange(512 + head * 64, 512 + (head + 1) * 64))
        v_idx = add_i64(graph, f"{base}_h{head}_v_idx", np.arange(1024 + head * 64, 1024 + (head + 1) * 64))
        shape_2d = add_i64(graph, f"{base}_h{head}_shape_2d", [9, 64])
        shape_score = add_i64(graph, f"{base}_h{head}_shape_score", [1, 1, 9, 9])
        shape_v4 = add_i64(graph, f"{base}_h{head}_shape_v4", [1, 1, 9, 64])
        shape_ctx3 = add_i64(graph, f"{base}_h{head}_shape_ctx3", [1, 9, 64])

        q = f"{base}_h{head}_q"
        k = f"{base}_h{head}_k"
        v = f"{base}_h{head}_v"
        q2 = f"{base}_h{head}_q2"
        k2 = f"{base}_h{head}_k2"
        v4 = f"{base}_h{head}_v4"
        score2 = f"{base}_h{head}_score2"
        score4 = f"{base}_h{head}_score4"
        scaled = f"{base}_h{head}_scaled"
        masked = f"{base}_h{head}_masked"
        softmax = f"{base}_h{head}_softmax"
        ctx4 = f"{base}_h{head}_ctx4"
        ctx3 = f"{base}_h{head}_ctx3"
        context_parts.append(ctx3)

        new_nodes.extend([
          helper.make_node("Gather", ["p_linear_6", q_idx], [q], name=f"{base}_h{head}_GatherQ", axis=2),
          helper.make_node("Gather", ["p_linear_6", k_idx], [k], name=f"{base}_h{head}_GatherK", axis=2),
          helper.make_node("Gather", ["p_linear_6", v_idx], [v], name=f"{base}_h{head}_GatherV", axis=2),
          helper.make_node("Reshape", [q, shape_2d], [q2], name=f"{base}_h{head}_ReshapeQ2"),
          helper.make_node("Reshape", [k, shape_2d], [k2], name=f"{base}_h{head}_ReshapeK2"),
          helper.make_node("Reshape", [v, shape_v4], [v4], name=f"{base}_h{head}_ReshapeV4"),
          helper.make_node("Gemm", [q2, k2], [score2], name=f"{base}_h{head}_ScoreGemm", transB=1),
          helper.make_node("Reshape", [score2, shape_score], [score4], name=f"{base}_h{head}_ReshapeScore4"),
          helper.make_node("Mul", [score4, scale], [scaled], name=f"{base}_h{head}_Scale"),
          helper.make_node("Where", [mask, fill_value, scaled], [masked], name=f"{base}_h{head}_MaskedFill"),
          helper.make_node("Softmax", [masked], [softmax], name=f"{base}_h{head}_Softmax", axis=-1),
          helper.make_node("MatMul", [softmax, v4], [ctx4], name=f"{base}_h{head}_ContextMatMul"),
          helper.make_node("Reshape", [ctx4, shape_ctx3], [ctx3], name=f"{base}_h{head}_ReshapeContext3"),
        ])

      new_nodes.append(helper.make_node("Concat", context_parts, ["p_view_2"], name=f"{base}_ConcatHeads", axis=2))
      inserted = True

    if node.name in attention_node_names:
      continue
    new_nodes.append(node)

  if not inserted:
    raise RuntimeError("failed to insert rewritten attention block")
  del graph.node[:]
  graph.node.extend(new_nodes)
  return len(attention_node_names)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--in-model", required=True, type=Path)
  parser.add_argument("--out-model", required=True, type=Path)
  parser.add_argument("--lower-gelu", action="store_true")
  parser.add_argument("--lower-layernorm", action="store_true")
  parser.add_argument("--check", action="store_true")
  args = parser.parse_args()

  model = onnx.load(args.in_model)
  removed = rewrite_attention(model)
  lowered = lower_gelu_sigmoid(model) if args.lower_gelu else 0
  lowered_ln = lower_layer_norm(model) if args.lower_layernorm else 0
  if args.check:
    onnx.checker.check_model(model)
    model = onnx.shape_inference.infer_shapes(model)

  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(model, args.out_model)
  print(f"wrote {args.out_model}")
  print(f"removed_attention_nodes={removed}")
  print(f"lowered_gelu={lowered}")
  print(f"lowered_layernorm={lowered_ln}")
  print(f"size={args.out_model.stat().st_size}")
  print(f"sha256={sha256_file(args.out_model)}")


if __name__ == "__main__":
  main()
