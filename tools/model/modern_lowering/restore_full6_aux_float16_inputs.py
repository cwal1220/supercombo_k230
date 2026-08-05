#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import onnx
from onnx import TensorProto, helper


AUX_INPUTS = [
  "features_buffer",
  "desire_pulse",
  "traffic_convention",
  "action_t",
]


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def replace_node_inputs(model: onnx.ModelProto, replacements: dict[str, str]) -> None:
  for node in model.graph.node:
    for i, name in enumerate(node.input):
      if name in replacements:
        node.input[i] = replacements[name]


def restore_aux_f16_inputs(model: onnx.ModelProto) -> int:
  graph = model.graph
  graph_inputs = {value.name: value for value in graph.input}
  replacements = {name: f"{name}_k230_internal_f32" for name in AUX_INPUTS}

  for name in AUX_INPUTS:
    if name not in graph_inputs:
      raise RuntimeError(f"missing graph input: {name}")
    input_type = graph_inputs[name].type.tensor_type
    if input_type.elem_type != TensorProto.FLOAT:
      raise RuntimeError(f"expected {name} to be FLOAT before restore, got {input_type.elem_type}")
    input_type.elem_type = TensorProto.FLOAT16

  replace_node_inputs(model, replacements)

  cast_nodes = [
    helper.make_node(
      "Cast",
      [name],
      [replacements[name]],
      name=f"k230_restore_{name}_f16_to_f32",
      to=TensorProto.FLOAT,
    )
    for name in AUX_INPUTS
  ]
  original_nodes = list(graph.node)
  del graph.node[:]
  graph.node.extend(cast_nodes + original_nodes)
  return len(cast_nodes)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--in-model", required=True, type=Path)
  parser.add_argument("--out-model", required=True, type=Path)
  parser.add_argument("--check", action="store_true")
  args = parser.parse_args()

  model = onnx.load(args.in_model)
  added = restore_aux_f16_inputs(model)
  if args.check:
    onnx.checker.check_model(model)
    model = onnx.shape_inference.infer_shapes(model)

  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(model, args.out_model)
  print(f"wrote {args.out_model}")
  print(f"added_casts={added}")
  print(f"size={args.out_model.stat().st_size}")
  print(f"sha256={sha256_file(args.out_model)}")


if __name__ == "__main__":
  main()
