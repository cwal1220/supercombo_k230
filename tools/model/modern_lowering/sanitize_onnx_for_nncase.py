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


def convert_type_proto(type_proto) -> int:
  changed = 0
  if type_proto.HasField("tensor_type") and type_proto.tensor_type.elem_type == TensorProto.FLOAT16:
    type_proto.tensor_type.elem_type = TensorProto.FLOAT
    changed += 1
  return changed


def convert_tensor_proto(tensor) -> int:
  if tensor.data_type != TensorProto.FLOAT16:
    return 0
  arr = numpy_helper.to_array(tensor).astype(np.float32)
  new_tensor = numpy_helper.from_array(arr, tensor.name)
  tensor.CopyFrom(new_tensor)
  return 1


def convert_attribute(attr) -> int:
  changed = 0
  if attr.type == onnx.AttributeProto.TENSOR:
    changed += convert_tensor_proto(attr.t)
  elif attr.type == onnx.AttributeProto.TENSORS:
    for tensor in attr.tensors:
      changed += convert_tensor_proto(tensor)
  elif attr.type == onnx.AttributeProto.GRAPH:
    changed += convert_graph(attr.g)
  elif attr.type == onnx.AttributeProto.GRAPHS:
    for graph in attr.graphs:
      changed += convert_graph(graph)
  return changed


def convert_graph(graph) -> int:
  changed = 0
  for value in list(graph.input) + list(graph.output) + list(graph.value_info):
    changed += convert_type_proto(value.type)
  for init in graph.initializer:
    changed += convert_tensor_proto(init)
  for node in graph.node:
    for attr in node.attribute:
      changed += convert_attribute(attr)
  return changed


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--in-model", required=True, type=Path)
  parser.add_argument("--out-model", required=True, type=Path)
  parser.add_argument("--float32", action="store_true")
  parser.add_argument("--dedupe-opsets", action="store_true")
  parser.add_argument("--name-empty-nodes", action="store_true")
  parser.add_argument("--remove-reshape-allowzero", action="store_true")
  parser.add_argument("--check", action="store_true")
  args = parser.parse_args()

  model = onnx.load(args.in_model)
  stats = {
    "converted_f16_items": 0,
    "renamed_nodes": 0,
    "removed_allowzero": 0,
    "opsets_before": [(op.domain, op.version) for op in model.opset_import],
  }

  if args.float32:
    stats["converted_f16_items"] += convert_graph(model.graph)
    for node in model.graph.node:
      if node.op_type == "Cast":
        for attr in node.attribute:
          if attr.name == "to" and attr.i == TensorProto.FLOAT16:
            attr.i = TensorProto.FLOAT
            stats["converted_f16_items"] += 1

  if args.name_empty_nodes:
    used = {node.name for node in model.graph.node if node.name}
    for i, node in enumerate(model.graph.node):
      if not node.name:
        base = f"{node.op_type}_{i}"
        name = base
        suffix = 0
        while name in used:
          suffix += 1
          name = f"{base}_{suffix}"
        node.name = name
        used.add(name)
        stats["renamed_nodes"] += 1

  if args.remove_reshape_allowzero:
    for node in model.graph.node:
      if node.op_type != "Reshape":
        continue
      keep = []
      for attr in node.attribute:
        if attr.name == "allowzero":
          stats["removed_allowzero"] += 1
        else:
          keep.append(attr)
      del node.attribute[:]
      node.attribute.extend(keep)

  if args.dedupe_opsets:
    best: dict[str, int] = {}
    for opset in model.opset_import:
      best[opset.domain] = max(best.get(opset.domain, 0), opset.version)
    del model.opset_import[:]
    for domain, version in sorted(best.items()):
      opset = model.opset_import.add()
      opset.domain = domain
      opset.version = version

  stats["opsets_after"] = [(op.domain, op.version) for op in model.opset_import]

  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  if args.check:
    onnx.checker.check_model(model)
  onnx.save(model, args.out_model)

  print(f"wrote {args.out_model}")
  print(f"size={args.out_model.stat().st_size}")
  print(f"sha256={sha256_file(args.out_model)}")
  for key, value in stats.items():
    print(f"{key}={value}")


if __name__ == "__main__":
  main()
