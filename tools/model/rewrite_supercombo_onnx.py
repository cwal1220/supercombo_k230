#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper, shape_inference


TRAJECTORY_SIZE = 33
PLAN_MHP_N = 5
PLAN_STRIDE = TRAJECTORY_SIZE * 15 * 2 + 1


def attrs(node: onnx.NodeProto) -> dict:
  out = {}
  for attr in node.attribute:
    out[attr.name] = helper.get_attribute_value(attr)
  return out


def inferred_float_shapes(model: onnx.ModelProto) -> dict[str, list[int]]:
  inferred = shape_inference.infer_shapes(model)
  shapes = {}
  for value in list(inferred.graph.input) + list(inferred.graph.value_info) + list(inferred.graph.output):
    tensor_type = value.type.tensor_type
    if tensor_type.elem_type != TensorProto.FLOAT:
      continue
    dims = []
    for dim in tensor_type.shape.dim:
      if not dim.dim_value:
        break
      dims.append(dim.dim_value)
    if dims:
      shapes[value.name] = dims
  return shapes


def rewrite_gemm_split(model: onnx.ModelProto) -> int:
  initializers = {t.name: t for t in model.graph.initializer}
  producers = {out: node for node in model.graph.node for out in node.output}
  consumers = {}
  for node in model.graph.node:
    for inp in node.input:
      consumers.setdefault(inp, []).append(node)

  remove = set()
  insert_after = {}
  replacements = {}
  new_initializers = []
  rewritten = 0

  for split in list(model.graph.node):
    if split.op_type != "Split":
      continue

    split_attrs = attrs(split)
    axis = int(split_attrs.get("axis", 0))
    sizes = list(split_attrs.get("split", []))
    if axis != 1 or not sizes:
      continue

    gemm = producers.get(split.input[0])
    if gemm is None or gemm.op_type != "Gemm":
      continue

    gemm_attrs = attrs(gemm)
    trans_b = int(gemm_attrs.get("transB", 0))
    alpha = float(gemm_attrs.get("alpha", 1.0))
    beta = float(gemm_attrs.get("beta", 1.0))
    if trans_b != 1 or alpha != 1.0 or beta != 1.0:
      continue

    if len(gemm.input) < 3:
      continue
    weight = initializers.get(gemm.input[1])
    bias = initializers.get(gemm.input[2])
    if weight is None or bias is None:
      continue

    weight_np = numpy_helper.to_array(weight)
    bias_np = numpy_helper.to_array(bias)
    if weight_np.shape[0] != sum(sizes) or bias_np.shape[0] != sum(sizes):
      continue

    new_nodes = []
    start = 0
    for idx, (size, split_out) in enumerate(zip(sizes, split.output)):
      end = start + size
      w_name = f"{gemm.input[1]}__split_{idx}"
      b_name = f"{gemm.input[2]}__split_{idx}"
      y_name = split_out
      new_initializers.append(numpy_helper.from_array(weight_np[start:end].astype(weight_np.dtype), w_name))
      new_initializers.append(numpy_helper.from_array(bias_np[start:end].astype(bias_np.dtype), b_name))
      new_nodes.append(helper.make_node(
        "Gemm",
        [gemm.input[0], w_name, b_name],
        [y_name],
        name=f"{gemm.name}_split_{idx}",
        alpha=1.0,
        beta=1.0,
        transB=1,
      ))
      replacements[split_out] = y_name
      start = end

    insert_after[gemm.name] = new_nodes
    remove.add(gemm.name)
    remove.add(split.name)
    rewritten += 1

  if rewritten == 0:
    return 0

  nodes = []
  for node in model.graph.node:
    if node.name in remove:
      if node.name in insert_after:
        nodes.extend(insert_after[node.name])
      continue
    for idx, inp in enumerate(node.input):
      if inp in replacements:
        node.input[idx] = replacements[inp]
    nodes.append(node)

  del model.graph.node[:]
  model.graph.node.extend(nodes)
  model.graph.initializer.extend(new_initializers)
  return rewritten


def decompose_elu(model: onnx.ModelProto, allowed_names: set[str] | None = None) -> int:
  one_name = "k230_rewrite_elu_one"
  existing_initializers = {t.name for t in model.graph.initializer}
  if one_name not in existing_initializers:
    model.graph.initializer.append(numpy_helper.from_array(np.array(1.0, dtype=np.float32), one_name))

  nodes = []
  rewritten = 0
  for node in model.graph.node:
    if node.op_type != "Elu":
      nodes.append(node)
      continue
    if allowed_names is not None and node.name not in allowed_names:
      nodes.append(node)
      continue

    node_attrs = attrs(node)
    alpha = float(node_attrs.get("alpha", 1.0))
    if alpha != 1.0:
      nodes.append(node)
      continue

    x = node.input[0]
    y = node.output[0]
    base = node.name or f"Elu_decomposed_{rewritten}"
    neg_x = f"{base}__neg_x"
    relu_neg_x = f"{base}__relu_neg_x"
    neg_relu_neg_x = f"{base}__neg_relu_neg_x"
    exp_neg_relu_neg_x = f"{base}__exp_neg_relu_neg_x"
    exp_minus_one = f"{base}__exp_minus_one"
    relu_x = f"{base}__relu_x"

    nodes.extend([
      helper.make_node("Neg", [x], [neg_x], name=f"{base}__NegX"),
      helper.make_node("Relu", [neg_x], [relu_neg_x], name=f"{base}__ReluNegX"),
      helper.make_node("Neg", [relu_neg_x], [neg_relu_neg_x], name=f"{base}__NegReluNegX"),
      helper.make_node("Exp", [neg_relu_neg_x], [exp_neg_relu_neg_x], name=f"{base}__Exp"),
      helper.make_node("Sub", [exp_neg_relu_neg_x, one_name], [exp_minus_one], name=f"{base}__SubOne"),
      helper.make_node("Relu", [x], [relu_x], name=f"{base}__ReluX"),
      helper.make_node("Add", [relu_x, exp_minus_one], [y], name=f"{base}__Add"),
    ])
    rewritten += 1

  del model.graph.node[:]
  model.graph.node.extend(nodes)
  return rewritten


def insert_identity_depthwise_before_elu(model: onnx.ModelProto, allowed_names: set[str] | None = None) -> int:
  shapes = inferred_float_shapes(model)
  existing_initializers = {t.name for t in model.graph.initializer}
  new_initializers = []
  nodes = []
  rewritten = 0

  def unique_name(base: str) -> str:
    if base not in existing_initializers:
      existing_initializers.add(base)
      return base
    idx = 1
    while f"{base}_{idx}" in existing_initializers:
      idx += 1
    name = f"{base}_{idx}"
    existing_initializers.add(name)
    return name

  for node in model.graph.node:
    if node.op_type != "Elu":
      nodes.append(node)
      continue
    if allowed_names is not None and node.name not in allowed_names:
      nodes.append(node)
      continue

    input_shape = shapes.get(node.input[0])
    if input_shape is None or len(input_shape) != 4 or input_shape[1] <= 0:
      nodes.append(node)
      continue

    channels = int(input_shape[1])
    base = node.name or f"EluIdentityDW_{rewritten}"
    weight_name = unique_name(f"{base}__idw")
    bias_name = unique_name(f"{base}__idb")
    conv_out = f"{base}__identity_out"

    new_initializers.append(numpy_helper.from_array(np.ones((channels, 1, 1, 1), dtype=np.float32), weight_name))
    new_initializers.append(numpy_helper.from_array(np.zeros((channels,), dtype=np.float32), bias_name))
    nodes.append(helper.make_node(
      "Conv",
      [node.input[0], weight_name, bias_name],
      [conv_out],
      name=f"{base}__IdentityDW",
      group=channels,
      kernel_shape=[1, 1],
      pads=[0, 0, 0, 0],
      strides=[1, 1],
      dilations=[1, 1],
    ))
    new_node = onnx.NodeProto()
    new_node.CopyFrom(node)
    new_node.input[0] = conv_out
    nodes.append(new_node)
    rewritten += 1

  del model.graph.node[:]
  model.graph.node.extend(nodes)
  model.graph.initializer.extend(new_initializers)
  return rewritten


def rewrite_summarizer_infeats_as_conv(model: onnx.ModelProto) -> int:
  initializers = {t.name: t for t in model.graph.initializer}

  # These two flatten outputs come from the twin vision towers and both have shape [1, 32, 4, 8].
  # Flatten+Concat+Gemm is mathematically equivalent to channel-concat + Conv(4x8) + Flatten.
  patterns = [
    {
      "concat": "Concat_226",
      "gemm": "Gemm_227",
      "out": "input.480",
      "prefix": "policy_summarizer",
    },
    {
      "concat": "Concat_273",
      "gemm": "Gemm_274",
      "out": "input.552",
      "prefix": "temporal_summarizer",
    },
  ]

  graph_nodes = {node.name: node for node in model.graph.node}
  if "Flatten_132" not in graph_nodes or "Flatten_225" not in graph_nodes:
    return 0

  new_initializers = []
  replacements: dict[str, list[onnx.NodeProto]] = {}
  remove_names = {"Flatten_132", "Flatten_225"}

  for pattern in patterns:
    concat = graph_nodes.get(pattern["concat"])
    gemm = graph_nodes.get(pattern["gemm"])
    if concat is None or gemm is None:
      return 0
    if concat.op_type != "Concat" or gemm.op_type != "Gemm":
      return 0
    if list(concat.input) != ["input.276", "input.476"]:
      return 0
    if gemm.input[0] != concat.output[0]:
      return 0

    weight = initializers.get(gemm.input[1])
    bias = initializers.get(gemm.input[2])
    if weight is None or bias is None:
      return 0

    weight_np = numpy_helper.to_array(weight)
    bias_np = numpy_helper.to_array(bias)
    if weight_np.shape != (512, 2048) or bias_np.shape != (512,):
      return 0

    left = weight_np[:, :1024].reshape(512, 32, 4, 8)
    right = weight_np[:, 1024:].reshape(512, 32, 4, 8)
    conv_weight = np.concatenate([left, right], axis=1).astype(np.float32)
    conv_bias = bias_np.astype(np.float32)

    w_name = f"k230_{pattern['prefix']}__conv_w"
    b_name = f"k230_{pattern['prefix']}__conv_b"
    conv_out = f"k230_{pattern['prefix']}__conv_out"
    new_initializers.append(numpy_helper.from_array(conv_weight, w_name))
    new_initializers.append(numpy_helper.from_array(conv_bias, b_name))

    replacements[pattern["concat"]] = [
      helper.make_node(
        "Conv",
        ["k230_concat_vision_4d", w_name, b_name],
        [conv_out],
        name=f"k230_{pattern['prefix']}__Conv4x8",
        kernel_shape=[4, 8],
        pads=[0, 0, 0, 0],
        strides=[1, 1],
        dilations=[1, 1],
      ),
      helper.make_node(
        "Flatten",
        [conv_out],
        [pattern["out"]],
        name=f"k230_{pattern['prefix']}__Flatten",
        axis=1,
      ),
    ]
    remove_names.add(pattern["concat"])
    remove_names.add(pattern["gemm"])

  nodes = []
  inserted_shared_concat = False
  for node in model.graph.node:
    if node.name == "Flatten_225" and not inserted_shared_concat:
      nodes.append(helper.make_node(
        "Concat",
        ["onnx::Flatten_588", "onnx::Flatten_681"],
        ["k230_concat_vision_4d"],
        name="k230_concat_vision_4d",
        axis=1,
      ))
      inserted_shared_concat = True

    if node.name in remove_names:
      if node.name in replacements:
        nodes.extend(replacements[node.name])
      continue

    nodes.append(node)

  del model.graph.node[:]
  model.graph.node.extend(nodes)
  model.graph.initializer.extend(new_initializers)
  return len(patterns)


def rewrite_gru_update(model: onnx.ModelProto) -> int:
  one_name = "k230_rewrite_gru_one"
  existing_initializers = {t.name for t in model.graph.initializer}
  if one_name not in existing_initializers:
    model.graph.initializer.append(numpy_helper.from_array(np.array(1.0, dtype=np.float32), one_name))

  producers = {out: node for node in model.graph.node for out in node.output}
  consumers = {}
  for node in model.graph.node:
    for inp in node.input:
      consumers.setdefault(inp, []).append(node)

  remove = set()
  replace = {}
  rewritten = 0

  for add in list(model.graph.node):
    if add.op_type != "Add" or len(add.input) != 2:
      continue

    tanh_value = None
    mul = None
    for inp in add.input:
      producer = producers.get(inp)
      if producer is not None and producer.op_type == "Mul":
        mul = producer
      else:
        tanh_value = inp
    if mul is None or tanh_value is None:
      continue

    sub = None
    z_value = None
    for inp in mul.input:
      producer = producers.get(inp)
      if producer is not None and producer.op_type == "Sub":
        sub = producer
      else:
        z_value = inp
    if sub is None or z_value is None:
      continue
    if list(sub.input) != ["initial_state", tanh_value]:
      continue
    if tanh_value not in consumers or len(consumers[tanh_value]) < 2:
      continue

    base = add.name or f"GruUpdate_{rewritten}"
    one_minus_z = f"{base}__one_minus_z"
    weighted_new = f"{base}__weighted_new"
    weighted_state = f"{base}__weighted_state"
    new_out = add.output[0]
    replace[add.name] = [
      helper.make_node("Sub", [one_name, z_value], [one_minus_z], name=f"{base}__SubOneMinusZ"),
      helper.make_node("Mul", [one_minus_z, tanh_value], [weighted_new], name=f"{base}__MulNew"),
      helper.make_node("Mul", [z_value, "initial_state"], [weighted_state], name=f"{base}__MulState"),
      helper.make_node("Add", [weighted_new, weighted_state], [new_out], name=f"{base}__AddWeighted"),
    ]
    remove.add(sub.name)
    remove.add(mul.name)
    remove.add(add.name)
    rewritten += 1

  if rewritten == 0:
    return 0

  nodes = []
  for node in model.graph.node:
    if node.name in remove:
      if node.name in replace:
        nodes.extend(replace[node.name])
      continue
    nodes.append(node)

  del model.graph.node[:]
  model.graph.node.extend(nodes)
  return rewritten


def split_final_plan_output(model: onnx.ModelProto, plan_prob_shift: float = 0.0,
                            plan_prob_delta: bool = False,
                            plan_prob_delta_after_gemm: bool = False,
                            plan_prob_center_before_delta: bool = False) -> int:
  if plan_prob_center_before_delta and not plan_prob_delta_after_gemm:
    raise ValueError("centering requires --plan-prob-delta-after-gemm")
  if plan_prob_delta and plan_prob_delta_after_gemm:
    raise ValueError("choose one plan probability delta strategy")
  shapes = inferred_float_shapes(model)
  initializers = {t.name: t for t in model.graph.initializer}
  plan_gemm = None
  final_concat = None
  for node in model.graph.node:
    if node.name == "Gemm_379" and node.op_type == "Gemm":
      plan_gemm = node
    elif node.name == "Concat_388" and node.op_type == "Concat":
      final_concat = node

  if plan_gemm is None or final_concat is None:
    return 0
  if len(plan_gemm.input) < 3:
    return 0

  weight = numpy_helper.to_array(initializers[plan_gemm.input[1]])
  bias = numpy_helper.to_array(initializers[plan_gemm.input[2]])
  if weight.shape[0] != PLAN_MHP_N * PLAN_STRIDE or bias.shape[0] != PLAN_MHP_N * PLAN_STRIDE:
    return 0

  new_initializers = []
  new_plan_outputs = []
  new_output_shapes = {}
  new_plan_nodes = []
  raw_prob_outputs = []
  ref_prob_idx = PLAN_STRIDE - 1
  # KPU final bias addition uses FP16 even with INT16 activations. The original
  # ~-320 common bias gives 0.25 ULPs, losing small differences before Sub.
  # A common bias shift cancels exactly in the delta, without changing weights.
  common_bias = np.mean(bias[PLAN_STRIDE-1::PLAN_STRIDE], dtype=np.float64)
  for plan_idx in range(PLAN_MHP_N):
    base = plan_idx * PLAN_STRIDE
    chunks = [
      ("data", base, base + PLAN_STRIDE - 1),
      ("prob", base + PLAN_STRIDE - 1, base + PLAN_STRIDE),
    ]
    for suffix, start, end in chunks:
      w_name = f"{plan_gemm.input[1]}__plan_{plan_idx}_{suffix}"
      b_name = f"{plan_gemm.input[2]}__plan_{plan_idx}_{suffix}"
      out_name = f"k230_plan_{plan_idx}_{suffix}"
      gemm_out_name = out_name
      chunk_weight = weight[start:end].astype(weight.dtype).copy()
      chunk_bias = bias[start:end].astype(bias.dtype).copy()
      if suffix == "prob" and plan_prob_delta_after_gemm:
        if plan_prob_center_before_delta:
          chunk_bias -= np.array(common_bias, dtype=chunk_bias.dtype)
        gemm_out_name = f"{out_name}__raw"
        raw_prob_outputs.append(gemm_out_name)
      elif suffix == "prob" and plan_prob_delta:
        chunk_weight -= weight[ref_prob_idx:ref_prob_idx + 1].astype(weight.dtype)
        chunk_bias -= bias[ref_prob_idx:ref_prob_idx + 1].astype(bias.dtype)
      elif suffix == "prob" and plan_prob_shift != 0.0:
        chunk_bias += np.array(plan_prob_shift, dtype=chunk_bias.dtype)
      new_initializers.append(numpy_helper.from_array(chunk_weight, w_name))
      new_initializers.append(numpy_helper.from_array(chunk_bias, b_name))
      new_plan_nodes.append(helper.make_node(
        "Gemm",
        [plan_gemm.input[0], w_name, b_name],
        [gemm_out_name],
        name=f"{plan_gemm.name}_plan_{plan_idx}_{suffix}",
        alpha=1.0,
        beta=1.0,
        transB=1,
      ))
      new_plan_outputs.append(out_name)
      new_output_shapes[out_name] = [1, end - start]

  if plan_prob_delta_after_gemm:
    if len(raw_prob_outputs) != PLAN_MHP_N:
      raise RuntimeError("raw probability outputs were not generated")
    ref_prob = raw_prob_outputs[0]
    for plan_idx, raw_prob in enumerate(raw_prob_outputs):
      new_plan_nodes.append(helper.make_node(
        "Sub",
        [raw_prob, ref_prob],
        [f"k230_plan_{plan_idx}_prob"],
        name=f"{plan_gemm.name}_plan_{plan_idx}_prob_delta_after_gemm",
      ))

  output_names = []
  for inp in final_concat.input:
    if inp == plan_gemm.output[0]:
      output_names.extend(new_plan_outputs)
    else:
      output_names.append(inp)

  nodes = []
  for node in model.graph.node:
    if node.name == plan_gemm.name:
      nodes.extend(new_plan_nodes)
    elif node.name == final_concat.name:
      continue
    else:
      nodes.append(node)

  del model.graph.node[:]
  model.graph.node.extend(nodes)
  model.graph.initializer.extend(new_initializers)

  del model.graph.output[:]
  for name in output_names:
    shape = new_output_shapes.get(name, shapes.get(name))
    if shape is None:
      raise RuntimeError(f"shape not found for output {name}")
    model.graph.output.append(helper.make_tensor_value_info(name, TensorProto.FLOAT, shape))

  return 1


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--in-model", required=True, type=Path)
  parser.add_argument("--out-model", required=True, type=Path)
  parser.add_argument("--gemm-split", action="store_true")
  parser.add_argument("--decompose-elu", action="store_true")
  parser.add_argument("--decompose-elu-names", default="", help="comma-separated Elu node names to decompose")
  parser.add_argument("--identity-dw-before-elu", action="store_true")
  parser.add_argument("--identity-dw-before-elu-names", default="", help="comma-separated Elu node names to wrap with an identity depthwise 1x1 Conv")
  parser.add_argument("--summarizer-infeats-as-conv", action="store_true")
  parser.add_argument("--gru-update", action="store_true")
  parser.add_argument("--split-plan-output", action="store_true")
  parser.add_argument("--plan-prob-shift", default=0.0, type=float)
  parser.add_argument("--plan-prob-delta", action="store_true")
  parser.add_argument("--plan-prob-delta-after-gemm", action="store_true")
  parser.add_argument("--plan-prob-center-before-delta", action="store_true",
                      help="remove common logit bias before KPU FP16 rounding; preserve delta outputs")
  args = parser.parse_args()
  if args.plan_prob_center_before_delta and not (args.split_plan_output and args.plan_prob_delta_after_gemm):
    parser.error("--plan-prob-center-before-delta requires --split-plan-output and --plan-prob-delta-after-gemm")

  model = onnx.load(args.in_model)
  counts = {}
  if args.gemm_split:
    counts["gemm_split"] = rewrite_gemm_split(model)
  if args.decompose_elu:
    elu_names = {name.strip() for name in args.decompose_elu_names.split(",") if name.strip()}
    counts["elu"] = decompose_elu(model, elu_names or None)
  if args.identity_dw_before_elu:
    elu_names = {name.strip() for name in args.identity_dw_before_elu_names.split(",") if name.strip()}
    counts["identity_dw_before_elu"] = insert_identity_depthwise_before_elu(model, elu_names or None)
  if args.summarizer_infeats_as_conv:
    counts["summarizer_infeats_as_conv"] = rewrite_summarizer_infeats_as_conv(model)
  if args.gru_update:
    counts["gru_update"] = rewrite_gru_update(model)
  if args.split_plan_output:
    counts["split_plan_output"] = split_final_plan_output(
      model,
      args.plan_prob_shift,
      args.plan_prob_delta,
      args.plan_prob_delta_after_gemm,
      args.plan_prob_center_before_delta,
    )
  if not counts:
    counts["gemm_split"] = rewrite_gemm_split(model)
  if sum(counts.values()) == 0:
    raise RuntimeError("no patterns were rewritten")

  onnx.checker.check_model(model)
  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(model, args.out_model)
  print("rewrites", counts)
  print(args.out_model)


if __name__ == "__main__":
  main()
