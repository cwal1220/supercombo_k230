#!/usr/bin/env python3
"""image 입력(input_imgs, big_input_imgs)을 uint8로 바꾸고 Cast를 삽입한다.

워프 커널은 0..255 정수를 만들므로 uint8 입력 + 그래프 내 Cast(float32)는
기존 float 입력과 비트 동일한 값을 타워에 전달한다. 입력 텐서 쓰기 대역폭이
1/4로 줄고 런타임의 int->float 변환이 사라진다.
"""
import argparse
from pathlib import Path

import onnx
from onnx import TensorProto, helper, numpy_helper

import numpy as np


IMAGE_INPUTS = ("input_imgs", "big_input_imgs")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--in-model", type=Path, required=True)
  parser.add_argument("--out-model", type=Path, required=True)
  parser.add_argument("--op", choices=("cast", "dequantize"), default="dequantize",
                      help="uint8->float 변환 방식. dequantize(QDQ 관용구)가 "
                           "컴파일러 융합에 유리하다.")
  parser.add_argument("--image-inputs", nargs="+", default=list(IMAGE_INPUTS),
                      help="uint8로 바꿀 이미지 입력 이름")
  args = parser.parse_args()
  image_inputs = tuple(args.image_inputs)

  model = onnx.load(str(args.in_model))
  graph = model.graph

  retyped = []
  for value in graph.input:
    if value.name not in image_inputs:
      continue
    if value.type.tensor_type.elem_type != TensorProto.FLOAT:
      raise RuntimeError(f"{value.name} is not float32; already retyped?")
    value.type.tensor_type.elem_type = TensorProto.UINT8
    retyped.append(value.name)

  if sorted(retyped) != sorted(image_inputs):
    raise RuntimeError(f"expected image inputs {image_inputs}, retyped {retyped}")

  # 입력 이름은 유지한다(PTQ npz 키·런타임 바인딩과 일치). 대신 float
  # 소비자들이 변환 출력(name_f32)을 읽도록 참조를 바꾼다.
  if args.op == "dequantize":
    # DequantizeLinear는 opset 10부터. 그래프의 연산자들은 9→10에서 의미가
    # 변하지 않음을 확인하고 승격한다.
    for opset in model.opset_import:
      if opset.domain == "" and opset.version < 10:
        opset.version = 10
    graph.initializer.extend([
        numpy_helper.from_array(np.float32(1.0), "image_dq_scale"),
        numpy_helper.from_array(np.uint8(0), "image_dq_zero"),
    ])

  convert_nodes = []
  for name in image_inputs:
    f32_name = name + "_f32"
    for node in graph.node:
      for i, node_input in enumerate(node.input):
        if node_input == name:
          node.input[i] = f32_name
    if args.op == "dequantize":
      convert_nodes.append(helper.make_node(
          "DequantizeLinear",
          inputs=[name, "image_dq_scale", "image_dq_zero"],
          outputs=[f32_name], name=f"DQ_{name}_u8_to_f32"))
    else:
      convert_nodes.append(helper.make_node(
          "Cast", inputs=[name], outputs=[f32_name],
          name=f"Cast_{name}_u8_to_f32", to=TensorProto.FLOAT))

  # 변환 노드는 그래프 맨 앞(토폴로지 순서 유지)에 둔다.
  for node in reversed(convert_nodes):
    graph.node.insert(0, node)

  onnx.checker.check_model(model)
  args.out_model.parent.mkdir(parents=True, exist_ok=True)
  onnx.save(model, str(args.out_model))
  op_name = "DequantizeLinear" if args.op == "dequantize" else "Cast"
  print(f"retyped {', '.join(image_inputs)} -> uint8 with {op_name}: {args.out_model}")


if __name__ == "__main__":
  main()
