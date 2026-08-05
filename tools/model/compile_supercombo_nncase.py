#!/usr/bin/env python3
import argparse
import collections
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto


ONNX_TO_NUMPY = {
  TensorProto.FLOAT: np.float32,
  TensorProto.FLOAT16: np.float16,
  TensorProto.UINT8: np.uint8,
  TensorProto.INT8: np.int8,
  TensorProto.UINT16: np.uint16,
  TensorProto.INT16: np.int16,
  TensorProto.UINT32: np.uint32,
  TensorProto.INT32: np.int32,
  TensorProto.INT64: np.int64,
}


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def read_model(path: Path) -> bytes:
  with path.open("rb") as f:
    return f.read()


def input_specs(path: Path) -> list[tuple[str, tuple[int, ...], np.dtype]]:
  model = onnx.load(path)
  initializer_names = {init.name for init in model.graph.initializer}
  specs = []
  for value in model.graph.input:
    if value.name in initializer_names:
      continue
    tensor_type = value.type.tensor_type
    dtype = ONNX_TO_NUMPY.get(tensor_type.elem_type)
    if dtype is None:
      raise RuntimeError(f"unsupported input dtype for {value.name}: {tensor_type.elem_type}")
    shape = []
    for dim in tensor_type.shape.dim:
      if not dim.dim_value:
        raise RuntimeError(f"dynamic input shape is not supported: {value.name}")
      shape.append(int(dim.dim_value))
    specs.append((value.name, tuple(shape), np.dtype(dtype)))
  return specs


def model_metadata(path: Path, specs: list[tuple[str, tuple[int, ...], np.dtype]]) -> dict:
  model = onnx.load(path)
  input_types = {
    value.name: TensorProto.DataType.Name(value.type.tensor_type.elem_type)
    for value in model.graph.input
  }
  outputs = []
  for value in model.graph.output:
    tensor_type = value.type.tensor_type
    outputs.append({
      "name": value.name,
      "dtype": TensorProto.DataType.Name(tensor_type.elem_type),
      "shape": [int(dim.dim_value) if dim.dim_value else dim.dim_param or "?"
                for dim in tensor_type.shape.dim],
    })
  return {
    "model": path.name,
    "model_size": path.stat().st_size,
    "model_sha256": sha256_file(path),
    "inputs": [
      {
        "name": name,
        "shape": list(shape),
        "numpy_dtype": str(dtype),
        "onnx_dtype": input_types[name],
      }
      for name, shape, dtype in specs
    ],
    "outputs": outputs,
    "op_counts": dict(collections.Counter(node.op_type for node in model.graph.node).most_common()),
  }


def random_int(rng: np.random.Generator, shape: tuple[int, ...], dtype: np.dtype) -> np.ndarray:
  if dtype == np.dtype(np.uint8):
    return rng.integers(0, 256, size=shape, dtype=np.uint8)
  if dtype == np.dtype(np.int8):
    return rng.integers(-128, 128, size=shape, dtype=np.int8)
  info = np.iinfo(dtype)
  return rng.integers(info.min, info.max, size=shape, dtype=dtype)


def random_calibration(specs: list[tuple[str, tuple[int, ...], np.dtype]], samples: int) -> list[list[np.ndarray]]:
  rng = np.random.default_rng(20260524)
  data = []
  for name, shape, dtype in specs:
    input_samples = []
    for _ in range(samples):
      if np.issubdtype(dtype, np.integer):
        arr = random_int(rng, shape, dtype)
      elif name in ("input_imgs", "big_input_imgs"):
        arr = rng.normal(0.0, 1.0, shape).astype(np.float32)
      elif name == "traffic_convention" and tuple(shape) == (1, 2):
        arr = np.array([[1.0, 0.0]], dtype=dtype)
      else:
        arr = np.zeros(shape, dtype=dtype)
      input_samples.append(arr)
    data.append(input_samples)
  return data


def npz_calibration(path: Path, specs: list[tuple[str, tuple[int, ...], np.dtype]], samples: int | None = None) -> list[list[np.ndarray]]:
  loaded = np.load(path)
  arrays = []
  sample_count = None
  for name, shape, dtype in specs:
    if name not in loaded:
      raise RuntimeError(f"calibration NPZ is missing {name!r}; keys={list(loaded.keys())}")
    arr = loaded[name].astype(dtype)
    if tuple(arr.shape) == shape:
      arr = arr.reshape((1,) + shape)
    elif tuple(arr.shape[1:]) != shape:
      raise RuntimeError(
        f"calibration {name!r} has shape {arr.shape}, expected (N,{','.join(map(str, shape))})")
    if samples is not None:
      arr = arr[:samples]
    if arr.shape[0] == 0:
      raise RuntimeError(f"calibration {name!r} has no samples")
    if sample_count is None:
      sample_count = arr.shape[0]
    elif sample_count != arr.shape[0]:
      raise RuntimeError(f"calibration sample count mismatch for {name!r}")
    sample_list = [np.ascontiguousarray(x.reshape(shape)) for x in arr]
    arrays.append(sample_list)
  return arrays


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--model", required=True, type=Path)
  parser.add_argument("--out", required=True, type=Path)
  parser.add_argument("--dump-dir", default=Path("dump/supercombo_k230"), type=Path)
  parser.add_argument("--target", default="k230")
  parser.add_argument("--no-dump-ir", action="store_true")
  parser.add_argument("--no-dump-asm", action="store_true")
  parser.add_argument("--ptq", action="store_true")
  parser.add_argument("--samples", default=5, type=int)
  parser.add_argument("--calib-npz", type=Path)
  parser.add_argument("--calibrate-method", default="Kld")
  parser.add_argument("--quant-type", default="uint8")
  parser.add_argument("--w-quant-type", default="uint8")
  parser.add_argument("--quant-scheme", type=Path)
  parser.add_argument("--use-mix-quant", action="store_true")
  parser.add_argument("--quant-scheme-strict", action="store_true")
  parser.add_argument("--use-mse-quant-w", action="store_true")
  parser.add_argument("--finetune-weights-method", default="NoFineTuneWeights")
  parser.add_argument("--dump-quant-error", action="store_true")
  parser.add_argument("--export-quant-scheme", action="store_true")
  parser.add_argument("--export-weight-range-by-channel", action="store_true")
  parser.add_argument("--metadata-json", type=Path)
  args = parser.parse_args()

  args.out.parent.mkdir(parents=True, exist_ok=True)
  args.dump_dir.mkdir(parents=True, exist_ok=True)

  import nncase

  compile_options = nncase.CompileOptions()
  compile_options.target = args.target
  compile_options.dump_ir = not args.no_dump_ir
  compile_options.dump_asm = not args.no_dump_asm
  compile_options.dump_dir = str(args.dump_dir)
  compile_options.preprocess = False

  specs = input_specs(args.model)
  metadata = model_metadata(args.model, specs)
  metadata.update({
    "target": args.target,
    "ptq": args.ptq,
    "samples": args.samples if args.ptq else None,
    "calibration_npz": args.calib_npz.name if args.calib_npz else None,
    "calibration_npz_sha256": sha256_file(args.calib_npz) if args.calib_npz else None,
    "calibrate_method": args.calibrate_method if args.ptq else None,
    "quant_type": args.quant_type if args.ptq else None,
    "w_quant_type": args.w_quant_type if args.ptq else None,
    "use_mix_quant": args.use_mix_quant if args.ptq else None,
    "use_mse_quant_w": args.use_mse_quant_w if args.ptq else None,
    "finetune_weights_method": args.finetune_weights_method if args.ptq else None,
    "dump_quant_error": args.dump_quant_error if args.ptq else None,
    "export_quant_scheme": args.export_quant_scheme if args.ptq else None,
    "export_weight_range_by_channel": (
      args.export_weight_range_by_channel if args.ptq else None
    ),
    "quant_scheme": args.quant_scheme.name if args.quant_scheme else None,
    "quant_scheme_strict_mode": args.quant_scheme_strict if args.ptq else None,
  })
  compiler = nncase.Compiler(compile_options)
  compiler.import_onnx(read_model(args.model), nncase.ImportOptions())

  if args.ptq:
    ptq_options = nncase.PTQTensorOptions()
    cali_data = npz_calibration(args.calib_npz, specs, args.samples) if args.calib_npz else random_calibration(specs, args.samples)
    ptq_options.samples_count = len(cali_data[0])
    ptq_options.calibrate_method = args.calibrate_method
    ptq_options.quant_type = args.quant_type
    ptq_options.w_quant_type = args.w_quant_type
    ptq_options.use_mix_quant = args.use_mix_quant
    ptq_options.use_mse_quant_w = args.use_mse_quant_w
    ptq_options.finetune_weights_method = args.finetune_weights_method
    ptq_options.dump_quant_error = args.dump_quant_error
    ptq_options.export_quant_scheme = args.export_quant_scheme
    ptq_options.export_weight_range_by_channel = args.export_weight_range_by_channel
    if args.quant_scheme is not None:
      ptq_options.quant_scheme = str(args.quant_scheme)
    ptq_options.quant_scheme_strict_mode = args.quant_scheme_strict
    ptq_options.set_tensor_data(cali_data)
    compiler.use_ptq(ptq_options)

  start = time.monotonic()
  compiler.compile()
  elapsed = time.monotonic() - start

  args.out.write_bytes(compiler.gencode_tobytes())
  metadata.update({
    "compile_elapsed_s": elapsed,
    "kmodel": args.out.name,
    "kmodel_size": args.out.stat().st_size,
    "kmodel_sha256": sha256_file(args.out),
  })
  print(f"compiled {args.out} in {elapsed:.2f}s")
  print(f"kmodel_size={args.out.stat().st_size / 1024 / 1024:.2f} MiB")
  print(f"kmodel_sha256={metadata['kmodel_sha256']}")
  if args.metadata_json:
    args.metadata_json.parent.mkdir(parents=True, exist_ok=True)
    args.metadata_json.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
  main()
