#!/usr/bin/env python3
import argparse
import os
import time
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto


ONNX_TO_NUMPY = {
  TensorProto.FLOAT: np.float32,
  TensorProto.UINT8: np.uint8,
  TensorProto.INT8: np.int8,
  TensorProto.UINT16: np.uint16,
  TensorProto.INT16: np.int16,
  TensorProto.UINT32: np.uint32,
  TensorProto.INT32: np.int32,
  TensorProto.INT64: np.int64,
}


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
  loaded = np.load(path, allow_pickle=False)
  arrays = []
  counts = set()
  for name, shape, dtype in specs:
    arr = loaded[name]
    counts.add(len(arr))
    if arr.shape[1:] not in (shape, shape[1:]):
      raise ValueError(f"invalid calibration shape for {name}: {arr.shape}, expected [N,{shape}]")
    if not np.isfinite(arr).all():
      raise ValueError(f"nonfinite calibration input: {name}")
    if np.issubdtype(dtype, np.integer):
      limits = np.iinfo(dtype)
      if arr.min() < limits.min or arr.max() > limits.max or not np.array_equal(arr, np.round(arr)):
        raise ValueError(f"lossy calibration cast to {dtype}: {name}")
    arr = arr.astype(dtype)
    if samples is not None:
      arr = arr[:samples]
    sample_list = []
    for x in arr:
      if tuple(x.shape) == shape[1:]:
        x = x.reshape(shape)
      sample_list.append(np.ascontiguousarray(x))
    arrays.append(sample_list)
  loaded.close()
  if len(counts) != 1 or not arrays or not arrays[0]:
    raise ValueError(f"empty or mismatched calibration counts: {counts}")
  if samples is not None and samples > next(iter(counts)):
    raise ValueError(f"requested {samples} calibration samples but only {next(iter(counts))} exist")
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
  parser.add_argument("--dump-quant-error", action="store_true")
  parser.add_argument("--export-quant-scheme", action="store_true")
  parser.add_argument("--export-weight-range-by-channel", action="store_true")
  parser.add_argument("--finetune-weights-method", choices=("NoFineTuneWeights", "UseSquant"), default="NoFineTuneWeights")
  args = parser.parse_args()
  if args.samples <= 0:
    parser.error("--samples must be positive")
  if args.target == 'k230' and args.quant_type == args.w_quant_type == 'int16':
    parser.error("K230 does not support INT16 activations and INT16 weights together")
  if args.no_dump_ir and (args.export_quant_scheme or args.dump_quant_error):
    parser.error("quantization diagnostics require IR dumping; omit --no-dump-ir")

  args.out.parent.mkdir(parents=True, exist_ok=True)
  args.dump_dir.mkdir(parents=True, exist_ok=True)

  import nncase
  if args.use_mse_quant_w:
    import inspect
    if 'use_mse_quant_w' not in inspect.getsource(nncase.Compiler.use_ptq):
      raise RuntimeError("This nncase Python binding silently ignores use_mse_quant_w; "
                         "choose a supported option instead of producing an unchanged model")

  compile_options = nncase.CompileOptions()
  compile_options.target = args.target
  compile_options.dump_ir = not args.no_dump_ir
  compile_options.dump_asm = not args.no_dump_asm
  compile_options.dump_dir = str(args.dump_dir)
  compile_options.preprocess = False

  specs = input_specs(args.model)
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
    ptq_options.dump_quant_error = args.dump_quant_error
    ptq_options.export_quant_scheme = args.export_quant_scheme
    ptq_options.export_weight_range_by_channel = args.export_weight_range_by_channel
    ptq_options.finetune_weights_method = args.finetune_weights_method
    if args.quant_scheme is not None:
      ptq_options.quant_scheme = str(args.quant_scheme)
    ptq_options.quant_scheme_strict_mode = args.quant_scheme_strict
    ptq_options.set_tensor_data(cali_data)
    compiler.use_ptq(ptq_options)

  start = time.monotonic()
  print(f"compiling {args.model} target={args.target} "
        f"PTQ={args.ptq} samples={len(cali_data[0]) if args.ptq else 0} "
        f"activation={args.quant_type} weights={args.w_quant_type} "
        f"calibration={args.calibrate_method} finetune={args.finetune_weights_method}", flush=True)
  compiler.compile()
  elapsed = time.monotonic() - start

  args.out.write_bytes(compiler.gencode_tobytes())
  print(f"compiled {args.out} in {elapsed:.2f}s")
  print(f"kmodel_size={args.out.stat().st_size / 1024 / 1024:.2f} MiB")


if __name__ == "__main__":
  main()
