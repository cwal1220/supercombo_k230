#!/usr/bin/env bash
# Builds a candidate in a separate directory; never installs it on the board.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo"
python_bin="${PYTHON_BIN:-python3}"
source_onnx="${SOURCE_ONNX:?Set SOURCE_ONNX to the original supercombo.onnx}"
calib_npz="${CALIB_NPZ:?Set CALIB_NPZ to an NPZ under this repository}"
samples="${CALIB_SAMPLES:?Set CALIB_SAMPLES to the exact calibration sample count}"
out_dir="${CANDIDATE_DIR:-models/quantization_work/candidate}"
calib_npz="$("$python_bin" -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).resolve().relative_to(Path.cwd()))' "$calib_npz")"
out_dir="$("$python_bin" -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).resolve().relative_to(Path.cwd()))' "$out_dir")"
mkdir -p "$out_dir"
"$python_bin" tools/model/rewrite_supercombo_onnx.py \
  --in-model "$source_onnx" --out-model "$out_dir/base.onnx" \
  --gemm-split --split-plan-output --plan-prob-delta-after-gemm \
  --plan-prob-center-before-delta \
  --identity-dw-before-elu --identity-dw-before-elu-names Elu_223
"$python_bin" tools/model/retype_image_inputs_uint8.py \
  --in-model "$out_dir/base.onnx" --out-model "$out_dir/uint8.onnx" --op dequantize
bash tools/model/run_nncase.sh python -u tools/model/compile_supercombo_nncase.py \
  --model "$out_dir/uint8.onnx" --out "$out_dir/supercombo.kmodel" \
  --dump-dir "$out_dir/dump" --target k230 --ptq --calib-npz "$calib_npz" \
  --samples "$samples" --calibrate-method "${CALIBRATE_METHOD:-NoClip}" \
  --quant-type int16 --w-quant-type uint8 \
  --finetune-weights-method "${FINETUNE_WEIGHTS_METHOD:-NoFineTuneWeights}" \
  --no-dump-ir --no-dump-asm
shasum -a 256 "$source_onnx" "$calib_npz" "$out_dir/uint8.onnx" "$out_dir/supercombo.kmodel" \
  > "$out_dir/manifest.sha256"
