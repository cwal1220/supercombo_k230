#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${REPO_DIR}/models"
OPENPILOT_DIR="${OPENPILOT_DIR:-/Users/chan/Documents/openpilot_c2}"
DOCKER_IMAGE="${DOCKER_IMAGE:-supercombo-nncase-k230:2.11.0-sdk}"
SOURCE_ONNX="${SOURCE_ONNX:-${OPENPILOT_DIR}/selfdrive/modeld/models/supercombo.onnx}"
# bias correction runs ONNX Runtime on the host (the Docker image has no onnxruntime).
PYTHON_BIN="${PYTHON_BIN:-python3}"

REWRITTEN_ONNX="${MODEL_DIR}/onnx/supercombo_rewritten.onnx"
INTERMEDIATE_ONNX="${MODEL_DIR}/onnx/supercombo_base.onnx"
FINAL_ONNX="${MODEL_DIR}/onnx/supercombo_uint8.onnx"
PTQ_BASE_NPZ="${MODEL_DIR}/ptq/supercombo_calib_mixed173.npz"
PTQ_K230_NPZ="${MODEL_DIR}/ptq/supercombo_calib_k230_127.npz"
PTQ_NPZ="${MODEL_DIR}/work/supercombo_calib_mixed300.npz"
QUANT_SCHEME="${MODEL_DIR}/ptq/supercombo_quant_scheme_bychannel.json"
FINAL_KMODEL="${MODEL_DIR}/supercombo.kmodel"
COMPILED_KMODEL="${MODEL_DIR}/work/supercombo-full.kmodel"

mkdir -p "${MODEL_DIR}/onnx" "${MODEL_DIR}/ptq" "${MODEL_DIR}/work/nncase_dump"

if [[ "${REGEN_PTQ:-0}" == "1" ]]; then
  echo 'The PTQ set is the preserved 173-sample mix plus 127 committed native K230 samples.' >&2
  echo 'See tools/model/QUANTIZATION.md to regenerate native samples; the legacy 77-sample generator is not equivalent.' >&2
  exit 1
fi
test -f "${SOURCE_ONNX}"
test -f "${PTQ_BASE_NPZ}"
test -f "${PTQ_K230_NPZ}"
test -f "${QUANT_SCHEME}"
"${PYTHON_BIN}" -c 'import numpy, onnx, onnxruntime' || {
  echo "PYTHON_BIN=${PYTHON_BIN} needs numpy, onnx and onnxruntime for bias correction" >&2
  exit 1
}

docker run --rm --platform linux/amd64 \
  -e DOTNET_EnableWriteXorExecute=0 \
  -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 \
  -e COMPlus_ReadyToRun=0 \
  -e COMPlus_ZapDisable=1 \
  -v "${REPO_DIR}:/work" \
  -v "${SOURCE_ONNX}:/source.onnx:ro" \
  -w /work \
  "${DOCKER_IMAGE}" \
  python -u tools/model/rewrite_supercombo_onnx.py \
    --in-model /source.onnx \
    --out-model /work/models/onnx/supercombo_rewritten.onnx \
    --gemm-split \
    --split-plan-output \
    --plan-prob-delta-after-gemm \
    --plan-prob-center-before-delta \
    --identity-dw-before-elu \
    --identity-dw-before-elu-names Elu_223 \
    --tanh-via-sigmoid

# 173 C2/K230 혼합 + 127 K230 = 300장 PTQ 세트
"${PYTHON_BIN}" - "${PTQ_BASE_NPZ}" "${PTQ_K230_NPZ}" "${PTQ_NPZ}" <<'EOF'
import sys
import numpy as np
a, b = np.load(sys.argv[1], allow_pickle=False), np.load(sys.argv[2], allow_pickle=False)
merged = {}
for name in ("input_imgs", "big_input_imgs", "desire", "traffic_convention", "initial_state"):
  values = np.concatenate([a[name], b[name]])
  merged[name] = values.astype(np.uint8 if name.endswith("imgs") else np.float32)
assert len(merged["desire"]) == 300
np.savez_compressed(sys.argv[3], **merged)
EOF

# 가중치를 nncase 채널별 uint8 격자로 미리 반올림하고 bias를 보정한다(tools/model/QUANTIZATION.md).
"${PYTHON_BIN}" "${REPO_DIR}/tools/model/prequant_bias_correct.py" \
  --in-model "${REWRITTEN_ONNX}" \
  --out-model "${INTERMEDIATE_ONNX}" \
  --calib-npz "${PTQ_NPZ}" \
  --samples 48 \
  --scheme "${QUANT_SCHEME}"

# 이미지 입력을 uint8 + DequantizeLinear(scale=1)로 바꾼다. 값은 float 입력과
# 비트 동일하게 유지되고, 런타임 워프의 float 변환과 4배 쓰기 대역폭이 사라진다.
docker run --rm --platform linux/amd64 \
  -v "${REPO_DIR}:/work" \
  -w /work \
  "${DOCKER_IMAGE}" \
  python -u tools/model/retype_image_inputs_uint8.py \
    --in-model /work/models/onnx/supercombo_base.onnx \
    --out-model /work/models/onnx/supercombo_uint8.onnx \
    --op dequantize

# nncase 2.11이 300장 보정에서 드물게 시작 직후 SIGSEGV로 죽으므로 최대 3회 재시도한다.
compiled=0
for attempt in 1 2 3; do
  if docker run --rm --platform linux/amd64 \
    --ulimit core=0 \
    -e DOTNET_EnableWriteXorExecute=0 \
    -e COMPlus_EnableWriteXorExecute=0 \
    -e COMPlus_TieredCompilation=0 \
    -e COMPlus_ReadyToRun=0 \
    -e COMPlus_ZapDisable=1 \
    -e OMP_NUM_THREADS=4 \
    -v "${REPO_DIR}:/work" \
    -w /work \
    "${DOCKER_IMAGE}" \
    python -u tools/model/compile_supercombo_nncase.py \
      --model /work/models/onnx/supercombo_uint8.onnx \
      --out /work/models/work/supercombo-full.kmodel \
      --dump-dir /work/models/work/nncase_dump \
      --target k230 \
      --ptq \
      --calib-npz /work/models/work/supercombo_calib_mixed300.npz \
      --samples 300 \
      --calibrate-method NoClip \
      --quant-type int16 \
      --w-quant-type uint8 \
      --finetune-weights-method NoFineTuneWeights \
      --no-dump-ir \
      --no-dump-asm; then
    compiled=1
    break
  fi
  echo "nncase compile attempt ${attempt} failed" >&2
done
[[ "${compiled}" == "1" ]]

mv "${COMPILED_KMODEL}" "${FINAL_KMODEL}"

(
  cd "${MODEL_DIR}"
  shasum -a 256 supercombo.kmodel onnx/supercombo_rewritten.onnx onnx/supercombo_base.onnx \
    onnx/supercombo_uint8.onnx ptq/supercombo_calib_mixed173.npz \
    ptq/supercombo_calib_mixed173_metadata.json ptq/supercombo_calib_k230_127.npz \
    ptq/supercombo_calib_k230_127_metadata.json ptq/supercombo_quant_scheme_bychannel.json > manifest.sha256
)

shasum -a 256 "${FINAL_KMODEL}" "${FINAL_ONNX}" "${PTQ_NPZ}"
