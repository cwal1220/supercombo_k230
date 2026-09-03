#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${REPO_DIR}/models"
OPENPILOT_DIR="${OPENPILOT_DIR:-/Users/chan/Documents/openpilot_c2}"
DOCKER_IMAGE="${DOCKER_IMAGE:-supercombo-nncase-k230:2.11.0-sdk}"
SOURCE_ONNX="${SOURCE_ONNX:-${OPENPILOT_DIR}/selfdrive/modeld/models/supercombo.onnx}"

INTERMEDIATE_ONNX="${MODEL_DIR}/onnx/supercombo_base.onnx"
FINAL_ONNX="${MODEL_DIR}/onnx/supercombo_uint8.onnx"
PTQ_NPZ="${MODEL_DIR}/ptq/supercombo_calib_mixed173.npz"
FINAL_KMODEL="${MODEL_DIR}/supercombo.kmodel"
COMPILED_KMODEL="${MODEL_DIR}/work/supercombo-full.kmodel"

mkdir -p "${MODEL_DIR}/onnx" "${MODEL_DIR}/ptq" "${MODEL_DIR}/work/nncase_dump"

if [[ "${REGEN_PTQ:-0}" == "1" ]]; then
  echo 'The current model uses the preserved mixed 173-sample PTQ set.' >&2
  echo 'See tools/model/QUANTIZATION.md to regenerate native K230 samples; the legacy 77-sample generator is not equivalent.' >&2
  exit 1
fi
test -f "${SOURCE_ONNX}"
test -f "${PTQ_NPZ}"

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
    --out-model /work/models/onnx/supercombo_base.onnx \
    --gemm-split \
    --split-plan-output \
    --plan-prob-delta-after-gemm \
    --plan-prob-center-before-delta \
    --identity-dw-before-elu \
    --identity-dw-before-elu-names Elu_223

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

docker run --rm --platform linux/amd64 \
  --ulimit core=0 \
  -e DOTNET_EnableWriteXorExecute=0 \
  -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 \
  -e COMPlus_ReadyToRun=0 \
  -e COMPlus_ZapDisable=1 \
  -e OMP_NUM_THREADS=4 \
  -v "${REPO_DIR}:/work" \
  -v "${OPENPILOT_DIR}:/openpilot_c2" \
  -w /work \
  "${DOCKER_IMAGE}" \
  python -u tools/model/compile_supercombo_nncase.py \
    --model /work/models/onnx/supercombo_uint8.onnx \
    --out /work/models/work/supercombo-full.kmodel \
    --dump-dir /work/models/work/nncase_dump \
    --target k230 \
    --ptq \
    --calib-npz /work/models/ptq/supercombo_calib_mixed173.npz \
    --samples 173 \
    --calibrate-method NoClip \
    --quant-type int16 \
    --w-quant-type uint8 \
    --finetune-weights-method UseSquant \
    --no-dump-ir \
    --no-dump-asm

mv "${COMPILED_KMODEL}" "${FINAL_KMODEL}"

(
  cd "${MODEL_DIR}"
  shasum -a 256 supercombo.kmodel onnx/supercombo_base.onnx \
    onnx/supercombo_uint8.onnx ptq/supercombo_calib_mixed173.npz \
    ptq/supercombo_calib_mixed173_metadata.json > manifest.sha256
)

shasum -a 256 "${FINAL_KMODEL}" "${FINAL_ONNX}" "${PTQ_NPZ}"
