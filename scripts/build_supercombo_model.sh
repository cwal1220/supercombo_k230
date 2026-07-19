#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${REPO_DIR}/models"
OPENPILOT_DIR="${OPENPILOT_DIR:-/Users/chan/Documents/openpilot_c2}"
PYTHON_BIN="${PYTHON_BIN:-${OPENPILOT_DIR}/model_viz_experiment/.venv/bin/python}"
DOCKER_IMAGE="${DOCKER_IMAGE:-supercombo-nncase-k230:2.11.0-sdk}"
SOURCE_ONNX="${SOURCE_ONNX:-${OPENPILOT_DIR}/selfdrive/modeld/models/supercombo.onnx}"
DATA_ROOT="${DATA_ROOT:-${OPENPILOT_DIR}/device_collected}"

INTERMEDIATE_ONNX="${MODEL_DIR}/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx"
FINAL_ONNX="${MODEL_DIR}/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero.onnx"
PTQ_NPZ="${MODEL_DIR}/ptq/supercombo_balanced80_calib.npz"
FINAL_KMODEL="${MODEL_DIR}/supercombo.kmodel"

mkdir -p "${MODEL_DIR}/onnx" "${MODEL_DIR}/ptq" "${MODEL_DIR}/work/nncase_dump"

"${PYTHON_BIN}" "${REPO_DIR}/tools/model/rewrite_supercombo_onnx.py" \
  --in-model "${SOURCE_ONNX}" \
  --out-model "${INTERMEDIATE_ONNX}" \
  --gemm-split \
  --split-plan-output \
  --plan-prob-delta-after-gemm \
  --identity-dw-before-elu \
  --identity-dw-before-elu-names Elu_223

"${PYTHON_BIN}" "${REPO_DIR}/tools/model/remove_supercombo_big_input.py" \
  --input "${INTERMEDIATE_ONNX}" \
  --output "${FINAL_ONNX}" \
  --mode fold-zero \
  --keep-big-input

if [[ "${REGEN_PTQ:-0}" == "1" ]]; then
  "${PYTHON_BIN}" "${REPO_DIR}/tools/model/make_supercombo_calibration.py" \
    --data-root "${DATA_ROOT}" \
    --model "${SOURCE_ONNX}" \
    --out "${PTQ_NPZ}" \
    --samples 80 \
    --warmup 5 \
    --per-segment 1 \
    --max-segments 80 \
    --warp-mode openpilot \
    --segment-list "${PKG_DIR}/ptq/balanced80_segments.txt" \
    --metadata-out "${PKG_DIR}/ptq/supercombo_balanced80_calib_metadata.json"
fi

docker run --rm --platform linux/amd64 \
  -e DOTNET_EnableWriteXorExecute=0 \
  -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 \
  -e COMPlus_ReadyToRun=0 \
  -e COMPlus_ZapDisable=1 \
  -v "${REPO_DIR}:/work" \
  -v "${OPENPILOT_DIR}:/openpilot_c2" \
  -w /work \
  "${DOCKER_IMAGE}" \
  python -u tools/model/compile_supercombo_nncase.py \
    --model /work/models/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero.onnx \
    --out /work/models/supercombo.kmodel \
    --dump-dir /work/models/work/nncase_dump \
    --target k230 \
    --ptq \
    --calib-npz /work/models/ptq/supercombo_balanced80_calib.npz \
    --samples 80 \
    --calibrate-method NoClip \
    --quant-type int16 \
    --w-quant-type uint8 \
    --no-dump-ir \
    --no-dump-asm

(
  cd "${MODEL_DIR}"
  find . -path ./variants -prune -o -path ./work -prune -o \
    -name manifest.sha256 -prune -o -type f -print0 | \
    sort -z | xargs -0 shasum -a 256 > manifest.sha256
)

shasum -a 256 "${FINAL_KMODEL}" "${FINAL_ONNX}" "${PTQ_NPZ}"
