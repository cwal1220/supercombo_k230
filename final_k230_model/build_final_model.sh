#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${PKG_DIR}/.." && pwd)"
OPENPILOT_DIR="${OPENPILOT_DIR:-/Users/chan/Documents/openpilot_c2}"
PYTHON_BIN="${PYTHON_BIN:-${OPENPILOT_DIR}/model_viz_experiment/.venv/bin/python}"
DOCKER_IMAGE="${DOCKER_IMAGE:-supercombo-nncase-k230:2.11.0-sdk}"
SOURCE_ONNX="${SOURCE_ONNX:-${OPENPILOT_DIR}/selfdrive/modeld/models/supercombo.onnx}"
DATA_ROOT="${DATA_ROOT:-${OPENPILOT_DIR}/device_collected}"

INTERMEDIATE_ONNX="${PKG_DIR}/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx"
FINAL_ONNX="${PKG_DIR}/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero.onnx"
PTQ_NPZ="${PKG_DIR}/ptq/supercombo_balanced80_calib.npz"
FINAL_KMODEL="${PKG_DIR}/model/supercombo.kmodel"
FULLNAME_KMODEL="${PKG_DIR}/model/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero_int16a_uint8w_balanced80_noclip.kmodel"

mkdir -p "${PKG_DIR}/onnx" "${PKG_DIR}/ptq" "${PKG_DIR}/model" "${PKG_DIR}/work/nncase_dump"

"${PYTHON_BIN}" "${REPO_DIR}/model_tools/rewrite_supercombo_onnx.py" \
  --in-model "${SOURCE_ONNX}" \
  --out-model "${INTERMEDIATE_ONNX}" \
  --gemm-split \
  --split-plan-output \
  --plan-prob-delta-after-gemm \
  --identity-dw-before-elu \
  --identity-dw-before-elu-names Elu_223

"${PYTHON_BIN}" "${REPO_DIR}/model_tools/remove_supercombo_big_input.py" \
  --input "${INTERMEDIATE_ONNX}" \
  --output "${FINAL_ONNX}" \
  --mode fold-zero \
  --keep-big-input

if [[ "${REGEN_PTQ:-0}" == "1" ]]; then
  "${PYTHON_BIN}" "${REPO_DIR}/model_tools/make_supercombo_calibration.py" \
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
  python -u model_tools/compile_supercombo_nncase.py \
    --model /work/final_k230_model/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero.onnx \
    --out /work/final_k230_model/model/supercombo.kmodel \
    --dump-dir /work/final_k230_model/work/nncase_dump \
    --target k230 \
    --ptq \
    --calib-npz /work/final_k230_model/ptq/supercombo_balanced80_calib.npz \
    --samples 80 \
    --calibrate-method NoClip \
    --quant-type int16 \
    --w-quant-type uint8 \
    --no-dump-ir \
    --no-dump-asm

ln -sf supercombo.kmodel "${FULLNAME_KMODEL}"

(
  cd "${PKG_DIR}"
  find model onnx ptq verification -type f -print0 | sort -z | xargs -0 shasum -a 256 > MANIFEST.sha256
)

shasum -a 256 "${FINAL_KMODEL}" "${FINAL_ONNX}" "${PTQ_NPZ}"
