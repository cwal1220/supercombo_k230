#!/usr/bin/env bash
# openpilot v0.9.4 supercombo을 K230 kmodel로 빌드한다.
#
#   1. 배포 ONNX(fp16)를 받아 fp32로 정규화한다.
#   2. 이미지 입력을 uint8 + DequantizeLinear(scale=1)로 바꾼다. 값은 그대로고
#      런타임 워프의 float 변환과 4배 쓰기 대역폭이 사라진다.
#   3. 실주행 캘리브레이션 npz로 PTQ(int16 활성 / uint8 가중치) 컴파일한다.
#
# PTQ npz는 tools/model/make_calibration.py가 녹화 주행에서 만든다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${REPO_DIR}/models"
DOCKER_IMAGE="${DOCKER_IMAGE:-supercombo-nncase-k230:2.11.0-sdk}"
OPENPILOT_TAG="${OPENPILOT_TAG:-v0.9.4}"
SOURCE_ONNX="${SOURCE_ONNX:-${MODEL_DIR}/work/supercombo-${OPENPILOT_TAG}.onnx}"
PTQ_NPZ="${PTQ_NPZ:-${MODEL_DIR}/ptq/supercombo_calib.npz}"
PTQ_SAMPLES="${PTQ_SAMPLES:-60}"

FP32_ONNX="${MODEL_DIR}/onnx/supercombo_base.onnx"
FINAL_ONNX="${MODEL_DIR}/onnx/supercombo_uint8.onnx"
FINAL_KMODEL="${MODEL_DIR}/supercombo.kmodel"
COMPILED_KMODEL="${MODEL_DIR}/work/supercombo-full.kmodel"

mkdir -p "${MODEL_DIR}/onnx" "${MODEL_DIR}/ptq" "${MODEL_DIR}/work/nncase_dump"

if [[ ! -f "${SOURCE_ONNX}" ]]; then
  echo "fetching openpilot ${OPENPILOT_TAG} supercombo.onnx"
  curl -fsSL -o "${SOURCE_ONNX}" \
    "https://github.com/commaai/openpilot/raw/${OPENPILOT_TAG}/selfdrive/modeld/models/supercombo.onnx"
fi

if [[ ! -f "${PTQ_NPZ}" ]]; then
  echo "missing PTQ calibration: ${PTQ_NPZ}" >&2
  echo "build it with tools/model/make_calibration.py" >&2
  exit 1
fi

docker_run() {
  docker run --rm --platform linux/amd64 \
    -e DOTNET_EnableWriteXorExecute=0 \
    -e COMPlus_EnableWriteXorExecute=0 \
    -e COMPlus_TieredCompilation=0 \
    -e COMPlus_ReadyToRun=0 \
    -e COMPlus_ZapDisable=1 \
    -v "${REPO_DIR}:/work" \
    -w /work \
    "${DOCKER_IMAGE}" "$@"
}

docker_run python -u tools/model/sanitize_onnx_for_nncase.py \
  --in-model "/work/${SOURCE_ONNX#${REPO_DIR}/}" \
  --out-model "/work/${FP32_ONNX#${REPO_DIR}/}" \
  --float32 --dedupe-opsets --name-empty-nodes --remove-reshape-allowzero --check

docker_run python -u tools/model/retype_image_inputs_uint8.py \
  --in-model "/work/${FP32_ONNX#${REPO_DIR}/}" \
  --out-model "/work/${FINAL_ONNX#${REPO_DIR}/}" \
  --op dequantize

docker_run python -u tools/model/compile_supercombo_nncase.py \
  --model "/work/${FINAL_ONNX#${REPO_DIR}/}" \
  --out "/work/${COMPILED_KMODEL#${REPO_DIR}/}" \
  --dump-dir /work/models/work/nncase_dump \
  --target k230 \
  --ptq \
  --calib-npz "/work/${PTQ_NPZ#${REPO_DIR}/}" \
  --samples "${PTQ_SAMPLES}" \
  --calibrate-method NoClip \
  --quant-type int16 \
  --w-quant-type uint8 \
  --no-dump-ir \
  --no-dump-asm

mv "${COMPILED_KMODEL}" "${FINAL_KMODEL}"

(
  cd "${MODEL_DIR}"
  find . -path ./work -prune -o \
    -name manifest.sha256 -prune -o -type f -print0 | \
    sort -z | xargs -0 shasum -a 256 > manifest.sha256
)
echo "built ${FINAL_KMODEL}"
