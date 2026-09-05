#!/usr/bin/env bash
# openpilot v0.9.4 supercombo을 K230 kmodel로 빌드한다.
#
#   1. 배포 ONNX(fp16)를 받아 fp32로 정규화한다.
#   2. Conv/Gemm 가중치를 nncase 채널별 uint8 격자로 미리 반올림하고, 보정 입력으로
#      레이어 출력 평균 이동을 bias에서 상쇄한다(tools/model/prequant_bias_correct.py).
#   3. 이미지 입력을 uint8 + DequantizeLinear(scale=1)로 바꾼다. 값은 그대로고
#      런타임 워프의 float 변환과 4배 쓰기 대역폭이 사라진다.
#   4. 실주행 캘리브레이션 npz(60 + K230 120)로 PTQ(int16 활성 / uint8 가중치) 컴파일한다.
#
# PTQ npz는 tools/model/make_calibration.py가 녹화 주행에서 만든다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${REPO_DIR}/models"
DOCKER_IMAGE="${DOCKER_IMAGE:-supercombo-nncase-k230:2.11.0-sdk}"
OPENPILOT_TAG="${OPENPILOT_TAG:-v0.9.4}"
SOURCE_ONNX="${SOURCE_ONNX:-${MODEL_DIR}/work/supercombo-${OPENPILOT_TAG}.onnx}"
PTQ_BASE_NPZ="${MODEL_DIR}/ptq/supercombo_calib.npz"
PTQ_K230_NPZ="${MODEL_DIR}/ptq/supercombo_calib_k230_120.npz"
PTQ_NPZ="${MODEL_DIR}/work/supercombo_calib_mixed180.npz"
QUANT_SCHEME="${MODEL_DIR}/ptq/supercombo_quant_scheme_bychannel.json"
# bias correction은 호스트 ONNX Runtime으로 돈다(Docker 이미지에는 onnxruntime이 없다).
PYTHON_BIN="${PYTHON_BIN:-python3}"

FP32_ONNX="${MODEL_DIR}/onnx/supercombo_base.onnx"
BC_ONNX="${MODEL_DIR}/onnx/supercombo_prequant.onnx"
FINAL_ONNX="${MODEL_DIR}/onnx/supercombo_uint8.onnx"
FINAL_KMODEL="${MODEL_DIR}/supercombo.kmodel"
COMPILED_KMODEL="${MODEL_DIR}/work/supercombo-full.kmodel"

mkdir -p "${MODEL_DIR}/onnx" "${MODEL_DIR}/ptq" "${MODEL_DIR}/work/nncase_dump"

if [[ ! -f "${SOURCE_ONNX}" ]]; then
  echo "fetching openpilot ${OPENPILOT_TAG} supercombo.onnx"
  curl -fsSL -o "${SOURCE_ONNX}" \
    "https://github.com/commaai/openpilot/raw/${OPENPILOT_TAG}/selfdrive/modeld/models/supercombo.onnx"
fi

for required in "${PTQ_BASE_NPZ}" "${PTQ_K230_NPZ}" "${QUANT_SCHEME}"; do
  if [[ ! -f "${required}" ]]; then
    echo "missing ${required}" >&2
    echo "PTQ npz: tools/model/make_calibration.py, scheme: compile with --export-quant-scheme --export-weight-range-by-channel" >&2
    exit 1
  fi
done
"${PYTHON_BIN}" -c 'import numpy, onnx, onnxruntime' || {
  echo "PYTHON_BIN=${PYTHON_BIN} needs numpy, onnx and onnxruntime for bias correction" >&2
  exit 1
}

docker_run() {
  docker run --rm --platform linux/amd64 \
    --ulimit core=0 \
    -e DOTNET_EnableWriteXorExecute=0 \
    -e COMPlus_EnableWriteXorExecute=0 \
    -e COMPlus_TieredCompilation=0 \
    -e COMPlus_ReadyToRun=0 \
    -e COMPlus_ZapDisable=1 \
    -e OMP_NUM_THREADS=4 \
    -v "${REPO_DIR}:/work" \
    -w /work \
    "${DOCKER_IMAGE}" "$@"
}

docker_run python -u tools/model/sanitize_onnx_for_nncase.py \
  --in-model "/work/${SOURCE_ONNX#${REPO_DIR}/}" \
  --out-model "/work/${FP32_ONNX#${REPO_DIR}/}" \
  --float32 --dedupe-opsets --name-empty-nodes --remove-reshape-allowzero --check

# 60(초기 3루트) + 120(K230 7루트) = 180장 PTQ 세트
"${PYTHON_BIN}" - "${PTQ_BASE_NPZ}" "${PTQ_K230_NPZ}" "${PTQ_NPZ}" <<'PY'
import sys
import numpy as np
a, b = np.load(sys.argv[1], allow_pickle=False), np.load(sys.argv[2], allow_pickle=False)
merged = {}
for name in ("input_imgs", "big_input_imgs", "desire", "traffic_convention", "nav_features", "features_buffer"):
  values = np.concatenate([a[name], b[name]])
  merged[name] = values.astype(np.uint8 if name.endswith("imgs") else np.float32)
assert len(merged["desire"]) == 180
np.savez_compressed(sys.argv[3], **merged)
PY

"${PYTHON_BIN}" "${REPO_DIR}/tools/model/prequant_bias_correct.py" \
  --in-model "${FP32_ONNX}" \
  --out-model "${BC_ONNX}" \
  --calib-npz "${PTQ_NPZ}" \
  --samples 96 \
  --scheme "${QUANT_SCHEME}"

docker_run python -u tools/model/retype_image_inputs_uint8.py \
  --in-model "/work/${BC_ONNX#${REPO_DIR}/}" \
  --out-model "/work/${FINAL_ONNX#${REPO_DIR}/}" \
  --op dequantize

# nncase 2.11이 큰 보정 세트에서 드물게 시작 직후 SIGSEGV로 죽으므로 최대 3회 재시도한다.
compiled=0
for attempt in 1 2 3; do
  if docker_run python -u tools/model/compile_supercombo_nncase.py \
    --model "/work/${FINAL_ONNX#${REPO_DIR}/}" \
    --out "/work/${COMPILED_KMODEL#${REPO_DIR}/}" \
    --dump-dir /work/models/work/nncase_dump \
    --target k230 \
    --ptq \
    --calib-npz "/work/${PTQ_NPZ#${REPO_DIR}/}" \
    --samples 180 \
    --calibrate-method NoClip \
    --quant-type int16 \
    --w-quant-type uint8 \
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
  find . -path ./work -prune -o -path ./onnx -prune -o -path ./quantization_work -prune -o \
    -name manifest.sha256 -prune -o -type f -print0 | \
    sort -z | xargs -0 shasum -a 256 > manifest.sha256
)
echo "built ${FINAL_KMODEL}"
