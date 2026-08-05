#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

mode="${1:-install}"
case "${mode}" in
  onnx|kmodel|install) ;;
  *) echo "Usage: $0 [onnx|kmodel|install]" >&2; exit 2 ;;
esac

source_onnx="${SOURCE_ONNX:-models/source/driving_supercombo.onnx}"
calibration_npz="${CALIBRATION_NPZ:-models/ptq/driving_supercombo_full6_288.npz}"
lowering_tools="${K230_LOWERING_TOOLS:-tools/model/modern_lowering}"
nncase_image="${NNCASE_IMAGE:-supercombo-nncase-k230:2.11.0-sdk-flat}"
nncase_python="${NNCASE_PYTHON:-/opt/nncase-venv/bin/python}"
nncase_plugin_path="${NNCASE_PLUGIN_PATH:-/opt/nncase-venv/lib/python3.10/site-packages/nncase/modules/kpu}"
work_dir="${MODEL_WORK_DIR:-models/work/modern}"

expected_source_sha="659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b"
expected_calibration_sha="aa13949b1a99128de65288747e6662420fd3eac209e2d55c99b0b30c0c18d8cb"
expected_lowered_sha="a5f92c39da3db830d9adbb36005aa35d1dc5ce8569fb4c90f8a2715af092e675"
expected_kmodel_sha="49ed812db587d48c6dfdcc26d8e42d2e69a5d0717527bb3dd74dfe4f088bfed1"

lowered_onnx="${work_dir}/driving_supercombo.k230.onnx"
compiled_kmodel="${work_dir}/driving_supercombo.k230.int16a_uint8w_noclip.kmodel"
compile_metadata="${work_dir}/compile_metadata.json"

sha256() {
  shasum -a 256 "$1" | awk '{print $1}'
}

require_sha() {
  local path="$1"
  local expected="$2"
  if [[ ! -f "${path}" ]]; then
    echo "Missing required input: ${path}" >&2
    exit 1
  fi
  local actual
  actual="$(sha256 "${path}")"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "SHA-256 mismatch for ${path}: expected ${expected}, got ${actual}" >&2
    exit 1
  fi
}

lowering_scripts=(
  sanitize_onnx_for_nncase.py
  rewrite_full_front_dequant_for_k230.py
  rewrite_fixed_ops_for_k230.py
  rewrite_main_full_attention_for_k230.py
  restore_full6_aux_float16_inputs.py
)

require_sha "${source_onnx}" "${expected_source_sha}"
for script in "${lowering_scripts[@]}"; do
  if [[ ! -f "${lowering_tools}/${script}" ]]; then
    echo "Missing lowering tool: ${lowering_tools}/${script}" >&2
    echo "Set K230_LOWERING_TOOLS to the reviewed modern-model lowering toolkit." >&2
    exit 1
  fi
done
mkdir -p "${work_dir}"

docker run --rm --platform linux/amd64 \
  -e DOTNET_EnableWriteXorExecute=0 \
  -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 \
  -e COMPlus_ReadyToRun=0 \
  -e COMPlus_ZapDisable=1 \
  -e NNCASE_PYTHON="${nncase_python}" \
  -e NNCASE_PLUGIN_PATH="${nncase_plugin_path}" \
  -v "${repo_dir}:/work" \
  -v "$(cd "$(dirname "${source_onnx}")" && pwd)/$(basename "${source_onnx}"):/source.onnx:ro" \
  -v "$(cd "${lowering_tools}" && pwd):/lowering:ro" \
  -w /work \
  "${nncase_image}" bash -lc '
set -euo pipefail
out=/work/'"${work_dir}"'
"$NNCASE_PYTHON" /lowering/sanitize_onnx_for_nncase.py \
  --in-model /source.onnx --out-model "$out/01_f32.onnx" \
  --float32 --dedupe-opsets --name-empty-nodes --remove-reshape-allowzero --check
"$NNCASE_PYTHON" /lowering/rewrite_full_front_dequant_for_k230.py \
  --in-model "$out/01_f32.onnx" --out-model "$out/02_frontdq.onnx" --check
"$NNCASE_PYTHON" /lowering/rewrite_fixed_ops_for_k230.py \
  --in-model "$out/02_frontdq.onnx" --out-model "$out/03_fixed_ops.onnx" --check
"$NNCASE_PYTHON" /lowering/rewrite_main_full_attention_for_k230.py \
  --in-model "$out/03_fixed_ops.onnx" \
  --out-model "$out/04_attention_gelu_layernorm.onnx" \
  --lower-gelu --lower-layernorm --check
"$NNCASE_PYTHON" /lowering/restore_full6_aux_float16_inputs.py \
  --in-model "$out/04_attention_gelu_layernorm.onnx" \
  --out-model "$out/driving_supercombo.k230.onnx" --check
'

require_sha "${lowered_onnx}" "${expected_lowered_sha}"
echo "Lowered ONNX: ${lowered_onnx} (${expected_lowered_sha})"
[[ "${mode}" == "onnx" ]] && exit 0

require_sha "${calibration_npz}" "${expected_calibration_sha}"
docker run --rm --platform linux/amd64 \
  -e DOTNET_EnableWriteXorExecute=0 \
  -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 \
  -e COMPlus_ReadyToRun=0 \
  -e COMPlus_ZapDisable=1 \
  -e NNCASE_PLUGIN_PATH="${nncase_plugin_path}" \
  -v "${repo_dir}:/work" \
  -v "$(cd "$(dirname "${calibration_npz}")" && pwd)/$(basename "${calibration_npz}"):/calibration.npz:ro" \
  -w /work \
  "${nncase_image}" "${nncase_python}" tools/model/compile_supercombo_nncase.py \
    --model "/work/${lowered_onnx}" \
    --out "/work/${compiled_kmodel}" \
    --dump-dir /tmp/driving_supercombo_nncase_dump \
    --target k230 --ptq --samples 288 \
    --calib-npz /calibration.npz \
    --calibrate-method NoClip --quant-type int16 --w-quant-type uint8 \
    --finetune-weights-method NoFineTuneWeights \
    --no-dump-ir --no-dump-asm \
    --metadata-json "/work/${compile_metadata}"

require_sha "${compiled_kmodel}" "${expected_kmodel_sha}"
echo "KModel: ${compiled_kmodel} (${expected_kmodel_sha})"
[[ "${mode}" == "kmodel" ]] && exit 0

install -m 0644 "${compiled_kmodel}" models/supercombo.kmodel
install -m 0644 "${compile_metadata}" models/verification/modern_model_compile_metadata.json
(
  cd models
  find . -path ./variants -prune -o -path ./work -prune -o \
    -name manifest.sha256 -prune -o -type f -print0 | \
    sort -z | xargs -0 shasum -a 256 > manifest.sha256
)
echo "Installed models/supercombo.kmodel and refreshed models/manifest.sha256"
