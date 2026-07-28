# K230 supercombo model

This directory is the cleaned final package for the K230 `supercombo.kmodel`
currently installed in `openpilot_c2_k230`.

## Contents

- `supercombo.kmodel`
  - K230 model with both the medmodel and sbigmodel image towers active.
- `onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx`
  - Final rewritten ONNX used for compile.
- `ptq/supercombo_balanced80_calib.npz`
  - PTQ data used for the final model.
- `ptq/balanced80_segments.txt`
  - Ordered segment list used to build the balanced 80-sample PTQ set.
- `ptq/*metadata*.json`
  - PTQ sample and segment metadata.
- `verification/balanced80_quant_verification_summary.json`
  - Historical folded-big model quantization verification.
- `verification/full_big_input_board_verification_summary.json`
  - Current full dual-input model build and board timing summary.
- `verification/openpilot_c2_master_comparison.json`
  - Exact reference commit, preprocessing compatibility, FP32 graph equivalence,
    and 80-sample K230 quantization comparison against `openpilot_c2` master.
- `variants/no_big_input/`
  - Historical 4-input variant that removes `big_input_imgs`.
  - It is not compatible with the current 5-input dual-warp runner.
- `manifest.sha256`
  - SHA-256 manifest for preserved regular files.

## Final hashes

```text
38f771be302871244c7daf74f57c1a7471c7b3af2bdfbea09bdf65402c9e7b4d  supercombo.kmodel
1c25f80c26c2b3dd162d837be5bd6e131e1626e087183a4fbb6d0d005f5c6e82  onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx
0eafd30925b807898f61663cdb7b3fb289c7c4a5100444870bdf97ee5aff8bc1  ptq/supercombo_balanced80_calib.npz
```

## Input contract

The model keeps the same 5-input runtime contract used by the current K230
runner:

```text
0 input_imgs          [1, 12, 128, 256]
1 big_input_imgs      [1, 12, 128, 256]
2 desire              [1, 8]
3 traffic_convention  [1, 2]
4 initial_state       [1, 512]
```

Both image inputs are active. The K230 runtime samples the same `640x360` NV12
frame through two calibrated virtual cameras and keeps independent two-frame
histories:

- `input_imgs`: medmodel, focal length 910, center `(256, 47.6)`.
- `big_input_imgs`: sbigmodel, focal length 455, center `(256, 151.8)`.

This matches the single-camera C2 path: it reuses the same source buffer, but
does not copy the already-warped medmodel tensor into the big input.

The runtime uses compact fixed-point sampling tables and a C908 RVV kernel.
Each warped current frame is written directly into its nncase image tensor,
then advanced to the previous-frame half after inference. Host and board
verification covers legacy-LUT equivalence, direct history layout, scalar/RVV
preprocessing, and full-model scalar/RVV raw outputs; all comparisons are
bit-exact within the K230 implementation. This does not claim pixel or model
output identity with the original C2 OpenCL/FP32 path. Detailed hashes and timing are in
`verification/full_big_input_board_verification_summary.json`.

The comparison with `openpilot_c2` master is recorded separately in
`verification/openpilot_c2_master_comparison.json`. The fixed12 K230 warp is
numerically close to, but not bit-exact with, the original OpenCL interpolation.
The FP32 ONNX rewrite preserves every non-plan-logit output exactly and preserves
the plan argmax. The INT16/UINT8 Kmodel is quantized; it selected the same plan
hypothesis as the original FP32 ONNX in 60 of 80 saved PTQ samples.

The model uses INT16 activations, UINT8 weights, the `NoClip` calibration
method, and the balanced 80-sample PTQ set recorded in `ptq/`.

## K230 graph rewrites

`rewrite_supercombo_onnx.py` applies the same nncase-compatible rewrites used by
the previous K230 artifact:

- Split two `Gemm -> Split` patterns into per-output Gemm nodes.
- Split the final plan head and calculate plan-probability deltas after Gemm.
- Insert an identity depthwise 1x1 Conv before `Elu_223`.
- Keep the original GRU update expression; `--gru-update` is intentionally not
  used.

## Rebuild

From `/Users/chan/Documents/supercombo_k230`:

```sh
scripts/build_supercombo_model.sh
```

By default, the script reuses the preserved PTQ NPZ in this directory. To rebuild
the PTQ NPZ from the saved balanced segment list first:

```sh
REGEN_PTQ=1 scripts/build_supercombo_model.sh
```

Required local inputs:

- `/Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx`
- Docker image `supercombo-nncase-k230:2.11.0-sdk`

Regenerating PTQ data additionally requires the saved driving data and a Python
environment supplied through `PYTHON_BIN` with OpenCV, ONNX Runtime, and the
script dependencies installed.

The compile runs under `linux/amd64` with the Rosetta/.NET mitigation flags used
for this Mac/Colima setup.

## Manual command outline

Create the no-GRU rewritten full ONNX:

```sh
docker run --rm --platform linux/amd64 \
  -v /Users/chan/Documents/supercombo_k230:/work \
  -v /Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx:/source.onnx:ro \
  -w /work \
  supercombo-nncase-k230:2.11.0-sdk \
  python -u tools/model/rewrite_supercombo_onnx.py \
    --in-model /source.onnx \
    --out-model /work/models/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx \
    --gemm-split \
    --split-plan-output \
    --plan-prob-delta-after-gemm \
    --identity-dw-before-elu \
    --identity-dw-before-elu-names Elu_223
```

Optional PTQ regeneration:

```sh
"${PYTHON_BIN}" \
  tools/model/make_supercombo_calibration.py \
  --data-root /Users/chan/Documents/openpilot_c2/device_collected \
  --model /Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx \
  --out models/ptq/supercombo_balanced80_calib.npz \
  --samples 80 \
  --warmup 5 \
  --per-segment 1 \
  --max-segments 80 \
  --warp-mode openpilot \
  --segment-list models/ptq/balanced80_segments.txt \
  --metadata-out models/ptq/supercombo_balanced80_calib_metadata.json
```

Compile with nncase:

```sh
docker run --rm --platform linux/amd64 \
  -e DOTNET_EnableWriteXorExecute=0 \
  -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 \
  -e COMPlus_ReadyToRun=0 \
  -e COMPlus_ZapDisable=1 \
  -v /Users/chan/Documents/supercombo_k230:/work \
  -v /Users/chan/Documents/openpilot_c2:/openpilot_c2 \
  -w /work \
  supercombo-nncase-k230:2.11.0-sdk \
  python -u tools/model/compile_supercombo_nncase.py \
    --model /work/models/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx \
    --out /work/models/work/supercombo-full.kmodel \
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
mv models/work/supercombo-full.kmodel models/supercombo.kmodel
```
