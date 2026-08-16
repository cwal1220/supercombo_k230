# K230 supercombo model

This directory is the cleaned final package for the K230 `supercombo.kmodel`
currently installed in `openpilot_c2_k230`.

## Contents

- `supercombo.kmodel`
  - K230 model with both the medmodel and sbigmodel image towers active.
- `onnx/supercombo_base.onnx`
  - Final rewritten ONNX used for compile.
- `ptq/supercombo_calib.npz`
  - PTQ data used for the final model.
- `ptq/supercombo_calib_segments.txt`
  - Ordered segment list used to build the balanced 77-sample PTQ set (3 degenerate frames removed from the original 80).
- `ptq/*metadata*.json`
  - PTQ sample and segment metadata.
- `verification/full_big_input_board_verification_summary.json`
  - Current full dual-input model build and board timing summary.
- `verification/openpilot_c2_master_comparison.json`
  - Exact reference commit, preprocessing compatibility, FP32 graph equivalence,
    and 80-sample K230 quantization comparison against `openpilot_c2` master.
- `manifest.sha256`
  - SHA-256 manifest for preserved regular files.

## Final hashes

```text
168e6a15886301250b9b554a04156ad0a399a04f5bd93b97ea0a5c72c0eba05d  supercombo.kmodel
1c25f80c26c2b3dd162d837be5bd6e131e1626e087183a4fbb6d0d005f5c6e82  onnx/supercombo_base.onnx
af8e2c4548fab5b482f32244041b3b928b2dccb3a9644e290c5495b69846f42e  onnx/supercombo_uint8.onnx
69656304316814b7b98ab4af26e31482f10d212478cee18165d80160291cbe2e  ptq/supercombo_calib.npz
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

Both image inputs are active. The K230 runtime samples the same `1280x720` NV12
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
method, and the balanced 77-sample PTQ set recorded in `ptq/`.

## K230 graph rewrites

`rewrite_supercombo_onnx.py` applies the same nncase-compatible rewrites used by
the previous K230 artifact:

- Split two `Gemm -> Split` patterns into per-output Gemm nodes.
- Split the final plan head and calculate plan-probability deltas after Gemm.
- Insert an identity depthwise 1x1 Conv before `Elu_223`.
- Keep the original GRU update expression; `--gru-update` is intentionally not
  used.

## Rebuild

From `/Users/chan/Documents/k230/supercombo_k230`:

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
for this Mac/Colima setup. The Docker image is reproducible from
`tools/model/docker/Dockerfile` (nncase 2.11.0 + nncase-kpu + .NET 7 runtime):

```sh
docker build --platform linux/amd64 -t supercombo-nncase-k230:2.11.0-sdk \
  tools/model/docker
```

The image inputs are uint8 at the kmodel boundary: after the base rewrite,
`tools/model/retype_image_inputs_uint8.py --op dequantize` retypes
`input_imgs`/`big_input_imgs` to uint8 with a `DequantizeLinear(scale=1)` that
nncase folds into the KPU input stage. The 0..255 integer pixels are
bit-identical to what the former float32 inputs carried, the runtime warp skips
the int-to-float convert and writes a quarter of the bytes, and the NPU input
stage gets cheaper (run 29.1 -> 28.0 ms). Board equivalence against the float
kmodel over a held-out continuous 100-frame drive: mean output diff 0.008
(1/7 of the accepted quantization error), flat recurrent divergence, plan
argmax 97/100 with all three mismatches inside one int16 quantization step.

## Manual command outline

Create the no-GRU rewritten full ONNX:

```sh
docker run --rm --platform linux/amd64 \
  -v /Users/chan/Documents/k230/supercombo_k230:/work \
  -v /Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx:/source.onnx:ro \
  -w /work \
  supercombo-nncase-k230:2.11.0-sdk \
  python -u tools/model/rewrite_supercombo_onnx.py \
    --in-model /source.onnx \
    --out-model /work/models/onnx/supercombo_base.onnx \
    --gemm-split \
    --split-plan-output \
    --plan-prob-delta-after-gemm \
    --identity-dw-before-elu \
    --identity-dw-before-elu-names Elu_223
```

Retype the image inputs to uint8:

```sh
docker run --rm --platform linux/amd64 \
  -v /Users/chan/Documents/k230/supercombo_k230:/work \
  -w /work \
  supercombo-nncase-k230:2.11.0-sdk \
  python -u tools/model/retype_image_inputs_uint8.py \
    --in-model /work/models/onnx/supercombo_base.onnx \
    --out-model /work/models/onnx/supercombo_uint8.onnx \
    --op dequantize
```

Optional PTQ regeneration:

```sh
"${PYTHON_BIN}" \
  tools/model/make_supercombo_calibration.py \
  --data-root /Users/chan/Documents/openpilot_c2/device_collected \
  --model /Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx \
  --out models/ptq/supercombo_calib.npz \
  --samples 77 \
  --warmup 5 \
  --per-segment 1 \
  --max-segments 80 \
  --warp-mode openpilot \
  --segment-list models/ptq/supercombo_calib_segments.txt \
  --metadata-out models/ptq/supercombo_calib_metadata.json
```

Compile with nncase:

```sh
docker run --rm --platform linux/amd64 \
  -e DOTNET_EnableWriteXorExecute=0 \
  -e COMPlus_EnableWriteXorExecute=0 \
  -e COMPlus_TieredCompilation=0 \
  -e COMPlus_ReadyToRun=0 \
  -e COMPlus_ZapDisable=1 \
  -v /Users/chan/Documents/k230/supercombo_k230:/work \
  -v /Users/chan/Documents/openpilot_c2:/openpilot_c2 \
  -w /work \
  supercombo-nncase-k230:2.11.0-sdk \
  python -u tools/model/compile_supercombo_nncase.py \
    --model /work/models/onnx/supercombo_uint8.onnx \
    --out /work/models/work/supercombo-full.kmodel \
    --dump-dir /work/models/work/nncase_dump \
    --target k230 \
    --ptq \
    --calib-npz /work/models/ptq/supercombo_calib.npz \
    --samples 77 \
    --calibrate-method NoClip \
    --quant-type int16 \
    --w-quant-type uint8 \
    --no-dump-ir \
    --no-dump-asm
mv models/work/supercombo-full.kmodel models/supercombo.kmodel
```
