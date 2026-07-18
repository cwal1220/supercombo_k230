# Final K230 supercombo model

This directory is the cleaned final package for the K230 `supercombo.kmodel`
currently installed in `openpilot_c2_k230`.

## Contents

- `model/supercombo.kmodel`
  - Drop-in model name for `openpilot_c2_k230/selfdrive/modeld/models/`.
- `model/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero_int16a_uint8w_balanced80_noclip.kmodel`
  - Symlink to `model/supercombo.kmodel`, with the full build name.
- `onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx`
  - Rewritten ONNX before big-image tower pruning.
- `onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero.onnx`
  - Final ONNX used for compile.
- `ptq/supercombo_balanced80_calib.npz`
  - PTQ data used for the final model.
- `ptq/balanced80_segments.txt`
  - Ordered segment list used to build the balanced 80-sample PTQ set.
- `ptq/*metadata*.json`
  - PTQ sample and segment metadata.
- `verification/balanced80_quant_verification_summary.json`
  - Board timing and quantization sanity summary.
- `no_big_input/`
  - 4-input variant that removes the `big_input_imgs` dummy input itself.
  - This requires a runtime input-order change and is not a drop-in replacement
    for the current 5-input K230 runner.
- `MANIFEST.sha256`
  - SHA-256 manifest for preserved regular files.

## Final hashes

```text
b6462b3374361ed0a0d65127d00da2af48da65b0b02d23ae096ec43d07f34822  model/supercombo.kmodel
6361d76e52e30351424c3fe0866fba33fd0112297be94e019a0ec8a67467203c  onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero.onnx
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

`big_input_imgs` is preserved as an ONNX/kmodel input for compatibility, but the
big-image tower is folded out of the final graph with a zero-big-image feature.
The current runtime may keep writing zeros to input index 1.

## Rebuild

From `/Users/chan/Documents/supercombo_k230`:

```sh
./final_k230_model/build_final_model.sh
```

By default, the script reuses the preserved PTQ NPZ in this directory. To rebuild
the PTQ NPZ from the saved balanced segment list first:

```sh
REGEN_PTQ=1 ./final_k230_model/build_final_model.sh
```

Required local inputs:

- `/Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx`
- `/Users/chan/Documents/openpilot_c2/device_collected`
- `/Users/chan/Documents/openpilot_c2/model_viz_experiment/.venv/bin/python`
- Docker image `supercombo-nncase-k230:2.11.0-sdk`

The compile runs under `linux/amd64` with the Rosetta/.NET mitigation flags used
for this Mac/Colima setup.

## Manual command outline

Create the no-GRU rewritten ONNX:

```sh
/Users/chan/Documents/openpilot_c2/model_viz_experiment/.venv/bin/python \
  model_tools/rewrite_supercombo_onnx.py \
  --in-model /Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx \
  --out-model final_k230_model/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx \
  --gemm-split \
  --split-plan-output \
  --plan-prob-delta-after-gemm \
  --identity-dw-before-elu \
  --identity-dw-before-elu-names Elu_223
```

Remove the unused big-image tower while keeping the input slot:

```sh
/Users/chan/Documents/openpilot_c2/model_viz_experiment/.venv/bin/python \
  model_tools/remove_supercombo_big_input.py \
  --input final_k230_model/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx \
  --output final_k230_model/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_keepinput_foldzero.onnx \
  --mode fold-zero \
  --keep-big-input
```

Optional PTQ regeneration:

```sh
/Users/chan/Documents/openpilot_c2/model_viz_experiment/.venv/bin/python \
  model_tools/make_supercombo_calibration.py \
  --data-root /Users/chan/Documents/openpilot_c2/device_collected \
  --model /Users/chan/Documents/openpilot_c2/selfdrive/modeld/models/supercombo.onnx \
  --out final_k230_model/ptq/supercombo_balanced80_calib.npz \
  --samples 80 \
  --warmup 5 \
  --per-segment 1 \
  --max-segments 80 \
  --warp-mode openpilot \
  --segment-list final_k230_model/ptq/balanced80_segments.txt \
  --metadata-out final_k230_model/ptq/supercombo_balanced80_calib_metadata.json
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
```
