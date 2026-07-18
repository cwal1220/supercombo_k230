# No-big-input K230 supercombo variant

This directory contains a 4-input variant of the final balanced80 K230
`supercombo` model.

It removes the `big_input_imgs` dummy input itself. This is different from the
main final model in `../model/supercombo.kmodel`, which keeps `big_input_imgs`
as an unused compatibility input.

## Compatibility

This kmodel is not drop-in compatible with the current 5-input K230 runtime.
The runtime must stop writing input index 1 as `big_input_imgs` and use this
input order instead:

```text
0 input_imgs          [1, 12, 128, 256]
1 desire              [1, 8]
2 traffic_convention  [1, 2]
3 initial_state       [1, 512]
```

Outputs remain 22.

## Artifacts

- `onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_dropinput_foldzero.onnx`
  - Final 4-input ONNX.
- `model/supercombo_no_big_input_int16a_uint8w_balanced80_noclip.kmodel`
  - Final 4-input K230 kmodel.
- `model/supercombo_no_big_input.kmodel`
  - Symlink to the final kmodel.
- `MANIFEST.sha256`
  - SHA-256 manifest for regular files in this variant.

## Hashes

```text
af36582f1909deaec2d9bd82f506cfc3deb0f4edf31b299046ea49444d31d589  model/supercombo_no_big_input_int16a_uint8w_balanced80_noclip.kmodel
6f510d686d4dea830a6fa1fc04cfac16c9c294621077a533fb82561ddcb6fcb7  onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_dropinput_foldzero.onnx
0eafd30925b807898f61663cdb7b3fb289c7c4a5100444870bdf97ee5aff8bc1  ../ptq/supercombo_balanced80_calib.npz
```

## Build Commands

Create the 4-input ONNX from the no-GRU intermediate ONNX:

```sh
/Users/chan/Documents/openpilot_c2/model_viz_experiment/.venv/bin/python \
  model_tools/remove_supercombo_big_input.py \
  --input final_k230_model/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx \
  --output final_k230_model/no_big_input/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_dropinput_foldzero.onnx \
  --mode fold-zero
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
    --model /work/final_k230_model/no_big_input/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta_nobig_dropinput_foldzero.onnx \
    --out /work/final_k230_model/no_big_input/model/supercombo_no_big_input_int16a_uint8w_balanced80_noclip.kmodel \
    --dump-dir /work/final_k230_model/no_big_input/work/nncase_dump \
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

Observed compile time on this Mac/Colima path was `567.64s`; output size was
`22.45 MiB`.
