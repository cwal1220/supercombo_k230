# Model rewrite and compile notes

This directory keeps the scripts used to reproduce the K230 target model from
the original openpilot `supercombo.onnx`.

## Final artifact

- `models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel`

The original ONNX is available in the openpilot fork:

- [supercombo.onnx](https://github.com/cwal1220/openpilot_c2/blob/master/selfdrive/modeld/models/supercombo.onnx)

The rewritten ONNX is an intermediate generated artifact and is ignored by Git.
Regenerate it with the command below when recompiling the kmodel.

## What changed in the ONNX graph

Starting from the original openpilot `selfdrive/modeld/models/supercombo.onnx`,
the final graph applies these mathematically equivalent rewrites:

1. `--gemm-split`
   - Replaces selected `Gemm -> Split` heads with multiple smaller `Gemm` nodes.
   - This avoids a large post-Gemm split pattern that was awkward for nncase/K230.
2. `--gru-update`
   - Rewrites the GRU update pattern into K230-friendlier primitive arithmetic.
3. `--split-plan-output --plan-prob-delta-after-gemm`
   - Splits the final plan output into explicit K230 outputs.
   - Emits plan probability delta outputs used by the runtime/parser.
4. `--identity-dw-before-elu --identity-dw-before-elu-names Elu_223`
   - Inserts a depthwise 1x1 Conv with all-one weights and zero bias immediately
     before `Elu_223`.
   - This is an identity transform, so it is mathematically equivalent to the
     original graph.
   - On K230/nncase this changed the surrounding layout/context enough for the
     `Elu_223` region to stay on the native GNNE activation path instead of
     falling back through an `Exp` decomposition.

The final inspected nncase dump had no `Unary(Exp)` fallback for this ELU region;
`Elu_223` lowered through a native `GNNEActivation` path.

## Recreate the ONNX

From the parent openpilot workspace where the original model exists:

```sh
python3 model_tools/rewrite_supercombo_onnx.py \
  --in-model selfdrive/modeld/models/supercombo.onnx \
  --out-model models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta.onnx \
  --gemm-split \
  --gru-update \
  --split-plan-output \
  --plan-prob-delta-after-gemm \
  --identity-dw-before-elu \
  --identity-dw-before-elu-names Elu_223
```

Expected script output includes non-zero rewrite counts for `gemm_split`,
`gru_update`, `split_plan_output`, and `identity_dw_before_elu`.

## Build calibration data from real driving logs

The checked-in kmodel name uses `real80`, meaning PTQ calibration was generated
from 80 real driving samples. The calibration script depends on the broader
openpilot/model visualization workspace (`cereal`, `model_viz_experiment`,
OpenCV, ffmpeg, onnxruntime), so it is documented here rather than treated as a
self-contained runtime dependency.

Example:

```sh
python3 model_tools/make_supercombo_calibration.py \
  --data-root device_collected \
  --model selfdrive/modeld/models/supercombo.onnx \
  --out out/supercombo_real80_calib.npz \
  --samples 80 \
  --warmup 5 \
  --per-segment 2 \
  --max-segments 50
```

## Compile the kmodel with nncase

Use nncase `2.11.0`, matching `fetch_nncase_runtime.sh` in the runtime tree.
The final target uses INT16 activation quantization, UINT8 weight quantization,
80 real calibration samples, and `NoClip` calibration.

```sh
python3 model_tools/compile_supercombo_nncase.py \
  --model models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta.onnx \
  --out models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel \
  --target k230 \
  --ptq \
  --calib-npz out/supercombo_real80_calib.npz \
  --samples 80 \
  --calibrate-method NoClip \
  --quant-type int16 \
  --w-quant-type uint8 \
  --no-dump-ir \
  --no-dump-asm
```

The runtime app loads the generated kmodel from `models/` and still performs the
live `NV12 512x256 -> YUV6 float` input preparation itself.
