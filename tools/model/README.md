# Model conversion tools

The production model is the modern six-input `driving_supercombo` documented in
`../../models/README.md`. Run `../../scripts/build_supercombo_model.sh`; do not
use the legacy five-input rewrite pipeline for deployment.

## Production pipeline

1. `modern_lowering/sanitize_onnx_for_nncase.py`
   normalizes float constants/opsets and removes unsupported attributes.
2. `modern_lowering/rewrite_full_front_dequant_for_k230.py`
   lowers the quantized image frontend into the K230-compatible boundary.
3. `modern_lowering/rewrite_fixed_ops_for_k230.py`
   replaces unsupported fixed operators.
4. `modern_lowering/rewrite_main_full_attention_for_k230.py`
   lowers GELU, layer normalization, and attention support operations.
5. `modern_lowering/restore_full6_aux_float16_inputs.py`
   restores the exact six-input ABI expected by the runtime.
6. `compile_supercombo_nncase.py`
   validates ONNX/NPZ shapes, applies real-sample PTQ, emits compiler metadata,
   and writes the KModel.

The selected profile uses 288 real logging samples, `NoClip`, INT16
activations, UINT8 weights, and no output affine. Source, calibration, lowered
ONNX, and final KModel hashes are checked by the build script.

## Legacy audit tools

The following scripts remain only to reproduce archived five-input artifacts:

- `rewrite_supercombo_onnx.py`
- `remove_supercombo_big_input.py`
- `make_supercombo_calibration.py`

They must not overwrite `models/supercombo.kmodel` in the modern runtime.
