# Model tools

For the quantization experiments, real K230 driving-data loader,
independent/recurrent board comparisons, and candidate build workflow, see
[QUANTIZATION.md](QUANTIZATION.md) and the measured reports
([2026-09-03](../../models/verification/quantization_20260903.md),
[2026-09-04](../../models/verification/quantization_20260904.md)).

This directory contains the scripts used to rewrite openpilot `supercombo.onnx`
and compile a K230 `.kmodel`.

The current cleaned final artifact package is:

- `../../models/`

Use `../../models/README.md` for the exact final ONNX, PTQ, compile
commands, hashes, and input contract. That package is the source of truth for
the current full dual-image-input K230 model with the original GRU update expression.

## Scripts

- `rewrite_supercombo_onnx.py`
  - Applies graph rewrites such as Gemm/Split replacement, plan-output split,
    plan probability delta output, optional GRU update rewrite, the
    identity depthwise Conv before `Elu_223`, and `--tanh-via-sigmoid`
    (Tanh as `2*Sigmoid(2x)-1`, avoiding the biased KPU int16 Tanh table).
  - The current final model intentionally omits `--gru-update`.

- `prequant_bias_correct.py`
  - Rounds Conv/Gemm weights onto nncase's per-channel uint8 grid inside the
    ONNX graph and cancels the resulting per-channel output mean shift in the
    biases (sequential, ONNX Runtime on calibration samples). Needs the
    by-channel `QuantScheme.json` for an exact grid.

- `make_supercombo_calibration.py`
  - Builds PTQ NPZ data from local driving logs.
  - Supports an ordered `--segment-list`, metadata output, and `--warp-mode`.

- `compile_supercombo_nncase.py`
  - Imports ONNX into nncase, applies optional PTQ data, and writes `.kmodel`.
  - On this Mac, the practical K230 path is the `linux/amd64` Docker image
    `supercombo-nncase-k230:2.11.0-sdk` with the Rosetta/.NET mitigation flags
    documented in `../../models/README.md`.

## Current final model summary

- Final ONNX:
  - `../../models/onnx/supercombo_base.onnx`
- Final kmodel:
  - `../../models/supercombo.kmodel`
- PTQ data:
  - `../../models/ptq/supercombo_calib.npz`
- Quantization:
  - INT16 activations, UINT8 weights, `NoClip`, 77 balanced samples (3 degenerate frames removed).
- Runtime compatibility:
  - Keeps `input_imgs`, `big_input_imgs`, `desire`, `traffic_convention`, and
    `initial_state` inputs, with both image towers active.
