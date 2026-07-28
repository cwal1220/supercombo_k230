# Model tools

This directory contains the scripts used to rewrite openpilot `supercombo.onnx`
and compile a K230 `.kmodel`.

The current cleaned final artifact package is:

- `../../models/`

Use `../../models/README.md` for the exact final ONNX, PTQ, compile
commands, hashes, and input contract. That package is the source of truth for
the current no-GRU, full dual-image-input K230 model.

## Scripts

- `rewrite_supercombo_onnx.py`
  - Applies graph rewrites such as Gemm/Split replacement, plan-output split,
    plan probability delta output, optional GRU update rewrite, and the
    identity depthwise Conv before `Elu_223`.
  - The current final model intentionally omits `--gru-update`.

- `remove_supercombo_big_input.py`
  - Folds the zero `big_input_imgs` tower contribution into the summarizer Gemm
    bias and prunes the unreachable big-image tower.
  - Use `--keep-big-input` for compatibility with the existing 5-input K230
    runtime.

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
  - `../../models/onnx/supercombo_gemm_split3_iddwelu223_nogru_splitplan_delta.onnx`
- Final kmodel:
  - `../../models/supercombo.kmodel`
- PTQ data:
  - `../../models/ptq/supercombo_balanced80_calib.npz`
- Quantization:
  - INT16 activations, UINT8 weights, `NoClip`, 80 balanced samples.
- Runtime compatibility:
  - Keeps `input_imgs`, `big_input_imgs`, `desire`, `traffic_convention`, and
    `initial_state` inputs, with both image towers active.
