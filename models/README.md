# K230 supercombo model

`supercombo.kmodel` is the 2026-09-04 candidate selected in
[the K230 measurements](verification/quantization_20260904.md), built on the
2026-09-03 model (common plan-bias centering, INT16 activations / UINT8 weights,
NoClip, 173 mixed C2/K230 calibration samples). It adds three changes that target
the error sources measured on the board:

- the GRU `Tanh` is computed as `2*Sigmoid(2x)-1` (the KPU int16 Tanh table is
  biased toward zero by up to 0.038, the Sigmoid table by at most 0.006),
- Conv/Gemm weights are pre-rounded onto nncase's per-channel uint8 grid and the
  biases are corrected for the resulting per-channel output mean shift,
- the PTQ set is extended with 127 native K230 samples (300 total).

`SQuant` is no longer used: the pre-rounded weights leave nothing for it to tune.
Both image towers and the original GRU expression remain.

On six held-out K230 routes (1,320 recurrent frames, independent of the
calibration data) plan agreement with the FP32 model went from 87.5 % to
94.5 %, selected-path y MAE from 15.6 cm to 11.3 cm, non-plan-logit MAE from
0.093 to 0.062 and recurrent-state MAE from 0.0184 to 0.0075; pose MAE moved
from 0.041 to 0.047. Pure inference time is unchanged. See the report for every metric, the rejected alternatives and limits.
These are conversion-error measurements, not vehicle tracking accuracy.

## Preserved artifacts

- `supercombo.kmodel`: selected K230 model, uploaded to `model/supercombo.kmodel`.
- `ptq/supercombo_calib_mixed173.npz` and metadata: the 2026-09-03 mix
  (77 C2 + 72 native K230 + 24 lane-change samples).
- `ptq/supercombo_calib_k230_127.npz` and metadata: 127 native K230 samples from
  ten routes (2026-08-16 to 2026-09-03, held-in segments only), appended to the
  mix at build time to form the 300-sample PTQ set.
- `ptq/supercombo_quant_scheme_bychannel.json`: nncase `QuantScheme.json` exported
  with `export_weight_range_by_channel`; provides the exact per-channel weight
  grid for `tools/model/prequant_bias_correct.py`.
- `onnx/supercombo_rewritten.onnx`, `onnx/supercombo_base.onnx` (after bias
  correction) and `onnx/supercombo_uint8.onnx`: generated ONNX artifacts,
  excluded from Git and reproducible from the source model.
- `manifest.sha256`: hashes of the current model, generated graphs and PTQ
  inputs. Run `shasum -a 256 -c manifest.sha256` in `models/` after a build.
- `verification/quantization_20260904.{md,json}` and
  `verification/quantization_20260903.{md,json}`: measured comparisons.

The older `openpilot_c2_master_comparison.json` and
`full_big_input_board_verification_summary.json` describe earlier artifacts.

Current model SHA-256:

```text
2e9759c73001e587aebdc66d7473479a9752ba9bbb7d67a3744347675b7ade99
```

## Runtime contract

```text
0 input_imgs          [1, 12, 128, 256] uint8
1 big_input_imgs      [1, 12, 128, 256] uint8
2 desire              [1, 8]           float32
3 traffic_convention  [1, 2]           float32
4 initial_state       [1, 512]         float32
```

Outputs flatten to 6524 float32 values; the last 512 feed the next frame.
The five plan scores are deltas relative to plan 0. Common bias is removed
before the KPU's intermediate rounding, preserving small score differences.

Both towers consume the same 1280x720 NV12 camera frame through independent
fixed12 warps and previous/current histories. The virtual med camera has focal
length 910 and center (256, 47.6); sbig has focal length 455 and center
(256, 151.8). The runtime writes uint8 tensors directly using its C908 RVV
kernel. This preserves the existing K230 input/output contract; it does not
claim pixel identity with the original C2 OpenCL preprocessing.

## Rebuild

From the repository root:

```sh
PYTHON_BIN=../.model-venv/bin/python scripts/build_supercombo_model.sh
```

Required inputs:

- The original `openpilot_c2/selfdrive/modeld/models/supercombo.onnx`, by default
  under `/Users/chan/Documents/openpilot_c2`; override `SOURCE_ONNX` if needed.
  Original ONNX SHA-256:
  `50c7fc8565ac69a4b9a0de122e961326820e78bf13659255a89d0ed04be030d5`.
- Docker image `supercombo-nncase-k230:2.11.0-sdk`, buildable from
  `tools/model/docker/Dockerfile`.
- A host Python with `numpy`, `onnx` and `onnxruntime` (`PYTHON_BIN`) for the
  bias correction step; the Docker image has no ONNX Runtime.
- The committed PTQ NPZ files and the quant scheme in `ptq/`.

The build applies Gemm splitting, common plan-bias centering followed by plan-0
subtraction, the identity depthwise Conv before `Elu_223`, the Tanh rewrite,
weight pre-quantization with bias correction (48 calibration samples), and uint8
image input retyping. It compiles with NoClip, INT16/UINT8 and no weight
fine-tuning on the merged 300-sample set, retrying the compiler up to three
times because nncase 2.11 occasionally dies at startup. The script replaces the
default local model only after compilation succeeds and regenerates its manifest.

`REGEN_PTQ=1` is rejected; native data generation and the evaluation workflow are
documented in [QUANTIZATION.md](../tools/model/QUANTIZATION.md).
Use `scripts/build_quantization_candidate.sh` for a separate experimental build.

For runtime cross-build and upload, see [build and deploy](../docs/build-and-deploy.md).
