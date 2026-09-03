# K230 supercombo model

`supercombo.kmodel` is the quantization candidate selected in the
[2026-09-03 K230 measurements](verification/quantization_20260903.md), now promoted
as this branch's default model. It combines common plan-bias centering,
INT16 activations / UINT8 weights, NoClip, SQuant, and 173 mixed C2/K230
calibration samples. Both image towers and the original GRU expression remain.

On 640 held-out recurrent frames, plan agreement with the original FP32 model
improved from 555/640 to 590/640, selected-path y MAE from 8.03 to 6.94 cm,
and non-plan-logit RMSE from 0.3144 to 0.2705. Pure inference remained about
27.8 ms. Pose and desire errors regressed slightly, pose maximum error increased,
and recurrent-state error remains; see the report for all methods and limits.
These are conversion-error measurements, not vehicle tracking accuracy.

## Preserved artifacts

- `supercombo.kmodel`: selected K230 model, uploaded to `model/supercombo.kmodel`.
- `ptq/supercombo_calib_mixed173.npz`: exact compilation inputs, committed with
  the model. The order is 77 C2 + 72 native K230 + 24 lane-change samples.
- `ptq/supercombo_calib_mixed173_metadata.json`: source hashes, clip parameters,
  and native sample provenance.
- `onnx/supercombo_base.onnx` and `onnx/supercombo_uint8.onnx`: generated ONNX
  artifacts, excluded from Git and reproducible from the source model.
- `manifest.sha256`: hashes of the current model, generated graphs, calibration
  NPZ and metadata. Run `shasum -a 256 -c manifest.sha256` in `models/` after build.
- `verification/quantization_20260903.{md,json}` and its manifest directory:
  measured comparisons, including the old branch model at commit `6c86a4a`.
- `ptq/supercombo_calib.npz` and its original metadata: preserved legacy 77-sample
  C2 data, used as part of the mixed set and for historical comparisons.

The older `openpilot_c2_master_comparison.json` and
`full_big_input_board_verification_summary.json` describe earlier artifacts.
Their model hashes and timing results do not describe the current binary.
The board's pre-deployment model came from a different branch; its hash is
recorded in the quantization report.

Current model SHA-256:

```text
3bc1c91da706f99245168a6e231d17609a5b1b8b23d7ad1aabb70b749293d8a9
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
scripts/build_supercombo_model.sh
```

Required inputs:

- The original `openpilot_c2/selfdrive/modeld/models/supercombo.onnx`, by default
  under `/Users/chan/Documents/openpilot_c2`; override `SOURCE_ONNX` if needed.
- Original ONNX SHA-256:
  `50c7fc8565ac69a4b9a0de122e961326820e78bf13659255a89d0ed04be030d5`.
- Docker image `supercombo-nncase-k230:2.11.0-sdk`, buildable from
  `tools/model/docker/Dockerfile`.
- The committed 173-sample NPZ and metadata in `ptq/`.

The build applies Gemm splitting, common plan-bias centering followed by plan-0
subtraction, identity depthwise Conv before `Elu_223`, and uint8 image input
retyping. It retains the original GRU expression and compiles with NoClip and
`UseSquant`. The script replaces the default local model only after compilation
succeeds and regenerates its artifact manifest.

`REGEN_PTQ=1` from the old 77-sample workflow is rejected because it cannot
reproduce the mixed set. Native data generation, exact merge order and evaluation
commands are documented in [QUANTIZATION.md](../tools/model/QUANTIZATION.md).
Use `scripts/build_quantization_candidate.sh` for a separate experimental build.

For runtime cross-build and upload, see [build and deploy](../docs/build-and-deploy.md).
