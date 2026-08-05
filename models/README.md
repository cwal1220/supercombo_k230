# K230 modern driving model

`supercombo.kmodel` is the production six-input `driving_supercombo` artifact
for the K230 runtime.

```text
49ed812db587d48c6dfdcc26d8e42d2e69a5d0717527bb3dd74dfe4f088bfed1  supercombo.kmodel
```

## Provenance and compile profile

- Source: sunnypilot `driving_supercombo.onnx`, commit
  `1a07e4722853c0606b0e1caa8f300a371e342948`
- Source ONNX SHA-256:
  `659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b`
- K230-lowered ONNX SHA-256:
  `a5f92c39da3db830d9adbb36005aa35d1dc5ce8569fb4c90f8a2715af092e675`
- Calibration NPZ SHA-256:
  `aa13949b1a99128de65288747e6662420fd3eac209e2d55c99b0b30c0c18d8cb`
- Compiler: nncase 2.11.0, K230 target
- PTQ: 288 real logging samples, `NoClip`, INT16 activations, UINT8 weights
- Output: native float32 output; no output affine

The previous mixed-precision/affine artifact (`908ec085…`) is rejected because
offline comparison with the source ONNX showed compressed plan range and
degraded lane/edge heads. The selected full-INT16 artifact preserves the vision
heads and is paired with the runtime's guarded lane-center lateral/yaw
correction.

## ABI

```text
0 img                 uint8   [1, 12, 128, 256]
1 big_img             uint8   [1, 12, 128, 256]
2 features_buffer     float16 [1, 24, 512]
3 desire_pulse        float16 [1, 25, 8]
4 traffic_convention  float16 [1, 2]
5 action_t            float16 [1, 2]

0 outputs             float32 [1, 2576]
```

The runtime validates this ABI before inference and implements the four-frame
image cadence, hidden-feature queue, desire max pooling, and lane-change guard.
The model has no stop-line head.

## Rebuild

The source ONNX and 115 MiB calibration set are intentionally not committed.
Place them at the ignored default paths or pass explicit paths:

```sh
mkdir -p models/source
cp /path/to/driving_supercombo.onnx models/source/driving_supercombo.onnx

SOURCE_ONNX=/path/to/driving_supercombo.onnx \
CALIBRATION_NPZ=/path/to/full6_real_logging_calib.npz \
  scripts/build_supercombo_model.sh install
```

The script verifies every source, lowered graph, calibration set, and final
KModel hash. Modes are `onnx`, `kmodel`, and `install`. `install` updates
`models/supercombo.kmodel`, compile metadata, and `manifest.sha256`.

## Evidence

- `modern_model_metadata.json`: provenance, ABI, K230 camera geometry, runtime contract
- `verification/modern_model_compile_metadata.json`: exact compiler inputs/options
- `verification/modern_model_board_validation.json`: board replay and quality results
- `../docs/modern_model_migration.md`: rationale, deployment, and safety gates

Legacy five-input ONNX/PTQ files and no-big-input variants are audit artifacts
only. They are not compatible with this runtime.
