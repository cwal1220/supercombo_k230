# Modern driving model migration

This branch runs a six-input, 2576-output modern `driving_supercombo` model on
K230. The model, runtime ABI, camera geometry, and parser must be deployed as a
single unit.

## Selected artifact

- Source: sunnypilot `driving_supercombo.onnx` at
  `1a07e4722853c0606b0e1caa8f300a371e342948`
- Source SHA-256:
  `659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b`
- KModel SHA-256:
  `49ed812db587d48c6dfdcc26d8e42d2e69a5d0717527bb3dd74dfe4f088bfed1`
- Compiler: nncase 2.11.0, 288 real logging samples, `NoClip`, INT16
  activations, UINT8 weights, no output affine

The selected source keeps the validated 2576-output ABI. Newer 2580-output
models require a different parser and were not mixed into this runtime.

The legacy `supercombo_k230` graph tricks were reviewed rather than copied
literally: its Gemm/Split, `Elu_223`, and GRU rewrites do not exist in this
attention model. The reusable principles are real driving PTQ samples,
KPU-friendly lowering, exact ABI checks, and selective validation on the board.

## Why the previous KModel was rejected

The earlier mixed uint8/int16 model with a 2576-channel affine had good timing,
but offline comparison with the source ONNX showed compressed plan range and
degraded lane/edge heads. The full INT16 artifact (`49ed812d…`) is therefore
the selected vision base.

Full INT16 preserves lane and road-edge geometry substantially better. Its
policy lateral position remains quantization-sensitive, so production applies
a guarded correction from the two inner lane lines. The correction requires
high lane probability, low uncertainty, and plausible 2–5 m widths; it is
disabled during lane changes. Longitudinal, vertical, time, and x outputs are
not replaced.

## Camera geometry

Live capture and `SCNV12R1` replay both use the calibrated K230 OV5647 geometry:
reference size 1920×1080, `fx=1625.742`, `fy=1585.983`, `cx=946.135`, and
`cy=537.341`. Intrinsics are scaled to the actual frame size, and the calibrated
Brown-Conrady distortion is applied consistently to model-input sampling and
overlay projection.

## Runtime ABI

| Input | Type and shape | Runtime source |
| --- | --- | --- |
| `img` | `uint8[1,12,128,256]` | medmodel current frame plus four-tick history |
| `big_img` | `uint8[1,12,128,256]` | sbigmodel current frame plus four-tick history |
| `features_buffer` | `float16[1,24,512]` | 96-frame hidden queue sampled every four frames |
| `desire_pulse` | `float16[1,25,8]` | 100-frame desire queue max-pooled by four |
| `traffic_convention` | `float16[1,2]` | right-hand traffic default |
| `action_t` | `float16[1,2]` | lateral 0.2 s, longitudinal 0.5 s |

The output is `float32[1,2576]`. Non-finite output, malformed plan geometry,
and invalid lateral targets are rejected before publishing model state.

## Validation

Host checks cover the modern model context, output parser, lane-plan fusion,
K230 calibration/projection, and scalar/RVV preprocessing equivalence. On the
board, a 45-second live K230-camera full-pipeline run sustained 20.022 FPS for
901 model frames, 29.987 camera FPS, and the 100 Hz control loop with no model
or camera errors. The model process used the lowest real-time round-robin
priority (`SCHED_RR:1`).

Exact compiler and validation records are under `models/verification/`.

## Build and deployment

Rebuild the model with the hash-checked pipeline:

```sh
SOURCE_ONNX=/path/to/driving_supercombo.onnx \
CALIBRATION_NPZ=/path/to/full6_real_logging_calib.npz \
  scripts/build_supercombo_model.sh install
```

Then build and upload the complete runtime:

```sh
K230_BUILD_DIR=build-k230-sdk \
  scripts/upload_to_board.sh root@192.168.219.111
```

The uploader verifies the selected model hash, stops the service, snapshots the
installed runtime, verifies the transferred model, and leaves the service
stopped unless restart is explicitly requested.

## Vehicle-validation gates

1. Parked, Panda receive-only: verify camera/model/overlay, thermal behavior,
   parser health, and K230-camera lane projection with TX disabled.
2. Closed-course shadow mode: compare curvature and lane selection without
   steering or longitudinal transmission.
3. Closed-course lateral-only: low speed, safety driver, immediate disengage.
4. Longitudinal control: enable only after lead distance and relative-speed
   output have been reviewed on a current K230 road log.

The host regression and live K230 full-pipeline runtime gates pass. K230-camera
road-quality and moving-vehicle closed-loop tests have not been run and must not
be represented as passed.
