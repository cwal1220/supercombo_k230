# Modern driving model migration

This branch replaces the legacy five-input `supercombo` model with the selected
six-input `driving_supercombo` model. The binary artifact and the runtime ABI
must be deployed together.

## Selected artifact

- Source: sunnypilot `driving_supercombo.onnx` at
  `1a07e4722853c0606b0e1caa8f300a371e342948`
- Source SHA-256:
  `659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b`
- Kmodel SHA-256:
  `908ec08594776d0060e26dbd7adca68831dc88433a175940a7fe89cce30c151d`
- Compiler: nncase 2.11.0, 288 real calibration samples, mixed uint8/int16
  activations, uint8 weights, KLD calibration, embedded output affine

The model was selected after comparing current comma, sunnypilot, FrogPilot,
dragonpilot and openpilotkr artifacts. The newest comma model changes the output
ABI to 2580 values, is substantially larger, and has no K230 timing or PTQ
evidence. The selected model retains the already validated 2576-output ABI and
has a reproducible K230 conversion.

The useful ideas carried over from the original `cwal1220/supercombo_k230`
conversion are KPU-friendly graph boundaries, real driving PTQ samples and
selective precision. Its literal `Gemm/Split`, `Elu_223` and GRU rewrites do not
exist in this attention-based model and were not copied blindly.

## Runtime ABI

| Input | Type and shape | Runtime source |
| --- | --- | --- |
| `img` | `uint8[1,12,128,256]` | current medmodel frame plus the frame from four model ticks earlier |
| `big_img` | `uint8[1,12,128,256]` | current sbigmodel frame plus the frame from four model ticks earlier |
| `features_buffer` | `float16[1,24,512]` | 96-frame hidden queue sampled every four frames |
| `desire_pulse` | `float16[1,25,8]` | 100-frame desire queue, max pooled in groups of four |
| `traffic_convention` | `float16[1,2]` | right-hand traffic default |
| `action_t` | `float16[1,2]` | lateral 0.2 s, longitudinal 0.5 s |

The output is `float32[1,2576]`. The runtime rejects any other tensor count,
shape or type at startup. It also rejects non-finite output, malformed plan
geometry and invalid lateral targets before publishing a valid model state.
The selected model has no stop-line head, so stop-line state is explicitly
invalid instead of being read from a legacy offset.

## Performance evidence

The final binary was tested on the target board with the production camera
device and with a 1,200-frame local-drive replay.

| Test | Result |
| --- | --- |
| Standalone Kmodel | 33.729 ms NPU mean, 34.692 ms total, 28.825 FPS |
| Full 1,200-frame replay | 22.48 FPS, p95 46.424 ms, p99 47.490 ms, max 48.484 ms, 0 errors |
| Live camera, 1,200 model frames | steady cadence 20.003 FPS, p95 49.209 ms, p99 50.282 ms, 0 model/camera errors |

The live test had 23 of 1,195 steady samples above 50 ms (1.925%), with a
51.783 ms maximum. The absolute-phase scheduler prevented this small inference
jitter from accumulating: mean publish interval was 49.993 ms. This passes the
sustained 20 FPS gate, but it is not a claim that every single frame completes
inside 50 ms.

Exact measurements and offline quality figures are in
`models/verification/modern_model_board_validation.json`.

## Deployment and rollback

Build the complete runtime, not only `k230_modeld`, because the model parser,
IPC validity gate and camera ABI header changed together. Then deploy with:

```sh
K230_BUILD_DIR=build-k230-modern \
  scripts/upload_to_board.sh root@192.168.219.111
```

The uploader verifies the model SHA-256, stops the service, snapshots the
current runtime to `/root/supercombo_k230/rollback`, verifies the transferred
model again and leaves the service stopped. To restore the snapshot:

```sh
scripts/rollback_board.sh root@192.168.219.111
```

Do not use `K230_RESTART_AFTER_UPLOAD=1` for a first deployment. Start it only
after completing the parked-car gate below.

## Vehicle-validation gates

Vehicle control remains disabled until all gates pass in order:

1. **Parked, Panda receive-only**: start camera/model/overlay with
   `K230_ENABLE_CONTROL=0`, `K230_PANDA_TX=0`; verify 20 Hz for at least ten
   minutes, zero parser errors, correct lane/edge overlay and no thermal
   throttling.
2. **Closed-course shadow mode**: drive with steering and longitudinal TX still
   disabled; compare desired curvature, lead distance and lane selection
   against the previous model over straight, curved and lane-change sections.
3. **Closed-course lateral-only**: enable steering at low speed with a safety
   driver and immediate disengagement access. Reject the build for oscillation,
   path jumps, stale model state or repeated inference overruns.
4. **Closed-course longitudinal**: enable only after lead probability and
   distance have been reviewed. The fresh 120-frame comparison showed weaker
   plan correlation than the earlier held-out sequence, so a moving-vehicle
   shadow log is mandatory before this step.

The offline and parked-camera gates are complete. A moving-vehicle closed-loop
test has not been performed by this migration and must not be represented as
passed.
