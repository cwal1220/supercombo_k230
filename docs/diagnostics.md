# Diagnostics

[← Documentation index](../README.md)

Benchmark and diagnostic utilities live under `benchmarks/` and are not built by
default. Build them explicitly with:

```sh
cmake -S . -B build/host-checks \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build build/host-checks -j2
```

CAN/panda payload checks can be run on a host before connecting the car:

```sh
cmake --build build/host-checks --target check_panda_can_codec -j2
./build/host-checks/bin/check_panda_can_codec
```

## NV12 replay

To run an existing `SCNV12R1` replay through the split model process on the
board:

```sh
SUPERCOMBO_REPLAY_NV12=/root/supercombo_k230/replay_120.scnv12 \
  ./k230_modeld model/supercombo.kmodel 0
```

## Model swap verification

A kmodel swap changes three things at once: the warp input, the temporal
plumbing, and the network. Verify them together by running the same NV12
frames through the board and through the host.

```sh
# host: build the replay and the fp32 reference for the candidate ONNX
python tools/model/make_replay.py \
  --route <route_dir> --out /tmp/verify --frames 100 --skip 400 \
  --model models/onnx/supercombo_uint8.onnx \
  --rpy "$(python3 -c 'import json;print(",".join(map(str,json.load(open("params/calibration.json"))["rpy_rad"])))')"

# board: same frames through the runtime, dumping raw outputs
SUPERCOMBO_REPLAY_NV12=/root/verify/replay.scnv12 \
SUPERCOMBO_RAW_DUMP=/root/verify/board_raw.bin \
SUPERCOMBO_CALIB_AUTO=0 \
  ./k230_modeld model/<candidate>.kmodel 0
```

The `--rpy` value must be the board's stored calibration, because the
calibration service overrides `SUPERCOMBO_INPUT_WARP_*` on every frame:
those variables only seed the first warp. Compare `board_raw.bin` against
`host_ref.npy` on the slices that drive control (plan lateral offset, lane
positions) rather than on the raw vector, and check that the feature/hidden
slice evolves smoothly — a dead temporal buffer still produces plausible
single-frame output.

Measured for the v0.9.4 swap (100 night-highway frames, int16 PTQ, uint8
image inputs): plan lateral 0.024 m mean / 0.062 m max at 2 s, lane position
0.048 m mean, feature buffer frame-to-frame correlation 0.77.

## Lateral bias

If the car holds one side of the lane, `tools/model/lane_bias.py` says whether
the camera calibration is responsible:

```sh
python tools/model/lane_bias.py <route_dir> [<route_dir> ...]
```

It reads the recorded `modelState` and `controlState`, keeps straight engaged
stretches, and fits the perceived lane-centre offset against distance:

| term | meaning |
| --- | --- |
| translation (m) | camera off the vehicle centreline, or the car genuinely off-centre. **Camera intrinsics cannot produce this** -- a principal-point or focal-length error acts about the camera, so its lateral effect is exactly zero at `x=0`. |
| rotation (rad/m) | the calibration-shaped term. A wrong `cx` of `dcx` pixels appears here as roughly `dcx/fx`. |

It also prints the tuning that was active on that drive from the route's own
`params/` snapshot, the mean steering angle needed to hold a straight line, and
the mean curvature command, so a control bias can be told apart from a
perception bias.

Straightness is judged from the model's own 48 m path, never from the steering
angle: when the car needs a non-zero angle to go straight, an `|angle| < k`
filter keeps one side of the curve distribution and manufactures a rotation
term that is not there. On the 2026-08-19 route that mistake reported
-4.22 mrad where the honest figure is -0.74 mrad.

Measured on the two logged drives, both with `camera_offset_m = 0`:

| drive | `path_offset_m` | translation | rotation |
| --- | ---: | ---: | ---: |
| 2026-08-16 | 0.00 | +19.9 cm | +0.18 mrad |
| 2026-08-19 | 0.07 | +7.2 cm | -0.74 mrad |

The rotation term is under 1 mrad on both, so the left-hugging on those drives
was a lateral offset, not a camera-matrix error.

## Recording format

`k230_recordd` writes the event log as 60 s chunks in `events/NNN.bin`, each
starting with an 8-byte `K230LOG1` magic, a version word, and fixed 16-byte
record headers. The current version is `5`.

| Record type | Payload |
| --- | ---: |
| `CanRx` / `CanTx` | variable CAN batch |
| `ModelState` | 3256 B |
| `ControlState` | 240 B |
| `PandaState` | 96 B |

Older recordings are not `ModelState`-compatible: version 1 carried 4384 B
including unused lateral draft fields, versions 2–3 carried 4080 B including the
stop-line block that openpilot v0.9.4 does not emit, and version 4 carried
4048 B including plan position stds and orientations that nothing read.
Version 2 also kept a
single route-level `events.bin`; CAN logging alone (~0.5 MB/s) filled the 988 MB
tmpfs staging in about 30 minutes on long drives and silently killed the rest of
the recording, which is why version 3 rotates event chunks alongside video
segments. `tools/model/k230_route.py` reads both the v3 and v4 layouts.

## Lateral dataset extraction

`extract_lateral_dataset` replays a recording and writes one CSV row per
`ControlState` record (~62 Hz), joining the CAN state decoded by the runtime's
own `vehicle_can` so signs and scaling match the board exactly:

```sh
cmake --build build/host-checks --target extract_lateral_dataset -j2
./build/host-checks/bin/extract_lateral_dataset out.csv <route>/events/*.bin
```

Version 2 route-level `events.bin` files work as well; a file truncated by the
tmpfs fill stops at the zero-filled gap with the byte offset on stderr.

Columns: time, wheel speed, `active`/`desire`/`block`, steering angle, driver
and applied torque (`tq_norm` is sign-corrected and divided by `steer_max`),
ESP12 lateral/longitudinal acceleration and yaw rate, the live bank estimate
and its roll equivalent, requested and measured curvature, and the NNFF-style
future values `la_p03..p15` / `roll_p03..p15` taken from the recorded model
plan.

Two deviations from the runtime are deliberate. The bank filter runs at row
rate with a `dt`-derived alpha rather than the controller's fixed 100 Hz step,
and future lateral acceleration is `v * dpsi/dt` off the plan's yaw angles
because the recorded `ModelState` has no plan acceleration. Both were checked
against known results: straight-line bank reproduces -0.178 on the 8-28 route
(logged: -0.176), and the total-least-squares torque fit over the 8-19 route
returns `latAccelFactor` 4.00, the value that route was fit to.

## Related documents

- [Recovery procedures](recovery.md)
- [CAN stability plan](can_stability_plan.md)
- [Departure alerts](departure_alerts.md)
- [YG panda port notes](yg_panda_port.md)
