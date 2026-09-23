# Diagnostics

This folder keeps standalone experiments out of the production runtime path.
They are not built by the default CMake target.

The `check_*` self-checks build and run in one step:

```sh
./scripts/run_host_checks.sh
```

Build the remaining benchmarks and tools explicitly. Host-only targets need
neither nncase, OpenCV, nor the K230 display libraries:

```sh
cmake -S . -B build-host \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_DIAGNOSTICS=ON
cmake --build build-host --target bench_input_warp_overhead -j2
./build-host/bin/bench_input_warp_overhead 3000
```

Board-only targets (`bench_capture_nv12`, `bench_ai2d_resize`, `bench_kmodel`,
`run_kmodel_sequence`, `probe_drm_planes`) need the K230 runtime build; add
`-DSUPERCOMBO_BUILD_DIAGNOSTICS=ON` to the cross-build in `build/`.

Available utilities. The `check_*` self-checks are listed with what each one
covers in [docs/verification.md](../docs/verification.md#host-self-tests);
`scripts/run_host_checks.sh` builds and runs all of them, including
`check_param_server.py`.

Host tools (`build-host`, no board libraries):

- `replay_lateral_learners`: runs the controlsd paramsd/torqued learners
  (`LateralLearners`) over a recording and prints the learned steer ratio,
  stiffness, angle offset, roll and torque factor. `--inputs`/`--torque-inputs`
  dump the per-tick inputs and `--outputs`/`--torque-outputs` the published
  messages for comparison against a reference implementation; `--torque-cache`
  chains drives the way the runtime cache does. `--steering route/params/steering.json`
  uses the recording's own tuning (priors, delay) instead of the code defaults, and
  `--upstream-schedule` observes steering at 20 Hz like upstream instead of every tick.
- `replay_planner`: re-runs `LateralPlanner` over recorded `ModelState` /
  `ControlState` records and writes what the planner asked for as CSV.
- `replay_closed_loop`: re-runs the whole lateral loop over the same records with
  the vehicle response simulated, so a control change moves the car instead of
  being scored against a frozen recording. Lane geometry is rotated into the
  simulated car's frame by the pose it has drifted (`dy`, `dpsi`), and ticks with
  driver torque or no `active` resync to the recording, so each free-running
  segment starts from the real pose. `SIM_*` environment variables set the plant
  (`WN`, `ZETA`, `DELAY`, `GAIN`, `GAIN_PTS`) and the controller (`SAD`,
  `KP_RAW`, `KI_RAW`, `KF_RAW`); `SIM_OPEN_LOOP=1` freezes the pose so the run
  measures how well the plant reproduces the recorded drive. It prints that
  reproduction score (overall and per speed band) on stdout; pass `-` as the
  output path to skip the CSV. See
  [docs/closed_loop_replay.md](../docs/closed_loop_replay.md).
- `extract_lateral_dataset`: one CSV row per `ControlState` record with the CAN
  state decoded by the runtime's own `vehicle_can`.
- `bench_nv12_to_yuv6`: CPU `NV12 512x256 -> YUV6 float` conversion timing.
- `bench_input_warp_overhead`: compares direct YUV6 packing with the calibrated
  homography `NV12 -> YUV6` input-warp path.
- `hud_snapshot`: renders the HUD scenarios (or a recorded model/control pair)
  to `K230ARGB` frames and prints draw timings. Needs OpenCV, so it builds where
  `k230_overlayd` builds; see [docs/diagnostics.md](../docs/diagnostics.md).

Board tools (`SUPERCOMBO_BUILD_DIAGNOSTICS=ON` in the cross build):

- `bench_capture_nv12`: `/dev/video2` NV12 capture timing.
- `bench_ai2d_resize`: K230 AI2D crop/resize timing experiment.
- `bench_kmodel`: NPU latency for any kmodel, using zero-filled inputs shaped
  from the model itself.
- `run_kmodel_sequence`: run a `K230MSQ1` input sequence through a kmodel and
  dump the raw outputs, for host-vs-board comparison.
- `probe_drm_planes`: inspect DRM planes and optional ARGB plane commit.

Shared sources, not targets: `check_harness.h` (`require`/`near`/`run_checks`),
`control_fixtures.h` (a ready K7 vehicle state for the control checks), and
`can_replay.*` (the `K230CAN1` fixture reader behind `check_control_replay`'s
replay mode; `tools/control/export_can_fixture.py` writes those fixtures).
