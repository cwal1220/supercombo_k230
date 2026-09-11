# Diagnostics

This folder keeps standalone experiments out of the production runtime path.
They are not built by the default CMake target.

The `check_*` and `verify_*` self-checks build and run in one step:

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

Available utilities:

- `bench_nv12_to_yuv6`: CPU `NV12 512x256 -> YUV6 float` conversion timing.
- `bench_input_warp_overhead`: compares direct YUV6 packing with the calibrated
  homography `NV12 -> YUV6` input-warp path.
- `verify_calibration_equivalence`: checks the pose-based online calibration
  state machine, manual/online model-input feedback policy, medmodel homography
  matrix, `transform_scale_buffer(0.5)` UV handling, and YUV6 plane order
  against independent openpilot-formula references.
- `bench_capture_nv12`: `/dev/video2` `NV12 512x256` capture timing.
- `bench_ai2d_resize`: K230 AI2D crop/resize timing experiment.
- `bench_kmodel`: NPU latency for any kmodel, using zero-filled inputs shaped
  from the model itself.
- `run_kmodel_sequence`: run a `K230MSQ1` input sequence through a kmodel and
  dump the raw outputs, for host-vs-board comparison.
- `probe_drm_planes`: inspect DRM planes and optional ARGB plane commit.
- `check_model_output_parser`: sanity-check one `SCODMP1` raw-output dump.
