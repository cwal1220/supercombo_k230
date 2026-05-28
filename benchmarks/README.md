# Benchmarks and diagnostics

This folder keeps standalone experiments out of the production runtime path.
They are not built by the default CMake target.

Build them explicitly when needed:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build build -j2
```

Available utilities:

- `bench_nv12_to_yuv6`: CPU `NV12 512x256 -> YUV6 float` conversion timing.
- `bench_capture_nv12`: `/dev/video2` `NV12 512x256` capture timing.
- `bench_ai2d_resize`: K230 AI2D crop/resize timing experiment.
- `sequence_runner`: run a prebuilt tensor sequence through a kmodel and dump
  raw outputs.
- `probe_drm_planes`: inspect DRM planes and optional ARGB plane commit.
- `check_model_output_parser`: sanity-check one `SCODMP1` raw-output dump.
