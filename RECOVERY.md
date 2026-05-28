# K230 supercombo recovery guide

This directory is the final reusable native C++ app bundle for K230.

## Final model

Use the `iddwelu223` model. This is the graph where `Elu_223` is kept on the
native K230 path by inserting a mathematically identity depthwise 1x1 Conv before
that ELU.

Required artifacts:

- `models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel`

The matching ONNX is not part of the runtime repository because it is larger
than GitHub's normal file-size limit.

## What must be preserved

- `src/`: C++ app, camera input, YUV6 preprocessing, nncase runtime wrapper, overlay drawing
- `include/`: K230 display/v4l2/mmz headers copied from the K230 Linux SDK
- `deps/`: nncase K230 runtime headers and static libraries
- `CMakeLists.txt`: primary board-native build recipe
- `Makefile`: rollback board-native build recipe
- `k230_manager.py`: minimal openpilot-style process supervisor
- `fetch_nncase_runtime.sh`: recreates `deps/` from Kendryte nncase v2.11.0 release

Important files inside `deps/`:

- `deps/include/nncase/runtime/interpreter.h`
- `deps/lib/libNncase.Runtime.Native.a`
- `deps/lib/libnncase.rt_modules.k230.a`
- `deps/lib/libfunctional_k230.a`

## Board prerequisites

On a freshly flashed board:

```sh
apt-get update
apt-get install -y g++ cmake make libdrm-dev curl
```

The flashed image must already include:

- `/usr/lib/riscv64-linux-gnu/libdisplay.so`
- `/usr/lib/riscv64-linux-gnu/libv4l2-drm.so`
- `/usr/lib/riscv64-linux-gnu/libdrm.so.2`
- `/dev/mmz`

## Restore to board

From the host:

```sh
ssh root@192.168.219.115 'rm -rf /root/supercombo_native && mkdir -p /root/supercombo_native'
scp -r tools/k230/supercombo_native/* root@192.168.219.115:/root/supercombo_native/
```

If `deps/` was not copied, recreate it on the board:

```sh
cd /root/supercombo_native
./fetch_nncase_runtime.sh
```

## Build

```sh
cd /root/supercombo_native
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

The expected output is:

- `/root/supercombo_native/supercombo.elf`
- `/root/supercombo_native/k230_overlay`
- `/root/supercombo_native/k230_camerad`
- `/root/supercombo_native/k230_modeld`

The Makefile remains available as a rollback build path:

```sh
make clean
make -j2
```

## Run

```sh
cd /root/supercombo_native
./k230_manager.py models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

Expected split-runtime behavior:

- `k230_overlay` starts first, opens `/dev/video1` through the verified
  `v4l2_drm` preview path, owns the display, waits for displayed preview
  frames, then writes `/tmp/k230_display_ready`.
- The manager waits for `/tmp/k230_display_ready`, then starts `k230_camerad`.
  `k230_camerad` then opens `/dev/video2` and publishes `roadAiFrame` metadata
  for frames in `/dev/shm/k230_road_ai`.
- `k230_modeld` consumes the latest shared NV12 frame and publishes compact
  `modelState`.
- `k230_overlay` subscribes to compact `modelState`, then draws only
  plan/lane/road-edge/lead on an ARGB8888 overlay plane. It does not use Qt,
  touch, buttons, or a status HUD.
- Display overlay may render below 30 fps; the model loop is the priority.

Expected behavior:

- LCD preview appears through `/dev/video1` as `800x480 NV12` with DRM rotation
  for the panel's native `480x800` mode.
- Overlay draws plan, lanes, road edges, and the primary front-vehicle marker.
- Runtime logs print `camerad`, `modeld`, and `overlay` FPS/errors separately.
- `errors` should remain `0`.

Optional profiling:

```sh
SUPERCOMBO_PROFILE=1 ./supercombo.elf models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

Headless replay from collected driving logs:

```sh
# Host side, from this repository:
python3 tools/k230/export_replay_nv12.py \
  --segment-dir device_collected/2026-05-23_1600_combined/part1_20260523_174953/1970-01-01--09-00-59--44 \
  --out tools/k230/out/replay_nv12/replay_120.scnv12 \
  --frames 120
scp tools/k230/out/replay_nv12/replay_120.scnv12 \
  root@192.168.219.115:/root/supercombo_native/

# Board side:
cd /root/supercombo_native
SUPERCOMBO_REPLAY_NV12=/root/supercombo_native/replay_120.scnv12 \
  ./supercombo.elf models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

The split model process can also be tested headlessly:

```sh
SUPERCOMBO_REPLAY_NV12=/root/supercombo_native/replay_120.scnv12 \
SUPERCOMBO_MAX_FRAMES=20 \
  ./k230_modeld models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

Replay mode does not open the camera or display. It feeds the same
`run_frame_nv12()` path used by live capture, so it is useful for validating
model execution, pose logging, raw dumps, and online calibration from stored
segments.

The app asks the K230 V4L2 path for `NV12 512x256` on the AI stream by default
through `v4l2_drm_context.crop_size`, then performs the final `NV12 -> YUV6
float` layout conversion on CPU. The default crop rectangle is the full sensor
image, `1920x1080+0+0`. `SUPERCOMBO_NV12_CROP_X`,
`SUPERCOMBO_NV12_CROP_Y`, `SUPERCOMBO_NV12_CROP_WIDTH`, and
`SUPERCOMBO_NV12_CROP_HEIGHT` can override the crop rectangle for tuning.
`SUPERCOMBO_NV12_WIDTH` and `SUPERCOMBO_NV12_HEIGHT` can override the output
size for experiments. In the split runtime, `k230_overlay` owns the
`/dev/video1` preview path, publishes display readiness after preview frames are
displayed, and the manager starts `k230_camerad` after that barrier.

Overlay projection can be adjusted without rebuilding:

```sh
SUPERCOMBO_CALIB_PITCH_DEG=1.0 SUPERCOMBO_CALIB_YAW_DEG=-0.5 \
  ./supercombo.elf models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

`SUPERCOMBO_LOG_POSE=1` prints the supercombo pose head every 30 inferred frames.
The pose-based online calibrator is enabled by default and only affects overlay
projection after it reaches `validBlocks >= 5`. Set `SUPERCOMBO_LOG_CALIB=1` to
print accepted/rejected sample counts, status, rpy, and spread. Set
`SUPERCOMBO_CALIB_AUTO=0` to keep the fixed zero/manual projection. If any of
`SUPERCOMBO_CALIB_ROLL_DEG`, `SUPERCOMBO_CALIB_PITCH_DEG`, or
`SUPERCOMBO_CALIB_YAW_DEG` is provided, that manual projection wins and online
calibration is not applied to the overlay.

The overlay projection defaults to the legacy K230 road-frame projection used by
the pre-refactor runtime. Set `SUPERCOMBO_PROJECTION_MODE=openpilot` to compare
against the openpilot `view_from_calib` rotation.
`SUPERCOMBO_LOG_CONTROL=1` prints a draft lateral target from the best plan; it
is only diagnostic and does not send steering or CAN commands.

The front-vehicle marker is enabled by default. Set `SUPERCOMBO_DRAW_LEAD=0` to
hide it, or adjust `SUPERCOMBO_LEAD_PROB_THRESHOLD` if the marker appears too
often or too rarely.

Use the app crop path only; standalone `v4l2-ctl` / `v4l2-drm` crop tests can
leave the camera device in a bad state.

## Known build detail

The board GCC assembler does not accept the T-Head mnemonic `dcache.civa`.
`include/thead.h` and `src/mmz.c` use the equivalent raw instruction form:

```c
__asm volatile(".insn i 0x0b, 0, x0, %0, 0x027" : : "r"(op_addr));
```

Do not replace it with `dcache.civa` unless building with a toolchain whose
assembler supports the `xtheadcmo` extension mnemonic.
