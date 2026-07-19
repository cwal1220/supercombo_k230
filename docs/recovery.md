# K230 supercombo recovery guide

This directory is the final reusable native C++ app bundle for K230.

## Final model

Use the `iddwelu223` model. This is the graph where `Elu_223` is kept on the
native K230 path by inserting a mathematically identity depthwise 1x1 Conv before
that ELU.

Required runtime artifact:

- `models/supercombo.kmodel`

The original ONNX is available in the openpilot fork:

- [supercombo.onnx](https://github.com/cwal1220/openpilot_c2/blob/master/selfdrive/modeld/models/supercombo.onnx)

The rewritten ONNX is an intermediate generated artifact and is intentionally
not tracked in this repository. See `tools/model/` for the rewrite/compile
scripts and exact reproduction commands.

## What must be preserved

- `src/`: C++ app, camera input, YUV6 preprocessing, nncase runtime wrapper, overlay drawing
- `include/`: K230 display/v4l2/mmz headers copied from the K230 Linux SDK
- `deps/`: nncase K230 runtime headers and static libraries
- `CMakeLists.txt`: primary board-native build recipe
- `Makefile`: rollback board-native build recipe
- `k230_manager.py`: minimal openpilot-style process supervisor
- `scripts/fetch_nncase_runtime.sh`: recreates `deps/` from Kendryte nncase v2.11.0 release

Important files inside `deps/`:

- `deps/include/nncase/runtime/interpreter.h`
- `deps/lib/libNncase.Runtime.Native.a`
- `deps/lib/libnncase.rt_modules.k230.a`
- `deps/lib/libfunctional_k230.a`

## Board prerequisites

On a freshly flashed board, install the packages needed to clone, build, and run
the split runtime:

```sh
apt-get update
apt-get install -y \
  ca-certificates \
  cmake \
  curl \
  g++ \
  git \
  libdrm-dev \
  libopencv-dev \
  make \
  python3
```

Package purpose:

- `g++`, `make`, `cmake`: board-native C/C++ build
- `libdrm-dev`: DRM headers used by the overlay/display path
- `libopencv-dev`: OpenCV headers/libraries used by the overlay renderer
- `curl`, `ca-certificates`: `scripts/fetch_nncase_runtime.sh` download support
- `git`: fresh GitHub restore
- `python3`: `k230_manager.py`

If the bundle is restored by copying files from the host with `scp`, `git` is
not required for compilation itself. It is only required for a fresh repository
checkout.

The flashed image must already include:

- `/usr/lib/riscv64-linux-gnu/libdisplay.so`
- `/usr/lib/riscv64-linux-gnu/libv4l2-drm.so`
- `/usr/lib/riscv64-linux-gnu/libdrm.so.2`
- `/dev/mmz`

## Restore to board

From the host:

```sh
ssh root@192.168.219.115 'mkdir -p /root/supercombo_k230'
rsync -a --exclude .git /Users/chan/Documents/supercombo_k230/ \
  root@192.168.219.115:/root/supercombo_k230/
```

If `deps/` was not copied, recreate it on the board:

```sh
cd /root/supercombo_k230
./scripts/fetch_nncase_runtime.sh
```

## Build

```sh
cd /root/supercombo_k230
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

The expected output is:

- `/root/supercombo_k230/supercombo.elf`
- `/root/supercombo_k230/k230_overlay`
- `/root/supercombo_k230/k230_camerad`
- `/root/supercombo_k230/k230_modeld`

The Makefile remains available as a rollback build path:

```sh
make clean
make -j2
```

## Run

```sh
cd /root/supercombo_k230
./scripts/k230_manager.py models/supercombo.kmodel 0
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
SUPERCOMBO_PROFILE=1 ./supercombo.elf models/supercombo.kmodel 0
```

Headless replay from collected driving logs:

```sh
# Host side, from this repository:
python3 tools/k230/export_replay_nv12.py \
  --segment-dir device_collected/2026-05-23_1600_combined/part1_20260523_174953/1970-01-01--09-00-59--44 \
  --out tools/k230/out/replay_nv12/replay_120.scnv12 \
  --frames 120
scp tools/k230/out/replay_nv12/replay_120.scnv12 \
  root@192.168.219.115:/root/supercombo_k230/

# Board side:
cd /root/supercombo_k230
SUPERCOMBO_REPLAY_NV12=/root/supercombo_k230/replay_120.scnv12 \
  ./supercombo.elf models/supercombo.kmodel 0
```

The split model process can also be tested headlessly:

```sh
SUPERCOMBO_REPLAY_NV12=/root/supercombo_k230/replay_120.scnv12 \
SUPERCOMBO_MAX_FRAMES=20 \
  ./k230_modeld models/supercombo.kmodel 0
```

Replay mode does not open the camera or display. It feeds the same
`run_frame_nv12()` path used by live capture, so it is useful for validating
model execution and online calibration from stored segments.

The app asks the K230 V4L2 path for fixed `NV12 512x256` on the AI stream,
using the full sensor crop `1920x1080+0+0`, then performs the final
`NV12 -> YUV6 float` layout conversion on CPU. In the split runtime,
`k230_overlay` owns the `/dev/video1` preview path, publishes
`/tmp/k230_display_ready` after 30 displayed preview frames, and the manager
starts `k230_camerad` after that barrier. The manager times out after 7000 ms
and then starts the camera/model pipeline anyway.

Overlay projection can be adjusted without rebuilding:

```sh
SUPERCOMBO_CALIB_PITCH_DEG=1.0 SUPERCOMBO_CALIB_YAW_DEG=-0.5 \
  ./supercombo.elf models/supercombo.kmodel 0
```

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

The front-vehicle marker is enabled by default with probability threshold `0.5`.

Standalone benchmark and diagnostic utilities live in `benchmarks/` and are not
built by default:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build build -j2
```

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
