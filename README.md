# K230 native supercombo app

This is the final live app path for the K230 target. It uses the `supercombo`
kmodel generated from the ONNX graph where `Elu_223` is kept native by inserting
an identity depthwise 1x1 Conv before it.

Final runtime artifact:

- `models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel`

The original model is available in the openpilot fork:

- [supercombo.onnx](https://github.com/cwal1220/openpilot_c2/blob/master/selfdrive/modeld/models/supercombo.onnx)

The rewritten ONNX is an intermediate generated artifact and is intentionally
not tracked in this repository. See `model_tools/` for the rewrite/compile
scripts and exact reproduction commands.

Board prerequisites:

For a freshly flashed board, install the build/runtime helper packages first:

```sh
apt-get update
apt-get install -y \
  ca-certificates \
  cmake \
  curl \
  g++ \
  git \
  libdrm-dev \
  libusb-1.0-0-dev \
  libopencv-dev \
  make \
  python3
```

Package purpose:

- `g++`, `make`, `cmake`: board-native C/C++ build
- `libdrm-dev`: DRM headers used by the overlay/display path
- `libusb-1.0-0-dev`: optional panda USB/CAN bridge build
- `libopencv-dev`: OpenCV headers/libraries used by the overlay renderer
- `curl`, `ca-certificates`: `fetch_nncase_runtime.sh` download support
- `git`: fresh clone from GitHub
- `python3`: `k230_manager.py`

If the directory is copied to the board with all files already present, `git` is
not needed for building. It is only needed for a fresh repository checkout.

The flashed image must already include these runtime libraries/devices:
  - `libdisplay.so`
  - `libv4l2-drm.so`
  - `libdrm.so.2`

Build on the board:

```sh
cd /root/supercombo_native
./fetch_nncase_runtime.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./k230_manager.py models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

Build the optional panda bridge when `libusb-1.0-0-dev` is installed:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSUPERCOMBO_BUILD_PANDA=ON
cmake --build build -j2
```

Mac Docker cross-build:

The repository can also be cross-built from macOS using the Buildroot host
tools produced by `k230_linux_sdk`. This is required to keep the target glibc
ABI at or below the board's glibc 2.33. First fetch the nncase runtime deps and
build the `k230_canmv_01studio_defconfig` SDK output:

```sh
cd /Users/chan/Documents/supercombo_k230
./fetch_nncase_runtime.sh
```

Then configure and build with the SDK host wrapper and Xuantie toolchain. The
script defaults to the SDK and toolchain paths under
`/Users/chan/Documents/K230`; override `K230_LINUX_SDK_DIR` or
`K230_XUANTIE_TOOLCHAIN_DIR` when needed:

```sh
scripts/configure_mac_rv64.sh

docker run --rm --platform linux/amd64 \
  -v "$PWD":/work \
  -v /Users/chan/Documents/K230/downloads/k230_linux_sdk/output/k230_canmv_01studio_defconfig/host:/sdk \
  -v /Users/chan/Documents/K230/.docker-toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2:/opt/toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2 \
  -w /work \
  ubuntu:24.04 \
  bash -lc 'export PATH=/sdk/bin:$PATH; cmake --build build-k230-sdk -j$(nproc)'
```

Upload the rebuilt runtime files to the board:

```sh
K230_RSYNC_RSH="sshpass -p '<password>' ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password -o StrictHostKeyChecking=no" \
  scripts/upload_to_board.sh root@192.168.219.115
```

`build-k230-sdk/` is local generated output and is not tracked. Do not use a
generic Ubuntu riscv64 compiler for board binaries: it can link against a newer
glibc than the flashed K230 image provides.

The historical Makefile is still kept as a rollback build path:

```sh
make -j2
```

Openpilot-style split runtime:

```sh
cd /root/supercombo_native
./k230_manager.py models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

The split runtime gives the model pipeline priority and keeps display work to a
minimal passive overlay subscriber:

- `k230_manager.py`
  - starts/stops/restarts `k230_overlay`, `k230_camerad`, and `k230_modeld`
  - publishes `managerState` to `/dev/shm/k230_manager_state`
  - starts `k230_overlay`, then `k230_camerad`, then `k230_modeld`
  - runs `k230_modeld` at a higher priority than display overlay by default
- `k230_overlay`
  - is overlay-only despite the historical name
  - owns the LCD directly through `libdisplay`/DRM and does not use Qt or touch
  - opens `/dev/video1` through the verified `v4l2_drm` preview path
  - uses the ChanLKAS landscape layout: preview and ARGB overlay are both
    rendered at logical `800x480` and rotated together for the native `480x800`
    panel
  - subscribes to compact `modelState` and draws only plan/lane/road-edge/lead
    on a double-buffered ARGB8888 overlay plane
  - renders the K230 driving HUD adapted from `openpilot_c2_k230/previewd`,
    using the same full `800x480` composition: center speed, top
    `CRUISE`/`OPENPILOT` panels, middle `ROAD`/`SYSTEM` panels, bottom
    `VEHICLE`/`K7 CONTROL` panels, and a centered status alert
  - fills that layout with camera/model/display FPS, inference time,
    CPU/temperature/memory, process health, Panda state, vehicle speed,
    steering torque, and K7 control state
  - writes `/tmp/k230_display_ready` after preview/display setup is complete
    and several preview frames have actually been displayed
- `k230_camerad`
  - starts after the manager sees `/tmp/k230_display_ready`
  - captures `/dev/video2` as `NV12 512x256`
  - copies frames into `/dev/shm/k230_road_ai`, a 4-slot shared NV12 ring
  - publishes only frame metadata as `roadAiFrame`
- `k230_modeld`
  - subscribes to latest `roadAiFrame`, reads the shared frame slot, runs
    nncase, parses supercombo output, updates calibration/control diagnostics,
    and publishes compact `modelState`
- `k230_pandad` (optional)
  - enabled with `K230_ENABLE_PANDA=1` when built with
    `-DSUPERCOMBO_BUILD_PANDA=ON`
  - connects to panda over `libusb`, publishes compact panda health and CAN
    receive batches, and can relay raw `sendcan` batches
  - defaults to shadow mode: `K230_PANDA_TX=0`, so no CAN frames are transmitted
- `k230_k7_controlsd` (optional K7 controller)
  - enabled with `K230_ENABLE_CONTROL=1`
  - runs the openpilot-compatible lane planner and Acados lateral MPC in a
    worker, with the KIA K7 YG HEV torque controller and
    `LKAS11`/`CLU11`/`MDPS12` packer at 100 Hz
  - consumes model path, lane, road-edge, and vehicle-state IPC
  - publishes generated raw `sendcan` batches for `k230_pandad`
  - publishes compact `controlState` diagnostics for the display HUD
  - does not transmit by itself; actual TX still requires `K230_PANDA_TX=1`

Large AI frames are never sent through the small-message IPC. `k230_overlay`
does not consume the shared AI frame ring for display; preview stays on the
K230 `v4l2_drm` display path, while `k230_modeld` consumes the shared `512x256`
AI ring. This keeps the split runtime close to openpilot's process boundaries
without paying the cost of Cap'n Proto/cereal in v1.

The model path captures the AI stream as `NV12 512x256` through `/dev/video2`
crop/resize, prepares the YUV6 recurrent inputs, runs nncase runtime directly,
and publishes compact `modelState`. The overlay display process uses
`/dev/video1` for preview and `/dev/video2` remains dedicated to the AI stream.
The model input preparation always uses calibrated homography sampling followed
by `NV12 -> YUV6` packing. The 01Studio ISP output is already normalized to the
medmodel view, so the default source intrinsics are `fx=fy=910`, `cx=256`, and
`cy=47.6`; zero calibration therefore preserves the ISP pixels exactly.
`SUPERCOMBO_INPUT_WARP_FX/FY/CX/CY` can override these values for a separately
measured camera pipeline. The medmodel transform feeds the current and previous
frames into `input_imgs`.

Calibration/input-warp equivalence checks:

- `benchmarks/verify_calibration_equivalence.cc` is a host-only verifier for
  the openpilot-derived calibration and input-warp math. It checks the
  pose-based calibration state machine, manual-vs-online feedback policy,
  medmodel homography matrix, UV `transform_scale_buffer(0.5)` handling, and
  YUV6 plane order (`Y00, Y10, Y01, Y11, U, V`) without requiring nncase or K230
  display libraries.
- The verifier intentionally treats model-input feedback as roll-free, matching
  openpilot's `get_view_frame_from_road_frame(0, pitch, yaw, model_height)`
  extrinsic matrix. `rpyCalib` may contain a tiny roll internally in openpilot,
  but that roll is not fed into `modeld`.
- The verifier checks the ISP-normalized medmodel defaults and compares its
  matrix with the original openpilot formula.
- Automatic calibration requires both CAN `vEgo` and camera-odometry
  `trans[0]` above 15 mph, matching openpilot's acceptance gate.
- The final graph does not consume `big_input_imgs`; its compatibility input is
  initialized to zero once and skipped during per-frame preprocessing.

Run the host-only verifier:

```sh
cmake -S . -B /tmp/supercombo_k230_verify \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build /tmp/supercombo_k230_verify \
  --target verify_calibration_equivalence bench_input_warp_overhead -j2
./verify_calibration_equivalence
./bench_input_warp_overhead 3000
```

Latest local/Pi checks:

- macOS host: `verify_calibration_equivalence: PASS`
- Raspberry Pi 4 aarch64: `verify_calibration_equivalence: PASS`
- Raspberry Pi 4 warp timing over 3000 frames:
  direct pack `0.251 ms`, identity warp `1.821 ms`, pitch `1.836 ms`, so the
  calibrated warp adds about `1.57-1.58 ms/frame` on Cortex-A72.

Runtime structure:

- `src/main.cc`
  - app bootstrap only: config load, signal handling, live/replay dispatch.
- `src/app_config.*`
  - parses the small runtime option set once at startup.
- `src/input_source.*`
  - normalizes live `/dev/video2 NV12` and `SCNV12R1` replay files to
    `Nv12Frame`.
- `src/model_output.*`
  - owns the supercombo raw-output layout and exposes parsed plan, lanes, road
    edges, leads, and pose.
- `src/model_input_transform.*`
  - direct `NV12 -> calibrated warped YUV6` input transform. It fuses
    homography sampling and openpilot-compatible YUV6 packing without creating
    an intermediate RGB or warped image buffer.
- `src/calibration_service.*`
  - wraps pose-based online calibration, manual override, projection policy, and
    the model-input calibration feedback loop.
- `src/projection.*`
  - converts model road coordinates to display pixels. Default mode is
    the K230 road-frame projection used by the pre-refactor runtime;
    openpilot-style `view_from_calib` remains available for experiments.
- `src/overlay_renderer.*`
  - draws plan/lane/road-edge/lead overlay with OpenCV into the CPU ARGB8888
    buffer used by the split DRM overlay process and monolithic rollback app.
- `src/openpilot_lateral_planner.*`
  - applies openpilot lane probability/width logic, lane-change state, and the
    generated Acados lateral MPC solver to produce curvature targets.
- `src/lateral_control.*`, `src/k7_lateral_controller.*`
  - hold the planner target and apply openpilot lag-adjusted curvature before
    the validated K7 torque/CAN path.
- `src/supercombo_runtime.*`
  - owns the live/replay pipeline and thread coordination.
- `src/k230_ipc.*`
  - owns the `/dev/shm` latest-message channels and shared NV12 frame ring used
    by the split runtime.
- `src/k230_overlay.cc`, `src/k230_camerad.cc`, `src/k230_modeld.cc`
  - openpilot-style process split. `k230_overlay` is the direct DRM overlay process;
    `k230_camerad` and `k230_modeld` keep the camera/model path independent.
- `k230_manager.py`
  - minimal supervisor and heartbeat publisher. It is intentionally not a full
    openpilot manager clone.
- `src/panda_client.*`, `src/k230_pandad.cc`
  - optional panda USB bridge. It handles USB, health, heartbeat, receive CAN,
    and the final TX gate, but does not generate vehicle control messages.
- `src/k230_k7_controlsd.cc`, `src/k7_lateral_controller.*`
  - standalone K7 YG HEV lateral controller using the validated Hyundai CAN
    bus split, torque limits, counters, checksums, 60 kph MDPS helper, and a
    20 Hz planner worker separated from the 100 Hz control loop.

Useful runtime options:

- `SUPERCOMBO_PROFILE=1`
  - prints model pipeline averages. `k230_overlay` also uses this for overlay
    draw/present timing.
- `SUPERCOMBO_CALIB_ROLL_DEG`, `SUPERCOMBO_CALIB_PITCH_DEG`,
  `SUPERCOMBO_CALIB_YAW_DEG`
  - overrides the overlay projection calibration in degrees. Defaults are all
    zero. If any of these are set, manual projection calibration wins and online
    calibration is not applied to the overlay.
- `SUPERCOMBO_CALIB_AUTO=0`
  - disables pose-based online overlay calibration and keeps the fixed
    zero/manual projection.
- `SUPERCOMBO_PROJECTION_MODE=openpilot|legacy`
  - selects the overlay projection math. Default is `legacy`, matching the
    pre-refactor K230 road-frame projection. `openpilot` enables the openpilot
    `view_from_calib` rotation for comparison.
- `SUPERCOMBO_LOG_CALIB=1`
  - prints the online calibrator status, accepted/rejected sample counts,
    valid block count, rpy, and spread.
- `SUPERCOMBO_INPUT_WARP_ROLL_DEG`, `SUPERCOMBO_INPUT_WARP_PITCH_DEG`,
  `SUPERCOMBO_INPUT_WARP_YAW_DEG`
  - optional model-input warp calibration in degrees. If unset, the input warp
    reuses `SUPERCOMBO_CALIB_*` values. Without a manual override, pose-based
    online calibration feeds back into the next frame's model input warp,
    matching openpilot's `cameraOdometry -> liveCalibration -> modeld` loop.
- `SUPERCOMBO_INPUT_WARP_FX`, `SUPERCOMBO_INPUT_WARP_FY`,
  `SUPERCOMBO_INPUT_WARP_CX`, `SUPERCOMBO_INPUT_WARP_CY`
  - optional camera intrinsics for the resized source frame. Defaults are
    derived from the calibrated OV5647 `1920x1080` matrix and the active
    crop/resize geometry.
- `SUPERCOMBO_REPLAY_NV12=/path/to/replay.scnv12`
  - runs headless from a preconverted `512x256 NV12` replay file instead of
    opening the camera and display. This is for validating inference and online
    calibration from collected logs.
  - also works with `k230_modeld` directly for split-runtime parser/model tests.
- `SUPERCOMBO_MAX_FRAMES=N`
  - stops after `N` inferred frames. This is mainly useful with replay mode.
- `K230_ENABLE_PANDA=1`
  - manager also starts `k230_pandad`. The binary must have been built with
    `-DSUPERCOMBO_BUILD_PANDA=ON`.
- `K230_ENABLE_CONTROL=1`
  - manager starts `k230_pandad` and `k230_k7_controlsd`.
  - no openpilot checkout or Python native extension is required.
- `K230_PANDA_SAFETY=nooutput|silent|hyundai|hyundaiCommunity|allOutput`
  - panda safety mode for `k230_pandad`. Default is `nooutput`.
  - collected KIA K7 YG HEV logs from the current openpilot fork report
    `safety=hyundaiCommunity:0`, `sccBus=-1`, `mdpsBus=1`, and `sasBus=1`.
    Use `K230_PANDA_SAFETY=hyundaiCommunity` for shadow/TX experiments unless
    a newer fingerprint proves otherwise.
- `K230_PANDA_TX=1`
  - allows `k230_pandad` to relay raw `/dev/shm/k230_sendcan` batches to panda.
    Default is `0`; shadow mode blocks transmission.
- `K230_PANDA_ENGAGED=1`
  - sends panda heartbeat as engaged, only meaningful with `K230_PANDA_TX=1`.
    Default is disengaged.
- `K230_PANDA_IDLE_US=5000`
  - sleep time used by `k230_pandad` when panda returns no CAN frames and no
    pending `sendcan` batch exists. This keeps USB-only or parked shadow runs
    from stealing scheduler time from `k230_modeld`.
- `K230_K7_CONTROL=0|1`
  - enables the standalone controller. Default is `1`; Panda TX remains
    independently blocked unless `K230_PANDA_TX=1`.
- `K230_K7_FORCE_ENGAGED=0|1`
  - bypasses the SET/CANCEL engage latch for offline replay only. Default is
    `0` and must remain `0` in a vehicle.
- `K230_K7_STEERING_PARAMS=/path/to/steering_params.json`
  - optionally overrides the validated built-in K7 steering parameters.

Fixed production defaults:

- AI capture is fixed at `/dev/video2`, `NV12 512x256`, full sensor crop
  `1920x1080+0+0`.
- Preview is fixed at `/dev/video1`; the manager waits for
  `/tmp/k230_display_ready` before opening the AI stream.
- The ready barrier waits for 30 displayed preview frames and times out after
  7000 ms.
- Child process priorities are fixed as `camerad=0`, `modeld=-5`,
  `overlay=10`, optional `pandad=10`, optional `controlsd=5`.
- The front-vehicle marker is always enabled with probability threshold `0.5`.

Benchmark and diagnostic utilities live under `benchmarks/` and are not built
by default. Build them explicitly with:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build build -j2
```

CAN/panda payload checks can be run on a host before connecting the car:

```sh
python3 benchmarks/check_k230_can_payload.py
cmake --build build --target check_panda_can_codec -j2
./check_panda_can_codec
```

Create a replay file on the host from collected openpilot logs:

```sh
python3 ../export_replay_nv12.py \
  --segment-dir /path/to/segment-with-fcamera \
  --out /tmp/replay_120.scnv12 \
  --frames 120
```

Then copy it to the board and run:

```sh
SUPERCOMBO_REPLAY_NV12=/root/supercombo_native/replay_120.scnv12 \
  ./supercombo.elf models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```
