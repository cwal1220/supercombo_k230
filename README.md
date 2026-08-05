# K230 x openpilot

<p align="center">
  <img src="docs/assets/k230-openpilot-k7-hero.png"
       alt="K230 x openpilot for the KIA K7 YG HEV"
       width="100%">
</p>

<p align="center">
  <strong>Native openpilot perception and lateral control on Kendryte K230</strong><br>
  KIA K7 YG HEV · dual-camera model input · Panda USB/CAN · 800x480 driving HUD
</p>

| Target | Vehicle | Model | Runtime |
| --- | --- | --- | --- |
| Kendryte K230 / 01Studio | KIA K7 YG HEV | openpilot supercombo | C++ split pipeline |

**[Board setup](#board-setup) · [Build and deploy](#build-and-deploy) · [Runtime](#split-runtime) · [Model pipeline](#model-pipeline) · [Options](#runtime-options) · [Diagnostics](#diagnostics)**

> [!WARNING]
> This is experimental vehicle-control software. Keep Panda safety enabled and
> validate changes in a controlled environment before road use.

## Overview

This is the live app path for the K230 target. The current artifact is a modern
six-input `driving_supercombo` model with uint8 image inputs, float16 temporal
context inputs and a float32 2576-value output.

### Model artifact

- `models/supercombo.kmodel`
- SHA-256: `49ed812db587d48c6dfdcc26d8e42d2e69a5d0717527bb3dd74dfe4f088bfed1`

The selected source is sunnypilot's `driving_supercombo.onnx` at commit
`1a07e4722853c0606b0e1caa8f300a371e342948`. Its selection, exact hashes,
six-input ABI, board measurements and safe rollout procedure are documented in
[`docs/modern_model_migration.md`](docs/modern_model_migration.md).

### Rebuild the model

The source ONNX and 288-sample calibration NPZ are kept outside Git because of
their size. The build script verifies their known hashes, performs the reviewed
modern-model lowering, compiles full-INT16 activations with UINT8 weights, and
verifies the final KModel hash before installation:

```sh
SOURCE_ONNX=/path/to/driving_supercombo.onnx \
CALIBRATION_NPZ=/path/to/full6_real_logging_calib.npz \
  scripts/build_supercombo_model.sh install
```

Use `onnx` or `kmodel` instead of `install` to stop before changing the tracked
artifact. Full details and exact intermediate hashes are in
[`models/README.md`](models/README.md).

## Board Setup

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
  python3 \
  python3-pip
```

### Package purpose

- `g++`, `make`, `cmake`: board-native C/C++ build
- `libdrm-dev`: DRM headers used by the overlay/display path
- `libusb-1.0-0-dev`: optional panda USB/CAN bridge build
- `libopencv-dev`: OpenCV headers/libraries used by the overlay renderer
- `curl`, `ca-certificates`: `scripts/fetch_nncase_runtime.sh` download support
- `git`: fresh clone from GitHub
- `python3`: `k230_manager.py`와 K7 파라미터 웹 서버
- `python3-pip`: FastAPI/uvicorn 설치

Install the parameter server dependencies:

```sh
python3 -m pip install -r scripts/requirements-param-server.txt
```

If the directory is copied to the board with all files already present, `git` is
not needed for building. It is only needed for a fresh repository checkout.

The flashed image must already include the K230 camera/display devices and these
target runtime libraries:

- `libdisplay.so`
- `libv4l2-drm.so`
- `libdrm.so.2`

## Build And Deploy

### Native board build

```sh
cd /root/supercombo_k230
./scripts/fetch_nncase_runtime.sh
cmake -S . -B build-native \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_PANDA=ON
cmake --build build-native -j2
cmake --install build-native --prefix /root/supercombo_k230
./k230_manager.py
```

The Panda build is required by the manager's default full-pipeline mode. For a
camera/model/display-only build, omit `SUPERCOMBO_BUILD_PANDA` and start the
manager with `K230_ENABLE_CONTROL=0 K230_ENABLE_PANDA=0`.

### macOS SDK cross-build

The repository can also be cross-built from macOS using the Buildroot host
tools produced by `k230_linux_sdk`. This is required to keep the target glibc
ABI at or below the board's glibc 2.33. First fetch the nncase runtime deps and
build the `k230_canmv_01studio_defconfig` SDK output:

```sh
cd /Users/chan/Documents/supercombo_k230
./scripts/fetch_nncase_runtime.sh
```

Then configure and build with the SDK host wrapper and Xuantie toolchain. The
script defaults to the SDK and toolchain paths under
`/Users/chan/Documents/K230`; override `K230_LINUX_SDK_DIR` or
`K230_XUANTIE_TOOLCHAIN_DIR` when needed:

```sh
scripts/configure_k230_cross_build.sh

docker run --rm --platform linux/amd64 \
  -v "$PWD":/work \
  -v /Users/chan/Documents/K230/downloads/k230_linux_sdk/output/k230_canmv_01studio_defconfig/host:/sdk \
  -v /Users/chan/Documents/K230/.docker-toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2:/opt/toolchain/Xuantie-900-gcc-linux-6.6.0-glibc-x86_64-V3.0.2 \
  -w /work \
  ubuntu:24.04 \
  bash -lc 'export PATH=/sdk/bin:$PATH; cmake --build build-k230-sdk -j$(nproc)'
```

### Upload to the board

Upload the rebuilt runtime files:

```sh
K230_SSH="sshpass -p '<password>' ssh" \
K230_SCP="sshpass -p '<password>' scp" \
  scripts/upload_to_board.sh root@192.168.219.111
```

The upload script reads binaries from `build-k230-sdk/bin` by default. Set
`K230_BUILD_DIR=build-native` for an on-board build or `K230_BIN_DIR` for a custom
binary directory. It verifies the modern model hash, stops the service, creates
a single rollback snapshot under `/root/supercombo_k230/rollback`, and leaves
the service stopped unless `K230_RESTART_AFTER_UPLOAD=1` is explicitly set.
Use `scripts/rollback_board.sh` to restore that snapshot.

Runtime tuning and calibration JSON files already present under `params/` are
never overwritten by the upload script. Repository defaults are copied to
`params.defaults/` and seed a runtime file only when that file does not exist.

All CMake executables are written below the selected build directory in `bin/`.
`build-k230-sdk/` is local generated output and is not tracked. Do not use a
generic Ubuntu riscv64 compiler for board binaries: it can link against a newer
glibc than the flashed K230 image provides.

## Split Runtime

### Managed startup

```sh
cd /root/supercombo_k230
./k230_manager.py
```

The no-argument command selects the installed `model/supercombo.kmodel` (or the
source-tree `models/supercombo.kmodel`), debug mode `0`, K7 control enabled,
Panda TX enabled, and the FastAPI parameter server enabled. Each setting can
still be overridden with its environment variable; the model and debug mode can
also be passed as command-line arguments.

On the board, the image-provided `/etc/init.d/S35supercombo_k230` starts the
same command automatically. `scripts/upload_to_board.sh` verifies that service
and removes the legacy `S95supercombo_k230` script if present. Manual service
controls are:

```sh
/etc/init.d/S35supercombo_k230 start
/etc/init.d/S35supercombo_k230 stop
/etc/init.d/S35supercombo_k230 restart
/etc/init.d/S35supercombo_k230 status
```

### Runtime processes

The split runtime gives the model pipeline priority and keeps display work to a
minimal passive overlay subscriber:

- `k230_manager.py`
  - supervises `k230_overlayd`, `k230_camerad`, `k230_modeld`, and
    `k230_recordd`, plus
    `k230_pandad`, `k230_controlsd`, and the parameter server when enabled
  - publishes `managerState` to `/dev/shm/k230_manager_state`
  - starts `k230_overlayd`, waits for display readiness, then starts camera,
    model, Panda, control, and parameter-server processes
  - runs `k230_modeld` at a higher priority than display overlay by default
- `k230_overlayd`
  - is overlay-only despite the historical name
  - owns the LCD directly through `libdisplay`/DRM and does not use Qt or touch
  - opens `/dev/video1` through the verified `v4l2_drm` preview path
  - uses the ChanLKAS landscape layout: preview and ARGB overlay are both
    rendered at logical `800x480` and rotated together for the native `480x800`
    panel
  - subscribes to compact `modelState` and draws only plan/lane/road-edge/lead
    on a double-buffered ARGB8888 overlay plane
  - renders the K230 driving HUD adapted from `openpilot_c2_k230/previewd`,
    using the full `800x480` composition: center speed, left-side
    `OPENPILOT`/`CONTROL`/`DRIVE`/`TPMS` panels, right-side
    `SYSTEM`/`HEALTH`/`CALIBRATION`/`LEAD` panels, and a centered status alert
  - fills that layout with camera/model/display FPS, inference time,
    CPU/temperature/memory/storage, process health, Panda state, vehicle speed,
    steering torque, and K7 control state
  - writes `/tmp/k230_display_ready` after preview/display setup is complete
    and several preview frames have actually been displayed
- `k230_camerad`
  - starts after the manager sees `/tmp/k230_display_ready`
  - captures `/dev/video2` as `NV12 640x360`
  - copies frames into `/dev/shm/k230_road_ai`, a 4-slot shared NV12 ring
  - publishes only frame metadata as `roadAiFrame`
- `k230_modeld`
  - subscribes to latest `roadAiFrame`, reads the shared frame slot, runs
    nncase, parses supercombo output, updates calibration/control diagnostics,
    and publishes compact `modelState`
- `k230_recordd`
  - follows the exact frame selected by `k230_modeld` instead of independently
    sampling the camera stream
  - keeps `/dev/video0` MVX ready, but while recording is off performs only one
    warm-up encode and then leaves the VPU idle
  - records `NV12 640x360` as 5 Mbps HEVC at 20 FPS with 60-second,
    independently decodable segments; `frames.bin` preserves each source frame
    ID and capture timestamp
  - records dedicated non-blocking CAN RX/TX copies plus model, control, and
    Panda state in `events.bin`, and snapshots all runtime JSON parameters
  - runs at nice level 15 so logging cannot take priority over model or control
- `k230_pandad` (optional)
  - enabled with `K230_ENABLE_PANDA=1` when built with
    `-DSUPERCOMBO_BUILD_PANDA=ON`
  - connects to panda over `libusb`, publishes compact panda health and ordered
    CAN receive batches, and can relay ordered `sendcan` batches
  - standalone default is shadow mode (`K230_PANDA_TX=0`); the manager's
    full-pipeline default enables TX
- `k230_controlsd` (optional K7 controller)
  - enabled with `K230_ENABLE_CONTROL=1`
  - runs the openpilot-compatible lane planner and Acados lateral MPC in a
    worker, with the KIA K7 YG HEV torque controller and
    `LKAS11`/`CLU11`/`MDPS12` packer at 100 Hz
  - consumes model path, lane, road-edge, and vehicle-state IPC
  - uses the vision lead distance and relative speed to adjust the stock
    fixed-speed cruise setting with rate-limited `SET-`/`RES+` CLU11 pulses;
    the first driver SET speed remains the maximum. Closing distance is
    projected through the measured 1.5 km/h/s vehicle response, repeated
    `SET-` pulses wait for that response, and `RES+` cannot immediately reverse
    a recent slowdown
  - publishes generated raw `sendcan` batches for `k230_pandad`
  - publishes compact `controlState` diagnostics for the display HUD
  - does not transmit by itself; actual TX still requires `K230_PANDA_TX=1`

Large AI frames are never sent through the small-message IPC. `k230_overlayd`
does not consume the shared AI frame ring for display; preview stays on the
K230 `v4l2_drm` display path, while `k230_modeld` consumes the shared `640x360`
AI ring and publishes its selected frame metadata to `k230_recordd`. This keeps
the split runtime close to openpilot's process boundaries
without paying the cost of Cap'n Proto/cereal in v1.

## Model Pipeline

The model path captures the AI stream as `NV12 640x360` through `/dev/video2`
crop/resize, samples independent `512x256` medmodel and sbigmodel views from
the same source frame, prepares four-frame YUV6 histories plus the modern
feature/desire context queues, runs nncase runtime directly, and publishes compact
`modelState`. The overlay display process uses
`/dev/video1` for preview and `/dev/video2` remains dedicated to the AI stream.
The model input preparation always uses calibrated homography sampling fused
with `NV12 -> YUV6` conversion. Compact fixed-point lookup tables and a C908
RVV indexed-gather kernel write the current frame directly into the second half
of each nncase image tensor; after inference, that half is copied to the
previous-frame half. The shared camera frame is copied once into a cacheable
buffer because C908 `vluxei32.v` is not reliable on the `/dev/shm` ring mapping.
No warped image, YUV6 staging tensor, or full `[previous,current]` pack buffer is
created. The auxiliary inputs use float16 exactly as required by the model ABI.
The source intrinsics are scaled from the calibrated K230 camera
matrix, so the default `640x360` path uses `fx=541.91`, `fy=528.66`,
`cx=315.38`, and `cy=179.11`.
The K230 OV5647 Brown-Conrady distortion coefficients are applied while
sampling the model input and while projecting the overlay. Replay uses the
same K230 camera calibration scaled to the replay frame dimensions.
`SUPERCOMBO_INPUT_WARP_FX/FY/CX/CY` can override these values for a separately
measured camera pipeline. The medmodel transform feeds `img`; the wider
sbigmodel transform independently feeds `big_img`. Each image input combines
the current frame with the frame selected by the model's four-frame cadence.
The selected full-INT16 KModel preserves the vision heads but its lateral
policy is quantization-sensitive. A confidence/uncertainty/width-gated fusion
uses the two inner lane lines for plan y/yaw only and automatically disengages
during lane changes. Set `SUPERCOMBO_LANE_PLAN_FUSION=0` only for A/B testing.

## Verification

### Calibration and input-warp equivalence

- `benchmarks/verify_calibration_equivalence.cc` is a host-only verifier for
  the openpilot-derived calibration and input-warp math. It checks the
  pose-based calibration state machine, manual-vs-online feedback policy,
  medmodel/sbigmodel homography matrices, UV `transform_scale_buffer(0.5)`
  handling, and YUV6 plane order (`Y00, Y10, Y01, Y11, U, V`) without requiring
  nncase or K230 display libraries.
- The verifier intentionally treats model-input feedback as roll-free, matching
  openpilot's `get_view_frame_from_road_frame(0, pitch, yaw, model_height)`
  extrinsic matrix. `rpyCalib` may contain a tiny roll internally in openpilot,
  but that roll is not fed into `modeld`.
- The verifier checks the ISP-normalized medmodel defaults and compares its
  matrix with the original openpilot formula.
- The verifier compares every output byte from the compact lookup tables with
  the previous 24-byte/sample lookup format across 12 medmodel/sbigmodel pose
  cases. It also checks two-frame direct tensor history against the previous
  pack path with a full-buffer `memcmp`.
- Against openpilot's OpenCL interpolation on the actual `640x360` K230 source,
  the worst 12-case mean absolute pixel difference is `0.314/255` and the worst
  absolute difference is `7/255`. The paths use the same projection and YUV6
  layout, but are not bit-exact because openpilot quantizes coordinates to
  1/32 pixel with 15-bit coefficients while K230 uses 12-bit coefficients.
- Automatic calibration requires both CAN `vEgo` and camera-odometry
  `trans[0]` above 15 mph, matching openpilot's acceptance gate.
- The final graph keeps both visual towers live. `img` uses the
  910-pixel-focal medmodel virtual camera and `big_img` uses the
  455-pixel-focal sbigmodel virtual camera, matching the single-camera C2 path.

Run the host-only verifier:

```sh
cmake -S . -B /tmp/supercombo_k230_verify \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build /tmp/supercombo_k230_verify \
  --target verify_calibration_equivalence bench_input_warp_overhead -j2
/tmp/supercombo_k230_verify/bin/verify_calibration_equivalence
/tmp/supercombo_k230_verify/bin/bench_input_warp_overhead 3000
```

## Source Layout

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
    homography and camera-distortion sampling with openpilot-compatible YUV6
    packing without creating an intermediate RGB or warped image buffer. Its
    compact fixed-point LUT is 16 bytes/sample instead of 24, and the K230
    build uses an exact C908 RVV kernel with a scalar fallback.
- `src/lane_plan_fusion.*`
  - guarded lateral/yaw correction from high-confidence inner lane lines; it
    preserves policy x/time/z/longitudinal outputs and skips lane changes.
- `src/calibration_service.*`
  - wraps pose-based online calibration, manual override, projection policy, and
    the model-input calibration feedback loop.
- `src/projection.*`
  - converts model road coordinates through the openpilot-style
    `view_from_calib` matrix and compensates for the rotated `800x480` display.
- `src/overlay_renderer.*`
  - draws plan/lane/road-edge/lead overlay with OpenCV into the CPU ARGB8888
    buffer used by the split DRM overlay process.
- `src/openpilot_lateral_planner.*`
  - applies openpilot lane probability/width logic, lane-change state, and the
    generated Acados lateral MPC solver to produce curvature targets.
- `src/lateral_control.*`, `src/lateral_controller.*`
  - expose model-side lateral diagnostics and apply the planner's lag-adjusted
    curvature through the validated K7 torque/CAN path. `k230_controlsd`
    tolerates a single malformed plan frame by holding the last usable path for
    at most 150 ms; the normal 250 ms model freshness timeout remains a hard
    safety gate, so a stale or invalid model still removes control. A transient
    Panda health-snapshot gap is similarly limited to 100 ms; a fresh,
    transport-ready `controls_allowed=0` is never held. Other health faults
    are released after that short hold if they persist.
- `src/k230_ipc.*`
  - owns the `/dev/shm` latest-message channels and shared NV12 frame ring used
    by the split runtime.
- `src/k230_overlayd.cc`, `src/k230_camerad.cc`, `src/k230_modeld.cc`
  - openpilot-style process split. `k230_overlayd` is the direct DRM overlay process;
    `k230_camerad` and `k230_modeld` keep the camera/model path independent.
- `src/k230_recordd.cc`, `src/mvx_v4l2_encoder.*`, `src/recording_writer.*`
  - low-priority data recorder, direct MVX V4L2 M2M encoder, timestamp index,
    compact event log, route segmentation, and storage-reserve guard.
- `k230_manager.py`
  - minimal supervisor and heartbeat publisher. It is intentionally not a full
    openpilot manager clone.
- `src/panda_client.*`, `src/k230_pandad.cc`
  - optional panda USB bridge. It handles USB, health, heartbeat, receive CAN,
    and the final TX gate, but does not generate vehicle control messages.
- `src/k230_controlsd.cc`, `src/lateral_controller.*`
  - standalone K7 YG HEV lateral controller using the validated Hyundai CAN
    bus split, torque limits, counters, checksums, 60 kph MDPS helper, and a
    20 Hz planner worker separated from the 100 Hz control loop.

## Runtime Options

- `K230_KMODEL=/path/to/supercombo.kmodel`
  - overrides the model selected by `k230_manager.py`.
- `SUPERCOMBO_MODEL_FPS=N`
  - selects the model loop target from 1 to 30 FPS. The default is 20 FPS.
- `SUPERCOMBO_PROFILE=1`
  - prints model pipeline averages. `k230_overlayd` also uses this for overlay
    draw/present timing.
- `SUPERCOMBO_CALIB_ROLL_DEG`, `SUPERCOMBO_CALIB_PITCH_DEG`,
  `SUPERCOMBO_CALIB_YAW_DEG`
  - overrides the overlay projection calibration in degrees. If any value is
    set, manual projection calibration wins and online calibration is not
    applied to the overlay. Otherwise the saved calibration is restored and
    automatic calibration remains active.
- `SUPERCOMBO_CALIB_AUTO=0`
  - disables pose-based online overlay calibration and keeps the restored or
    manually supplied projection.
- `SUPERCOMBO_LOG_CALIB=1`
  - prints the online calibrator status, accepted/rejected sample counts,
    valid block count, rpy, and spread.
- `K230_PARAMS_DIR=/path/to/params`
  - overrides the shared runtime parameter directory. The default is
    `params/` relative to the runtime working directory.
  - stable online calibration is stored atomically in
    `params/calibration.json` and restored before the first model frame.
    Manual `SUPERCOMBO_CALIB_*` values take precedence and seed this file.
- `K230_RECORD_ROOT=/path/to/recordings`
  - overrides the route output directory. The default is `recordings/` under
    the runtime working directory. Recording is toggled live through the web
    UI or `params/recording.json`; at least 5 GiB and 10% free space is kept.
- `K230_RECORD_CODEC=/dev/video0`
  - overrides the MVX hardware codec device used by `k230_recordd`.
- `SUPERCOMBO_INPUT_WARP_ROLL_DEG`, `SUPERCOMBO_INPUT_WARP_PITCH_DEG`,
  `SUPERCOMBO_INPUT_WARP_YAW_DEG`
  - optional model-input warp calibration in degrees. If unset, the input warp
    reuses `SUPERCOMBO_CALIB_*` values. Without a manual override, pose-based
    online calibration feeds back into the next frame's model input warp,
    matching openpilot's `cameraOdometry -> liveCalibration -> modeld` loop.
- `SUPERCOMBO_INPUT_WARP_FX`, `SUPERCOMBO_INPUT_WARP_FY`,
  `SUPERCOMBO_INPUT_WARP_CX`, `SUPERCOMBO_INPUT_WARP_CY`
  - optional camera intrinsics for the resized source frame. Defaults are
    derived from the calibrated K230 `1920x1080` camera matrix.
- `SUPERCOMBO_INPUT_DIST_K1`, `SUPERCOMBO_INPUT_DIST_K2`,
  `SUPERCOMBO_INPUT_DIST_P1`, `SUPERCOMBO_INPUT_DIST_P2`,
  `SUPERCOMBO_INPUT_DIST_K3`
  - optional Brown-Conrady lens-distortion override. Defaults come from the
    calibrated K230 OV5647 camera.
- `SUPERCOMBO_LANE_PLAN_FUSION=0|1`
  - confidence-gated correction for the quantization-sensitive policy lateral
    path and yaw. It uses only the two high-confidence inner lane lines,
    rejects implausible widths/uncertainty, and is disabled during a lane
    change. It is enabled by default; set `0` for A/B diagnostics.
- `SUPERCOMBO_WARP_SCALAR=1`
  - disables the C908 RVV input-warp kernel for diagnostics and uses the
    bit-exact scalar fallback.
- `K230_MODELD_RT_PRIORITY=0..99`
  - requests `SCHED_RR` for `k230_modeld`; the default is the lowest real-time
    priority, `1`, which sustained 20 FPS without disturbing the 100 Hz control
    loop in full-pipeline board validation. Set `0` to use the normal scheduler.
    The manager startup log reports both the requested and actual scheduler so
    missing privileges are visible.
- `SUPERCOMBO_REPLAY_NV12=/path/to/replay.scnv12`
  - when launching `k230_modeld` directly, runs headless from an `SCNV12R1`
    NV12 replay file instead of opening the camera and display. Width, height,
    and frame count are read from the replay header. This is for validating
    inference and online calibration from collected logs.
- `SUPERCOMBO_MAX_FRAMES=N`
  - stops after `N` inferred frames. This is mainly useful with replay mode.
- `K230_ENABLE_PANDA=1`
  - manager also starts `k230_pandad`. The binary must have been built with
    `-DSUPERCOMBO_BUILD_PANDA=ON`.
- `K230_ENABLE_CONTROL=1`
  - manager starts `k230_pandad` and `k230_controlsd`. This is the manager
    default.
  - no openpilot checkout or Python native extension is required.
- `K230_PANDA_SAFETY=nooutput|silent|hyundai|hyundaiCommunity|allOutput`
  - panda safety mode for `k230_pandad`. Its standalone default is `nooutput`;
    the managed full pipeline defaults to `hyundaiCommunity:0`.
  - collected KIA K7 YG HEV logs from the current openpilot fork report
    `safety=hyundaiCommunity:0`, `sccBus=-1`, `mdpsBus=1`, and `sasBus=1`.
    Use `K230_PANDA_SAFETY=hyundaiCommunity` for shadow/TX experiments unless
    a newer fingerprint proves otherwise.
- `K230_PANDA_TX=1`
  - allows `k230_pandad` to relay ordered `/dev/shm/k230_sendcan` batches to panda.
    Its standalone default is `0`; the managed full pipeline defaults to `1`.
- `K230_PANDA_ENGAGED=1`
  - sends panda heartbeat as engaged, only meaningful with `K230_PANDA_TX=1`.
    Its standalone default is disengaged; the managed full pipeline defaults to engaged.
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
- `K230_K7_ADAPTIVE_CRUISE=0|1`
  - enables vision-based adjustment of the stock fixed-speed cruise setpoint.
    Default is `1`. The JSON defaults send at most one five-frame button pulse
    per second and pace repeated `SET-` commands with the measured vehicle
    deceleration response. It yields to driver buttons and pedals and never
    commands brakes; the driver remains responsible for braking when the stock
    cruise cannot maintain a safe following distance.
- `K230_K7_STEERING_PARAMS=/path/to/steering_params.json`
  - overrides the default `params/yg_steering.json` file.
- `K230_K7_DRIVING_PARAMS=/path/to/driving_params.json`
  - overrides `params/yg_driving.json`, which contains model/CAN freshness,
    inactive release, MDPS 60 kph spoof, and lateral motion limits.
- `K230_K7_ADAPTIVE_CRUISE_PARAMS=/path/to/adaptive_cruise.json`
  - overrides `params/yg_adaptive_cruise.json`. The web editor exposes it
    as a separate vision-cruise menu; valid changes are hot-reloaded on the
    next 100 Hz control tick without restarting the pipeline.
- `K230_ENABLE_PARAM_SERVER=0|1`
  - starts the FastAPI parameter editor with the manager. It defaults to `1`
    when `K230_ENABLE_CONTROL=1`.
- `K230_PARAM_HOST=address`, `K230_PARAM_PORT=port`
  - select the parameter editor listen address and port. Defaults are
    `0.0.0.0:8080`.
- `K230_DISPLAY_PARAMS=/path/to/display.json`
  - overrides `params/display.json`. The web editor controls the active-high
    GPIO25 backlight and its 20 kHz PWM5 brightness without stopping the video
    pipeline.
- `K230_PIEZO_BUZZER=0`
  - disables the passive-piezo PWM alerts while preserving LCD alerts. The
    default is enabled; if the board cannot access the PWM/IOMUX interfaces,
    the worker reports the failure and LCD alerts remain active.
- `K230_PIEZO_PIN=46|47`
  - selects the board piezo pin. The default is pin 46 (`PWM2`, ALT2); pin 47
    selects `PWM3`.

The board piezo path reads IOMUX through `/dev/mem` and applies the pin mux with
the board-provided `devmem` utility (normally `/sbin/devmem`). If that utility
or the PWM sysfs interface is unavailable, the pipeline continues with LCD
alerts only.

The piezo module also provides distinct `engage`, `disengage`, and `unable`
(engage refused) tones. Engagement uses a fixed-duty ascending sequence and
disengagement a descending sequence so passive-piezo playback remains clean.
A refused engage request shows its gate reason on the HUD and plays the
`unable` tone once.

The tracked JSON files in `params/` are the source of truth for K7 steering,
driving, vision-cruise, recording, and display configuration. Changes are
written atomically. Control changes are signaled to `k230_controlsd` and
also detected by its 100 ms fallback poll, while recording and display changes
are applied directly by their owning processes.
`params/calibration.json` is also tracked as the initial calibration seed. The
runtime replaces it atomically when a stable calibration is learned, while
`scripts/upload_to_board.sh` preserves an existing runtime copy and installs
the repository copy under `params.defaults/` as a fallback.

### Live parameter editor

Open the editor at `http://<board-ip>:8080`. It has no authentication and must
only be exposed on a trusted vehicle or development network. It can also be
started directly:

```sh
python3 scripts/param_server.py --host 0.0.0.0 --port 8080
```

### Production defaults

- AI capture defaults to `/dev/video2`, `NV12 640x360`, full sensor crop
  `1920x1080+0+0`. `SUPERCOMBO_NV12_WIDTH/HEIGHT` and
  `SUPERCOMBO_NV12_CROP_X/Y/WIDTH/HEIGHT` provide diagnostic overrides.
- Preview is fixed at `/dev/video1`; the manager waits for
  `/tmp/k230_display_ready` before opening the AI stream.
- The ready barrier waits for 30 displayed preview frames and times out after
  7000 ms.
- Child process priorities are fixed as `camerad=0`, `modeld=-5`,
  `overlay=10`, optional `pandad=-10`, optional `controlsd=-8`.
- The front-vehicle marker is always enabled with probability threshold `0.5`.
- Final desired curvature is clamped to `0.3 1/m`; it is intentionally not a
  runtime tuning option.

## Diagnostics

Benchmark and diagnostic utilities live under `benchmarks/` and are not built
by default. Build them explicitly with:

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

### NV12 replay

To run an existing `SCNV12R1` replay through the split model process on the board:

```sh
SUPERCOMBO_REPLAY_NV12=/root/supercombo_k230/replay_120.scnv12 \
  ./k230_modeld model/supercombo.kmodel 0
```
