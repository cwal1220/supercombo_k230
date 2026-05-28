# K230 native supercombo app

This is the final live app path for the K230 target. It uses the `supercombo`
kmodel generated from the ONNX graph where `Elu_223` is kept native by inserting
an identity depthwise 1x1 Conv before it.

Final model artifacts:

- `models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel`

The matching ONNX is intentionally not stored in this repository because it is
larger than GitHub's normal file-size limit. Recreate or archive it separately
when model conversion work is needed.

Board prerequisites:

- `g++`, `cmake`, `make`
- `libdrm-dev`
- runtime libraries already present on the flashed image:
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
  - configures preview as `800x480 NV12` with DRM rotation for the native
    `480x800` panel
  - subscribes to compact `modelState` and draws only plan/lane/road-edge/lead
    on an ARGB8888 overlay plane
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

Large AI frames are never sent through the small-message IPC. `k230_overlay`
does not consume the shared AI frame ring for display; preview stays on the
K230 `v4l2_drm` display path, while `k230_modeld` consumes the shared `512x256`
AI ring. This keeps the split runtime close to openpilot's process boundaries
without paying the cost of Cap'n Proto/cereal in v1.

The model path captures the AI stream as `NV12 512x256` through `/dev/video2`
crop/resize, prepares the YUV6 recurrent inputs, runs nncase runtime directly,
and publishes compact `modelState`. The overlay display process uses
`/dev/video1` for preview and `/dev/video2` remains dedicated to the AI stream.

Runtime structure:

- `src/main.cc`
  - app bootstrap only: config load, signal handling, live/replay dispatch.
- `src/app_config.*`
  - parses all `SUPERCOMBO_*` environment options once at startup.
- `src/input_source.*`
  - normalizes live `/dev/video2 NV12` and `SCNV12R1` replay files to
    `Nv12Frame`.
- `src/model_output.*`
  - owns the supercombo raw-output layout and exposes parsed plan, lanes, road
    edges, leads, and pose.
- `src/calibration_service.*`
  - wraps pose-based online calibration, manual override, and projection policy.
- `src/projection.*`
  - converts model road coordinates to display pixels. Default mode is
    the K230 road-frame projection used by the pre-refactor runtime;
    openpilot-style `view_from_calib` remains available for experiments.
- `src/overlay_renderer.*`
  - draws plan/lane/road-edge/lead overlay with the CPU ARGB8888 raster path
    for both the split DRM overlay process and the monolithic rollback app.
- `src/lateral_control.*`
  - computes a diagnostic `LateralTarget` skeleton only; it does not send CAN or
    steering commands.
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

Useful runtime options:

- `SUPERCOMBO_PROFILE=1`
  - prints per-frame pipeline averages for preprocessing, tensor input, nncase
    run, and output handling.
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
- `SUPERCOMBO_LOG_POSE=1`
  - prints the supercombo pose head every 30 inferred frames.
- `SUPERCOMBO_LOG_CONTROL=1`
  - prints the draft lateral target derived from the best plan. It does not send
    CAN or steering commands.
- `SUPERCOMBO_DRAW_LEAD=0`
  - disables the front-vehicle marker. By default the app draws a small white
    triangle/red dot at the primary supercombo lead position.
- `SUPERCOMBO_LEAD_PROB_THRESHOLD`
  - minimum lead probability for drawing the front-vehicle marker. Default is
    `0.5`.
- `SUPERCOMBO_REPLAY_NV12=/path/to/replay.scnv12`
  - runs headless from a preconverted `512x256 NV12` replay file instead of
    opening the camera and display. This is for validating inference,
    raw-output dumps, pose logging, and online calibration from collected logs.
  - also works with `k230_modeld` directly for split-runtime parser/model tests.
- `SUPERCOMBO_MAX_FRAMES=N`
  - stops after `N` inferred frames. This is mainly useful with replay mode.
- `SUPERCOMBO_AI_START_PREVIEW_FRAMES=N`
  - monolithic `supercombo.elf` rollback option only. The split runtime does
    its startup barrier with `K230_DISPLAY_READY_PREVIEW_FRAMES`.
- `SUPERCOMBO_NV12_CROP_X/Y/WIDTH/HEIGHT`
  - overrides the `/dev/video2` crop rectangle for tuning.
- `SUPERCOMBO_NV12_WIDTH` and `SUPERCOMBO_NV12_HEIGHT`
  - override the capture size for experiments.
- `K230_OVERLAY_CMD`
  - manager-only override for the display overlay process. Keep the default
    `./k230_overlay` for onroad preview.
- `K230_OVERLAY_VIDEO_DEVICE=1`
  - selects the preview video device. Default is `/dev/video1`.
- `K230_DISPLAY_READY_FILE=/tmp/k230_display_ready`
  - changes the file path used by the display-ready barrier.
- `K230_DISPLAY_READY_PREVIEW_FRAMES=30`
  - number of displayed preview frames required before `k230_overlay` publishes
    the ready file.
- `K230_DISPLAY_READY_TIMEOUT_MS=7000`
  - maximum time the manager waits for display readiness before starting the
    camera/model pipeline anyway.
- `K230_OVERLAY_PROFILE=1`
  - prints average display-stage times for ARGB overlay drawing and DRM present.
- `K230_CAMERAD_NICE`, `K230_MODELD_NICE`, `K230_OVERLAY_NICE`
  - manager child niceness adjustments. Defaults are `0`, `-5`, and `10`,
    respectively, so model inference has priority over display overlay work.
- `K230_CAMERAD_CMD`, `K230_MODELD_CMD`
  - manager-only command overrides for experiments.
- `K230_NO_RESTART=1`
  - makes the manager exit when a child exits instead of restarting it.
- `K230_DISABLE_OVERLAY=1`, `K230_DISABLE_CAMERA=1`, `K230_DISABLE_MODEL=1`
  - manager-only switches for isolated process tests.

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
