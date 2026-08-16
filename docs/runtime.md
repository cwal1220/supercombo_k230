# Split runtime

[← Documentation index](../README.md)

## Managed startup

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

## Runtime processes

The split runtime gives the model pipeline priority and keeps display work to a
minimal passive overlay subscriber.

### `k230_manager.py`

- supervises `k230_overlayd`, `k230_camerad`, `k230_modeld`, and `k230_recordd`,
  plus `k230_pandad`, `k230_controlsd`, and the parameter server when enabled
- publishes `managerState` to `/dev/shm/k230_manager_state`
- starts `k230_overlayd`, waits for display readiness, then starts camera,
  model, Panda, control, and parameter-server processes
- runs `k230_modeld` at a higher priority than display overlay by default

### `k230_overlayd`

- is overlay-only despite the historical name
- owns the LCD directly through `libdisplay`/DRM and does not use Qt or touch
- opens `/dev/video1` through the verified `v4l2_drm` preview path
- uses the ChanLKAS landscape layout: preview and ARGB overlay are both rendered
  at logical `800x480` and rotated together for the native `480x800` panel
- subscribes to compact `modelState` and draws only plan/lane/road-edge/lead on a
  double-buffered ARGB8888 overlay plane
- renders the K230 driving HUD adapted from `openpilot_c2_k230/previewd`, using
  the full `800x480` composition: center speed, left-side
  `OPENPILOT`/`CONTROL`/`DRIVE`/`TPMS` panels, right-side
  `SYSTEM`/`HEALTH`/`CALIBRATION`/`LEAD` panels, and a centered status alert
- fills that layout with camera/model/display FPS, inference time,
  CPU/temperature/memory/storage, process health, Panda state, vehicle speed,
  steering torque, and K7 control state
- writes `/tmp/k230_display_ready` after preview/display setup is complete and
  several preview frames have actually been displayed

### `k230_camerad`

- starts after the manager sees `/tmp/k230_display_ready`
- captures `/dev/video2` as `NV12 1280x720`
- copies frames into `/dev/shm/k230_road_ai`, an 8-slot shared NV12 ring
- publishes only frame metadata as `roadAiFrame`

### `k230_modeld`

- subscribes to latest `roadAiFrame`, reads the shared frame slot, runs nncase,
  parses supercombo output, updates calibration diagnostics, and publishes
  compact `modelState`

### `k230_recordd`

- follows the exact frame selected by `k230_modeld` instead of independently
  sampling the camera stream
- keeps `/dev/video0` MVX ready, but while recording is off performs only one
  warm-up encode and then leaves the VPU idle
- records `NV12 1280x720` as 5 Mbps HEVC at 20 FPS with 60-second, independently
  decodable segments; `frames.bin` preserves each source frame ID and capture
  timestamp
- writes the active route to a tmpfs staging directory (`K230_RECORD_STAGING`,
  default `/tmp/record_staging`) and a mover thread copies closed files to the
  SD card sequentially, so SD latency spikes never stall the record path
- records dedicated non-blocking CAN RX/TX copies plus model, control, and Panda
  state in `events.bin`, and snapshots all runtime JSON parameters
- runs at nice level 15 so logging cannot take priority over model or control

### `k230_pandad` (optional)

- enabled with `K230_ENABLE_PANDA=1` when built with `-DSUPERCOMBO_BUILD_PANDA=ON`
- connects to panda over `libusb`, publishes compact panda health and ordered CAN
  receive batches, and can relay ordered `sendcan` batches
- standalone default is shadow mode (`K230_PANDA_TX=0`); the manager's
  full-pipeline default enables TX

### `k230_controlsd` (optional K7 controller)

- enabled with `K230_ENABLE_CONTROL=1`
- runs the openpilot-compatible lane planner and Acados lateral MPC in a worker,
  with the KIA K7 YG HEV torque controller and `LKAS11`/`CLU11`/`MDPS12` packer
  at 100 Hz
- consumes model path, lane, road-edge, and vehicle-state IPC
- uses the vision lead distance and relative speed to adjust the stock
  fixed-speed cruise setting with rate-limited `SET-`/`RES+` CLU11 pulses; the
  first driver SET speed remains the maximum. Closing distance is projected
  through the measured 1.5 km/h/s vehicle response, repeated `SET-` pulses wait
  for that response, and `RES+` cannot immediately reverse a recent slowdown
- publishes generated raw `sendcan` batches for `k230_pandad`
- publishes compact `controlState` diagnostics for the display HUD
- does not transmit by itself; actual TX still requires `K230_PANDA_TX=1`

## IPC boundaries

Large AI frames are never sent through the small-message IPC. `k230_overlayd`
does not consume the shared AI frame ring for display; preview stays on the K230
`v4l2_drm` display path, while `k230_modeld` consumes the shared `1280x720` AI
ring and publishes its selected frame metadata to `k230_recordd`. This keeps the
split runtime close to openpilot's process boundaries without paying the cost of
Cap'n Proto/cereal in v1.

The lateral plan has a single producer: the Acados MPC inside `k230_controlsd`.
`modelState` carries perception output only.
