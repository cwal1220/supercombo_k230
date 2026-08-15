# Runtime options

[← Documentation index](../README.md)

Boolean environment variables follow one convention: unset or empty uses the
default, `0`/`false`/`no`/`off`/`n` (case-insensitive) is false, anything else is
true.

## Model and calibration

- `K230_KMODEL=/path/to/supercombo.kmodel`
  - overrides the model selected by `k230_manager.py`.
- `SUPERCOMBO_MODEL_FPS=N`
  - selects the model loop target from 1 to 30 FPS. The default is 20 FPS.
- `SUPERCOMBO_PROFILE=1`
  - prints model pipeline averages. `k230_overlayd` also uses this for overlay
    draw/present timing.
- `SUPERCOMBO_CALIB_ROLL_DEG`, `SUPERCOMBO_CALIB_PITCH_DEG`,
  `SUPERCOMBO_CALIB_YAW_DEG`
  - override the overlay projection calibration in degrees. If any value is set,
    manual projection calibration wins and online calibration is not applied to
    the overlay. Otherwise the saved calibration is restored and automatic
    calibration remains active.
- `SUPERCOMBO_CALIB_AUTO=0`
  - disables pose-based online overlay calibration and keeps the restored or
    manually supplied projection.
- `SUPERCOMBO_LOG_CALIB=1`
  - prints the online calibrator status, accepted/rejected sample counts, valid
    block count, rpy, and spread.

## Input warp

- `SUPERCOMBO_INPUT_WARP_ROLL_DEG`, `SUPERCOMBO_INPUT_WARP_PITCH_DEG`,
  `SUPERCOMBO_INPUT_WARP_YAW_DEG`
  - optional model-input warp calibration in degrees. If unset, the input warp
    reuses `SUPERCOMBO_CALIB_*` values. Without a manual override, pose-based
    online calibration feeds back into the next frame's model input warp,
    matching openpilot's `cameraOdometry -> liveCalibration -> modeld` loop.
- `SUPERCOMBO_INPUT_WARP_FX`, `SUPERCOMBO_INPUT_WARP_FY`,
  `SUPERCOMBO_INPUT_WARP_CX`, `SUPERCOMBO_INPUT_WARP_CY`
  - optional camera intrinsics for the resized source frame. Defaults are derived
    from the calibrated K230 `1920x1080` camera matrix.
- `SUPERCOMBO_WARP_SCALAR=1`
  - disables the C908 RVV input-warp kernel for diagnostics and uses the
    bit-exact scalar fallback.

## Storage and replay

- `K230_PARAMS_DIR=/path/to/params`
  - overrides the shared runtime parameter directory. The default is `params/`
    relative to the runtime working directory. Stable online calibration is
    stored atomically in `params/calibration.json` and restored before the first
    model frame. Manual `SUPERCOMBO_CALIB_*` values take precedence and seed this
    file.
- `K230_RECORD_ROOT=/path/to/recordings`
  - overrides the route output directory. The default is `recordings/` under the
    runtime working directory. Recording is toggled live through the web UI or
    `params/recording.json`; at least 5 GiB and 10% free space is kept.
- `K230_RECORD_CODEC=/dev/video0`
  - overrides the MVX hardware codec device used by `k230_recordd`.
- `SUPERCOMBO_REPLAY_NV12=/path/to/replay.scnv12`
  - when launching `k230_modeld` directly, runs headless from an `SCNV12R1` NV12
    replay file instead of opening the camera and display. Width, height, and
    frame count are read from the replay header. This is for validating inference
    and online calibration from collected logs.
- `SUPERCOMBO_MAX_FRAMES=N`
  - stops after `N` inferred frames. This is mainly useful with replay mode.

## Panda

- `K230_ENABLE_PANDA=1`
  - manager also starts `k230_pandad`. The binary must have been built with
    `-DSUPERCOMBO_BUILD_PANDA=ON`.
- `K230_PANDA_SAFETY=nooutput|silent|hyundai|hyundaiCommunity|allOutput`
  - panda safety mode for `k230_pandad`. Its standalone default is `nooutput`;
    the managed full pipeline defaults to `hyundaiCommunity:0`.
  - collected KIA K7 YG HEV logs from the current openpilot fork report
    `safety=hyundaiCommunity:0`, `sccBus=-1`, `mdpsBus=1`, and `sasBus=1`. Use
    `K230_PANDA_SAFETY=hyundaiCommunity` for shadow/TX experiments unless a newer
    fingerprint proves otherwise.
- `K230_PANDA_TX=1`
  - allows `k230_pandad` to relay ordered `/dev/shm/k230_sendcan` batches to
    panda. Its standalone default is `0`; the managed full pipeline defaults
    to `1`.
- `K230_PANDA_ENGAGED=1`
  - sends panda heartbeat as engaged, only meaningful with `K230_PANDA_TX=1`. Its
    standalone default is disengaged; the managed full pipeline defaults to
    engaged.
- `K230_PANDA_IDLE_US=5000`
  - sleep time used by `k230_pandad` when panda returns no CAN frames and no
    pending `sendcan` batch exists. This keeps USB-only or parked shadow runs from
    stealing scheduler time from `k230_modeld`.

## K7 control

- `K230_ENABLE_CONTROL=1`
  - manager starts `k230_pandad` and `k230_controlsd`. This is the manager
    default. No openpilot checkout or Python native extension is required.
- `K230_K7_CONTROL=0|1`
  - enables the standalone controller. Default is `1`; Panda TX remains
    independently blocked unless `K230_PANDA_TX=1`.
- `K230_K7_FORCE_ENGAGED=0|1`
  - bypasses the SET/CANCEL engage latch for offline replay only. Default is `0`
    and must remain `0` in a vehicle.
- `K230_K7_ADAPTIVE_CRUISE=0|1`
  - enables vision-based adjustment of the stock fixed-speed cruise setpoint.
    Default is `1`. The JSON defaults send at most one five-frame button pulse per
    second and pace repeated `SET-` commands with the measured vehicle
    deceleration response. It yields to driver buttons and pedals and never
    commands brakes; the driver remains responsible for braking when the stock
    cruise cannot maintain a safe following distance.
- `K230_K7_STEERING_PARAMS=/path/to/steering_params.json`
  - overrides the default `params/yg_steering.json` file.
- `K230_K7_DRIVING_PARAMS=/path/to/driving_params.json`
  - overrides `params/yg_driving.json`, which contains model/CAN freshness,
    inactive release, MDPS 60 kph spoof, and lateral motion limits.
- `K230_K7_ADAPTIVE_CRUISE_PARAMS=/path/to/adaptive_cruise.json`
  - overrides `params/yg_adaptive_cruise.json`. The web editor exposes it as a
    separate vision-cruise menu; valid changes are hot-reloaded on the next 100 Hz
    control tick without restarting the pipeline.

## Parameter server and display

- `K230_ENABLE_PARAM_SERVER=0|1`
  - starts the FastAPI parameter editor with the manager. It defaults to `1` when
    `K230_ENABLE_CONTROL=1`.
- `K230_PARAM_HOST=address`, `K230_PARAM_PORT=port`
  - select the parameter editor listen address and port. Defaults are
    `0.0.0.0:8080`.
- `K230_DISPLAY_PARAMS=/path/to/display.json`
  - overrides `params/display.json`. The web editor controls the active-high
    GPIO25 backlight and its 20 kHz PWM5 brightness without stopping the video
    pipeline.

## Piezo alerts

- `K230_PIEZO_BUZZER=0`
  - disables the passive-piezo PWM alerts while preserving LCD alerts. The
    default is enabled; if the board cannot access the PWM/IOMUX interfaces, the
    worker reports the failure and LCD alerts remain active.
- `K230_PIEZO_PIN=46|47`
  - selects the board piezo pin. The default is pin 46 (`PWM2`, ALT2); pin 47
    selects `PWM3`.

The board piezo path reads IOMUX through `/dev/mem` and applies the pin mux with
the board-provided `devmem` utility (normally `/sbin/devmem`). If that utility or
the PWM sysfs interface is unavailable, the pipeline continues with LCD alerts
only.

The piezo module also provides distinct `engage`, `disengage`, and `unable`
(engage refused) tones. Engagement uses a fixed-duty ascending sequence and
disengagement a descending sequence so passive-piezo playback remains clean. A
refused engage request shows its gate reason on the HUD and plays the `unable`
tone once.

## Parameter files

The tracked JSON files in `params/` are the source of truth for K7 steering,
driving, vision-cruise, recording, and display configuration. Changes are written
atomically. Control changes are signaled to `k230_controlsd` and also detected by
its 100 ms fallback poll, while recording and display changes are applied
directly by their owning processes.

`params/calibration.json` is also tracked as the initial calibration seed. The
runtime replaces it atomically when a stable calibration is learned, while
`scripts/upload_to_board.sh` preserves an existing runtime copy and installs the
repository copy under `params.defaults/` as a fallback.

## Live parameter editor

Open the editor at `http://<board-ip>:8080`. It can also be started directly:

```sh
python3 scripts/param_server.py --host 0.0.0.0 --port 8080
```

> [!WARNING]
> The editor has no authentication and writes steering parameters that
> `k230_controlsd` hot-reloads while driving. Expose it only on a trusted vehicle
> or development network.

## Production defaults

- AI capture defaults to `/dev/video2`, `NV12 640x360`, full sensor crop
  `1920x1080+0+0`. `SUPERCOMBO_NV12_WIDTH/HEIGHT` and
  `SUPERCOMBO_NV12_CROP_X/Y/WIDTH/HEIGHT` provide diagnostic overrides.
- Preview is fixed at `/dev/video1`; the manager waits for
  `/tmp/k230_display_ready` before opening the AI stream.
- The ready barrier waits for 30 displayed preview frames and times out after
  7000 ms.
- Child process nice levels are fixed as `camerad=0`, `modeld=-15`,
  `overlayd=10`, `recordd=15`, optional `pandad=-10`, optional `controlsd=-8`,
  and `param_server=10`.
- The front-vehicle marker is always enabled with probability threshold `0.5`.
- Final desired curvature is clamped to `0.3 1/m`; it is intentionally not a
  runtime tuning option.
