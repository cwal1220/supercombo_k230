# Source layout

[← Documentation index](../README.md)

## Configuration and input

- `src/app_config.*`
  - parses the small runtime option set once at startup.
- `src/input_source.*`
  - normalizes live `/dev/video2 NV12` and `SCNV12R1` replay files to
    `Nv12Frame`.

## Perception

- `src/model_output.*`
  - owns the supercombo raw-output layout and exposes parsed plan, lanes, road
    edges, leads, and pose. Also owns the shared `T_IDXS`/`X_IDXS`
    trajectory grids.
- `src/model_input_transform.*`
  - direct `NV12 -> calibrated warped YUV6` input transform. It fuses homography
    sampling and openpilot-compatible YUV6 packing without creating an
    intermediate RGB or warped image buffer. Its compact fixed-point LUT is
    16 bytes/sample instead of 24, and the K230 build uses an exact C908 RVV
    kernel with a scalar fallback.
- `src/calibration_service.*`, `src/online_calibrator.*`
  - wrap pose-based online calibration, manual override, projection policy, and
    the model-input calibration feedback loop.
- `src/projection.*`
  - converts model road coordinates through the openpilot-style `view_from_calib`
    matrix and compensates for the rotated `800x480` display.

## Planning and control

- `src/openpilot_lateral_planner.*`
  - applies openpilot lane probability/width logic, lane-change state, and the
    generated Acados lateral MPC solver to produce curvature targets. This is the
    only producer of `LateralTarget`.
- `src/lateral_target.h`
  - declares `LateralTarget`, the planner-to-controller interface.
- `src/lateral_controller.*`, `src/openpilot_torque_controller.*`,
  `src/steering_params.*`, `src/hyundai_steering.*`
  - apply the planner's lag-adjusted curvature through the validated K7
    torque/CAN path.
- `src/lateral_path.*`
  - adapts `modelState` into the steering-usability path gate.
- `src/adaptive_cruise.*`, `src/departure_alert.*`
  - vision cruise setpoint control and departure alerting.
- `src/vehicle_can.*`, `src/hyundai_can.*`
  - K7 YG HEV CAN decode/encode and vehicle state.

### Control safety holds

`k230_controlsd` tolerates a single malformed plan frame by holding the last
usable path for at most 150 ms; the normal 250 ms model freshness timeout remains
a hard safety gate, so a stale or invalid model still removes control. A
transient Panda health-snapshot gap is similarly limited to 100 ms; a fresh,
transport-ready `controls_allowed=0` is never held. Other health faults are
released after that short hold if they persist.

## Processes and IPC

- `src/k230_ipc.*`
  - owns the `/dev/shm` latest-message channels and shared NV12 frame ring used
    by the split runtime.
- `src/k230_overlayd.cc`, `src/k230_camerad.cc`, `src/k230_modeld.cc`
  - openpilot-style process split. `k230_overlayd` is the direct DRM overlay
    process; `k230_camerad` and `k230_modeld` keep the camera/model path
    independent.
- `src/overlay_renderer.*`
  - draws plan/lane/road-edge/lead overlay with OpenCV into the CPU ARGB8888
    buffer used by the split DRM overlay process. Owns the engage-block label
    table shared with `k230_overlayd`.
- `src/k230_recordd.cc`, `src/mvx_v4l2_encoder.*`, `src/recording_writer.*`
  - low-priority data recorder, direct MVX V4L2 M2M encoder, timestamp index,
    compact event log, route segmentation, and storage-reserve guard.
- `src/panda_client.*`, `src/panda_can_codec.*`, `src/k230_pandad.cc`
  - optional panda USB bridge. It handles USB, health, heartbeat, receive CAN,
    and the final TX gate, but does not generate vehicle control messages.
- `src/k230_controlsd.cc`
  - standalone K7 YG HEV lateral controller using the validated Hyundai CAN bus
    split, torque limits, counters, checksums, 60 kph MDPS helper, and a 20 Hz
    planner worker separated from the 100 Hz control loop.
- `scripts/k230_manager.py`
  - minimal supervisor and heartbeat publisher. It is intentionally not a full
    openpilot manager clone.

## Shared helpers

- `src/common_utils.*`
  - shared-memory channels, `env_flag` environment parsing, clamping, CAN signal
    extraction, and timestamp freshness.
- `src/json_utils.*`
  - minimal JSON value readers and the clamped `parse_json_optional_*` helpers
    used by every parameter loader.
