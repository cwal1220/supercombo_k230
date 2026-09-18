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
- `src/model_temporal.h`
  - the supercombo temporal inputs (desire pulse history, feature buffer, the
    constant traffic-convention and nav inputs) without any nncase dependency,
    so `check_model_output_parser` can pin the v0.9.4 convention on the host.
- `src/model_input_transform.*`
  - direct `NV12 -> calibrated warped YUV6` input transform. It fuses homography
    sampling and openpilot-compatible YUV6 packing without creating an
    intermediate RGB or warped image buffer. Its compact fixed-point LUT is
    16 bytes/sample instead of 24, and the K230 build uses an exact C908 RVV
    kernel with a scalar fallback.
- `src/calibration_service.*`, `src/calibration_online.*`
  - wrap pose-based online calibration, manual override, projection policy, and
    the model-input calibration feedback loop.
- `src/projection.*`
  - converts model road coordinates through the openpilot-style `view_from_calib`
    matrix and compensates for the rotated `800x480` display.

## Planning and control

- `src/lateral_planner.*`
  - applies openpilot lane probability/width logic, lane-change state, and the
    lateral MPC in `src/lateral_mpc.*` to produce curvature targets. This is the
    only producer of `LateralTarget`.
- `src/lateral_mpc.*`
  - the lateral MPC itself: one Gauss-Newton SQP iteration per call over the
    openpilot 0.8.16 OCP, solved by a backward Riccati recursion. No external
    solver. See [Verification](verification.md#lateral-mpc-solver).
- `src/lateral_target.h`
  - declares `LateralTarget`, the planner-to-controller interface.
- `src/lateral_controller.*`, `src/lateral_torque.*`,
  `src/control_params.*`, `src/hyundai_can.*`
  - apply the planner's lag-adjusted curvature through the validated K7
    torque/CAN path.
- `src/lateral_path.*`
  - reduces `modelState` to the steering-usability gate (reach and point
    count). It computes no path geometry; curvature comes from the MPC.
- `src/adaptive_cruise.*`, `src/departure_alert.*`
  - vision cruise setpoint control and departure alerting.
- `src/can_frame.h`, `src/vehicle_can.*`, `src/hyundai_can.*`
  - `can_frame.h` holds the transport type and the K7 YG HEV address/bus table;
    `vehicle_can` decodes received frames into vehicle state, `hyundai_can`
    encodes LKAS11/CLU11/MDPS12 commands.

### Control safety holds

`src/control_holds.*` implements both holds as `PandaHealthGate` and
`PathHoldGate`; `check_control_replay` exercises their boundaries.
`k230_controlsd` tolerates a single malformed plan frame by holding the last
usable path for at most 150 ms; the normal 250 ms model freshness timeout remains
a hard safety gate, so a stale or invalid model still removes control. A
transient Panda health-snapshot gap is similarly limited to 100 ms; a fresh,
transport-ready `controls_allowed=0` is never held. Other health faults are
released after that short hold if they persist.

## Processes and IPC

- `src/ipc_messages.*`
  - every message that crosses `/dev/shm`: topic names, magics, channel headers,
    the `K230*State` snapshots with their `static_assert`s, and the
    `ParsedModelOutput` ↔ `K230ModelState` marshalling. Recording v5 stores
    `K230ModelState`, `K230ControlState`, and `K230PandaState` as-is, so their
    offsets are pinned here and tied to `kK230RecordingVersion`. Code that only
    reads or fills a message includes this and nothing else.
- `src/ipc_channels.*`
  - the `/dev/shm` channel implementations: latest-message channel, CAN queue,
    and the shared NV12 frame ring.
- `src/k230_overlayd.cc`, `src/k230_camerad.cc`, `src/k230_modeld.cc`
  - openpilot-style process split. `k230_overlayd` is the direct DRM overlay
    process; `k230_camerad` and `k230_modeld` keep the camera/model path
    independent.
- `src/overlay_renderer.*`
  - draws the HUD (panels, plan/lane/road-edge ribbons, lead marker, turn
    signals, alerts, traffic-signal sprites) with OpenCV into the CPU ARGB8888
    buffer used by the split DRM overlay process. Stateless apart from the
    preloaded sprites; the turn-signal phase comes from `k230_overlayd`.
- `src/overlay_state.*`
  - `OverlayHudState`, the `K230*State` → `OverlayHudState` mapping shared by
    `k230_overlayd` and `hud_snapshot`, the engage-block label table, and
    `OverlayAlertEvents`, which turns the controlsd event counters into the one
    piezo/toast alert a frame may play (baseline on first sight, rebaseline on a
    controlsd restart, reject > engage > disengage > departure). No OpenCV, so
    `check_overlay_state` pins all of it on the host.
- `src/system_monitor.*`
  - `/proc`, thermal-zone, and network sampling into `OverlayHudState`, called
    at 1 Hz by `k230_overlayd`.
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

- `src/utils_process.h`
  - what a process gets from the OS: environment variables (`env_flag` is the
    one boolean convention), the `params/` directory path, and the
    SIGINT/SIGTERM → stop-flag hookup used by every `k230_*d` main.
- `src/utils_math.h`
  - clamping, openpilot `interp`, degree/radian conversion.
- `src/utils_time.h`
  - `k230_now_ns` (`CLOCK_BOOTTIME`), the clock behind every timestamp that
    crosses a process boundary, and the freshness predicates for ns and
    CAN-seconds timestamps. Per-process scheduling may still use
    `std::chrono::steady_clock`.
- `src/utils_json.*`
  - minimal JSON value readers and the clamped `parse_json_optional_*` helpers
    used by every parameter loader.
