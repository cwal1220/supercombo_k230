# Verification

[← Documentation index](../README.md)

## Calibration and input-warp equivalence

- `diagnostics/verify_calibration_equivalence.cc` is a host-only verifier for the
  openpilot-derived calibration and input-warp math. It checks the pose-based
  calibration state machine, manual-vs-online feedback policy,
  medmodel/sbigmodel homography matrices, UV `transform_scale_buffer(0.5)`
  handling, and YUV6 plane order (`Y00, Y10, Y01, Y11, U, V`) without requiring
  nncase or K230 display libraries.
- The verifier intentionally treats model-input feedback as roll-free, matching
  openpilot's `get_view_frame_from_road_frame(0, pitch, yaw, model_height)`
  extrinsic matrix. `rpyCalib` may contain a tiny roll internally in openpilot,
  but that roll is not fed into `modeld`.
- The verifier checks the ISP-normalized medmodel defaults and compares its
  matrix with the original openpilot formula.
- The verifier compares every output byte from the compact lookup tables with the
  previous 24-byte/sample lookup format across 12 medmodel/sbigmodel pose cases.
  It also checks two-frame direct tensor history against the previous pack path
  with a full-buffer `memcmp`.
- Against openpilot's OpenCL interpolation on the actual `640x360` K230 source,
  the worst 12-case mean absolute pixel difference is `0.314/255` and the worst
  absolute difference is `7/255`. The paths use the same projection and YUV6
  layout, but are not bit-exact because openpilot quantizes coordinates to 1/32
  pixel with 15-bit coefficients while K230 uses 12-bit coefficients.
- Automatic calibration requires both CAN `vEgo` and camera-odometry `trans[0]`
  above 15 mph, matching openpilot's acceptance gate.
- The final graph keeps both visual towers live. `input_imgs` uses the
  910-pixel-focal medmodel virtual camera and `big_input_imgs` uses the
  455-pixel-focal sbigmodel virtual camera, matching the single-camera C2 path.

`scripts/run_host_checks.sh` already runs this verifier. To run it alone,
together with the warp benchmark:

```sh
cmake -S . -B build-host \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_DIAGNOSTICS=ON
cmake --build build-host \
  --target verify_calibration_equivalence bench_input_warp_overhead -j2
build-host/bin/verify_calibration_equivalence
build-host/bin/bench_input_warp_overhead 3000
```

## Lateral MPC solver

`src/lateral_mpc.*` replaces the prebuilt riscv64 acados/HPIPM runtime that used
to live in `deps/acados`. It solves the same OCP as openpilot 0.8.16's
`lateral_mpc_lib`. The problem was recovered from the generated solver's own
`.rodata` and cross-checked against openpilot's `lat_mpc.py`: T_IDXS shooting
nodes (16 intervals, 2.5 s), `idxbx=[2,3]` bounded at radians(90)/radians(50),
NONLINEAR_LS residuals `(y, (v+5)*psi, (v+5)*4*rate)`, ERK4 with one step per
interval, Gauss-Newton Hessian, and per-stage cost scaling by the time step.

Two deliberate reductions:

- The `x_ego` state is dropped. It appears in no cost term, no constraint, and no
  other state's dynamics, so the solver state is `(y, psi, curvature)`.
- One Gauss-Newton SQP iteration per call, with the LQ subproblem solved exactly
  by a backward Riccati recursion. This is the same real-time-iteration structure
  as acados' `SQP_RTI` with `qp_solver_iter_max = 1`, but the subproblem is
  solved to optimality instead of by a single interior-point step.

### Optimality (host, no second solver)

`diagnostics/check_lateral_mpc.cc` reimplements the dynamics and cost
independently and checks the warm-started fixed point across five scenarios
(standstill through 27 m/s): multiple-shooting defects stay below `1e-15` and the
central-difference gradient of the true objective below `1e-9` relative to the
cost. A wrong sensitivity in the RK4 forward VDE fails this check, since the
iteration would then settle where the linearized KKT holds but the true gradient
does not.

### A/B against the acados runtime (board, 2026-09-10)

400 cycles per speed with identical references, both solvers warm-started from
reset. `lockstep` feeds both the same initial curvature; `free` lets each feed
back its own.

| v (m/s) | curvature, all nodes | curvature_rate | curvature[0], free |
| --- | --- | --- | --- |
| 0 | 0 | 0 | 0 |
| 3 | 1.4e-5 | 2.7e-5 | 1.3e-6 |
| 12 | 1.3e-6 | 1.1e-6 | 1.3e-8 |
| 27 | 2.3e-8 | 4.4e-8 | 5.5e-9 |

Solve time on the board (C908, single core, with `k230_modeld` running, so the
minimum is the meaningful figure):

| Solver | min | p50 | p90 |
| --- | --- | --- | --- |
| acados | 964 us | 1368 us | 3246 us |
| `LateralMpc` | 33.5 us | 33.6 us | 33.8 us |

The millisecond tail is what mattered on a one-core board: 3.2 ms of the 50 ms
control cycle could disappear into a single solve.

### End-to-end replay

`planner_replay` over three segments of route `2026-09-07--02-21-46-703`
(3545 frames, 1217 of them at standstill, 0-76 kph). Both runs are board
binaries, so nothing here is host/target float noise; the host build produces a
byte-identical CSV to the board build.

| Segment | commanded curvature, max diff | RMS | signal range | frames differing |
| --- | --- | --- | --- | --- |
| 000 | 2.0e-6 | 1.8e-7 | 2.7e-3 | 27/1183 |
| 007 | 1.4e-5 | 6.3e-7 | 6.2e-3 | 50/1182 |
| 008 | 4.0e-6 | 2.3e-7 | 1.3e-2 | 34/1180 |

`mpcSolutionValid` and laneless mode never differ. The largest differences are at
low speed, matching the synthetic A/B; 1.4e-5 1/m is about 0.002 degrees of front
wheel angle, well below the torque command quantum.

To re-run the A/B, restore `deps/acados`, `benchmarks/acados_lateral_mpc.h`, and
`benchmarks/check_lateral_mpc_vs_acados.cc` from the commit that removed them
(they predate the `benchmarks/` -> `diagnostics/` rename).

### Steering-rate cost term (from 0.9.4)

openpilot 0.9.4's lat MPC splits the input cost into lateral-jerk and
steering-rate terms. Its steering-rate residual, `psi_accel / (v_ego + 0.1)`, is
the same quantity as this solver's `curvature_rate`, so the shipped weight of
700 transplants directly. All three of 0.9.4's extra terms were implemented and
swept over the three replay segments; only this one earned its place:

| Term | Effect at a sane weight | Verdict |
| --- | --- | --- |
| `steering_rate` (700) | command jerk -24.6% overall, -27.1% below 20 kph | kept |
| `lateral_jerk` (0.04) | -2.8%; it is the existing `rate` term with a different speed scaling | dropped as redundant |
| `lateral_accel` (0.02) | -0.0%; at a weight large enough to matter it distorts highway cornering | dropped |

Command jerk (RMS of `d(des)/dt` from `planner_replay`) with `steering_rate` at
700, by speed band:

| Segment | 0-5 kph | 5-20 | 20-50 | 50+ |
| --- | --- | --- | --- | --- |
| 000 | -42% | **+23%** | -14% | -3% |
| 007 | -61% | -40% | -8% | -4% |
| 008 | -46% | -31% | -13% | -3% |

Fourteen of the fifteen bands improve. The exception is a ~5 s creep window in
segment 000 (103 frames, 5-20 kph) where the model plan is collapsed to 8-18 m
and its references jump frame to frame; there the rate penalty makes the solver
lag a jumping reference and the command gets rougher. The effect scales
monotonically with the weight (+8.8% at 200, +23% at 700, +36% at 2000), so it is
a dial rather than a threshold: halving to 400 halves that regression and keeps
most of the benefit. `mpcSolutionValid` and laneless mode never differ, and the
command deviation from the pre-term baseline stays under 4.1e-3 1/m, confined to
below 20 kph (7e-5 above 50 kph).

Solve time is unchanged at 33.5 us on the board; the term is one extra weight in
the input Hessian.

### Rejected: more than one SQP iteration per cycle

Since a solve costs 33.5 us instead of 964 us, running 2-3 SQP iterations per
cycle looked like free tracking accuracy. It buys nothing, because the NLS
residuals are *linear* in the state and control at a fixed speed, so the
Gauss-Newton Hessian is exact and the only nonlinearity in the whole problem is
`sin/cos(psi)` at `|psi| < 0.05 rad`. One Newton step lands on the optimum.

Measured against the same frame solved to convergence, over a 200-cycle 20 Hz
sequence with a sinusoidal reference plus a 3.5 m lane-change step. The
controller consumes only `curvatures[0]` (which the initial-state equality fixes
to the passed value, so it is identical by construction) and `psis` interpolated
at the actuator delay, so the gap is reported as the equivalent curvature error
`dpsi / (v * delay)`:

| v | 1 iteration | 2 | 3 |
| --- | --- | --- | --- |
| 3 m/s | 5.3e-6 | 6.8e-8 | 9.4e-10 |
| 12 m/s | 2.5e-6 | 1.3e-8 | 8.6e-11 |
| 27 m/s | 4.5e-7 | 7.2e-10 | 1.3e-12 |

Cold start immediately after `reset()` is the same order (2.8e-6 at 3 m/s), so
the failure-recovery path needs no extra iterations either. For scale, one
iteration's residual suboptimality is ~40x smaller than the acados-vs-this-solver
difference already accepted on real logs, and ~2500x smaller than the command
range. `curvature_rates` are logged but never consumed by the controller.

### Rejected: per-node speed profile

0.9.4 passes the model's predicted speed profile per shooting node. This was
implemented and measured, then reverted. The profile itself is real (in a creep
window the model correctly predicts 3.4 -> 1.4 m/s over the horizon while the
plan only reaches 7.9 m, so the flat-speed assumption was clamping references to
the last knot), but the only source available without a recording-format change
is differentiating the plan's arc length, and that is unstable frame to frame at
low speed: the estimated speed jumped 3.7 -> 1.5 m/s between two frames 0.1 s
apart. Command jerk rose 1.3-1.8x on the two standstill-heavy segments, and a
physical acceleration-band clamp (+/-2.5 m/s^2 * t) only recovered half of it.
Revisit with the model's velocity head (plan knot floats 3-5, currently unparsed),
which needs `K230ModelState` to grow and the recording version to be bumped.

## Host self-tests

The same benchmark build produces self-checking binaries that need no board:

| Target | Covers |
| --- | --- |
| `check_control_replay` | K7 engage gates, torque limits, CAN frame build |
| `check_departure_alert` | departure alert state machine |
| `check_adaptive_cruise` | vision cruise button pacing and limits |
| `check_model_output_parser` | supercombo raw-output layout |
| `check_k230_can_queue` | shared-memory CAN queue |
| `check_panda_can_codec` | panda USB CAN packing/unpacking |
| `check_lateral_mpc` | lateral MPC optimality and solve time |

See [Diagnostics](diagnostics.md) for the build command and additional
on-board tools.
