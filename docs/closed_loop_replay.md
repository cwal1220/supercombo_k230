# Closed-loop replay

[← Documentation index](../README.md)

`diagnostics/replay_closed_loop.cc` replays a recording with the vehicle
response simulated, so a control change moves the car instead of being scored
against a frozen recording.

## Why the open-loop replay was not enough

`replay_planner` holds the vehicle response at whatever the recording captured.
Change a gain and the measured curvature does not move, so the error the
controller sees is fiction. That is how the 2026-09-21 prediction for the
mid-curve torque dropout came out at `36% -> 14%` offline while the road gave
`43% -> 46%` (`p=0.699`).

The missing piece is not graphics, it is the plant. Curvature to vehicle pose is
already in the code (`TorqueController::estimate_actual_curvature`); only torque
to lateral acceleration was missing.

## Pose-corrected replay

The recorded perception is reused, not re-rendered. The simulated car
accumulates a pose difference against the recorded car,

    dpsi' = v * (k_sim - k_rec)
    dy'   = v * dpsi

and every model frame is rotated and translated into the simulated car's frame
before the planner sees it. Real camera output, real model output, closed loop.

This holds while the deviation stays inside what the camera saw. Measured on
`route_711` and `route_829`, `|dy|` p90 is `0.07 m` and the worst segment peak is
`0.56 m`, against a `2 m` clamp that never fired.

Driver torque and inactive ticks are disturbances the simulation cannot
reproduce, so those ticks resync to the recording and the next free-running
segment starts from the real pose. The driver gate has hysteresis
(`150` counts in, `60` counts and `0.5 s` out); a single threshold shatters
`route_711` into 847 segments with a median length of `0.06 s`, because driver
torque is noisy even hands-off (p50 `14`, p90 `156` while active). With the
hysteresis the same route gives 35 segments over `398 s`, the longest `68 s`.

## Plant

Torque to lateral acceleration, as a pure delay and a second-order lag. The DC
gain is `1/kf` — the assumption the feedforward is already built on — times a
speed-scheduled correction for the error in that assumption.

The plant is accepted on **closed-loop reproduction**, not on an open-loop fit.
Run with `SIM_OPEN_LOOP=1`, the perception is exactly what the car saw and the
controller feeds back the simulated angle; if the plant is right, the simulated
lateral acceleration tracks the recorded one. An earlier attempt to identify the
plant by fitting recorded torque against recorded angle gave `tau = 2210 ms` at
14% explained and was not usable.

Identified on `route_711`, `wn = 10 rad/s`, `zeta = 4`, `delay = 0`,
gain `0.7 / 0.35 / 0.7 / 1.2` at `3 / 8 / 15 / 25 m/s`:

| route | ticks | R² open-loop | R² closed-loop |
| --- | --- | --- | --- |
| `route_711` (fit) | 38163 | 0.941 | 0.924 |
| `route_829` (held out) | 146803 | 0.965 | 0.959 |

Per speed band on the held-out route: `0.90 / 0.93 / 0.96 / 0.96` for
`<20 / 20-35 / 35-55 / >55 km/h`.

## What the numbers do and do not support

A naive model that assumes the car follows the requested curvature exactly
already explains `R² = 0.82` on `route_711`. The plant removes 62% of the error
that model leaves (RMSE `0.059 -> 0.036 m/s²`), which is what the identification
is worth — not the `0.94` on its own.

Rankings survive plant uncertainty. Across four plants spanning
`wn 6-14`, `zeta 1.5-4`, `delay 0-3` and flat-vs-scheduled gain, `SAD 0.46`
scores worse than `SAD 0.34` on every metric every time, matching the road. The
absolute gap moves; the order does not. Rank configurations with this tool, do
not read absolute numbers off it.

The optimum `SAD` is not one of the things it can find. The identification puts
the transport delay at 0 and carries the lag in an over-damped pole, and
lookahead compensation is worth less against a pure lag than against a transport
delay. The sweep therefore improves monotonically toward `SAD = 0`, which the
road does not support.

Two more limits are structural: `live_bank_compensation` is off because
`ControlState` carries no ESP12 lateral acceleration, so road camber lands in
the residual, and driver torque up to `150` counts is left in free segments as
an unmodelled disturbance.

## Coverage

The hands-off requirement is not a detail, it decides which questions the tool
can answer. Sorted by lateral acceleration over both routes:

| band | time | driver torque p50 | in a free segment |
| --- | --- | --- | --- |
| straight, `|a| < 0.3` | 2057 s | 9-11 | 83-87% |
| moderate, `0.3-1.0` | 232 s | 26-74 | 34-55% |
| curve, `|a| >= 1.0` | 55 s | 231-245 | 0-6% |

Real curves are curves the driver is co-steering, so the tool covers almost none
of them. Lane keeping on straights and moderate bends is what it measures.

Feeding the recorded driver torque into the plant would raise that coverage and
must not be done: the driver's torque was produced by the driver reacting to the
real car, so replaying it hands the simulation the real trajectory through the
back door and freezes the one response an A/B is supposed to change.

## Metrics

`lane_y` is the lane centre offset in the simulated car's frame. Unlike `dy` it
does not reference the recorded trajectory, so it does not favour whichever
configuration produced the recording. Use it, with the tracking error and the
rate-limit binding fraction, for A/B.

## Cost

`route_829`, 24 minutes of driving, replays in `0.6 s` on an M5. A full sweep of
840 plant candidates over `route_711` takes 31 s at 8-way parallelism.
