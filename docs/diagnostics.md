# Diagnostics

[← Documentation index](../README.md)

Benchmark and diagnostic utilities live under `benchmarks/` and are not built by
default. Build them explicitly with:

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

## NV12 replay

To run an existing `SCNV12R1` replay through the split model process on the
board:

```sh
SUPERCOMBO_REPLAY_NV12=/root/supercombo_k230/replay_120.scnv12 \
  ./k230_modeld model/supercombo.kmodel 0
```

## Recording format

`k230_recordd` writes the event log as 60 s chunks in `events/NNN.bin`, each
starting with an 8-byte `K230LOG1` magic, a version word, and fixed 16-byte
record headers. The current version is `3`.

| Record type | Payload |
| --- | ---: |
| `CanRx` / `CanTx` | variable CAN batch |
| `ModelState` | 4080 B |
| `ControlState` | 240 B |
| `PandaState` | 96 B |

Version 1 recordings are not compatible: `ModelState` was 4384 B before the
unused lateral draft fields were removed. Version 2 kept a single route-level
`events.bin`; CAN logging alone (~0.5 MB/s) filled the 988 MB tmpfs staging in
about 30 minutes on long drives and silently killed the rest of the recording,
which is why version 3 rotates event chunks alongside video segments.

## Lateral dataset extraction

`extract_lateral_dataset` replays a recording and writes one CSV row per
`ControlState` record (~62 Hz), joining the CAN state decoded by the runtime's
own `vehicle_can` so signs and scaling match the board exactly:

```sh
cmake --build build/host-checks --target extract_lateral_dataset -j2
./build/host-checks/bin/extract_lateral_dataset out.csv <route>/events/*.bin
```

Version 2 route-level `events.bin` files work as well; a file truncated by the
tmpfs fill stops at the zero-filled gap with the byte offset on stderr.

Columns: time, wheel speed, `active`/`desire`/`block`, steering angle, driver
and applied torque (`tq_norm` is sign-corrected and divided by `steer_max`),
ESP12 lateral/longitudinal acceleration and yaw rate, the live bank estimate
and its roll equivalent, requested and measured curvature, and the NNFF-style
future values `la_p03..p15` / `roll_p03..p15` taken from the recorded model
plan.

Two deviations from the runtime are deliberate. The bank filter runs at row
rate with a `dt`-derived alpha rather than the controller's fixed 100 Hz step,
and future lateral acceleration is `v * dpsi/dt` off the plan's yaw angles
because the recorded `ModelState` has no plan acceleration. Both were checked
against known results: straight-line bank reproduces -0.178 on the 8-28 route
(logged: -0.176), and the total-least-squares torque fit over the 8-19 route
returns `latAccelFactor` 4.00, the value that route was fit to.

## Related documents

- [Recovery procedures](recovery.md)
- [CAN stability plan](can_stability_plan.md)
- [Departure alerts](departure_alerts.md)
- [YG panda port notes](yg_panda_port.md)
