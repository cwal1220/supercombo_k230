# KIA K7 YG HEV Panda Port

## Runtime

- `k230_pandad` owns Panda USB through `libusb` and publishes CAN batches to
  the ordered `/dev/shm/k230_can` shared-memory ring queue.
- `k230_k7_controlsd` runs the standalone K7 controller at 100 Hz and publishes
  generated CAN batches to the ordered `/dev/shm/k230_sendcan` ring queue.
- `k230_pandad` is the final TX gate. `K230_PANDA_TX=0` is the default and
  prevents every generated frame from reaching USB.
- No openpilot checkout, Python DBC extension, or acados module is required on
  the K230 board.

The CAN queues have 64 slots, reject new batches instead of overwriting older
ones when full, and are drained in sequence order. Producer startup resets its
own queue generation, and `k230_pandad` drops TX batches older than 100 ms.
The one-second daemon logs expose queue depth, full/stale counts, Panda CAN
errors, blocked frames, heartbeat status, USB retries, and malformed RX batches.

The controller uses the validated K7 YG HEV bus split:

- RX bus 0: powertrain, cluster, SAS, brake, gear, and body state.
- RX bus 1: MDPS state.
- RX bus 2: camera `LKAS11` seed.
- TX bus 0: `LKAS11` at 100 Hz.
- TX bus 1: mirrored `LKAS11` at 100 Hz and `CLU11` at 50 Hz.
- TX bus 2: `MDPS12` at 100 Hz.

While steering is active below the MDPS threshold, the bus-1 `CLU11` helper
reports 60 kph (38 mph) and preserves the source decimal-speed field. This
matches the K7 branch in the reference openpilot controller.

Runtime parameters are stored in:

- `params/k7_yg_steering.json`: torque PID, steering limits, vehicle geometry,
  angle offset, and low-speed steering settings.
- `params/k7_yg_driving.json`: model/CAN freshness, inactive release duration,
  MDPS speed spoof, and lateral motion limits.
- `params/calibration.json`: generated camera calibration state. This file is
  preserved across application and board restarts.

## Build

Use the K230 Linux SDK host-toolchain cross-build procedure in the repository
`README.md`, then deploy with `scripts/upload_to_board.sh`.

The Buildroot SDK configuration must include `BR2_PACKAGE_LIBUSB=y`.

## Offline Validation

Export the collected drive log to the `K230CAN1` fixture format, then run:

```sh
cmake -S . -B build/host-checks \
  -DSUPERCOMBO_BUILD_RUNTIME=OFF \
  -DSUPERCOMBO_BUILD_BENCHMARKS=ON
cmake --build build/host-checks --target check_k7_control_replay
./check_k7_control_replay build/k7_drive.k230can
```

The 60.001 second K7 YG HEV fixture contains 43,273 CAN records. The expected
result is 5,970 messages each for bus-0 `LKAS11`, bus-1 `LKAS11`, and bus-2
`MDPS12`, plus 2,985 bus-1 `CLU11` messages. The checker also validates the
60 kph helper, model-path coordinate conversion, frame lengths, active ticks,
and torque bounds.

## Shadow Run

```sh
K230_ENABLE_CONTROL=1 \
K230_PANDA_TX=0 \
K230_PANDA_SAFETY=nooutput \
./scripts/k230_manager.py models/supercombo.kmodel 0
```

Use `hyundaiCommunity` only after the connected vehicle fingerprint and Panda
health confirm the expected K7 configuration. Collected logs from the current
vehicle report `mdpsBus=1`, `sasBus=1`, and `hyundaiCommunity:0`.

## TX Gates

Vehicle transmission requires every explicit setting below:

```sh
K230_ENABLE_CONTROL=1
K230_PANDA_TX=1
K230_PANDA_SAFETY=hyundaiCommunity
K230_PANDA_ENGAGED=1
```

Keep `K230_K7_FORCE_ENGAGED=0` in a vehicle. Engagement must come from the
vehicle SET/CANCEL button state. Before any closed-course TX test, verify Panda
USB RX, ignition, safety mode/param, `controls_allowed`, CAN freshness, checksum
counters, and zero blocked/error counts in shadow mode.
