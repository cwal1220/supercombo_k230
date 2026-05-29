# KIA K7 YG HEV panda port plan

## Current decision

- Vehicle target: KIA K7 YG HEV (`CAR.K7_HEV_YG` in the current openpilot fork).
- Panda firmware: keep the currently flashed firmware. Do not rebuild firmware on
  the K230 board.
- CAN generation: do not reimplement Hyundai/Kia LKAS messages in C++ for v1.
  Reuse the current openpilot Python `CarInterface`/`CarController`/DBC packer
  path because it already handles `LKAS11`, optional `MDPS12`, checksum, counters,
  bus mirroring, and K7 HEV safety parameters.

## Stage 1: shadow panda bridge

- `k230_pandad` owns panda USB through `libusb`.
- It publishes panda health to `/dev/shm/k230_panda_state`.
- It publishes received CAN batches to `/dev/shm/k230_can`.
- It creates `/dev/shm/k230_sendcan`, but CAN transmission is blocked by default.
- Default safety mode is `nooutput`.
- With only USB connected and no vehicle harness, expected health is
  approximately `voltage=5V`, `ignition_line=0`, `ignition_can=0`, and
  `rx=0`. This confirms panda USB communication only; it does not validate
  vehicle CAN wiring.
- Empty-CAN polling sleeps by default with `K230_PANDA_IDLE_US=5000`, and the
  manager starts `k230_pandad` at low priority so shadow mode does not steal
  modeld scheduler time.
- The collected K7 YG HEV logs from this fork report:
  - `sccBus=-1`
  - `mdpsBus=1`
  - `sasBus=1`
  - `safety=hyundaiCommunity:0`
- Therefore the shadow/TX candidate for this car is
  `K230_PANDA_SAFETY=hyundaiCommunity`, not the generic Hyundai hybrid
  `hyundai:param=2` path.

Build:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSUPERCOMBO_BUILD_PANDA=ON
cmake --build build -j2
```

Run shadow mode:

```sh
K230_ENABLE_PANDA=1 \
./k230_manager.py models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

## Stage 2: openpilot controller reuse

- `k230_controlsd.py` imports the existing openpilot Hyundai interface and
  forces the candidate to `CAR.K7_HEV_YG`.
- It is enabled with `K230_ENABLE_CONTROL=1`.
- It first builds a short CAN fingerprint from `/dev/shm/k230_can`, then calls
  `CarInterface.get_params(CAR.K7_HEV_YG, fingerprint)` so `sccBus`, `mdpsBus`,
  and safety model match the actual harness layout.
- Feed it:
  - `/dev/shm/k230_can` from `k230_pandad`
  - `/dev/shm/k230_model_state` from `k230_modeld`
  - the existing `LateralTarget` or a model path-derived desired curvature
- The existing openpilot controller produces raw CAN tuples.
- `k230_controlsd.py` publishes those tuples to `/dev/shm/k230_sendcan`.
- Keep `K230_PANDA_TX=0` until shadow logs prove counters, bus selection, and
  generated `LKAS11` payloads match the known-good openpilot behavior.

Run shadow controller:

```sh
K230_OPENPILOT_PATH=/root/openpilot_c2 \
K230_ENABLE_CONTROL=1 \
K230_PANDA_SAFETY=hyundaiCommunity \
./k230_manager.py models/supercombo_gemm_split3_iddwelu223_gru_splitplan_delta_int16a_uint8w_real80_noclip.kmodel 0
```

This still does not transmit CAN unless `K230_PANDA_TX=1` is set.

Local replay validation from collected logs:

```sh
uv venv /tmp/k230_openpilot_py311 --python 3.11
uv pip install --python /tmp/k230_openpilot_py311/bin/python \
  'scons==4.2.0' 'numpy<2' pycapnp Cython pycryptodome jinja2 crcmod

cd /Users/chan/Documents/openpilot_c2
PATH=/tmp/k230_openpilot_py311/bin:$PATH \
PYTHONPATH=/Users/chan/Documents/openpilot_c2:/Users/chan/Documents/openpilot_c2/pyextra \
ZMQ=1 \
scons -j4 opendbc/can/packer_pyx.so opendbc/can/parser_pyx.so

cd /Users/chan/Documents/supercombo_k230
rm -rf /tmp/k230_ipc_control_test
mkdir -p /tmp/k230_ipc_control_test
PATH=/tmp/k230_openpilot_py311/bin:$PATH \
PYTHONPATH=/Users/chan/Documents/openpilot_c2:/Users/chan/Documents/openpilot_c2/pyextra \
K230_OPENPILOT_PATH=/Users/chan/Documents/openpilot_c2 \
K230_IPC_DIR=/tmp/k230_ipc_control_test \
K230_CONTROLD_FINGERPRINT_MIN_ADDRS=20 \
./k230_controlsd.py

K230_IPC_DIR=/tmp/k230_ipc_control_test \
benchmarks/replay_controlsd_from_rlog.py \
  /Users/chan/Documents/openpilot_c2/device_collected/2026-05-23_1600_combined/part0_existing_device_realdata/1970-01-01--09-00-59--0/rlog.bz2 \
  --openpilot /Users/chan/Documents/openpilot_c2 \
  --max-can-events 1000 \
  --sleep-scale 0.2 \
  --curvature 0.01
```

Observed result on Mac with a Python 3.11 openpilot build:

- `can_events=1000`
- `can_frames=50716`
- `send_batches=901`
- `send_frames=3154`
- top generated send addresses:
  - `bus0:0x340`
  - `bus1:0x340`
  - `bus2:0x251`
  - `bus1:0x4f1`
- `k230_controlsd.py` runtime log reported `errors=0` after default-param shim
  fixes. Exact `send_batches` can vary slightly with controller startup timing
  during replay.

## Stage 3: controlled TX enable

- Enable real transmission only with all explicit gates set:
  - `K230_ENABLE_PANDA=1`
  - `K230_PANDA_TX=1`
  - `K230_PANDA_SAFETY=hyundaiCommunity`
  - `K230_PANDA_ENGAGED=1`
- Confirm panda health reports the expected safety model/param and
  `controls_allowed=1` before accepting steering output.
- Start with low-speed/private-road tests and log every send batch.

## Safety constraints

- `k230_pandad` must never create vehicle-specific steering messages.
- Raw send relay must remain disabled by default.
- The current openpilot CAN generation path is the source of truth for K7 YG HEV.
