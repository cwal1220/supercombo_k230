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

`k230_recordd` writes `events.bin` per route with an 8-byte `K230LOG1` magic, a
version word, and fixed 16-byte record headers. The current version is `2`.

| Record type | Payload |
| --- | ---: |
| `CanRx` / `CanTx` | variable CAN batch |
| `ModelState` | 4080 B |
| `ControlState` | 240 B |
| `PandaState` | 96 B |

Version 1 recordings are not compatible: `ModelState` was 4384 B before the
unused lateral draft fields were removed.

## Related documents

- [Recovery procedures](recovery.md)
- [CAN stability plan](can_stability_plan.md)
- [Departure alerts](departure_alerts.md)
- [YG panda port notes](yg_panda_port.md)
