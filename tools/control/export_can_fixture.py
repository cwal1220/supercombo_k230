#!/usr/bin/env python3
"""녹화 events/NNN.bin(K230LOG1) 하나를 gtest_control_replay의 K230CAN1 픽스처로 내보낸다.

수신 CAN(CanRx) 레코드만 담고, 타임스탬프는 첫 프레임을 0으로 다시 잰다.
사용: export_can_fixture.py <events/NNN.bin> <out.k230can>
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
from recording_reader import RECORD_CAN_RX, iter_event_records  # noqa: E402

BATCH_HEADER = struct.Struct("<II")        # K230RecordedCanBatchHeader
RECORDED_FRAME = struct.Struct("<IIIII64s")  # K230RecordedCanFrame
FIXTURE_MAGIC = b"K230CAN1"
FIXTURE_RECORD = struct.Struct("<QIBB8s2x")  # timestamp_us, address, bus, len, data, pad


def export(src: Path, dst: Path) -> tuple[int, float]:
    frames: list[tuple[int, int, int, int, bytes]] = []
    for rec in iter_event_records(src):
        if rec.type != RECORD_CAN_RX:
            continue
        count, _dropped = BATCH_HEADER.unpack_from(rec.payload, 0)
        pos = BATCH_HEADER.size
        for _ in range(count):
            if pos + RECORDED_FRAME.size > len(rec.payload):
                break
            address, bus, _bus_time, length, _flags, raw = RECORDED_FRAME.unpack_from(rec.payload, pos)
            pos += RECORDED_FRAME.size
            if length <= 8:
                frames.append((rec.timestamp_ns, address, bus, length, raw[:8]))
    if not frames:
        raise ValueError(f"{src}: no CAN frames")
    start_ns = frames[0][0]
    out = bytearray(FIXTURE_MAGIC + struct.pack("<IIQ", 1, FIXTURE_RECORD.size, len(frames)))
    for ts, address, bus, length, raw in frames:
        out += FIXTURE_RECORD.pack((ts - start_ns) // 1000, address, bus, length, raw)
    dst.write_bytes(out)
    return len(frames), (frames[-1][0] - start_ns) / 1e9


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    count, duration_s = export(Path(sys.argv[1]), Path(sys.argv[2]))
    print(f"{sys.argv[2]}: {count} frames, {duration_s:.3f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
