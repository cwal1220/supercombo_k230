#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from k230_controlsd import (
    CAN_BATCH_HEADER,
    CAN_BATCH_MAX_FRAMES,
    CAN_BATCH_SIZE,
    CAN_DLC_LENGTHS,
    CAN_FRAME,
    CanFrame,
    decode_can_batch,
    encode_can_batch,
)


def expect(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    frames = [
        CanFrame(address=0x123, src=0, dat=b"\x00\x01\x02\x03\x04\x05\x06\x07"),
        CanFrame(address=0x18DA10F1, src=2, dat=b"\xaa\xbb\xcc\xdd\xee\xff\x10\x20"),
    ]
    payload = encode_can_batch(frames)
    expect(len(payload) == CAN_BATCH_SIZE, "encoded batch size")
    decoded = decode_can_batch(payload)
    expect(decoded == frames, "encode/decode round-trip")

    flagged = bytearray(CAN_BATCH_SIZE)
    CAN_BATCH_HEADER.pack_into(flagged, 0, 0, 1, 1, 0, 0)
    CAN_FRAME.pack_into(flagged, CAN_BATCH_HEADER.size, 0x123, 0, 0, 8, 0x3,
                        b"\x01\x02\x03\x04\x05\x06\x07\x08".ljust(64, b"\x00"))
    decoded = decode_can_batch(bytes(flagged))
    expect(len(decoded) == 1, "flagged frame decoded")
    expect(decoded[0].src == 320, "returned/rejected flags map to openpilot src")

    invalids = [
        CanFrame(address=0x20000000, src=0, dat=b"\x00" * 8),
        CanFrame(address=0x123, src=4, dat=b"\x00" * 8),
        CanFrame(address=0x123, src=0, dat=b"\x00" * 9),
    ]
    payload = encode_can_batch(frames + invalids)
    _ts, valid, count, dropped, _reserved = CAN_BATCH_HEADER.unpack_from(payload, 0)
    expect(valid == 1, "encoded batch valid")
    expect(count == len(frames), "invalid TX frames skipped")
    expect(dropped == len(invalids), "invalid TX frames counted as dropped")

    too_many = [CanFrame(address=0x300 + i, src=i % 4, dat=b"\x00" * 8)
                for i in range(CAN_BATCH_MAX_FRAMES + 3)]
    payload = encode_can_batch(too_many)
    _ts, _valid, count, dropped, _reserved = CAN_BATCH_HEADER.unpack_from(payload, 0)
    expect(count == CAN_BATCH_MAX_FRAMES, "batch count capped")
    expect(dropped == 3, "truncated TX frames counted as dropped")

    for length in CAN_DLC_LENGTHS:
        payload = encode_can_batch([CanFrame(address=0x100 + length, src=0, dat=bytes(length))])
        decoded = decode_can_batch(payload)
        expect(len(decoded) == 1 and len(decoded[0].dat) == length, f"DLC length {length}")

    print("check_k230_can_payload: ok")


if __name__ == "__main__":
    main()
