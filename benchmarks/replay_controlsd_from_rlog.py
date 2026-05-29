#!/usr/bin/env python3
import argparse
import bz2
import sys
import time
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from k230_controlsd import (  # noqa: E402
    CAN_BATCH_SIZE,
    CAN_TOPIC,
    LATERAL_TARGET,
    MODEL_STATE_TOPIC,
    SENDCAN_TOPIC,
    CanFrame,
    LatestChannel,
    decode_can_batch,
    encode_can_batch,
    now_ns,
)


def load_events(openpilot_path: Path, rlog: Path):
    sys.path.insert(0, str(openpilot_path))
    from cereal import log  # noqa: WPS433

    return log.Event.read_multiple_bytes(bz2.decompress(rlog.read_bytes()))


def wait_sendcan(timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            return LatestChannel(SENDCAN_TOPIC, CAN_BATCH_SIZE, create=False)
        except FileNotFoundError:
            time.sleep(0.05)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay openpilot rlog CAN into k230_controlsd IPC.")
    parser.add_argument("rlog", type=Path)
    parser.add_argument("--openpilot", type=Path, default=Path("/Users/chan/Documents/openpilot_c2"))
    parser.add_argument("--max-can-events", type=int, default=1000)
    parser.add_argument("--sleep-scale", type=float, default=0.0)
    parser.add_argument("--curvature", type=float, default=0.01)
    parser.add_argument("--sendcan-timeout", type=float, default=3.0)
    args = parser.parse_args()

    can_pub = LatestChannel(CAN_TOPIC, CAN_BATCH_SIZE, create=True)
    model_pub = LatestChannel(MODEL_STATE_TOPIC, LATERAL_TARGET.size, create=True)
    model_pub.publish(LATERAL_TARGET.pack(1, 20.0, 0.0, 0.0, args.curvature))
    sendcan_sub = wait_sendcan(args.sendcan_timeout)

    events = load_events(args.openpilot, args.rlog)
    last_log_ns = 0
    can_events = 0
    can_frames = 0
    send_frames = 0
    send_batches = 0
    send_addr_counts = Counter()
    last_send_seq = 0
    start_ns = now_ns()

    for ent in events:
        try:
            if ent.which() != "can":
                continue
        except Exception:
            continue

        if args.sleep_scale > 0.0 and last_log_ns:
            dt = max(0, ent.logMonoTime - last_log_ns) * 1e-9 * args.sleep_scale
            if dt > 0:
                time.sleep(dt)
        last_log_ns = ent.logMonoTime

        frames = [CanFrame(address=int(msg.address), src=int(msg.src), dat=bytes(msg.dat))
                  for msg in ent.can]
        can_pub.publish(encode_can_batch(frames))
        can_events += 1
        can_frames += len(frames)

        if sendcan_sub is not None:
            seq, payload = sendcan_sub.read()
            if payload and seq != last_send_seq:
                last_send_seq = seq
                out_frames = decode_can_batch(payload)
                send_batches += 1
                send_frames += len(out_frames)
                send_addr_counts.update((frame.src, frame.address) for frame in out_frames)

        if can_events >= args.max_can_events:
            break

    elapsed = max((now_ns() - start_ns) * 1e-9, 1e-6)
    print(f"replay_controlsd: can_events={can_events} can_frames={can_frames} "
          f"send_batches={send_batches} send_frames={send_frames} "
          f"rate={can_events / elapsed:.1f} can_events/s")
    if send_addr_counts:
        top = " ".join(f"bus{bus}:{hex(addr)}x{count}"
                       for (bus, addr), count in send_addr_counts.most_common(12))
        print(f"replay_controlsd: send_top={top}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
