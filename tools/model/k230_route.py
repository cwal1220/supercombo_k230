"""Reader for K230 recordd routes (frames.bin / event log / road.hevc).

Binary layouts mirror src/recording_format.h and src/k230_ipc.h. Struct sizes
are asserted against the payload sizes found in the stream, so a layout drift
fails loudly instead of decoding garbage.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

FRAME_INDEX_MAGIC = b"K230IDX1"
EVENT_LOG_MAGIC = b"K230LOG1"

RECORD_CAN_RX = 1
RECORD_CAN_TX = 2
RECORD_MODEL_STATE = 3
RECORD_CONTROL_STATE = 4
RECORD_PANDA_STATE = 5

FRAME_INDEX_RECORD = np.dtype([
    ("frame_id", "<u8"),
    ("capture_timestamp_ns", "<u8"),
    ("encode_index", "<u8"),
    ("file_offset", "<u8"),
    ("packet_size", "<u4"),
    ("flags", "<u4"),
])

# K230ControlState: natural alignment, no packing pragma; all members are
# 4/8-byte so the layout is padding-free except the trailing 8-byte pad.
CONTROL_STATE = np.dtype([
    ("timestamp_ns", "<u8"),
    ("enabled", "<u4"), ("engaged", "<u4"), ("active", "<u4"),
    ("should_send", "<u4"), ("path_usable", "<u4"), ("seeds_ready", "<u4"),
    ("vehicle_fresh", "<u4"), ("steering_fault", "<u4"),
    ("left_blinker", "<u4"), ("right_blinker", "<u4"), ("cruise_active", "<u4"),
    ("gear", "<i4"),
    ("speed_kph", "<f4"), ("cruise_max_speed_kph", "<f4"),
    ("cruise_command_speed_kph", "<f4"), ("steering_angle_deg", "<f4"),
    ("desired_curvature", "<f4"), ("actual_curvature", "<f4"),
    ("normalized_output", "<f4"),
    ("desired_torque", "<i4"), ("apply_torque", "<i4"), ("driver_torque", "<i4"),
    ("desire", "<u4"),
    ("active_block", "S32"),
    ("radar_lead_valid", "<u4"), ("radar_lead_distance_m", "<f4"),
    ("radar_lead_relative_speed_mps", "<f4"),
    ("departure_alert_type", "<u4"), ("departure_alert_event_id", "<u4"),
    ("green_light_alert_armed", "<u4"),
    ("tpms_valid", "<u4"), ("tpms_unit", "<u4"),
    ("tpms_pressure_fl", "<f4"), ("tpms_pressure_fr", "<f4"),
    ("tpms_pressure_rl", "<f4"), ("tpms_pressure_rr", "<f4"),
    ("tpms_warning", "<u4"), ("hud_flags", "<u4"),
    ("engage_event_id", "<u4"), ("disengage_event_id", "<u4"),
    ("engage_reject_event_id", "<u4"),
    ("engage_reject_block", "S32"),
    ("ego_speed_kph", "<f4"),
])

# The head of K230ModelState (frame_id ..). Only the fields these tools need are
# decoded; the plan/lane payload in the middle is skipped by offset.
# K230CalibrationState is the struct's last member, so it is located from the
# end of the payload.
MODEL_STATE_HEAD = np.dtype([
    ("frame_id", "<u8"),
    ("capture_timestamp_ns", "<u8"),
    ("model_timestamp_ns", "<u8"),
    ("model_execution_ms", "<f4"),
    ("valid", "<u4"),
    ("best_plan", "<i4"),
    ("plan_probability", "<f4"),
])

CALIBRATION_STATE = np.dtype([
    ("status", "<u4"),
    ("valid_blocks", "<i4"),
    ("roll", "<f4"), ("pitch", "<f4"), ("yaw", "<f4"),
    ("spread", "<f4", (3,)),
])

TRAJECTORY_SIZE = 33
IPC_POINT = 12  # K230IpcPoint: three float32


def model_state_layout(version: int) -> dict[str, int]:
    """Byte offsets inside a K230ModelState payload, per recording version.

    The struct lost fields as the runtime dropped things nothing consumed, so
    a reader that hardcodes one version silently misreads the others:
      v3 and older carry stop_line (payload 4076 + 4 pad = 4080)
      v4 dropped stop_line (4048)
      v5 dropped plan_position_stds and plan_orientations (3256)
    Callers should check ``layout["__size__"]`` against the payload size they
    actually saw so a future layout change fails loudly instead of decoding
    garbage.
    """
    n, pt = TRAJECTORY_SIZE, IPC_POINT
    fields = [("frame_id", 8), ("capture_timestamp_ns", 8),
              ("model_timestamp_ns", 8), ("model_execution_ms", 4),
              ("valid", 4), ("best_plan", 4), ("plan_probability", 4),
              ("model_t", 4 * n), ("lane_t", 4 * n), ("plan", pt * n)]
    if version <= 4:
        fields += [("plan_position_stds", pt * n), ("plan_orientations", pt * n)]
    fields += [("lanes", 4 * pt * n), ("lane_probabilities", 16),
               ("lane_stds", 16), ("road_edges", 2 * pt * n),
               ("road_edge_stds", 8), ("desire_state", 32), ("lead", 24)]
    if version <= 3:
        fields += [("stop_line", 28)]
    fields += [("pose", 52), ("calibration", 32)]

    layout, offset = {}, 0
    for name, size in fields:
        layout[name] = offset
        offset += size
    layout["__size__"] = offset
    return layout


@dataclass
class SegmentInfo:
    path: Path
    width: int
    height: int
    fps: int
    segment_start_ns: int
    frames: np.ndarray  # FRAME_INDEX_RECORD array


@dataclass
class RouteEvents:
    control: np.ndarray            # CONTROL_STATE records
    control_log_ts: np.ndarray     # event-header timestamps for control records
    model_frame_id: np.ndarray
    model_capture_ts: np.ndarray
    model_rpy: np.ndarray          # [N, 3] calibration roll/pitch/yaw (rad)
    model_valid_blocks: np.ndarray


def read_segment_index(segment_dir: Path) -> SegmentInfo:
    data = (segment_dir / "frames.bin").read_bytes()
    magic, version, header_size, record_size, width, height, fps = struct.unpack_from(
        "<8sIIIIII", data, 0)
    if magic != FRAME_INDEX_MAGIC:
        raise ValueError(f"{segment_dir}: bad frames.bin magic {magic!r}")
    segment_start_ns, = struct.unpack_from("<Q", data, 32)
    if record_size != FRAME_INDEX_RECORD.itemsize:
        raise ValueError(f"{segment_dir}: index record size {record_size} != "
                         f"{FRAME_INDEX_RECORD.itemsize}")
    # an unclean stop can truncate the file mid-record; keep whole records only
    count = (len(data) - header_size) // FRAME_INDEX_RECORD.itemsize
    frames = np.frombuffer(data, FRAME_INDEX_RECORD, count=count, offset=header_size)
    return SegmentInfo(segment_dir, width, height, fps, segment_start_ns, frames)


def route_segments(route_dir: Path) -> list[SegmentInfo]:
    seg_root = route_dir / "segments"
    segs = []
    for seg_dir in sorted(seg_root.iterdir()):
        if not (seg_dir.is_dir() and (seg_dir / "frames.bin").exists()
                and (seg_dir / "road.hevc").exists()):
            continue
        try:
            info = read_segment_index(seg_dir)
        except (ValueError, struct.error) as error:
            print(f"skipping {seg_dir}: {error}")
            continue
        if len(info.frames):
            segs.append(info)
    return segs


def decode_route_yuv(segments: list[SegmentInfo]):
    """Yield (index_record, y, u, v) across a whole route in stream order.

    The segments of a route are 60 s slices of one continuous HEVC stream, and
    the MVX encoder splits large access units (keyframes) across several
    dequeued buffers, so the per-record packet boundaries in frames.bin are not
    reliable AU boundaries. The bytes ARE in stream order though: feed them
    through one ffmpeg parser + decoder for the whole route and pair decoded
    frames with index records by order (one encoder output per index record).
    """
    import av
    import re
    from collections import deque

    codec = av.CodecContext.create("hevc", "r")

    def au_starts(payload: bytes) -> int:
        count = 0
        for match in re.finditer(b"\x00\x00\x01", payload):
            pos = match.end()
            if pos + 2 < len(payload) and ((payload[pos] >> 1) & 0x3F) <= 31 \
                    and (payload[pos + 2] >> 7) & 1:
                count += 1
        return count

    # The MVX encoder does not keep a 1:1 mapping between dequeued buffers
    # (= index records) and access units around large keyframes, so map each
    # AU (in stream order) to the record whose byte range starts it. The
    # decoder also skips AUs with the MVX RPS/POC nonconformance, so packets
    # are tagged with their AU index as pts to keep the pairing exact.
    au_records: list[tuple[np.void, int]] = []
    for segment in segments:
        data = (segment.path / "road.hevc").read_bytes()
        for rec in segment.frames:
            payload = data[int(rec["file_offset"]):
                           int(rec["file_offset"]) + int(rec["packet_size"])]
            for _ in range(au_starts(payload)):
                au_records.append((rec, segment.height))

    au_index = 0

    def decode(packet):
        nonlocal au_index
        if packet is not None:
            packet.pts = au_index
            au_index += 1
        for frame in codec.decode(packet):
            rec, vis_h = au_records[frame.pts]
            yuv = frame.reformat(format="yuv420p")
            h, w = frame.height, frame.width
            y = np.asarray(yuv.planes[0]).reshape(h, yuv.planes[0].line_size)[:, :w]
            u = np.asarray(yuv.planes[1]).reshape(h // 2, yuv.planes[1].line_size)[:, : w // 2]
            v = np.asarray(yuv.planes[2]).reshape(h // 2, yuv.planes[2].line_size)[:, : w // 2]
            yield rec, y[:vis_h], u[: vis_h // 2], v[: vis_h // 2]

    for segment in segments:
        data = (segment.path / "road.hevc").read_bytes()
        # only bytes covered by index records are trustworthy; an unclean stop
        # can leave a partially written tail
        last = segment.frames[-1]
        data = data[: int(last["file_offset"]) + int(last["packet_size"])]
        for chunk_start in range(0, len(data), 1 << 20):
            for packet in codec.parse(data[chunk_start : chunk_start + (1 << 20)]):
                yield from decode(packet)
    for packet in codec.parse(b""):
        yield from decode(packet)
    yield from decode(None)
    if au_index < len(au_records):
        raise ValueError(f"parser produced {au_index} AUs, expected {len(au_records)}")


def route_event_files(route_dir: Path) -> list[Path]:
    """Event log files in record order: v3 rotates 60 s chunks in events/,
    v2 and older keep a single route-level events.bin."""
    chunk_dir = route_dir / "events"
    if chunk_dir.is_dir():
        return sorted(chunk_dir.glob("*.bin"))
    legacy = route_dir / "events.bin"
    return [legacy] if legacy.exists() else []


def read_route_events(route_dir: Path) -> RouteEvents | None:
    paths = [p for p in route_event_files(route_dir) if p.stat().st_size > 32]
    if not paths:
        return None

    controls, control_ts = [], []
    model_fid, model_cts, model_rpy, model_blocks = [], [], [], []
    for path in paths:
        data = path.read_bytes()
        magic, version, header_size = struct.unpack_from("<8sII", data, 0)
        if magic != EVENT_LOG_MAGIC:
            raise ValueError(f"{path}: bad event log magic {magic!r}")
        offset = header_size
        end = len(data)
        while offset + 16 <= end:
            ts, rtype, flags, payload = struct.unpack_from("<QHHI", data, offset)
            offset += 16
            if offset + payload > end:
                break  # truncated tail from an unclean stop
            if rtype == RECORD_CONTROL_STATE:
                if payload != CONTROL_STATE.itemsize + _pad8(CONTROL_STATE.itemsize):
                    raise ValueError(f"control state payload {payload} != "
                                     f"{CONTROL_STATE.itemsize} (+pad)")
                controls.append(np.frombuffer(data, CONTROL_STATE, count=1, offset=offset)[0])
                control_ts.append(ts)
            elif rtype == RECORD_MODEL_STATE:
                head = np.frombuffer(data, MODEL_STATE_HEAD, count=1, offset=offset)[0]
                # v4 and v5 end exactly on the calibration block. v3 and older
                # still carried stop_line, which left 4 bytes of trailing
                # padding after it (payload 4080).
                tail_pad = 0 if version >= 4 else 4
                calib = np.frombuffer(
                    data, CALIBRATION_STATE, count=1,
                    offset=offset + payload - CALIBRATION_STATE.itemsize - tail_pad)[0]
                model_fid.append(head["frame_id"])
                model_cts.append(head["capture_timestamp_ns"])
                model_rpy.append((calib["roll"], calib["pitch"], calib["yaw"]))
                model_blocks.append(calib["valid_blocks"])
            offset += payload

    if not model_fid and not controls:
        return None
    return RouteEvents(
        control=np.array(controls, dtype=CONTROL_STATE),
        control_log_ts=np.array(control_ts, dtype=np.uint64),
        model_frame_id=np.array(model_fid, dtype=np.uint64),
        model_capture_ts=np.array(model_cts, dtype=np.uint64),
        model_rpy=np.array(model_rpy, dtype=np.float32).reshape(-1, 3),
        model_valid_blocks=np.array(model_blocks, dtype=np.int32),
    )


def _pad8(size: int) -> int:
    return (8 - size % 8) % 8


def read_route_calibration(route_dir: Path) -> np.ndarray | None:
    path = route_dir / "params" / "calibration.json"
    if not path.exists():
        return None
    rpy = json.loads(path.read_text()).get("rpy_rad")
    return np.asarray(rpy, dtype=np.float32) if rpy else None
