"""Decode a recorded route into the exact tensors the device feeds the model.

The device warps one NV12 frame through two calibrated virtual cameras and
keeps a two-frame history per tower, so this yields, per new camera frame,
the uint8 12-channel pair for both towers. Frames come out in capture order;
nothing is resampled onto a fixed tick grid, because the runtime only infers
on new frames and an identical image pair reads as "not moving" to the model.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from k230_route import (decode_route_yuv, read_route_calibration,
                        read_route_events, route_segments)
from model_warp import WarpPair


def route_calibration(route: Path) -> np.ndarray:
    """rpy the device used on this route, for a matching host-side warp."""
    events = read_route_events(route)
    if events is not None and len(events.model_rpy):
        return np.median(events.model_rpy, axis=0).astype(np.float32)
    stored = read_route_calibration(route)
    if stored is None:
        raise SystemExit(f"{route}: no calibration in the recording")
    return stored


def iter_model_inputs(route: Path, rpy: np.ndarray | None = None,
                      skip: int = 0, limit: int | None = None):
    """Yield (frame_index, img12, big12, nv12) in capture order."""
    segments = route_segments(route)
    if not segments:
        raise SystemExit(f"{route}: no usable segments")
    if rpy is None:
        rpy = route_calibration(route)
    warp = WarpPair(segments[0].width, segments[0].height)
    warp.set_calibration(rpy)

    prev_med = prev_sbig = None
    emitted = 0
    for index, (_, y, u, v) in enumerate(decode_route_yuv(segments)):
        med, sbig = warp.warp(y, u, v)
        if prev_med is None:
            prev_med, prev_sbig = med, sbig
        if index >= skip:
            nv12 = np.empty(y.size + u.size + v.size, np.uint8)
            nv12[: y.size] = y.reshape(-1)
            uv = nv12[y.size:].reshape(y.shape[0] // 2, y.shape[1])
            uv[:, 0::2] = u
            uv[:, 1::2] = v
            yield (index,
                   np.concatenate([prev_med, med]),
                   np.concatenate([prev_sbig, sbig]),
                   nv12)
            emitted += 1
            if limit is not None and emitted >= limit:
                return
        prev_med, prev_sbig = med, sbig
