"""Numpy port of src/model_input_transform.cc.

Reproduces the device's fixed-point bilinear warp (kWeightBits=12) so the
student trains on the same pixel distribution the K230 runtime produces. The
output is the YUV6 tensor layout the supercombo models consume:
channels [Y(2x,2y), Y(2x,2y+1), Y(2x+1,2y), Y(2x+1,2y+1), U, V] at 128x256.
"""

from __future__ import annotations

import numpy as np

MODEL_W, MODEL_H = 512, 256
HALF_W, HALF_H = MODEL_W // 2, MODEL_H // 2
WEIGHT_BITS = 12
WEIGHT_SCALE = 1 << WEIGHT_BITS

# src/app_config.h: calibrated K230 camera at 1920x1080
CAMERA_FX_1080 = 1583.3981
CAMERA_FY_1080 = 1583.7622
CAMERA_CX_1080 = 954.9441
CAMERA_CY_1080 = 545.1774
CAMERA_HEIGHT_M = 1.22

GROUND_FROM_MEDMODEL = np.array([
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00],
    [-1.09890110e-03, 0.00000000e+00, 2.81318681e-01],
    [-1.84808520e-20, 9.00738606e-04, -4.28751576e-02],
], dtype=np.float32)

GROUND_FROM_SBIGMODEL = np.array([
    [0.00000000e+00, 7.31372216e-19, 1.00000000e+00],
    [-2.19780220e-03, 4.11497335e-19, 5.62637363e-01],
    [-5.46146580e-20, 1.80147721e-03, -2.73464241e-01],
], dtype=np.float32)


def default_intrinsics(width: int, height: int) -> tuple[float, float, float, float]:
    return (CAMERA_FX_1080 * width / 1920.0, CAMERA_FY_1080 * height / 1080.0,
            CAMERA_CX_1080 * width / 1920.0, CAMERA_CY_1080 * height / 1080.0)


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def projection_matrix(fx: float, fy: float, cx: float, cy: float,
                      rpy: np.ndarray, sbig: bool,
                      height_m: float = CAMERA_HEIGHT_M) -> np.ndarray:
    rot = _rotation_from_rpy(*np.asarray(rpy, dtype=np.float64))
    device_from_road = rot * np.array([1.0, -1.0, -1.0])
    view_from_road = device_from_road[[1, 2, 0], :]
    extrinsic = np.hstack([view_from_road,
                           np.array([[0.0], [height_m], [0.0]])])
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    camera_from_road = k @ extrinsic
    camera_from_ground = camera_from_road[:, [0, 1, 3]]
    ground_from_model = GROUND_FROM_SBIGMODEL if sbig else GROUND_FROM_MEDMODEL
    return (camera_from_ground @ ground_from_model.astype(np.float64)).astype(np.float32)


def _scale_transform(projection: np.ndarray, scale: float) -> np.ndarray:
    t_out = np.array([[1.0 / scale, 0, 0.5], [0, 1.0 / scale, 0.5], [0, 0, 1]],
                     dtype=np.float32)
    t_in = np.array([[scale, 0, -0.5 * scale], [0, scale, -0.5 * scale], [0, 0, 1]],
                    dtype=np.float32)
    return t_in @ projection @ t_out


class _PlaneMap:
    """Precomputed gather indices + fixed-point bilinear weights for one plane."""

    def __init__(self, projection: np.ndarray, src_w: int, src_h: int,
                 dst_scale: int, dst_x_offset: int, dst_y_offset: int):
        xs = np.arange(HALF_W, dtype=np.float32) * dst_scale + dst_x_offset
        ys = np.arange(HALF_H, dtype=np.float32) * dst_scale + dst_y_offset
        gx, gy = np.meshgrid(xs, ys)
        x0 = projection[0, 0] * gx + projection[0, 1] * gy + projection[0, 2]
        y0 = projection[1, 0] * gx + projection[1, 1] * gy + projection[1, 2]
        w0 = projection[2, 0] * gx + projection[2, 1] * gy + projection[2, 2]
        valid = np.abs(w0) > 1e-6
        with np.errstate(divide="ignore", invalid="ignore"):
            sx = np.where(valid, x0 / w0, 0.0)
            sy = np.where(valid, y0 / w0, 0.0)
        ix = np.floor(sx).astype(np.int64)
        iy = np.floor(sy).astype(np.int64)
        ix[~valid] = -2
        iy[~valid] = -2
        ax = (sx - ix).astype(np.float32)
        ay = (sy - iy).astype(np.float32)
        weights = np.stack([(1 - ax) * (1 - ay), ax * (1 - ay),
                            (1 - ax) * ay, ax * ay])
        corner_x = np.stack([ix, ix + 1, ix, ix + 1])
        corner_y = np.stack([iy, iy, iy + 1, iy + 1])
        in_bounds = ((corner_x >= 0) & (corner_x < src_w) &
                     (corner_y >= 0) & (corner_y < src_h))
        weights = np.clip(np.round(weights * WEIGHT_SCALE), 0, WEIGHT_SCALE)
        weights[~in_bounds | ~valid[None]] = 0
        self.weights = weights.astype(np.uint32)

        base_x = np.clip(ix, 0, src_w - 1)
        base_y = np.clip(iy, 0, src_h - 1)
        x_step = np.where((ix >= 0) & (ix + 1 < src_w), 1, 0)
        y_step = np.where((iy >= 0) & (iy + 1 < src_h), src_w, 0)
        base = base_y * src_w + base_x
        self.indices = np.stack([base, base + x_step, base + y_step,
                                 base + y_step + x_step]).astype(np.int64)

    def sample(self, plane_flat: np.ndarray) -> np.ndarray:
        pixels = plane_flat[self.indices].astype(np.uint32)
        acc = (pixels * self.weights).sum(axis=0, dtype=np.uint32)
        return np.minimum((acc + WEIGHT_SCALE // 2) >> WEIGHT_BITS, 255).astype(np.uint8)


class ModelInputWarp:
    """One model-frame warp (medmodel or sbigmodel) at fixed calibration."""

    def __init__(self, src_w: int, src_h: int, rpy: np.ndarray, sbig: bool,
                 intrinsics: tuple[float, float, float, float] | None = None):
        fx, fy, cx, cy = intrinsics or default_intrinsics(src_w, src_h)
        proj_y = projection_matrix(fx, fy, cx, cy, rpy, sbig)
        proj_uv = _scale_transform(proj_y, 0.5)
        self.src_w, self.src_h = src_w, src_h
        self.rpy = np.asarray(rpy, dtype=np.float32).copy()
        offsets = [(0, 0), (0, 1), (1, 0), (1, 1)]
        self.y_maps = [_PlaneMap(proj_y, src_w, src_h, 2, ox, oy)
                       for ox, oy in offsets]
        self.uv_map = _PlaneMap(proj_uv, src_w // 2, src_h // 2, 1, 0, 0)

    def warp(self, y: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """(720p planar YUV420) -> uint8 [6, 128, 256]."""
        out = np.empty((6, HALF_H, HALF_W), dtype=np.uint8)
        y_flat = np.ascontiguousarray(y).reshape(-1)
        for ch, plane_map in enumerate(self.y_maps):
            out[ch] = plane_map.sample(y_flat)
        out[4] = self.uv_map.sample(np.ascontiguousarray(u).reshape(-1))
        out[5] = self.uv_map.sample(np.ascontiguousarray(v).reshape(-1))
        return out


class WarpPair:
    """med + sbig warps with calibration-change caching."""

    def __init__(self, src_w: int, src_h: int,
                 intrinsics: tuple[float, float, float, float] | None = None):
        self.src_w, self.src_h = src_w, src_h
        self.intrinsics = intrinsics
        self._med: ModelInputWarp | None = None
        self._sbig: ModelInputWarp | None = None

    def set_calibration(self, rpy: np.ndarray):
        rpy = np.asarray(rpy, dtype=np.float32)
        if self._med is None or not np.allclose(self._med.rpy, rpy, atol=1e-7):
            self._med = ModelInputWarp(self.src_w, self.src_h, rpy, sbig=False,
                                       intrinsics=self.intrinsics)
            self._sbig = ModelInputWarp(self.src_w, self.src_h, rpy, sbig=True,
                                        intrinsics=self.intrinsics)

    def warp(self, y, u, v) -> tuple[np.ndarray, np.ndarray]:
        if self._med is None:
            raise RuntimeError("set_calibration() before warp()")
        return self._med.warp(y, u, v), self._sbig.warp(y, u, v)
