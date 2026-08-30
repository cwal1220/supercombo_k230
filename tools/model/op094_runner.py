"""Host-side sequential runner for the openpilot v0.9.4 supercombo ONNX.

Mirrors what `src/supercombo_model.cc` feeds the kmodel: a 100-tick desire
pulse history, a 99x128 feature buffer taken from the model's own output, and
zeroed nav features. The buffers only make sense along a drive, so callers run
frames in capture order.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import onnxruntime as ort

DESIRE_LEN = 8
DESIRE_HISTORY = 100
FEATURE_LEN = 128
FEATURE_HISTORY = 99
NAV_FEATURE_LEN = 256
OUTPUT_FLOATS = 6120
HIDDEN_SLICE = slice(5990, 5990 + FEATURE_LEN)
INPUT_ORDER = ["input_imgs", "big_input_imgs", "desire", "traffic_convention",
               "nav_features", "features_buffer"]


class Op094Runner:
    def __init__(self, model_path: str | Path):
        options = ort.SessionOptions()
        options.graph_optimization_level = \
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(str(model_path), options,
                                            providers=["CPUExecutionProvider"])
        self.types = {i.name: i.type for i in self.session.get_inputs()}
        missing = [n for n in INPUT_ORDER if n not in self.types]
        if missing:
            raise ValueError(f"not a v0.9.4 supercombo graph, missing {missing}")
        self.reset()

    def _dtype(self, name: str):
        kind = self.types[name]
        if kind == "tensor(uint8)":
            return np.uint8
        if kind == "tensor(float16)":
            return np.float16
        return np.float32

    def reset(self) -> None:
        self.features = np.zeros((FEATURE_HISTORY, FEATURE_LEN), np.float32)
        self.desire = np.zeros((DESIRE_HISTORY, DESIRE_LEN), np.float32)
        self.prev_desire = np.zeros(DESIRE_LEN, np.float32)

    def build_inputs(self, img: np.ndarray, big_img: np.ndarray,
                     desire: int = 0) -> dict:
        """img/big_img: uint8 [12, 128, 256] = previous and current YUV6."""
        vec = np.zeros(DESIRE_LEN, np.float32)
        if 0 < desire < DESIRE_LEN:
            vec[desire] = 1.0
        # rising edge only, like the runtime
        pulse = np.where(vec - self.prev_desire > 0.99, vec, 0.0)
        self.prev_desire = vec
        self.desire = np.roll(self.desire, -1, axis=0)
        self.desire[-1] = pulse
        return {
            "input_imgs": img[None].astype(self._dtype("input_imgs")),
            "big_input_imgs": big_img[None].astype(self._dtype("big_input_imgs")),
            "desire": self.desire[None].astype(self._dtype("desire")),
            "traffic_convention": np.array(
                [[1.0, 0.0]], self._dtype("traffic_convention")),
            "nav_features": np.zeros(
                (1, NAV_FEATURE_LEN), self._dtype("nav_features")),
            "features_buffer": self.features[None].astype(
                self._dtype("features_buffer")),
        }

    def step(self, inputs: dict) -> np.ndarray:
        out = self.session.run(None, inputs)[0][0].astype(np.float32)
        if out.size != OUTPUT_FLOATS:
            raise ValueError(f"unexpected output size {out.size}")
        self.features = np.roll(self.features, -1, axis=0)
        self.features[-1] = out[HIDDEN_SLICE]
        return out

    def run(self, img: np.ndarray, big_img: np.ndarray,
            desire: int = 0) -> np.ndarray:
        return self.step(self.build_inputs(img, big_img, desire))
