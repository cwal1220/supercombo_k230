#!/usr/bin/env python3
from __future__ import annotations

import ctypes
import errno
import fcntl
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict


GPIO_PIN = 25
PWM_CHANNEL = 2
PWM_PERIOD_NS = 50_000
PWM_BRIGHTNESS_CURVE = (
    (0, 0.0),
    (1, 0.008),
    (90, 0.15),
    (100, 1.0),
)
GPIO_CHIP = Path("/dev/gpiochip0")
PWM_CHIP = Path("/sys/class/pwm/pwmchip3")

_GPIO_V2_LINES_MAX = 64
_GPIO_V2_LINE_NUM_ATTRS_MAX = 10
_GPIO_V2_LINE_FLAG_OUTPUT = 1 << 3
_GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES = 2

# These structures mirror the Linux GPIO v2 ABI and avoid a runtime Python
# gpiod dependency on the board image.

class _LineAttributeData(ctypes.Union):
    _fields_ = [
        ("flags", ctypes.c_uint64),
        ("values", ctypes.c_uint64),
        ("debounce_period_us", ctypes.c_uint32),
    ]


class _LineAttribute(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_uint32),
        ("padding", ctypes.c_uint32),
        ("data", _LineAttributeData),
    ]


class _LineConfigAttribute(ctypes.Structure):
    _fields_ = [("attr", _LineAttribute), ("mask", ctypes.c_uint64)]


class _LineConfig(ctypes.Structure):
    _fields_ = [
        ("flags", ctypes.c_uint64),
        ("num_attrs", ctypes.c_uint32),
        ("padding", ctypes.c_uint32 * 5),
        ("attrs", _LineConfigAttribute * _GPIO_V2_LINE_NUM_ATTRS_MAX),
    ]


class _LineRequest(ctypes.Structure):
    _fields_ = [
        ("offsets", ctypes.c_uint32 * _GPIO_V2_LINES_MAX),
        ("consumer", ctypes.c_char * 32),
        ("config", _LineConfig),
        ("num_lines", ctypes.c_uint32),
        ("event_buffer_size", ctypes.c_uint32),
        ("padding", ctypes.c_uint32 * 5),
        ("fd", ctypes.c_int32),
    ]


class _LineValues(ctypes.Structure):
    _fields_ = [("bits", ctypes.c_uint64), ("mask", ctypes.c_uint64)]


def _iowr(ioctl_type: int, number: int, structure: type[ctypes.Structure]) -> int:
    return (3 << 30) | (ctypes.sizeof(structure) << 16) | (ioctl_type << 8) | number


_GPIO_V2_GET_LINE_IOCTL = _iowr(0xB4, 0x07, _LineRequest)
_GPIO_V2_LINE_SET_VALUES_IOCTL = _iowr(0xB4, 0x0F, _LineValues)


def duty_cycle_ns(brightness_percent: int) -> int:
    if not 0 <= brightness_percent <= 100:
        raise ValueError("brightness_percent must be between 0 and 100")
    for low, high in zip(PWM_BRIGHTNESS_CURVE, PWM_BRIGHTNESS_CURVE[1:]):
        if brightness_percent <= high[0]:
            position = (brightness_percent - low[0]) / (high[0] - low[0])
            on_ratio = low[1] + (high[1] - low[1]) * position
            return round(PWM_PERIOD_NS * (1 - on_ratio))
    raise AssertionError("brightness curve does not cover 100%")


class DisplayBacklight:
    def __init__(
        self,
        gpio_chip: Path = GPIO_CHIP,
        pwm_chip: Path = PWM_CHIP,
    ) -> None:
        self.gpio_chip = gpio_chip
        self.pwm_chip = pwm_chip
        self.pwm_path = pwm_chip / f"pwm{PWM_CHANNEL}"
        self._gpio_fd: int | None = None
        self._mode = "unknown"
        self._enabled = True
        self._brightness_percent = 100
        self._last_error = ""
        self._lock = threading.Lock()
        try:
            from k230 import iomux

            self._iomux = iomux
        except ImportError:
            self._iomux = None

    def status(self) -> Dict[str, Any]:
        available = (
            self._iomux is not None
            and self.gpio_chip.exists()
            and self.pwm_chip.exists()
        )
        return {
            "available": available,
            "enabled": self._enabled,
            "brightness_percent": self._brightness_percent,
            "mode": self._mode,
            "error": self._last_error,
        }

    def apply(self, config: Dict[str, Any]) -> None:
        enabled = config.get("enabled")
        brightness = config.get("brightness_percent")
        if not isinstance(enabled, bool):
            raise ValueError("display.enabled must be a boolean")
        if type(brightness) is not int or not 1 <= brightness <= 100:
            raise ValueError("display.brightness_percent must be an integer from 1 to 100")

        with self._lock:
            try:
                self._ensure_available()
                if not enabled:
                    self._set_gpio(False)
                elif brightness == 100:
                    self._set_gpio(True)
                else:
                    self._set_pwm(brightness)
                self._enabled = enabled
                self._brightness_percent = brightness
                self._last_error = ""
            except (OSError, RuntimeError, ValueError) as exc:
                self._last_error = str(exc)
                raise RuntimeError(f"display backlight update failed: {exc}") from exc

    def _ensure_available(self) -> None:
        if self._iomux is None:
            raise RuntimeError("k230.iomux is unavailable")
        if not self.gpio_chip.exists():
            raise RuntimeError(f"GPIO chip is unavailable: {self.gpio_chip}")
        if not self.pwm_chip.exists():
            raise RuntimeError(f"PWM chip is unavailable: {self.pwm_chip}")

    def _set_mux(self, function: int) -> None:
        current = self._iomux.read_pin(GPIO_PIN)
        mask = self._iomux.IO_SEL_MASK << self._iomux.IO_SEL_SHIFT
        updated = (current & ~mask) | (function << self._iomux.IO_SEL_SHIFT)
        if updated != current and not self._iomux.write_pin(
            GPIO_PIN, updated, confirm=False
        ):
            raise RuntimeError(f"failed to set IO{GPIO_PIN} mux")

    def _set_gpio(self, high: bool) -> None:
        self._disable_pwm()
        if self._mode != "gpio" or self._gpio_fd is None:
            self._close_gpio()
            self._set_mux(self._iomux.ALT0)
            self._gpio_fd = self._request_gpio_line(high)
        else:
            self._write_gpio_line(high)
        self._mode = "gpio"

    def _set_pwm(self, brightness_percent: int) -> None:
        duty = duty_cycle_ns(brightness_percent)
        if self._mode == "pwm" and self.pwm_path.is_dir():
            self._write_sysfs(self.pwm_path / "duty_cycle", duty)
            return

        self._close_gpio()
        self._set_mux(self._iomux.ALT1)
        self._export_pwm()

        enable_path = self.pwm_path / "enable"
        if self._read_sysfs(enable_path) != "0":
            self._write_sysfs(enable_path, 0)
        self._write_sysfs(self.pwm_path / "duty_cycle", 0)
        self._write_sysfs(self.pwm_path / "period", PWM_PERIOD_NS)
        polarity_path = self.pwm_path / "polarity"
        if self._read_sysfs(polarity_path) != "inversed":
            self._write_sysfs(polarity_path, "inversed")
        self._write_sysfs(self.pwm_path / "duty_cycle", duty)
        self._write_sysfs(enable_path, 1)
        self._mode = "pwm"

    def _export_pwm(self) -> None:
        if self.pwm_path.is_dir():
            return
        try:
            self._write_sysfs(self.pwm_chip / "export", PWM_CHANNEL)
        except OSError as exc:
            if exc.errno != errno.EBUSY:
                raise
        for _ in range(20):
            if self.pwm_path.is_dir():
                return
            time.sleep(0.01)
        raise RuntimeError(f"PWM channel did not appear: {self.pwm_path}")

    def _disable_pwm(self) -> None:
        if self.pwm_path.is_dir() and self._read_sysfs(self.pwm_path / "enable") != "0":
            self._write_sysfs(self.pwm_path / "enable", 0)

    def _request_gpio_line(self, high: bool) -> int:
        request = _LineRequest()
        request.offsets[0] = GPIO_PIN
        request.consumer = b"supercombo-display"
        request.config.flags = _GPIO_V2_LINE_FLAG_OUTPUT
        request.config.num_attrs = 1
        request.config.attrs[0].attr.id = _GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES
        request.config.attrs[0].attr.data.values = int(high)
        request.config.attrs[0].mask = 1
        request.num_lines = 1

        chip_fd = os.open(self.gpio_chip, os.O_RDONLY | os.O_CLOEXEC)
        try:
            buffer = bytearray(bytes(request))
            fcntl.ioctl(chip_fd, _GPIO_V2_GET_LINE_IOCTL, buffer, True)
            line_fd = _LineRequest.from_buffer_copy(buffer).fd
        finally:
            os.close(chip_fd)
        if line_fd < 0:
            raise RuntimeError(f"failed to request GPIO{GPIO_PIN}")
        return line_fd

    def _write_gpio_line(self, high: bool) -> None:
        if self._gpio_fd is None:
            raise RuntimeError("GPIO line is not requested")
        values = _LineValues(bits=int(high), mask=1)
        fcntl.ioctl(
            self._gpio_fd,
            _GPIO_V2_LINE_SET_VALUES_IOCTL,
            bytearray(bytes(values)),
            True,
        )

    def _close_gpio(self) -> None:
        if self._gpio_fd is not None:
            os.close(self._gpio_fd)
            self._gpio_fd = None

    @staticmethod
    def _read_sysfs(path: Path) -> str:
        return path.read_text(encoding="ascii").strip()

    @staticmethod
    def _write_sysfs(path: Path, value: int | str) -> None:
        with path.open("w", encoding="ascii") as file:
            file.write(str(value))
