#!/usr/bin/env python3
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.k230_display_control import duty_cycle_ns
from scripts.param_server import PARAM_METADATA, ParamStore


class FakeDisplayController:
    def __init__(self):
        self.applied = []

    def apply(self, document):
        self.applied.append(dict(document))

    def status(self):
        latest = self.applied[-1] if self.applied else {}
        return {
            "available": True,
            "enabled": latest.get("enabled", True),
            "brightness_percent": latest.get("brightness_percent", 100),
            "mode": "test",
            "error": "",
        }


class ParamStoreTest(unittest.TestCase):
    EXPECTED_QUICK = {
        "steering": [
            "path_offset_m",
            "steer_actuator_delay",
            "torque_kp_raw",
            "torque_kf_raw",
            "torque_friction_raw",
            "torque_angle_deadzone_raw",
        ],
        "driving": ["max_lateral_jerk", "max_lateral_accel"],
        "adaptive_cruise": [
            "enabled",
            "following_time_s",
            "standstill_gap_m",
            "gap_correction_gain",
            "max_slowdown_correction_mps",
            "lead_restore_delay_s",
            "command_interval_s",
        ],
        "recording": ["enabled"],
        "display": ["enabled", "brightness_percent"],
    }

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.paths = {
            "steering": root / "steering.json",
            "driving": root / "driving.json",
            "adaptive_cruise": root / "adaptive_cruise.json",
            "recording": root / "recording.json",
            "display": root / "display.json",
        }
        self.paths["steering"].write_text(
            json.dumps({"gain": 10, "enabled": True}), encoding="utf-8"
        )
        self.paths["driving"].write_text(
            json.dumps({"delay": 0.4}), encoding="utf-8"
        )
        self.paths["adaptive_cruise"].write_text(
            json.dumps({"following_time_s": 1.8}), encoding="utf-8"
        )
        self.paths["recording"].write_text(
            json.dumps({"enabled": False}), encoding="utf-8"
        )
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 100}), encoding="utf-8"
        )
        self.notifications = 0

        def notify():
            self.notifications += 1
            return [123]

        self.store = ParamStore(self.paths, notify)

    def tearDown(self):
        self.temporary.cleanup()

    def test_atomic_single_value_update(self):
        result = self.store.update("steering", {"gain": 12})
        self.assertEqual(result["notified_pids"], [123])
        self.assertEqual(self.notifications, 1)
        self.assertEqual(self.store.read_group("steering"), {"gain": 12, "enabled": True})
        self.assertEqual(list(self.paths["steering"].parent.glob("*.tmp")), [])

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(KeyError):
            self.store.update("driving", {"unknown": 1})
        self.assertEqual(self.notifications, 0)
        self.assertEqual(self.store.read_group("driving"), {"delay": 0.4})

    def test_recording_toggle_does_not_signal_controlsd(self):
        result = self.store.update("recording", {"enabled": True})
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(self.notifications, 0)

    def test_display_update_applies_hardware_without_signaling_controlsd(self):
        controller = FakeDisplayController()
        store = ParamStore(
            self.paths,
            self.store.notifier,
            display_controller=controller,
        )
        controller.applied.clear()
        result = store.update("display", {"brightness_percent": 35})
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(controller.applied, [{"enabled": True, "brightness_percent": 35}])
        self.assertEqual(store.read_group("display")["brightness_percent"], 35)

    def test_display_pwm_duty_uses_inversed_polarity_formula(self):
        self.assertEqual(duty_cycle_ns(100), 0)
        self.assertEqual(duty_cycle_ns(99), 4_250)
        self.assertEqual(duty_cycle_ns(90), 42_500)
        self.assertEqual(duty_cycle_ns(50), 45_691)
        self.assertEqual(duty_cycle_ns(10), 48_882)
        self.assertEqual(duty_cycle_ns(1), 49_600)
        self.assertEqual(duty_cycle_ns(0), 50_000)

    def test_missing_defaults_are_added_without_overwriting_tuning(self):
        defaults = Path(self.temporary.name) / "adaptive.defaults.json"
        defaults.write_text(
            json.dumps({"following_time_s": 1.8, "deceleration_rate_kph_per_s": 1.5}),
            encoding="utf-8",
        )
        self.paths["adaptive_cruise"].write_text(
            json.dumps({"following_time_s": 2.2}), encoding="utf-8"
        )
        store = ParamStore(
            self.paths,
            lambda: [],
            {"adaptive_cruise": defaults},
        )
        self.assertEqual(
            store.read_group("adaptive_cruise"),
            {"following_time_s": 2.2, "deceleration_rate_kph_per_s": 1.5},
        )

    def test_repository_params_have_complete_ui_metadata(self):
        root = Path(__file__).resolve().parents[1]
        for group, filename in (
            ("steering", "steering.json"),
            ("driving", "driving.json"),
            ("adaptive_cruise", "adaptive_cruise.json"),
            ("recording", "recording.json"),
            ("display", "display.json"),
        ):
            params = json.loads((root / "params" / filename).read_text(encoding="utf-8"))
            self.assertEqual(set(params), set(PARAM_METADATA[group]))
            for key, value in params.items():
                metadata = PARAM_METADATA[group][key]
                for field in ("label", "section", "description", "increase", "decrease"):
                    self.assertTrue(metadata.get(field), f"{group}.{key}.{field}")
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    for field in ("step", "min", "max"):
                        self.assertIsInstance(
                            metadata.get(field), (int, float), f"{group}.{key}.{field}"
                        )

    def test_quick_tuning_layout_is_explicit_and_stable(self):
        for group, expected in self.EXPECTED_QUICK.items():
            quick = {
                key: metadata
                for key, metadata in PARAM_METADATA[group].items()
                if metadata.get("quick")
            }
            self.assertEqual(set(quick), set(expected))
            self.assertTrue(all(metadata.get("quick_section") for metadata in quick.values()))
            orders = [metadata.get("quick_order") for metadata in quick.values()]
            self.assertTrue(all(isinstance(order, int) for order in orders))
            self.assertEqual(len(orders), len(set(orders)))
            ordered = sorted(quick, key=lambda key: quick[key]["quick_order"])
            self.assertEqual(ordered, expected)


if __name__ == "__main__":
    unittest.main()
