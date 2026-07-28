#!/usr/bin/env python3
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.k7_param_server import PARAM_METADATA, ParamStore


class ParamStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.paths = {
            "steering": root / "steering.json",
            "driving": root / "driving.json",
        }
        self.paths["steering"].write_text(
            json.dumps({"gain": 10, "enabled": True}), encoding="utf-8"
        )
        self.paths["driving"].write_text(
            json.dumps({"delay": 0.4}), encoding="utf-8"
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

    def test_repository_params_have_complete_ui_metadata(self):
        root = Path(__file__).resolve().parents[1]
        for group, filename in (
            ("steering", "k7_yg_steering.json"),
            ("driving", "k7_yg_driving.json"),
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


if __name__ == "__main__":
    unittest.main()
