"""Packaging checks for the 2026-10-20 merged board runtime."""
import hashlib
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "sdicamera+yolov8-pose+yolov8-face+yolov8-finger+hdmi.cpp"
BINARY = ROOT / "sdicamera+yolov8-pose+yolov8-face+yolov8-finger+hdmi"
PREBUILT = ROOT / "prebuilt" / "ZG" / BINARY.name


class MergedGesturePackageTests(unittest.TestCase):
    def test_pose_fallback_and_pulses_are_in_source(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("selectPoseWristHands", source)
        self.assertIn('udp["touch_face_pose"]', source)
        self.assertIn("gesturePulse(g_pulse_head", source)
        self.assertIn("gesturePulse(g_pulse_face", source)
        self.assertIn("gesturePulse(g_pulse_hug", source)

    def test_all_network_configs_enable_the_merged_geometry(self):
        configs = sorted((ROOT / "configs" / "ZG").glob("*.yaml"))
        self.assertGreaterEqual(len(configs), 4)
        for config in configs:
            text = config.read_text(encoding="utf-8")
            self.assertIn("region_above: 9.0", text, config.name)
            self.assertIn("region_below: 0.40", text, config.name)
            self.assertIn("region_half_width: 9.0", text, config.name)
            self.assertIn("touch_face_pose:", text, config.name)

    def test_runtime_and_prebuilt_binary_match(self):
        self.assertEqual(BINARY.read_bytes()[:4], b"\x7fELF")
        self.assertEqual(
            hashlib.sha256(BINARY.read_bytes()).digest(),
            hashlib.sha256(PREBUILT.read_bytes()).digest(),
        )


if __name__ == "__main__":
    unittest.main()

