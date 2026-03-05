"""
Drone Plaka Takip Sistemi - ALPR Birim Testleri
Calistir: python3 -m pytest tests/ -v
         veya: python3 tests/test_alpr.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import re
import unittest

from src.alpr_detector import ALPRDetector
from config.settings import ALPR


class TestPlateValidation(unittest.TestCase):
    def setUp(self):
        self.det = ALPRDetector()

    def test_valid_plates(self):
        valid = ["34ABC123", "06AA1234", "01A1234", "35ZZZ99", "41B234"]
        for p in valid:
            self.assertTrue(ALPRDetector._valid(p), f"Gecerli plaka reddedildi: {p}")

    def test_invalid_plates(self):
        invalid = ["ABCDEFG", "1234567", "34 ABC 123", "A34ABC12", ""]
        for p in invalid:
            self.assertFalse(ALPRDetector._valid(p), f"Gecersiz plaka kabul edildi: {p}")

    def test_lock_plate(self):
        self.det.lock_plate("34abc123")
        self.assertEqual(self.det.target_plate, "34ABC123")

    def test_clear_target(self):
        self.det.lock_plate("34ABC123")
        self.det.clear_target()
        self.assertIsNone(self.det.target_plate)

    def test_preprocess_shape(self):
        roi = np.zeros((30, 100, 3), dtype=np.uint8)
        result = ALPRDetector._preprocess(roi)
        self.assertEqual(len(result.shape), 2)  # Gri tonda olmali

    def test_empty_result_structure(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        result = ALPRDetector._empty(frame)
        self.assertIn("detected", result)
        self.assertIn("plate_text", result)
        self.assertIn("confidence", result)
        self.assertIn("bbox", result)
        self.assertIn("is_target", result)
        self.assertFalse(result["detected"])


class TestPIDTracker(unittest.TestCase):
    def test_pid_zero_error(self):
        from src.vehicle_tracker import _PIDAxis
        pid = _PIDAxis(0.05, 0.001, 0.02)
        out = pid.compute(0.0)
        self.assertAlmostEqual(out, 0.0, places=5)

    def test_pid_positive_error(self):
        from src.vehicle_tracker import _PIDAxis
        pid = _PIDAxis(0.05, 0.0, 0.0)
        out = pid.compute(100.0)
        self.assertGreater(out, 0.0)


class TestSettings(unittest.TestCase):
    def test_mavlink_defaults(self):
        from config.settings import MAVLink
        self.assertEqual(MAVLink.BAUD_RATE, 57600)

    def test_failsafe_thresholds(self):
        from config.settings import FailSafe
        self.assertGreater(FailSafe.BATTERY_WARN_PCT, FailSafe.BATTERY_RTL_PCT)
        self.assertGreater(FailSafe.BATTERY_RTL_PCT, FailSafe.BATTERY_LAND_PCT)


if __name__ == "__main__":
    print("Drone Plaka Takip - Birim Testleri")
    print("=" * 45)
    unittest.main(verbosity=2)
