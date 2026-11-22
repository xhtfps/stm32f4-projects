import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from lissajous_vision.boundary import BoundaryMeasurement
from lissajous_vision.fast_runtime import RuntimeConfig, _make_observation
from lissajous_vision.models import Measurement
from lissajous_vision.pipeline import AnalysisResult


class RuntimeConfigTests(unittest.TestCase):
    def test_default_control_deadline_is_sixty_seconds(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.json"
            with patch.dict(os.environ, {"LISSAJOUS_SERIAL_PORT": "COM12"}):
                config = RuntimeConfig.load(path)
        self.assertEqual(60_000, config.control_timeout_ms)

    def test_loads_serial_and_control_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.json"
            path.write_text(json.dumps({
                "serial_port": "COM8",
                "serial_baudrate": 230400,
                "ack_timeout_ms": 90,
                "hold_ms": 5100,
                "settle_frames": 2,
            }), encoding="utf-8")
            config = RuntimeConfig.load(path)
        self.assertEqual("COM8", config.serial_port)
        self.assertEqual(90, config.ack_timeout_ms)
        self.assertEqual(5100, config.hold_ms)
        self.assertEqual(2, config.settle_frames)

    def test_serial_port_environment_override(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.json"
            with patch.dict(os.environ, {"LISSAJOUS_SERIAL_PORT": "COM11"}):
                config = RuntimeConfig.load(path)
        self.assertEqual("COM11", config.serial_port)

    def test_rejects_invalid_control_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.json"
            path.write_text('{"control_timeout_ms": 0}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "control_timeout_ms"):
                RuntimeConfig.load(path)

    def test_maps_measurement_and_boundary_to_controller_observation(self):
        measurement = Measurement(
            valid=True,
            shape="ellipse",
            confidence=0.92,
            span_x_div=7.9,
            span_y_div=8.1,
            center_x_div=0.1,
            center_y_div=-0.1,
            line_error_norm=0.2,
            ellipse_error=0.03,
            infinity_error=0.4,
            score_2x=0.2,
        )
        result = AnalysisResult(
            measurement=measurement,
            source=np.zeros((2, 2, 3), dtype=np.uint8),
            warped=None,
            raw_mask=None,
            geometry_mask=None,
            corners=None,
        )
        boundary = BoundaryMeasurement(
            True, "", 2, 2, 1, 1, 1, 2, 1, 2, 0.95,
            7.9, 8.1, 0.0, 0.1, -0.1,
        )
        observed = _make_observation(result, boundary, probe_grid=100)
        self.assertTrue(observed.valid)
        self.assertEqual((1, 2), (observed.ratio_y, observed.ratio_x))
        self.assertIn(200, observed.candidates)
        self.assertGreater(observed.score_circle, observed.score_diagonal)
        self.assertEqual(8.1, observed.span_y_div)


if __name__ == "__main__":
    unittest.main()
