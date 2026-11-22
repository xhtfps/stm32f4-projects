import unittest
from types import SimpleNamespace

from lissajous_vision.fast_runtime import RuntimeConfig, _peak_relationship
from lissajous_vision.k230_compat import (
    FastSearchController,
    PROBE_TABLE,
    SUPPORTED_INPUT_GRIDS,
)


class FastRuntimeTests(unittest.TestCase):
    def test_loads_coarse_pc_probe_table(self):
        self.assertEqual((100, 200, 300, 500, 700), PROBE_TABLE)
        self.assertEqual(tuple(range(100, 1001, 100)), SUPPORTED_INPUT_GRIDS)

    def test_runtime_defaults_are_suitable_for_preview(self):
        config = RuntimeConfig()
        self.assertEqual("auto", config.grid_mode)
        self.assertEqual(320, config.panel_size)

    def test_controller_waits_for_applied(self):
        controller = FastSearchController()
        command = controller.start(1, 0)
        self.assertTrue(controller.waiting_for_applied)
        self.assertTrue(controller.on_applied(
            command.sequence, command.probe_id
        ))
        self.assertFalse(controller.waiting_for_applied)

    def test_peak_count_is_the_primary_multiplier(self):
        two_peaks = SimpleNamespace(valid=True, peak_count=2)
        self.assertIn("2 个波峰点", _peak_relationship(two_peaks))
        self.assertIn("2 倍", _peak_relationship(two_peaks))


if __name__ == "__main__":
    unittest.main()
