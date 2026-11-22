"""Regression tests for the two-stage fast SEARCH pre-filter.

Covers:
- fast_trace_stats() / make_fast_measurement() blob observer
- fill-ratio static gate: rotating envelopes are skipped cheaply
- complete-frame confirmation after the gate passes
- optional 2-frame counting vote (count_confirm_frames=2)
- bounded retry: a passing probe whose complete frames keep failing moves on
"""

import importlib.util
import math
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "main.py"
SPEC = importlib.util.spec_from_file_location("requirement5_main", MODULE_PATH)
main = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(main)


class FakeBlob:
    def __init__(self, x, y, w, h, pixels):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._p = pixels

    def rect(self):
        return (self._x, self._y, self._w, self._h)

    def pixels(self):
        return self._p


class FakeImg:
    def __init__(self, blobs):
        self._blobs = blobs

    def find_blobs(self, thresholds, roi=None, pixels_threshold=0, merge=False):
        return self._blobs


def count_measurement(probe_grid, input_grid, fill=None):
    count_x, count_y = main.expected_touch_counts(input_grid, probe_grid)
    value = {
        "valid": True,
        "strict_pair_valid": True,
        "left": count_x,
        "right": count_x,
        "top": count_y,
        "bottom": count_y,
        "ratio_x": count_x // math.gcd(count_x, count_y),
        "ratio_y": count_y // math.gcd(count_x, count_y),
        "trace_pixels": 1000,
        "span_x_div": 4.0,
        "span_y_div": 4.0,
    }
    if fill is not None:
        value["fill_ratio"] = fill
    return value


def fast_frame(fill, pixels=2000):
    return {"valid": True, "fill_ratio": fill, "trace_pixels": pixels,
            "span_x_div": 4.0, "span_y_div": 4.0}


class FastTraceStatsTests(unittest.TestCase):
    def test_no_blob_returns_none(self):
        self.assertIsNone(main.fast_trace_stats(FakeImg([])))

    def test_single_blob_fill_ratio(self):
        stats = main.fast_trace_stats(
            FakeImg([FakeBlob(100, 50, 200, 100, 5000)])
        )
        self.assertIsNotNone(stats)
        self.assertAlmostEqual(0.25, stats["fill_ratio"], places=6)
        self.assertEqual((100, 50, 300, 150), stats["bbox"])

    def test_multiple_blobs_union_bbox(self):
        stats = main.fast_trace_stats(FakeImg([
            FakeBlob(100, 50, 60, 40, 800),
            FakeBlob(180, 80, 80, 30, 900),
        ]))
        self.assertIsNotNone(stats)
        self.assertEqual((100, 50, 260, 110), stats["bbox"])
        self.assertEqual(1700, stats["pixels"])

    def test_fast_measurement_geometry(self):
        stats = {
            "valid": True, "pixels": 2000,
            "bbox": (89 + 10, 18 + 20, 89 + 210, 18 + 220),
            "fill_ratio": 0.2,
        }
        measurement = main.make_fast_measurement(stats)
        px_per_div_x = main.GRID_RECT[2] / main.GRID_DIVISIONS_X
        px_per_div_y = main.GRID_RECT[3] / main.GRID_DIVISIONS_Y
        self.assertAlmostEqual(
            200.0 / px_per_div_x, measurement["span_x_div"], places=6
        )
        self.assertAlmostEqual(
            200.0 / px_per_div_y, measurement["span_y_div"], places=6
        )
        self.assertAlmostEqual(0.2, measurement["fill_ratio"], places=6)


class SearchPreFilterTests(unittest.TestCase):
    @staticmethod
    def _controller_in_search(**kwargs):
        controller = main.Requirement5Controller(
            hold_ms=5000, track_update_ms=100000, **kwargs
        )
        now = 0
        command = controller.start(1, now)

        def apply(current):
            nonlocal now
            now += 1
            assert controller.on_applied(
                current["sequence"], current["probe_id"], now
            )

        apply(command)
        precheck = {"trace_pixels": 1000, "span_x_div": 4.0}
        for _ in range(5):
            now += 1
            command = controller.on_measurement(precheck, now)
        assert controller.state == controller.SEARCH
        # 应用 SEARCH 探测点，进入可接收测量帧的就绪状态
        now += 1
        assert controller.on_applied(
            command["sequence"], command["probe_id"], now
        )
        return controller, now, command

    def test_rotating_envelope_skips_probe(self):
        controller, now, command = self._controller_in_search()
        first_probe = controller.current_probe_grid
        # 高填充率 = 旋转包络：换下一个探测点
        result = controller.on_measurement(fast_frame(0.55), now)
        self.assertIsNotNone(result)
        self.assertIs(controller.SEARCH, controller.state)
        self.assertNotEqual(first_probe, controller.current_probe_grid)

    def test_static_fast_frame_waits_for_complete_frame(self):
        controller, now, command = self._controller_in_search()
        probe = controller.current_probe_grid
        result = controller.on_measurement(fast_frame(0.05), now)
        self.assertIsNone(result)
        self.assertTrue(controller.search_passed_fast)
        # 完整帧确认
        frame = count_measurement(probe, probe)
        result = controller.on_measurement(frame, now + 1)
        self.assertIs(controller.DUAL_PHASE, controller.state)
        self.assertEqual(probe, controller.input_grid)

    def test_complete_frame_high_fill_after_gate_moves_on(self):
        controller, now, command = self._controller_in_search()
        probe = controller.current_probe_grid
        self.assertIsNone(controller.on_measurement(fast_frame(0.05), now))
        # 完整帧（带 fill_ratio）却显示高填充：判定为噪声，换点
        frame = count_measurement(probe, probe, fill=0.6)
        result = controller.on_measurement(frame, now + 1)
        self.assertIsNotNone(result)
        self.assertIs(controller.SEARCH, controller.state)
        self.assertNotEqual(probe, controller.current_probe_grid)

    def test_legacy_frame_without_fill_confirms_immediately(self):
        controller, now, command = self._controller_in_search()
        probe = controller.current_probe_grid
        frame = count_measurement(probe, probe)
        result = controller.on_measurement(frame, now)
        self.assertIs(controller.DUAL_PHASE, controller.state)

    def test_confirm_vote_requires_two_identical_frames(self):
        controller, now, command = self._controller_in_search(
            count_confirm_frames=2
        )
        probe = controller.current_probe_grid
        frame = count_measurement(probe, probe)
        # 第一帧：vote=1，未达标
        self.assertIsNone(controller.on_measurement(frame, now))
        self.assertIs(controller.SEARCH, controller.state)
        # 第二帧相同：达标
        result = controller.on_measurement(frame, now + 1)
        self.assertIs(controller.DUAL_PHASE, controller.state)

    def test_confirm_vote_abandons_after_bounded_frames(self):
        controller, now, command = self._controller_in_search(
            count_confirm_frames=2
        )
        probe = controller.current_probe_grid
        # 三帧全部不同计数（抖动），投票无法达成 → 换点
        for index in range(main.SEARCH_COUNT_MAX_FRAMES):
            frame = count_measurement(probe + index + 1, probe)
            controller.on_measurement(frame, now + index)
        self.assertIs(controller.SEARCH, controller.state)
        self.assertNotEqual(probe, controller.current_probe_grid)

    def test_waiting_stage_bounded_then_moves_on(self):
        controller, now, command = self._controller_in_search()
        probe = controller.current_probe_grid
        # 快测通过 → 完整帧持续无效（fill 低但计数坏）
        self.assertIsNone(controller.on_measurement(fast_frame(0.05), now))
        for index in range(main.SEARCH_COUNT_MAX_FRAMES):
            bad = count_measurement(probe, probe, fill=0.05)
            bad["left"] = 3
            bad["right"] = 2
            controller.on_measurement(bad, now + 1 + index)
        self.assertIs(controller.SEARCH, controller.state)
        self.assertNotEqual(probe, controller.current_probe_grid)


if __name__ == "__main__":
    unittest.main()
