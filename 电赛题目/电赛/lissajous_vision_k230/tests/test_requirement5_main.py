import importlib.util
import math
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "main.py"
SPEC = importlib.util.spec_from_file_location("requirement5_main", MODULE_PATH)
main = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(main)


def count_measurement(probe_grid, input_grid):
    count_x, count_y = main.expected_touch_counts(input_grid, probe_grid)
    return {
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
        "center_x_div": 0.0,
        "center_y_div": 0.0,
        "bounds_div": (-2.0, 2.0, -2.0, 2.0),
    }


def target_measurement(shape, phase=90.0, target_phase=90.0):
    value = count_measurement(840 if shape == 3 else 420, 420)
    distance = abs(main.phase_delta_deg(phase, target_phase))
    value.update({
        "shape": {1: "diagonal", 2: "ellipse", 3: "infinity"}[shape],
        "shape_confidence": 0.98,
        "diagonal_pass": shape == 1 and distance < 0.01,
        "circle_pass": shape == 2 and distance < 0.01,
        "infinity_pass": shape == 3 and distance < 0.01,
        "error_diagonal": distance / 180.0 if shape == 1 else 10.0,
        "error_circle": distance / 180.0 if shape == 2 else 10.0,
        "error_infinity": distance / 180.0 if shape == 3 else 10.0,
    })
    return value


def drive_controller_to_hold(test_case, shape, target_phase):
    controller = main.Requirement5Controller(
        hold_ms=5000, track_update_ms=100000
    )
    now = 0
    command = controller.start(shape, now)

    def apply(current):
        nonlocal now
        now += 1
        test_case.assertTrue(controller.on_applied(
            current["sequence"], current["probe_id"], now
        ))

    apply(command)
    precheck = {"trace_pixels": 1000, "span_x_div": 4.0}
    for _ in range(4):
        now += 1
        test_case.assertIsNone(controller.on_measurement(precheck, now))
    now += 1
    command = controller.on_measurement(precheck, now)
    test_case.assertEqual(controller.SEARCH, controller.state)

    apply(command)
    input_grid = command["frequency_grid"]
    search_frame = count_measurement(command["frequency_grid"], input_grid)
    now += 1
    command = controller.on_measurement(search_frame, now)
    test_case.assertEqual(controller.DUAL_PHASE, controller.state)
    apply(command)
    now += 1
    command = controller.on_measurement(search_frame, now)
    test_case.assertEqual(controller.SECOND_PROBE, controller.state)

    apply(command)
    second_frame = count_measurement(command["frequency_grid"], input_grid)
    now += 1
    command = controller.on_measurement(second_frame, now)
    test_case.assertEqual(controller.PHASE_COARSE, controller.state)

    while controller.state == controller.PHASE_COARSE:
        apply(command)
        phase = command["phase_millideg"] / 1000.0
        frame = target_measurement(shape, phase, target_phase)
        now += 1
        test_case.assertIsNone(controller.on_measurement(frame, now))
        now += 1
        command = controller.on_measurement(frame, now)

    while controller.state == controller.PHASE_FINE:
        apply(command)
        phase = command["phase_millideg"] / 1000.0
        frame = target_measurement(shape, phase, target_phase)
        now += 1
        test_case.assertIsNone(controller.on_measurement(frame, now))
        now += 1
        command = controller.on_measurement(frame, now)

    test_case.assertEqual(controller.AMPLITUDE_FINE, controller.state)
    apply(command)
    good = target_measurement(shape, target_phase, target_phase)
    for _ in range(2):
        now += 1
        test_case.assertIsNone(controller.on_measurement(good, now))
    now += 1
    command = controller.on_measurement(good, now)
    test_case.assertEqual(controller.LOCK_TRACK, controller.state)
    apply(command)

    event = None
    for _ in range(3):
        now += 1
        event = controller.on_measurement(good, now)
    test_case.assertIsInstance(event, main.LockedEvent)
    test_case.assertEqual(controller.HOLD, controller.state)
    return controller, now, good


class FrequencyMathTests(unittest.TestCase):
    def test_fixed_probe_table_and_full_100hz_coverage(self):
        self.assertEqual(5, main.MAX_TOUCH_COUNT)
        self.assertEqual(328, len(main.PROBE_TABLE))
        self.assertEqual(328, len(set(main.PROBE_TABLE)))
        self.assertEqual([], main.probe_table_coverage())
        self.assertEqual(tuple(range(10, 1001)), main.SUPPORTED_INPUT_GRIDS)

    def test_strict_counts_and_exact_integer_candidate(self):
        self.assertEqual(
            (False, "LEFT_RIGHT_MISMATCH", 0, 0),
            main.strict_ratio_from_counts(3, 2, 1, 1),
        )
        self.assertEqual(
            (False, "TOP_BOTTOM_MISMATCH", 0, 0),
            main.strict_ratio_from_counts(3, 3, 1, 2),
        )
        self.assertEqual(
            (True, None, 2, 3),
            main.strict_ratio_from_counts(3, 3, 2, 2),
        )
        self.assertEqual(
            (False, "TOUCH_COUNT_OVERFLOW", 0, 0),
            main.strict_ratio_from_counts(6, 6, 4, 4),
        )
        self.assertEqual(
            630,
            main.exact_frequency_candidate(420, 3, 3, 2, 2),
        )
        self.assertIsNone(
            main.exact_frequency_candidate(421, 3, 3, 2, 2)
        )

    def test_every_legal_input_has_an_exact_search_frame(self):
        for input_grid in main.SUPPORTED_INPUT_GRIDS:
            recovered = False
            for probe_grid in main.PROBE_TABLE:
                count_x, count_y = main.expected_touch_counts(
                    input_grid, probe_grid
                )
                if max(count_x, count_y) > main.MAX_TOUCH_COUNT:
                    continue
                candidate = main.exact_frequency_candidate(
                    probe_grid, count_x, count_x, count_y, count_y
                )
                if candidate == input_grid:
                    recovered = True
                    break
            self.assertTrue(recovered, input_grid)

    def test_controller_reaches_the_worst_five_touch_probe(self):
        first_hits = []
        for input_grid in main.SUPPORTED_INPUT_GRIDS:
            for index, probe_grid in enumerate(main.PROBE_TABLE):
                count_x, count_y = main.expected_touch_counts(
                    input_grid, probe_grid
                )
                if max(count_x, count_y) <= main.MAX_TOUCH_COUNT:
                    first_hits.append((index, input_grid))
                    break
        hit_index, input_grid = max(first_hits)

        controller = main.Requirement5Controller(control_timeout_ms=100000)
        now = 0
        command = controller.start(1, now)

        def apply(current):
            nonlocal now
            now += 1
            self.assertTrue(controller.on_applied(
                current["sequence"], current["probe_id"], now
            ))

        apply(command)
        precheck = {"trace_pixels": 1000, "span_x_div": 4.0}
        for _ in range(5):
            now += 1
            command = controller.on_measurement(precheck, now)

        while controller.state == controller.SEARCH:
            apply(command)
            frame = count_measurement(
                command["frequency_grid"], input_grid
            )
            now += 1
            command = controller.on_measurement(frame, now)

        self.assertEqual(controller.DUAL_PHASE, controller.state)
        self.assertEqual(hit_index + 1, controller.probe_cursor)
        self.assertEqual(input_grid, controller.input_grid)
        counts = controller.first_counts
        self.assertLessEqual(max(counts), main.MAX_TOUCH_COUNT)


class ShapeAndPhaseTests(unittest.TestCase):
    def test_real_diagonal_circle_and_infinity_metrics(self):
        diagonal = [(-4.0 + index * 0.04, -4.0 + index * 0.04)
                    for index in range(201)]
        diagonal_metrics = main.diagonal_shape_metrics(diagonal)
        self.assertTrue(diagonal_metrics["pass"])
        self.assertLess(diagonal_metrics["eigen_ratio"], 0.001)

        circle = []
        infinity = []
        for index in range(720):
            angle = 2.0 * math.pi * index / 720.0
            circle.append((4.0 * math.cos(angle), 4.0 * math.sin(angle)))
            infinity.append((4.0 * math.sin(angle),
                             4.0 * math.sin(2.0 * angle)))
        circle_metrics = main.circle_shape_metrics(circle)
        infinity_metrics = main.infinity_shape_metrics(infinity)
        self.assertTrue(circle_metrics["pass"])
        self.assertLess(circle_metrics["radial_cv"], 0.001)
        self.assertTrue(infinity_metrics["pass"])
        self.assertGreaterEqual(infinity_metrics["mirror_iou_lr"], 0.90)

    def test_phase_wrap_unwrap_and_frequency_trim(self):
        self.assertEqual(-20.0, main.phase_delta_deg(350.0, 10.0))
        self.assertEqual(
            [350.0, 370.0, 390.0],
            main.unwrap_phase_degrees([350.0, 10.0, 30.0]),
        )
        history = [
            (0, 0.0), (250, 90.0), (500, 180.0),
            (750, 270.0), (1000, 0.0),
        ]
        self.assertAlmostEqual(
            1.0, main.phase_drift_frequency_trim_hz(history), places=6
        )


class ProtocolAndControllerTests(unittest.TestCase):
    def test_protocol_parser_and_formatter(self):
        parser = main.UartFrameParser()
        self.assertEqual([], parser.feed(b"junk$STAR"))
        self.assertEqual(["START,2", "APPLIED,7,9"], parser.feed(
            b"T,2\r\n$APPLIED,7,9\r\n"
        ))
        self.assertEqual(("START", 2), main.parse_uart_frame("START,2"))
        command = {
            "kind": "SET", "sequence": 1, "probe_id": 2,
            "frequency_millihz": 1234000, "phase_millideg": 37000,
            "amplitude_mv": 471, "shape": 3,
        }
        self.assertEqual(
            "$SET,1,2,1234000,37000,471,3\r\n",
            main.format_action(command),
        )

    def test_ack_timeout_retries_once_then_fails(self):
        controller = main.Requirement5Controller(
            ack_timeout_ms=20, max_ack_retries=1
        )
        command = controller.start(1, 0)
        self.assertIs(command, controller.tick(20))
        failed = controller.tick(40)
        self.assertIsInstance(failed, main.FailedEvent)
        self.assertEqual(main.Requirement5Controller.ERROR_UART_TIMEOUT,
                         failed.error_code)

    def test_tracking_reverses_after_a_worse_phase_perturbation(self):
        controller = main.Requirement5Controller(track_update_ms=150)
        controller.target_shape = 2
        controller.state = controller.LOCK_TRACK
        controller.target_span_div = 4.0
        controller.nominal_frequency_millihz = 42000000
        controller.locked_frequency_millihz = 42000000
        controller.phase_deg = 90.0
        controller.amplitude_mv = 471
        controller.last_track_ms = 0
        controller.last_trim_ms = 150
        controller.last_slow_amplitude_ms = 150

        optimum = target_measurement(2, 90.0, 90.0)
        first_move = controller._tracking_command(optimum, 150)
        self.assertEqual(91000, first_move["phase_millideg"])
        self.assertEqual(1, controller.track_direction)
        self.assertTrue(controller.on_applied(
            first_move["sequence"], first_move["probe_id"], 151
        ))

        worse = target_measurement(2, 91.0, 90.0)
        # A one-degree circle perturbation is still within the physical shape
        # gate; only its continuous score is worse than at the optimum.
        worse["circle_pass"] = True
        worse["error_circle"] = 0.2
        second_move = controller._tracking_command(worse, 300)
        self.assertEqual(-1, controller.track_direction)
        self.assertEqual(90000, second_move["phase_millideg"])

    def test_all_three_target_modes_reach_hold(self):
        for shape, target_phase in ((1, 0.0), (2, 90.0), (3, 0.0)):
            with self.subTest(shape=shape):
                controller, unused, unused_good = drive_controller_to_hold(
                    self, shape, target_phase
                )
                self.assertEqual(controller.HOLD, controller.state)

    def test_full_circle_control_holds_continuous_five_seconds(self):
        controller, locked_at, good = drive_controller_to_hold(self, 2, 90.0)

        # One invalid frame makes the required interval restart.
        bad = dict(good)
        bad["circle_pass"] = False
        now = locked_at + 1000
        self.assertIsNone(controller.on_measurement(bad, now))
        self.assertIsNone(controller.on_measurement(good, now + 4999))
        done = controller.on_measurement(good, now + 5000)
        self.assertIsInstance(done, main.DoneEvent)
        self.assertEqual(controller.DONE, controller.state)


if __name__ == "__main__":
    unittest.main()
