import math
import unittest

from lissajous_vision.fast_controller import (
    ControlStage,
    DoneEvent,
    FailedEvent,
    FastSearchController,
    LockedEvent,
    PROBE_TABLE,
    SUPPORTED_INPUT_GRIDS,
    VisionObservation,
)
from lissajous_vision.serial_protocol import SetCommand


def observation(
    *,
    ratio_y=1,
    ratio_x=2,
    shape="ellipse",
    span_y=8.0,
    score=0.9,
    candidates=(200,),
):
    return VisionObservation(
        valid=True,
        ratio_y=ratio_y,
        ratio_x=ratio_x,
        left=ratio_x,
        right=ratio_x,
        top=ratio_y,
        bottom=ratio_y,
        confidence=0.95,
        candidates=frozenset(candidates),
        shape=shape,
        shape_confidence=0.95,
        span_x_div=8.0,
        span_y_div=span_y,
        center_x_div=0.0,
        center_y_div=0.0,
        score_diagonal=score if shape == "diagonal" else 0.1,
        score_circle=score if shape == "ellipse" else 0.1,
        score_infinity=score if shape == "infinity" else 0.1,
    )


class FastControllerTests(unittest.TestCase):
    def test_probe_table_is_the_five_requested_coarse_frequencies(self):
        self.assertEqual((100, 200, 300, 500, 700), PROBE_TABLE)
        self.assertEqual(
            tuple(range(100, 1001, 100)), SUPPORTED_INPUT_GRIDS
        )

    def test_each_10khz_input_is_inferred_from_the_10khz_probe(self):
        for input_grid in SUPPORTED_INPUT_GRIDS:
            with self.subTest(input_khz=input_grid // 10):
                controller = FastSearchController()
                first = controller.start(2, 0)
                self.assertEqual(10_000_000, first.frequency_millihz)
                controller.on_applied(first.sequence, first.probe_id, 5)
                measured = observation(
                    ratio_y=1,
                    ratio_x=input_grid // 100,
                    candidates=(input_grid,),
                )
                second = controller.observe(measured, 20)
                self.assertIsInstance(second, SetCommand)
                controller.on_applied(second.sequence, second.probe_id, 25)
                family = controller.observe(measured, 40)
                self.assertIsInstance(family, SetCommand)
                self.assertEqual(
                    input_grid * 100_000,
                    controller.input_frequency_millihz,
                )

    def test_every_coarse_probe_recovers_every_supported_input(self):
        for probe_index, probe_grid in enumerate(PROBE_TABLE):
            for input_grid in SUPPORTED_INPUT_GRIDS:
                with self.subTest(
                    probe_khz=probe_grid // 10,
                    input_khz=input_grid // 10,
                ):
                    controller = FastSearchController()
                    now = 0
                    command = controller.start(2, now)
                    for _ in range(probe_index):
                        now += 5
                        controller.on_applied(
                            command.sequence, command.probe_id, now
                        )
                        now += 5
                        command = controller.observe(
                            VisionObservation(valid=False), now
                        )
                    self.assertEqual(
                        probe_grid * 100_000,
                        command.frequency_millihz,
                    )
                    divisor = math.gcd(input_grid, probe_grid)
                    measured = observation(
                        ratio_y=probe_grid // divisor,
                        ratio_x=input_grid // divisor,
                        candidates=(input_grid,),
                    )
                    now += 5
                    controller.on_applied(
                        command.sequence, command.probe_id, now
                    )
                    now += 5
                    command = controller.observe(measured, now)
                    controller.on_applied(
                        command.sequence, command.probe_id, now + 5
                    )
                    family = controller.observe(measured, now + 10)
                    self.assertIsInstance(family, SetCommand)
                    self.assertEqual(
                        input_grid * 100_000,
                        controller.input_frequency_millihz,
                    )

    def test_unpaired_side_counts_are_rejected(self):
        controller = FastSearchController()
        first = controller.start(1, 0)
        controller.on_applied(first.sequence, first.probe_id, 5)
        unpaired = VisionObservation(
            valid=True,
            ratio_y=1,
            ratio_x=2,
            left=2,
            right=1,
            top=1,
            bottom=1,
            confidence=0.95,
            candidates=frozenset((200,)),
            shape="diagonal",
        )
        next_probe = controller.observe(unpaired, 20)
        self.assertIsInstance(next_probe, SetCommand)
        self.assertEqual(2, next_probe.probe_id)
        self.assertEqual(20_000_000, next_probe.frequency_millihz)

    def test_off_step_input_candidate_is_rejected(self):
        controller = FastSearchController()
        first = controller.start(1, 0)
        controller.on_applied(first.sequence, first.probe_id, 5)
        next_probe = controller.observe(observation(
            ratio_y=2,
            ratio_x=3,
            shape="diagonal",
            candidates=(150,),
        ), 20)
        self.assertIsInstance(next_probe, SetCommand)
        self.assertEqual(2, next_probe.probe_id)
        self.assertEqual(20_000_000, next_probe.frequency_millihz)

    def test_search_uses_only_five_probes_then_fails(self):
        controller = FastSearchController()
        now = 0
        action = controller.start(1, now)
        issued = []
        for index, probe_grid in enumerate(PROBE_TABLE):
            self.assertIsInstance(action, SetCommand)
            issued.append(action.frequency_millihz)
            now += 5
            controller.on_applied(action.sequence, action.probe_id, now)
            now += 5
            action = controller.observe(VisionObservation(valid=False), now)
            if index < len(PROBE_TABLE) - 1:
                self.assertIsInstance(action, SetCommand)
        self.assertEqual(
            [grid * 100_000 for grid in PROBE_TABLE], issued
        )
        self.assertIsInstance(action, FailedEvent)
        self.assertIn("five coarse probe", action.reason)

    def test_controller_waits_for_matching_applied(self):
        controller = FastSearchController()
        command = controller.start(1, 0)
        self.assertIsInstance(command, SetCommand)
        self.assertTrue(controller.waiting_for_applied)
        self.assertFalse(controller.on_applied(
            command.sequence + 1, command.probe_id, 5
        ))
        self.assertTrue(controller.on_applied(
            command.sequence, command.probe_id, 6
        ))
        self.assertFalse(controller.waiting_for_applied)

    def test_candidate_always_requires_second_phase(self):
        controller = FastSearchController()
        first = controller.start(2, 0)
        controller.on_applied(first.sequence, first.probe_id, 5)
        second = controller.observe(observation(), 20)
        self.assertIsInstance(second, SetCommand)
        self.assertEqual(ControlStage.DUAL_PHASE, controller.stage)
        self.assertEqual(48_000, second.phase_millideg)
        self.assertIsNone(controller.input_frequency_millihz)

        controller.on_applied(second.sequence, second.probe_id, 25)
        family = controller.observe(observation(), 40)
        self.assertIsInstance(family, SetCommand)
        self.assertEqual(ControlStage.FAMILY_VERIFY, controller.stage)
        self.assertEqual(20_000_000, controller.input_frequency_millihz)
        self.assertEqual(20_000_000, family.frequency_millihz)
        self.assertEqual(90_000, family.phase_millideg)

    def test_dual_phase_mismatch_continues_search(self):
        controller = FastSearchController()
        first = controller.start(1, 0)
        controller.on_applied(first.sequence, first.probe_id, 5)
        second = controller.observe(observation(shape="diagonal"), 20)
        controller.on_applied(second.sequence, second.probe_id, 25)
        mismatch = observation(
            ratio_y=1, ratio_x=3, shape="diagonal", candidates=(300,)
        )
        next_probe = controller.observe(mismatch, 40)
        self.assertIsInstance(next_probe, SetCommand)
        self.assertEqual(ControlStage.SEARCH, controller.stage)
        self.assertEqual(2, next_probe.probe_id)

    def test_infinity_family_uses_twice_the_input_frequency(self):
        controller = FastSearchController()
        first = controller.start(3, 0)
        controller.on_applied(first.sequence, first.probe_id, 5)
        second = controller.observe(observation(shape="infinity"), 20)
        controller.on_applied(second.sequence, second.probe_id, 25)
        family = controller.observe(observation(shape="infinity"), 40)
        self.assertEqual(40_000_000, family.frequency_millihz)
        self.assertEqual(0, family.phase_millideg)

    def test_full_lock_phase_amplitude_verify_and_hold(self):
        controller = FastSearchController(hold_ms=5_000)
        now = 0
        command = controller.start(2, now)

        def apply(cmd, advance=5):
            nonlocal now
            now += advance
            self.assertTrue(controller.on_applied(cmd.sequence, cmd.probe_id, now))

        apply(command)
        now += 10
        command = controller.observe(observation(), now)
        apply(command)
        now += 10
        command = controller.observe(observation(), now)
        apply(command)
        now += 10
        command = controller.observe(
            observation(ratio_y=1, ratio_x=1, candidates=(200,)), now
        )
        self.assertEqual(ControlStage.PHASE_SCAN, controller.stage)

        while controller.stage in (ControlStage.PHASE_SCAN, ControlStage.PHASE_REFINE):
            apply(command)
            now += 5
            phase = command.phase_millideg / 1000.0
            distance = abs((phase - 90.0 + 180.0) % 360.0 - 180.0)
            command = controller.observe(
                observation(score=1.0 - distance / 180.0), now
            )
        self.assertEqual(ControlStage.AMPLITUDE_LOCK, controller.stage)
        apply(command)
        now += 5
        command = controller.observe(observation(span_y=6.0), now)
        self.assertEqual(500, command.amplitude_mv)
        apply(command)
        now += 5
        command = controller.observe(observation(span_y=8.0), now)
        self.assertEqual(ControlStage.VERIFY, controller.stage)
        apply(command)

        event = None
        for _ in range(3):
            now += 10
            event = controller.observe(observation(), now)
        self.assertIsInstance(event, LockedEvent)
        self.assertEqual(ControlStage.HOLD, controller.stage)
        locked_at = now
        self.assertIsNone(controller.observe(observation(), locked_at + 4_999))
        event = controller.observe(observation(), locked_at + 5_000)
        self.assertIsInstance(event, DoneEvent)
        self.assertEqual(ControlStage.DONE, controller.stage)

    def test_bad_hold_frame_restarts_timer(self):
        controller = FastSearchController(hold_ms=100)
        controller.stage = ControlStage.HOLD
        controller.target_shape = 2
        controller.input_frequency_millihz = 10_000_000
        controller._hold_started_ms = 0
        bad = VisionObservation(valid=False)
        self.assertIsNone(controller.observe(bad, 80))
        self.assertIsNone(controller.observe(observation(candidates=(100,)), 179))
        self.assertIsInstance(
            controller.observe(observation(candidates=(100,)), 180), DoneEvent
        )

    def test_ack_retry_then_failure_and_control_timeout(self):
        controller = FastSearchController(ack_timeout_ms=20, max_ack_retries=1)
        command = controller.start(1, 0)
        retry = controller.tick(20)
        self.assertEqual(command, retry)
        failed = controller.tick(40)
        self.assertIsInstance(failed, FailedEvent)

        controller = FastSearchController(control_timeout_ms=50)
        command = controller.start(1, 0)
        controller.on_applied(command.sequence, command.probe_id, 5)
        self.assertIsInstance(controller.tick(51), FailedEvent)


if __name__ == "__main__":
    unittest.main()
