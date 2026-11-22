"""Deterministic requirement-5 search and visual closed-loop controller."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import math
from typing import TypeAlias

from .serial_protocol import SetCommand


# All frequency grids use 100 Hz units.  The first PC milestone deliberately
# accepts only 10 kHz input steps and emits only these five coarse probes.
SUPPORTED_INPUT_GRIDS = tuple(range(100, 1001, 100))
PROBE_TABLE = (100, 200, 300, 500, 700)

if len(PROBE_TABLE) != 5 or len(set(PROBE_TABLE)) != 5:
    raise RuntimeError("the coarse PC probe table must contain five unique points")


class ControlStage(Enum):
    WAIT_START = auto()
    SEARCH = auto()
    DUAL_PHASE = auto()
    FAMILY_VERIFY = auto()
    PHASE_SCAN = auto()
    PHASE_REFINE = auto()
    AMPLITUDE_LOCK = auto()
    VERIFY = auto()
    HOLD = auto()
    DONE = auto()
    FAILED = auto()


@dataclass(frozen=True, slots=True)
class VisionObservation:
    valid: bool = False
    ratio_y: int = 0
    ratio_x: int = 0
    left: int = 0
    right: int = 0
    top: int = 0
    bottom: int = 0
    confidence: float = 0.0
    candidates: frozenset[int] = field(default_factory=frozenset)
    shape: str = "unknown"
    shape_confidence: float = 0.0
    span_x_div: float = 0.0
    span_y_div: float = 0.0
    center_x_div: float = 0.0
    center_y_div: float = 0.0
    score_diagonal: float = 0.0
    score_circle: float = 0.0
    score_infinity: float = 0.0

    def score_for(self, target_shape: int) -> float:
        if target_shape == 1:
            return self.score_diagonal
        if target_shape == 2:
            return self.score_circle
        return self.score_infinity


@dataclass(frozen=True, slots=True)
class LockedEvent:
    sequence: int
    input_frequency_millihz: int


@dataclass(frozen=True, slots=True)
class DoneEvent:
    sequence: int
    input_frequency_millihz: int


@dataclass(frozen=True, slots=True)
class FailedEvent:
    sequence: int
    error_code: int
    reason: str


ControllerAction: TypeAlias = SetCommand | LockedEvent | DoneEvent | FailedEvent


class FastSearchController:
    SEARCH_PHASE_MILLIDEG = 11_000
    SECOND_PHASE_OFFSET_MILLIDEG = 37_000

    def __init__(
        self,
        *,
        ack_timeout_ms: int = 80,
        max_ack_retries: int = 2,
        control_timeout_ms: int = 60_000,
        hold_ms: int = 5_000,
        minimum_confidence: float = 0.80,
        target_span_div: float = 8.0,
        span_tolerance_div: float = 0.2,
        verify_frames: int = 3,
        initial_amplitude_mv: int = 471,
        minimum_amplitude_mv: int = 20,
        maximum_amplitude_mv: int = 500,
    ) -> None:
        if ack_timeout_ms <= 0 or control_timeout_ms <= 0 or hold_ms < 0:
            raise ValueError("controller timeouts must be positive")
        if max_ack_retries < 0 or verify_frames <= 0:
            raise ValueError("retry and verification counts are invalid")
        self.ack_timeout_ms = ack_timeout_ms
        self.max_ack_retries = max_ack_retries
        self.control_timeout_ms = control_timeout_ms
        self.hold_ms = hold_ms
        self.minimum_confidence = minimum_confidence
        self.target_span_div = target_span_div
        self.span_tolerance_div = span_tolerance_div
        self.verify_frames = verify_frames
        self.initial_amplitude_mv = initial_amplitude_mv
        self.minimum_amplitude_mv = minimum_amplitude_mv
        self.maximum_amplitude_mv = maximum_amplitude_mv
        self.stage = ControlStage.WAIT_START
        self.target_shape = 0
        self.sequence = 0
        self.waiting_for_applied = False
        self.pending_command: SetCommand | None = None
        self.input_frequency_millihz: int | None = None
        self.output_frequency_millihz = 0
        self.output_phase_millideg = 0
        self.output_amplitude_mv = initial_amplitude_mv
        self._started_ms: int | None = None
        self._ack_deadline_ms = 0
        self._ack_retries = 0
        self._applied_ms: int | None = None
        self._probe_cursor = 0
        self._current_probe_grid = 0
        self._first_observation: VisionObservation | None = None
        self._first_candidates: frozenset[int] = frozenset()
        self._control_probe_id = 0x7FFF
        self._phase_scores: dict[int, float] = {}
        self._scan_index = 0
        self._best_phase = 0
        self._best_score = -math.inf
        self._refine_round = 0
        self._refine_candidates: list[int] = []
        self._refine_index = 0
        self._amplitude_corrections = 0
        self._verify_count = 0
        self._hold_started_ms: int | None = None

    @property
    def applied_ms(self) -> int | None:
        return self._applied_ms

    @property
    def current_probe_grid(self) -> int:
        return self._current_probe_grid

    @property
    def started_ms(self) -> int | None:
        return self._started_ms

    @property
    def hold_started_ms(self) -> int | None:
        return self._hold_started_ms

    def _next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0xFFFF
        if self.sequence == 0:
            self.sequence = 1
        return self.sequence

    def _next_control_probe_id(self) -> int:
        self._control_probe_id += 1
        if self._control_probe_id > 0xFFFF:
            self._control_probe_id = 0x8000
        return self._control_probe_id

    @staticmethod
    def _normalize_phase(phase_millideg: int) -> int:
        return phase_millideg % 360_000

    def _issue(
        self,
        frequency_millihz: int,
        phase_millideg: int,
        amplitude_mv: int,
        probe_id: int,
        now_ms: int,
    ) -> SetCommand:
        command = SetCommand(
            sequence=self._next_sequence(),
            probe_id=probe_id,
            frequency_millihz=frequency_millihz,
            phase_millideg=self._normalize_phase(phase_millideg),
            amplitude_mv=int(amplitude_mv),
            target_shape=self.target_shape,
        )
        self.pending_command = command
        self.waiting_for_applied = True
        self._ack_retries = 0
        self._ack_deadline_ms = now_ms + self.ack_timeout_ms
        self._applied_ms = None
        self.output_frequency_millihz = command.frequency_millihz
        self.output_phase_millideg = command.phase_millideg
        self.output_amplitude_mv = command.amplitude_mv
        return command

    def start(self, target_shape: int, now_ms: int) -> SetCommand:
        if target_shape not in (1, 2, 3):
            raise ValueError("target_shape must be 1, 2, or 3")
        self.__init__(
            ack_timeout_ms=self.ack_timeout_ms,
            max_ack_retries=self.max_ack_retries,
            control_timeout_ms=self.control_timeout_ms,
            hold_ms=self.hold_ms,
            minimum_confidence=self.minimum_confidence,
            target_span_div=self.target_span_div,
            span_tolerance_div=self.span_tolerance_div,
            verify_frames=self.verify_frames,
            initial_amplitude_mv=self.initial_amplitude_mv,
            minimum_amplitude_mv=self.minimum_amplitude_mv,
            maximum_amplitude_mv=self.maximum_amplitude_mv,
        )
        self.target_shape = target_shape
        self._started_ms = now_ms
        self.stage = ControlStage.SEARCH
        return self._issue_search_probe(now_ms)

    def _issue_search_probe(self, now_ms: int) -> SetCommand:
        if self._probe_cursor >= len(PROBE_TABLE):
            return self._fail(
                4, "all five coarse probe frequencies were exhausted"
            )  # type: ignore[return-value]
        self.stage = ControlStage.SEARCH
        self._current_probe_grid = PROBE_TABLE[self._probe_cursor]
        probe_id = self._probe_cursor + 1
        self._probe_cursor += 1
        return self._issue(
            self._current_probe_grid * 100_000,
            self.SEARCH_PHASE_MILLIDEG,
            self.initial_amplitude_mv,
            probe_id,
            now_ms,
        )

    def on_applied(
        self,
        sequence: int,
        probe_id: int,
        now_ms: int | None = None,
    ) -> bool:
        command = self.pending_command
        if (
            not self.waiting_for_applied
            or command is None
            or command.sequence != sequence
            or command.probe_id != probe_id
        ):
            return False
        self.waiting_for_applied = False
        self._applied_ms = self._ack_deadline_ms if now_ms is None else now_ms
        return True

    def on_error(
        self, sequence: int, probe_id: int, error_code: int
    ) -> FailedEvent | None:
        command = self.pending_command
        if command is None or command.sequence != sequence or command.probe_id != probe_id:
            return None
        return self._fail(10 + min(max(error_code, 1), 245), "F407 rejected SET")

    def _candidate_set(self, observation: VisionObservation) -> frozenset[int]:
        candidates = {
            value
            for value in observation.candidates
            if value in SUPPORTED_INPUT_GRIDS
        }
        if observation.ratio_y > 0 and observation.ratio_x > 0:
            numerator = self._current_probe_grid * observation.ratio_x
            if numerator % observation.ratio_y == 0:
                exact = numerator // observation.ratio_y
                if exact in SUPPORTED_INPUT_GRIDS:
                    candidates.add(exact)
        return frozenset(candidates)

    def _search_observation_valid(self, observation: VisionObservation) -> bool:
        return (
            observation.valid
            and observation.confidence >= self.minimum_confidence
            and observation.ratio_y > 0
            and observation.ratio_x > 0
            and observation.left > 0
            and observation.top > 0
            and observation.left == observation.right
            and observation.top == observation.bottom
        )

    @staticmethod
    def _dual_phase_consistent(
        first: VisionObservation, second: VisionObservation
    ) -> bool:
        if not first.valid or not second.valid:
            return False
        if (first.ratio_y, first.ratio_x) != (second.ratio_y, second.ratio_x):
            return False
        return (
            first.left,
            first.right,
            first.top,
            first.bottom,
        ) == (
            second.left,
            second.right,
            second.top,
            second.bottom,
        )

    def _expected_shape(self) -> str:
        return {1: "diagonal", 2: "ellipse", 3: "infinity"}[self.target_shape]

    def _family_valid(self, observation: VisionObservation) -> bool:
        expected_ratio = (2, 1) if self.target_shape == 3 else (1, 1)
        return (
            observation.valid
            and observation.confidence >= self.minimum_confidence
            and (observation.ratio_y, observation.ratio_x) == expected_ratio
            and observation.shape == self._expected_shape()
            and observation.shape_confidence >= 0.70
        )

    def _target_good(self, observation: VisionObservation) -> bool:
        return (
            observation.valid
            and observation.shape == self._expected_shape()
            and observation.shape_confidence >= 0.75
            and abs(observation.span_y_div - self.target_span_div)
            <= self.span_tolerance_div
            and abs(observation.center_x_div) <= 0.35
            and abs(observation.center_y_div) <= 0.35
        )

    def _family_command(self, now_ms: int) -> SetCommand:
        assert self.input_frequency_millihz is not None
        frequency = self.input_frequency_millihz * (2 if self.target_shape == 3 else 1)
        phase = 90_000 if self.target_shape == 2 else 0
        self.stage = ControlStage.FAMILY_VERIFY
        return self._issue(
            frequency, phase, self.initial_amplitude_mv,
            self._next_control_probe_id(), now_ms,
        )

    def _phase_command(self, phase: int, now_ms: int) -> SetCommand:
        return self._issue(
            self.output_frequency_millihz,
            phase,
            self.output_amplitude_mv,
            self._next_control_probe_id(),
            now_ms,
        )

    def _begin_phase_scan(self, now_ms: int) -> SetCommand:
        self.stage = ControlStage.PHASE_SCAN
        self._phase_scores.clear()
        self._scan_index = 0
        self._best_score = -math.inf
        return self._phase_command(0, now_ms)

    def _begin_refine_round(self, step_degrees: int, now_ms: int) -> SetCommand:
        self.stage = ControlStage.PHASE_REFINE
        step = step_degrees * 1000
        self._refine_candidates = [
            self._normalize_phase(self._best_phase - step),
            self._normalize_phase(self._best_phase + step),
        ]
        self._refine_index = 0
        return self._phase_command(self._refine_candidates[0], now_ms)

    def _begin_amplitude(self, now_ms: int) -> SetCommand:
        self.stage = ControlStage.AMPLITUDE_LOCK
        self._amplitude_corrections = 0
        return self._phase_command(self._best_phase, now_ms)

    def observe(
        self, observation: VisionObservation, now_ms: int
    ) -> ControllerAction | None:
        if self.waiting_for_applied or self.stage in (
            ControlStage.WAIT_START, ControlStage.DONE, ControlStage.FAILED
        ):
            return None
        timeout = self._check_control_timeout(now_ms)
        if timeout is not None:
            return timeout

        if self.stage == ControlStage.SEARCH:
            if not self._search_observation_valid(observation):
                return self._issue_search_probe(now_ms)
            candidates = self._candidate_set(observation)
            if not candidates:
                return self._issue_search_probe(now_ms)
            self._first_observation = observation
            self._first_candidates = candidates
            self.stage = ControlStage.DUAL_PHASE
            assert self.pending_command is not None
            return self._issue(
                self.output_frequency_millihz,
                self.SEARCH_PHASE_MILLIDEG + self.SECOND_PHASE_OFFSET_MILLIDEG,
                self.output_amplitude_mv,
                self.pending_command.probe_id,
                now_ms,
            )

        if self.stage == ControlStage.DUAL_PHASE:
            first = self._first_observation
            second_candidates = self._candidate_set(observation)
            if (
                first is None
                or not self._search_observation_valid(observation)
                or not self._dual_phase_consistent(first, observation)
                or not self._first_candidates.intersection(second_candidates)
            ):
                return self._issue_search_probe(now_ms)
            numerator = self._current_probe_grid * observation.ratio_x
            if numerator % observation.ratio_y != 0:
                return self._issue_search_probe(now_ms)
            input_grid = numerator // observation.ratio_y
            if input_grid not in self._first_candidates.intersection(second_candidates):
                return self._issue_search_probe(now_ms)
            self.input_frequency_millihz = input_grid * 100_000
            return self._family_command(now_ms)

        if self.stage == ControlStage.FAMILY_VERIFY:
            if not self._family_valid(observation):
                return self._issue_search_probe(now_ms)
            return self._begin_phase_scan(now_ms)

        if self.stage == ControlStage.PHASE_SCAN:
            phase = self.output_phase_millideg
            score = observation.score_for(self.target_shape) if observation.valid else -math.inf
            self._phase_scores[phase] = score
            if score > self._best_score:
                self._best_score = score
                self._best_phase = phase
            self._scan_index += 1
            if self._scan_index < 12:
                return self._phase_command(self._scan_index * 30_000, now_ms)
            self._refine_round = 1
            return self._begin_refine_round(10, now_ms)

        if self.stage == ControlStage.PHASE_REFINE:
            score = observation.score_for(self.target_shape) if observation.valid else -math.inf
            if score > self._best_score:
                self._best_score = score
                self._best_phase = self.output_phase_millideg
            self._refine_index += 1
            if self._refine_index < len(self._refine_candidates):
                return self._phase_command(
                    self._refine_candidates[self._refine_index], now_ms
                )
            if self._refine_round == 1:
                self._refine_round = 2
                return self._begin_refine_round(5, now_ms)
            return self._begin_amplitude(now_ms)

        if self.stage == ControlStage.AMPLITUDE_LOCK:
            if not observation.valid or observation.span_y_div <= 0.0:
                return None
            span_error = abs(observation.span_y_div - self.target_span_div)
            if span_error <= self.span_tolerance_div:
                self.stage = ControlStage.VERIFY
                self._verify_count = 0
                return self._phase_command(self._best_phase, now_ms)
            if self._amplitude_corrections >= 3:
                return self._fail(5, "amplitude did not converge after three corrections")
            corrected = round(
                self.output_amplitude_mv * self.target_span_div / observation.span_y_div
            )
            corrected = min(
                self.maximum_amplitude_mv,
                max(self.minimum_amplitude_mv, corrected),
            )
            self._amplitude_corrections += 1
            return self._issue(
                self.output_frequency_millihz,
                self._best_phase,
                corrected,
                self._next_control_probe_id(),
                now_ms,
            )

        if self.stage == ControlStage.VERIFY:
            if self._target_good(observation):
                self._verify_count += 1
            else:
                self._verify_count = 0
            if self._verify_count >= self.verify_frames:
                assert self.input_frequency_millihz is not None
                self.stage = ControlStage.HOLD
                self._hold_started_ms = now_ms
                return LockedEvent(self.sequence, self.input_frequency_millihz)
            return None

        if self.stage == ControlStage.HOLD:
            if not self._target_good(observation):
                self._hold_started_ms = now_ms
                return None
            if self._hold_started_ms is None:
                self._hold_started_ms = now_ms
                return None
            if now_ms - self._hold_started_ms >= self.hold_ms:
                assert self.input_frequency_millihz is not None
                self.stage = ControlStage.DONE
                return DoneEvent(self.sequence, self.input_frequency_millihz)
        return None

    def _check_control_timeout(self, now_ms: int) -> FailedEvent | None:
        if (
            self._started_ms is not None
            and self.stage not in (ControlStage.HOLD, ControlStage.DONE, ControlStage.FAILED)
            and now_ms - self._started_ms >= self.control_timeout_ms
        ):
            return self._fail(2, "control did not lock before the deadline")
        return None

    def tick(self, now_ms: int) -> ControllerAction | None:
        timeout = self._check_control_timeout(now_ms)
        if timeout is not None:
            return timeout
        if not self.waiting_for_applied or self.pending_command is None:
            return None
        if now_ms < self._ack_deadline_ms:
            return None
        if self._ack_retries < self.max_ack_retries:
            self._ack_retries += 1
            self._ack_deadline_ms = now_ms + self.ack_timeout_ms
            return self.pending_command
        return self._fail(1, "APPLIED acknowledgement timed out")

    def _fail(self, error_code: int, reason: str) -> FailedEvent:
        self.stage = ControlStage.FAILED
        self.waiting_for_applied = False
        return FailedEvent(self.sequence, error_code, reason)

    def fail(self, error_code: int, reason: str) -> FailedEvent:
        return self._fail(error_code, reason)
