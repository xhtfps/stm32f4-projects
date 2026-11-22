"""Bounded ASCII protocol shared by the PC vision master and F407 slave."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias


MAX_SEQUENCE = 65_535
MAX_PROBE_ID = 65_535
MIN_DDS_FREQUENCY_MILLIHZ = 1_000
MAX_DDS_FREQUENCY_MILLIHZ = 200_000_000
MIN_INPUT_FREQUENCY_MILLIHZ = 1_000_000
MAX_INPUT_FREQUENCY_MILLIHZ = 100_000_000
MAX_PHASE_MILLIDEG = 359_999
MAX_AMPLITUDE_MV = 1_000
MAX_ERROR_CODE = 255

# PC-side names for the three Lissajous target modes.  The DDS output remains
# a sine carrier; the selected mode describes the required frequency ratio and
# phase relationship used by the vision controller.
WAVEFORM_NAMES = {
    1: "diagonal",
    2: "circle",
    3: "infinity",
}


@dataclass(frozen=True, slots=True)
class StartFrame:
    target_shape: int


@dataclass(frozen=True, slots=True)
class AppliedFrame:
    sequence: int
    probe_id: int


@dataclass(frozen=True, slots=True)
class ErrorFrame:
    sequence: int
    probe_id: int
    error_code: int


InboundFrame: TypeAlias = StartFrame | AppliedFrame | ErrorFrame


@dataclass(frozen=True, slots=True)
class SetCommand:
    sequence: int
    probe_id: int
    frequency_millihz: int
    phase_millideg: int
    amplitude_mv: int
    target_shape: int


def encode_run(target_shape: int) -> bytes:
    """Ask an F407 slave to start PC-master vision control."""
    _bounded("target_shape", target_shape, 1, 3)
    return _line("RUN", target_shape)


def _bounded(name: str, value: int, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in {minimum}..{maximum}")
    return value


def _line(*fields: object) -> bytes:
    return ("$" + ",".join(str(field) for field in fields) + "\r\n").encode("ascii")


def encode_set(command: SetCommand) -> bytes:
    _bounded("sequence", command.sequence, 0, MAX_SEQUENCE)
    _bounded("probe_id", command.probe_id, 0, MAX_PROBE_ID)
    _bounded(
        "frequency_millihz", command.frequency_millihz,
        MIN_DDS_FREQUENCY_MILLIHZ, MAX_DDS_FREQUENCY_MILLIHZ,
    )
    _bounded("phase_millideg", command.phase_millideg, 0, MAX_PHASE_MILLIDEG)
    _bounded("amplitude_mv", command.amplitude_mv, 0, MAX_AMPLITUDE_MV)
    _bounded("target_shape", command.target_shape, 1, 3)
    return _line(
        "SET", command.sequence, command.probe_id,
        command.frequency_millihz, command.phase_millideg,
        command.amplitude_mv, command.target_shape,
    )


def _encode_input_status(kind: str, sequence: int, input_frequency_millihz: int) -> bytes:
    _bounded("sequence", sequence, 0, MAX_SEQUENCE)
    _bounded(
        "input_frequency_millihz", input_frequency_millihz,
        MIN_INPUT_FREQUENCY_MILLIHZ, MAX_INPUT_FREQUENCY_MILLIHZ,
    )
    return _line(kind, sequence, input_frequency_millihz)


def encode_locked(sequence: int, input_frequency_millihz: int) -> bytes:
    return _encode_input_status("LOCKED", sequence, input_frequency_millihz)


def encode_done(sequence: int, input_frequency_millihz: int) -> bytes:
    return _encode_input_status("DONE", sequence, input_frequency_millihz)


def encode_failed(sequence: int, error_code: int) -> bytes:
    _bounded("sequence", sequence, 0, MAX_SEQUENCE)
    _bounded("error_code", error_code, 1, MAX_ERROR_CODE)
    return _line("FAILED", sequence, error_code)


def encode_stop(sequence: int) -> bytes:
    _bounded("sequence", sequence, 0, MAX_SEQUENCE)
    return _line("STOP", sequence)


def _parse_decimal(token: str, minimum: int, maximum: int) -> int | None:
    if not token or not token.isascii() or not token.isdecimal():
        return None
    value = int(token, 10)
    return value if minimum <= value <= maximum else None


def _parse_line(payload: bytes) -> InboundFrame | None:
    try:
        text = payload.decode("ascii")
    except UnicodeDecodeError:
        return None
    tokens = text.split(",")
    if len(tokens) == 2 and tokens[0] == "START":
        target = _parse_decimal(tokens[1], 1, 3)
        return None if target is None else StartFrame(target)
    if len(tokens) == 3 and tokens[0] == "APPLIED":
        sequence = _parse_decimal(tokens[1], 0, MAX_SEQUENCE)
        probe_id = _parse_decimal(tokens[2], 0, MAX_PROBE_ID)
        if sequence is not None and probe_id is not None:
            return AppliedFrame(sequence, probe_id)
        return None
    if len(tokens) == 4 and tokens[0] == "ERROR":
        sequence = _parse_decimal(tokens[1], 0, MAX_SEQUENCE)
        probe_id = _parse_decimal(tokens[2], 0, MAX_PROBE_ID)
        error_code = _parse_decimal(tokens[3], 1, MAX_ERROR_CODE)
        if sequence is not None and probe_id is not None and error_code is not None:
            return ErrorFrame(sequence, probe_id, error_code)
    return None


class FrameParser:
    """Incrementally parse bounded frames while recovering at the next '$'."""

    def __init__(self, *, max_line_bytes: int = 128) -> None:
        if max_line_bytes < 8:
            raise ValueError("max_line_bytes must be at least 8")
        self.max_line_bytes = max_line_bytes
        self.error_count = 0
        self._payload = bytearray()
        self._collecting = False

    def feed(self, data: bytes) -> list[InboundFrame]:
        if not isinstance(data, bytes):
            raise TypeError("serial data must be bytes")
        frames: list[InboundFrame] = []
        for byte in data:
            if byte == ord("$"):
                if self._collecting and self._payload:
                    self.error_count += 1
                self._payload.clear()
                self._collecting = True
                continue
            if byte in (ord("\r"), ord("\n")):
                if self._collecting and self._payload:
                    frame = _parse_line(bytes(self._payload))
                    if frame is None:
                        self.error_count += 1
                    else:
                        frames.append(frame)
                self._payload.clear()
                self._collecting = False
                continue
            if not self._collecting:
                continue
            if len(self._payload) >= self.max_line_bytes:
                self.error_count += 1
                self._payload.clear()
                self._collecting = False
                continue
            self._payload.append(byte)
        return frames
