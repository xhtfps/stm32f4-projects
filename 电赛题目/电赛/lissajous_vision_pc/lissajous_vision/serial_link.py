"""Nonblocking ownership of the PC-to-F407 serial connection."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Any

import serial
from serial.tools import list_ports

from .serial_protocol import (
    FrameParser,
    InboundFrame,
    SetCommand,
    encode_done,
    encode_failed,
    encode_locked,
    encode_run,
    encode_set,
    encode_stop,
)


class SerialLinkError(RuntimeError):
    pass


def resolve_port(port: str, port_lister: Callable[[], Iterable[Any]]) -> str:
    requested = str(port).strip()
    if requested and requested.lower() != "auto":
        return requested
    available = list(port_lister())
    if not available:
        raise SerialLinkError("未检测到串口；请连接 USB-TTL 或在配置中填写 COM 口")
    if len(available) != 1:
        choices = ", ".join(
            f"{item.device} ({getattr(item, 'description', 'unknown')})"
            for item in available
        )
        raise SerialLinkError(f"检测到多个串口，请在配置中明确选择：{choices}")
    return str(available[0].device)


class SerialLink:
    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 230400,
        serial_factory: Callable[..., Any] | None = None,
        port_lister: Callable[[], Iterable[Any]] | None = None,
    ) -> None:
        if baudrate <= 0:
            raise ValueError("baudrate must be positive")
        selected = resolve_port(port, port_lister or list_ports.comports)
        factory = serial_factory or serial.Serial
        try:
            self._serial = factory(
                port=selected,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0,
            )
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(f"无法打开串口 {selected}：{exc}") from exc
        self.port = selected
        self.baudrate = baudrate
        self._parser = FrameParser()
        self._closed = False

    def _require_open(self) -> None:
        if self._closed or not bool(getattr(self._serial, "is_open", True)):
            raise SerialLinkError("串口已关闭")

    def poll(self) -> list[InboundFrame]:
        self._require_open()
        try:
            waiting = int(getattr(self._serial, "in_waiting", 0))
            data = self._serial.read(waiting) if waiting > 0 else b""
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(f"串口 {self.port} 读取失败：{exc}") from exc
        return self._parser.feed(data)

    def _write(self, payload: bytes) -> None:
        self._require_open()
        try:
            written = int(self._serial.write(payload))
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(f"串口 {self.port} 写入失败：{exc}") from exc
        if written != len(payload):
            raise SerialLinkError(
                f"串口写入不完整：需要 {len(payload)} 字节，实际 {written} 字节"
            )

    def send_set(self, command: SetCommand) -> None:
        self._write(encode_set(command))

    def send_run(self, target_shape: int) -> None:
        self._write(encode_run(target_shape))

    def send_locked(self, sequence: int, input_frequency_millihz: int) -> None:
        self._write(encode_locked(sequence, input_frequency_millihz))

    def send_done(self, sequence: int, input_frequency_millihz: int) -> None:
        self._write(encode_done(sequence, input_frequency_millihz))

    def send_failed(self, sequence: int, error_code: int) -> None:
        self._write(encode_failed(sequence, error_code))

    def send_stop(self, sequence: int) -> None:
        self._write(encode_stop(sequence))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._serial.close()
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(f"关闭串口 {self.port} 失败：{exc}") from exc

    def __enter__(self) -> "SerialLink":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
