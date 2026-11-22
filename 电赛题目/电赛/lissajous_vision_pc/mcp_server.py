"""Anthropic/MCP-compliant stdio server for the PC vision link.

The official Python MCP SDK owns JSON-RPC framing, initialization/version
negotiation, capability handling, tool schemas, and protocol error responses.
This module only contains domain operations and input/path validation.
"""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import re
import threading
import time
from typing import Any, Literal

import cv2
from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations

from lissajous_vision.boundary import (
    detect_boundary_contacts,
    measure_boundary_touches,
)
from lissajous_vision.config import load_config
from lissajous_vision.io_utils import parse_video_source, read_image, write_image
from lissajous_vision.pipeline import LissajousAnalyzer
from lissajous_vision.serial_link import SerialLink
from lissajous_vision.serial_protocol import SetCommand


PROJECT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = PROJECT_DIR.parent.resolve()
DEFAULT_CONFIG = PROJECT_DIR / "configs" / "my_camera.json"
MAX_CALLS_PER_MINUTE = 120


class _RateLimiter:
    def __init__(self, maximum: int, window_seconds: float) -> None:
        self.maximum = maximum
        self.window_seconds = window_seconds
        self._calls: list[float] = []
        self._lock = threading.Lock()

    def check(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._calls[:] = [
                timestamp for timestamp in self._calls
                if now - timestamp < self.window_seconds
            ]
            if len(self._calls) >= self.maximum:
                raise RuntimeError("MCP tool rate limit exceeded; retry later")
            self._calls.append(now)


_rate_limiter = _RateLimiter(MAX_CALLS_PER_MINUTE, 60.0)


def _check_call() -> None:
    _rate_limiter.check()


def _jsonable(value: Any) -> Any:
    if hasattr(value, "item"):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _workspace_path(value: str | Path, *, output: bool = False) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = WORKSPACE_DIR / path
    resolved = path.resolve(strict=False)
    root = (PROJECT_DIR / "outputs").resolve() if output else WORKSPACE_DIR
    if not resolved.is_relative_to(root):
        scope = "outputs directory" if output else "workspace"
        raise ValueError(f"path must stay inside the {scope}")
    return resolved


def _config_path(config_path: str | None) -> Path:
    path = DEFAULT_CONFIG if config_path is None else _workspace_path(config_path)
    if not path.is_file():
        raise ValueError(f"vision config does not exist: {path}")
    return path


def _source(value: int | str) -> int | str:
    if isinstance(value, bool):
        raise ValueError("source must be a camera index or URL/path string")
    if isinstance(value, int):
        if not 0 <= value <= 16:
            raise ValueError("camera source index must be in 0..16")
        return value
    if not isinstance(value, str) or not value.strip() or len(value) > 2048:
        raise ValueError("source must be a non-empty string no longer than 2048 characters")
    if any(ord(char) < 32 for char in value):
        raise ValueError("source contains control characters")
    return value


def _port(value: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 64:
        raise ValueError("port must be a non-empty string no longer than 64 characters")
    if any(char.isspace() or ord(char) < 32 for char in value):
        raise ValueError("port must not contain whitespace or control characters")
    if not re.fullmatch(r"(?:COM\d+|/dev/tty[\w.\-]+|loop://[\w./:\-]+)", value, re.IGNORECASE):
        raise ValueError("port must be a COM port, /dev/tty device, or pyserial loop URL")
    return value


def _camera(source: int | str) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(parse_video_source(str(_source(source))))
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"unable to open camera or video source: {source}")
    return capture


def _capture_one(source: int | str, warmup: int = 2) -> tuple[Any, float]:
    if isinstance(warmup, bool) or not 0 <= warmup <= 20:
        raise ValueError("warmup must be in 0..20")
    capture = _camera(source)
    try:
        for _ in range(warmup):
            capture.grab()
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera opened but frame capture failed")
        return frame, time.time()
    finally:
        capture.release()


def _analyzer(config_path: str | None, grid_mode: Literal["auto", "fixed"]) -> LissajousAnalyzer:
    config = load_config(_config_path(config_path))
    if grid_mode == "auto":
        config.grid.corners_norm = None
    return LissajousAnalyzer(
        config,
        auto_grid=grid_mode == "auto",
        lock_auto_grid=True,
        use_temporal=False,
    )


def _analyze(
    frame: Any,
    config_path: str | None,
    grid_mode: Literal["auto", "fixed"],
) -> dict[str, Any]:
    analyzer = _analyzer(config_path, grid_mode)
    result = analyzer.analyze(frame, timestamp_ms=time.time() * 1000.0)
    contacts = detect_boundary_contacts(result.raw_mask)
    boundary = measure_boundary_touches(result.raw_mask, contacts=contacts)
    measurement = result.measurement
    return _jsonable({
        "valid": bool(measurement.valid and boundary.valid),
        "shape": measurement.shape,
        "confidence": float(measurement.confidence),
        "span_x_div": float(measurement.span_x_div),
        "span_y_div": float(measurement.span_y_div),
        "center_x_div": float(measurement.center_x_div),
        "center_y_div": float(measurement.center_y_div),
        "score_1x": float(getattr(measurement, "score_1x", 0.0)),
        "score_2x": float(getattr(measurement, "score_2x", 0.0)),
        "boundary": asdict(boundary),
        "grid_corners_px": None if result.corners is None else result.corners.tolist(),
    })


READ_ONLY = ToolAnnotations(
    readOnlyHint=True,
    destructiveHint=False,
    idempotentHint=True,
    openWorldHint=False,
)
SERIAL_WRITE = ToolAnnotations(
    readOnlyHint=False,
    destructiveHint=False,
    idempotentHint=True,
    openWorldHint=False,
)

mcp = FastMCP(
    "lissajous-vision",
    instructions=(
        "PC-side Lissajous vision and F407 serial control. "
        "Camera tools are read-only; serial tools can change hardware output."
    ),
)


@mcp.tool(
    title="List Cameras",
    description="Enumerate camera indexes that can be opened on this PC.",
    annotations=READ_ONLY,
)
def list_cameras(max_index: int = 8) -> dict[str, Any]:
    _check_call()
    if isinstance(max_index, bool) or not 1 <= max_index <= 16:
        raise ValueError("max_index must be in 1..16")
    found = []
    for index in range(max_index):
        capture = cv2.VideoCapture(index)
        try:
            if capture.isOpened():
                found.append({
                    "index": index,
                    "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                })
        finally:
            capture.release()
    return {"cameras": found}


@mcp.tool(
    title="Capture Frame",
    description="Capture one frame and save it under the project outputs directory.",
    annotations=READ_ONLY,
)
def capture_frame(
    source: int | str = 0,
    output: str = "lissajous_vision_pc/outputs/mcp_capture_latest.jpg",
    warmup: int = 2,
) -> dict[str, Any]:
    _check_call()
    frame, timestamp = _capture_one(source, warmup)
    output_path = _workspace_path(output, output=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_image(output_path, frame)
    return {
        "source": str(source),
        "path": str(output_path),
        "timestamp": timestamp,
        "width": int(frame.shape[1]),
        "height": int(frame.shape[0]),
        "mean_bgr": [float(item) for item in frame.mean(axis=(0, 1))],
    }


@mcp.tool(
    title="Analyze Image",
    description="Analyze one workspace image with the existing Lissajous vision pipeline.",
    annotations=READ_ONLY,
)
def analyze_image(
    path: str,
    config: str | None = None,
    grid_mode: Literal["auto", "fixed"] = "fixed",
) -> dict[str, Any]:
    _check_call()
    image_path = _workspace_path(path)
    try:
        frame = read_image(image_path)
    except (OSError, ValueError) as exc:
        raise ValueError(f"unable to read image: {image_path}") from exc
    result = _analyze(frame, config, grid_mode)
    result["path"] = str(image_path)
    return result


@mcp.tool(
    title="Capture And Analyze",
    description="Capture one live camera frame and return its vision measurements.",
    annotations=READ_ONLY,
)
def capture_and_analyze(
    source: int | str = 0,
    config: str | None = None,
    grid_mode: Literal["auto", "fixed"] = "fixed",
    warmup: int = 2,
) -> dict[str, Any]:
    _check_call()
    frame, timestamp = _capture_one(source, warmup)
    result = _analyze(frame, config, grid_mode)
    result["timestamp"] = timestamp
    return result


@mcp.tool(
    title="Watch Camera",
    description="Capture and analyze multiple live frames, returning throughput and validity.",
    annotations=READ_ONLY,
)
def watch_camera(
    source: int | str = 0,
    frames: int = 30,
    config: str | None = None,
    grid_mode: Literal["auto", "fixed"] = "fixed",
) -> dict[str, Any]:
    _check_call()
    if isinstance(frames, bool) or not 1 <= frames <= 300:
        raise ValueError("frames must be in 1..300")
    analyzer = _analyzer(config, grid_mode)
    capture = _camera(source)
    observations: list[dict[str, Any]] = []
    started = time.perf_counter()
    try:
        for _ in range(frames):
            ok, frame = capture.read()
            if not ok:
                break
            result = analyzer.analyze(frame, timestamp_ms=time.time() * 1000.0)
            observations.append({
                "valid": bool(result.measurement.valid),
                "shape": result.measurement.shape,
                "confidence": float(result.measurement.confidence),
                "span_x_div": float(result.measurement.span_x_div),
                "span_y_div": float(result.measurement.span_y_div),
            })
    finally:
        capture.release()
    elapsed = max(time.perf_counter() - started, 1e-9)
    return {
        "source": str(source),
        "requested_frames": frames,
        "captured_frames": len(observations),
        "elapsed_ms": round(elapsed * 1000.0, 2),
        "fps": round(len(observations) / elapsed, 2),
        "valid_frames": sum(item["valid"] for item in observations),
        "latest": observations[-1] if observations else None,
    }


@mcp.tool(
    title="Run Target Shape",
    description="Send RUN to start F407 PC-master control: 1 diagonal, 2 circle, 3 infinity.",
    annotations=SERIAL_WRITE,
)
def run_shape(
    port: str,
    target_shape: Literal[1, 2, 3],
    baudrate: int = 230400,
) -> dict[str, Any]:
    _check_call()
    _port(port)
    if isinstance(baudrate, bool) or not 1 <= baudrate <= 4_000_000:
        raise ValueError("baudrate must be in 1..4000000")
    with SerialLink(port, baudrate=baudrate) as link:
        link.send_run(target_shape)
    return {"sent": "RUN", "target_shape": target_shape}


@mcp.tool(
    title="Set DDS Output",
    description="Send one validated DDS SET command to F407.",
    annotations=SERIAL_WRITE,
)
def set_output(
    port: str,
    sequence: int,
    frequency_millihz: int,
    target_shape: Literal[1, 2, 3],
    probe_id: int = 0,
    phase_millideg: int = 0,
    amplitude_mv: int = 471,
    baudrate: int = 230400,
) -> dict[str, Any]:
    _check_call()
    _port(port)
    if isinstance(baudrate, bool) or not 1 <= baudrate <= 4_000_000:
        raise ValueError("baudrate must be in 1..4000000")
    command = SetCommand(
        sequence=sequence,
        probe_id=probe_id,
        frequency_millihz=frequency_millihz,
        phase_millideg=phase_millideg,
        amplitude_mv=amplitude_mv,
        target_shape=target_shape,
    )
    with SerialLink(port, baudrate=baudrate) as link:
        link.send_set(command)
    return {"sent": "SET", "sequence": sequence, "probe_id": probe_id}


@mcp.tool(
    title="Stop Vision Control",
    description="Send STOP to F407 PC-master control.",
    annotations=SERIAL_WRITE,
)
def stop(
    port: str,
    sequence: int,
    baudrate: int = 230400,
) -> dict[str, Any]:
    _check_call()
    _port(port)
    if isinstance(baudrate, bool) or not 1 <= baudrate <= 4_000_000:
        raise ValueError("baudrate must be in 1..4000000")
    with SerialLink(port, baudrate=baudrate) as link:
        link.send_stop(sequence)
    return {"sent": "STOP", "sequence": sequence}


def main() -> int:
    # FastMCP's stdio transport guarantees that stdout contains MCP messages
    # only; diagnostics from OpenCV and this process belong on stderr.
    mcp.run(transport="stdio")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
