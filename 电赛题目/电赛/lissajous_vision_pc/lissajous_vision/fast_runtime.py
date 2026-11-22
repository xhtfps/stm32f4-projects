"""Pure-PC visual validation for the K230 requirement-5 image pipeline."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time

import cv2
import numpy as np

from .boundary import (
    BoundaryContacts,
    BoundaryMeasurement,
    candidate_input_grids,
    detect_boundary_contacts,
    measure_boundary_touches,
)
from .calibration import interactive_calibration
from .config import load_config
from .fast_controller import (
    ControlStage,
    DoneEvent,
    FailedEvent,
    FastSearchController,
    LockedEvent,
    VisionObservation,
)
from .io_utils import parse_video_source, write_image
from .mesh import PeakMeasurement, measure_peak_points
from .pipeline import AnalysisResult, LissajousAnalyzer
from .serial_link import SerialLink, SerialLinkError
from .serial_protocol import AppliedFrame, ErrorFrame, SetCommand, StartFrame
from .text_render import draw_text_lines_bgr
from .visualizer import annotate_source


@dataclass(slots=True)
class RuntimeConfig:
    camera_source: str = "1"
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: float = 60.0
    vision_config: str = "configs/my_camera.json"
    grid_mode: str = "auto"
    panel_size: int = 320
    print_every: int = 10
    max_frames: int = 0
    serial_port: str = "auto"
    serial_baudrate: int = 230400
    ack_timeout_ms: int = 80
    max_ack_retries: int = 2
    control_timeout_ms: int = 60_000
    hold_ms: int = 5_000
    settle_frames: int = 1
    minimum_confidence: float = 0.80
    target_span_div: float = 8.0
    span_tolerance_div: float = 0.2
    verify_frames: int = 3

    @classmethod
    def load(cls, path: str | Path) -> "RuntimeConfig":
        config_path = Path(path)
        data = (
            json.loads(config_path.read_text(encoding="utf-8"))
            if config_path.exists()
            else {}
        )
        values = {
            name: data[name]
            for name in cls.__dataclass_fields__
            if name in data
        }
        runtime = cls(**values)
        runtime.camera_source = os.getenv(
            "LISSAJOUS_CAMERA", runtime.camera_source
        )
        runtime.max_frames = int(os.getenv(
            "LISSAJOUS_MAX_FRAMES", runtime.max_frames
        ))
        runtime.serial_port = os.getenv(
            "LISSAJOUS_SERIAL_PORT", runtime.serial_port
        ).strip()
        if runtime.grid_mode not in ("auto", "fixed"):
            raise ValueError("grid_mode 必须是 auto 或 fixed")
        if not runtime.serial_port:
            raise ValueError("serial_port 不能为空")
        positive = (
            "serial_baudrate", "ack_timeout_ms", "control_timeout_ms",
            "hold_ms", "target_span_div", "span_tolerance_div",
            "verify_frames",
        )
        for name in positive:
            if getattr(runtime, name) <= 0:
                raise ValueError(f"{name} 必须大于 0")
        if runtime.max_ack_retries < 0:
            raise ValueError("max_ack_retries 不能小于 0")
        if runtime.settle_frames < 0:
            raise ValueError("settle_frames 不能小于 0")
        if not 0.0 <= runtime.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence 必须在 0..1")
        return runtime


@dataclass(slots=True)
class RuntimeTelemetry:
    set_sent_ms: int | None = None
    applied_ms: int | None = None
    set_to_applied_ms: int | None = None
    applied_to_frame_ms: int | None = None
    frame_process_ms: int | None = None
    control_started_ms: int | None = None
    last_error: str = ""


def _dispatch_action(
    link: SerialLink,
    action: SetCommand | LockedEvent | DoneEvent | FailedEvent | None,
    telemetry: RuntimeTelemetry,
    now_ms: int,
) -> bool:
    if action is None:
        return False
    if isinstance(action, SetCommand):
        link.send_set(action)
        telemetry.set_sent_ms = now_ms
        telemetry.applied_ms = None
        telemetry.applied_to_frame_ms = None
        return True
    if isinstance(action, LockedEvent):
        link.send_locked(action.sequence, action.input_frequency_millihz)
        return False
    if isinstance(action, DoneEvent):
        link.send_done(action.sequence, action.input_frequency_millihz)
        return False
    if isinstance(action, FailedEvent):
        telemetry.last_error = action.reason
        link.send_failed(action.sequence, action.error_code)
        return False
    raise TypeError(f"unsupported controller action: {type(action).__name__}")


def _make_observation(
    result: AnalysisResult,
    boundary: BoundaryMeasurement,
    *,
    probe_grid: int,
) -> VisionObservation:
    measurement = result.measurement
    candidates = candidate_input_grids(probe_grid, boundary)
    return VisionObservation(
        valid=bool(measurement.valid and boundary.valid),
        ratio_y=boundary.ratio_y,
        ratio_x=boundary.ratio_x,
        left=boundary.left,
        right=boundary.right,
        top=boundary.top,
        bottom=boundary.bottom,
        # High-ratio search traces are outside the final three-shape classifier;
        # boundary consistency is the reliable confidence during SEARCH.
        confidence=boundary.confidence,
        candidates=frozenset(candidates),
        shape=measurement.shape,
        shape_confidence=float(measurement.confidence),
        span_x_div=float(measurement.span_x_div),
        span_y_div=float(measurement.span_y_div),
        center_x_div=float(measurement.center_x_div),
        center_y_div=float(measurement.center_y_div),
        score_diagonal=math.exp(-5.0 * max(0.0, measurement.line_error_norm)),
        score_circle=math.exp(-5.0 * max(0.0, measurement.ellipse_error)),
        score_infinity=float(measurement.score_2x),
    )


def _make_analyzer(
    settings: RuntimeConfig,
    project_dir: Path,
    grid_mode: str,
) -> LissajousAnalyzer:
    config = load_config(project_dir / settings.vision_config)
    left_marker = [0.0, 0.44, 0.075, 0.57]
    if left_marker not in config.segmentation.artifact_masks_norm:
        config.segmentation.artifact_masks_norm.append(left_marker)
    if grid_mode == "auto":
        config.grid.corners_norm = None
    elif config.grid.corners_norm is None:
        raise RuntimeError("固定标定模式缺少 configs/my_camera.json 四角数据")
    return LissajousAnalyzer(
        config,
        auto_grid=grid_mode == "auto",
        lock_auto_grid=True,
        use_temporal=False,
    )


def _calibration_name(result: AnalysisResult, grid_mode: str) -> str:
    if result.corners is None:
        return "未识别"
    calibration = result.measurement.debug.get("calibration", "")
    names = {
        "configured": "固定标定",
        "auto": "本帧自动识别",
        "cached_auto": "自动识别已锁定",
    }
    return names.get(str(calibration), "自动识别" if grid_mode == "auto" else "固定标定")


def _grid_confidence(result: AnalysisResult) -> float | None:
    value = result.measurement.debug.get("grid_confidence")
    return float(value) if isinstance(value, (int, float)) else None


def _k230_grid_metrics(result: AnalysisResult) -> dict[str, float | bool]:
    if result.corners is None or result.source is None:
        return {"passed": False, "area_ratio": 0.0, "aspect": 0.0}
    _, _, width, height = cv2.boundingRect(
        np.rint(result.corners).astype(np.int32)
    )
    frame_height, frame_width = result.source.shape[:2]
    area_ratio = width * height / float(max(1, frame_width * frame_height))
    aspect = width / float(max(1, height))
    return {
        "passed": (
            area_ratio >= 0.15 and 0.60 <= aspect <= 1.50
        ),
        "area_ratio": area_ratio,
        "aspect": aspect,
    }


def _peak_relationship(peaks: PeakMeasurement) -> str:
    if not peaks.valid:
        return "无法判断波峰点"
    if peaks.peak_count <= 0:
        return "未找到有效波峰点"
    return f"{peaks.peak_count} 个波峰点，判为 {peaks.peak_count} 倍"


# Compatibility for existing small scripts/tests.
_mesh_relationship = _peak_relationship


def _format_terminal_line(
    frame_index: int,
    result: AnalysisResult,
    boundary: BoundaryMeasurement,
    mesh: PeakMeasurement,
    grid_mode: str,
    fps: float,
) -> str:
    grid_text = _calibration_name(result, grid_mode)
    confidence = _grid_confidence(result)
    grid_metrics = _k230_grid_metrics(result)
    confidence_text = "-" if confidence is None else f"{confidence:.3f}"
    trace_text = "有效" if boundary.valid else f"无效({boundary.error})"
    return (
        f"[第 {frame_index} 帧] 网格：{grid_text}，置信度：{confidence_text}；"
        f"轨迹：{trace_text}；四边触点：左{boundary.left}/右{boundary.right}/"
        f"上{boundary.top}/下{boundary.bottom}；辅助比例："
        f"{boundary.ratio_y}:{boundary.ratio_x}；波峰点主判："
        f"{_peak_relationship(mesh)}；波峰点置信度：{mesh.confidence:.3f}；"
        f"左右峰：{mesh.left_peaks}/{mesh.right_peaks}；"
        f"K230网格门限："
        f"{'通过' if grid_metrics['passed'] else '不通过'}"
        f"(面积{grid_metrics['area_ratio']:.3f}，长宽比{grid_metrics['aspect']:.3f})；"
        f"处理帧率：{fps:.1f} FPS"
    )


def _fit_panel(image: np.ndarray, label: str, panel_size: int) -> np.ndarray:
    label_height = 30
    canvas = np.full((panel_size, panel_size, 3), 18, dtype=np.uint8)
    available_height = panel_size - label_height
    height, width = image.shape[:2]
    scale = min(panel_size / width, available_height / height)
    resized = cv2.resize(
        image,
        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
        interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
    )
    x0 = (panel_size - resized.shape[1]) // 2
    y0 = label_height + (available_height - resized.shape[0]) // 2
    canvas[y0 : y0 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
    draw_text_lines_bgr(
        canvas, [label], origin=(8, 2), font_size=18, line_gap=0
    )
    return canvas


def _draw_contacts(image: np.ndarray, contacts: BoundaryContacts) -> None:
    if contacts.bbox is None:
        return
    left, top, right, bottom = contacts.bbox
    cv2.rectangle(image, (left, top), (right, bottom), (0, 165, 255), 3)
    side_points = (
        (contacts.left, (0, 0, 255)),
        (contacts.right, (255, 0, 0)),
        (contacts.top, (255, 0, 255)),
        (contacts.bottom, (255, 255, 0)),
    )
    for points, color in side_points:
        for index, point in enumerate(points, start=1):
            cv2.circle(image, point, 11, color, -1, cv2.LINE_AA)
            cv2.putText(
                image, str(index), (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA,
            )


def _draw_mesh_holes(image: np.ndarray, mesh: PeakMeasurement) -> None:
    if mesh.hole_mask is None or not mesh.centers:
        return
    contours, _ = cv2.findContours(
        mesh.hole_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    overlay = image.copy()
    overlay[mesh.hole_mask > 0] = (20, 120, 20)
    cv2.addWeighted(overlay, 0.35, image, 0.65, 0.0, image)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 4, cv2.LINE_AA)
    for index, center in enumerate(mesh.centers, start=1):
        cv2.circle(image, center, 18, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.putText(
            image, str(index), (center[0] - 7, center[1] + 7),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA,
        )


def _render_panels(
    result: AnalysisResult,
    contacts: BoundaryContacts,
    mesh: PeakMeasurement,
    panel_size: int,
) -> np.ndarray:
    source = annotate_source(result)
    if result.warped is None:
        warped = np.zeros((800, 800, 3), dtype=np.uint8)
    else:
        warped = result.warped.copy()
        height, width = warped.shape[:2]
        for index in range(9):
            x = int(round(index * (width - 1) / 8.0))
            y = int(round(index * (height - 1) / 8.0))
            cv2.line(warped, (x, 0), (x, height - 1), (80, 160, 80), 1)
            cv2.line(warped, (0, y), (width - 1, y), (80, 160, 80), 1)
    mask = (
        cv2.cvtColor(result.raw_mask, cv2.COLOR_GRAY2BGR)
        if result.raw_mask is not None
        else np.zeros_like(warped)
    )
    _draw_mesh_holes(warped, mesh)
    _draw_mesh_holes(mask, mesh)
    _draw_contacts(warped, contacts)
    _draw_contacts(mask, contacts)
    return np.hstack((
        _fit_panel(source, "原图与识别四角", panel_size),
        _fit_panel(warped, "透视校正后的 8x8 网格", panel_size),
        _fit_panel(mask, "黄色轨迹二值掩膜", panel_size),
    ))


def _render_validation_preview(
    result: AnalysisResult,
    boundary: BoundaryMeasurement,
    contacts: BoundaryContacts,
    mesh: PeakMeasurement,
    *,
    grid_mode: str,
    fps: float,
    panel_size: int,
    grid_success: int,
    total_frames: int,
    controller: FastSearchController,
    serial_port: str,
    telemetry: RuntimeTelemetry,
    now_ms: int,
) -> np.ndarray:
    panels = _render_panels(result, contacts, mesh, panel_size)
    header_height = 256
    header = np.zeros((header_height, panels.shape[1], 3), dtype=np.uint8)
    confidence = _grid_confidence(result)
    confidence_text = "-" if confidence is None else f"{confidence:.3f}"
    success_rate = 100.0 * grid_success / max(total_frames, 1)
    grid_metrics = _k230_grid_metrics(result)
    target_name = {0: "等待启动", 1: "对角线", 2: "圆/椭圆", 3: "无穷符号"}[
        controller.target_shape
    ]
    input_text = (
        "-" if controller.input_frequency_millihz is None
        else f"{controller.input_frequency_millihz / 1_000_000.0:.1f} Hz"
    )
    control_elapsed = (
        0 if controller.started_ms is None else now_ms - controller.started_ms
    )
    hold_elapsed = (
        0 if controller.hold_started_ms is None else now_ms - controller.hold_started_ms
    )
    def timing(value: int | None) -> str:
        return "-" if value is None else str(value)
    lines = [
        (
            f"PC-F407 视觉闭环  串口：{serial_port} @ 230400  "
            f"目标：{target_name}  阶段：{controller.stage.name}"
        ),
        (
            f"输入候选：{input_text}  DDS："
            f"{controller.output_frequency_millihz / 1_000_000.0:.1f} Hz / "
            f"{controller.output_phase_millideg / 1000.0:.1f} deg / "
            f"{controller.output_amplitude_mv} mV  "
            f"等待回执：{'是' if controller.waiting_for_applied else '否'}"
        ),
        (
            f"耗时：SET->APPLIED {timing(telemetry.set_to_applied_ms)} ms  "
            f"APPLIED->FRAME {timing(telemetry.applied_to_frame_ms)} ms  "
            f"处理 {timing(telemetry.frame_process_ms)} ms  "
            f"控制 {control_elapsed} ms  保持 {hold_elapsed} ms"
        ),
        (
            f"PC 网格验证  模式：{'自动找屏' if grid_mode == 'auto' else '固定标定'}  "
            f"结果：{_calibration_name(result, grid_mode)}  置信度：{confidence_text}"
        ),
        (
            f"K230网格门限：{'通过' if grid_metrics['passed'] else '不通过'}  "
            f"面积占比：{grid_metrics['area_ratio']:.3f} (需>=0.15)  "
            f"长宽比：{grid_metrics['aspect']:.3f} (需0.60~1.50)"
        ),
        (
            f"波峰点主判（左右边界峰值）：{mesh.peak_count} 个 -> "
            f"{mesh.peak_count if mesh.peak_count > 0 else '-'} 倍关系  "
            f"置信度：{mesh.confidence:.3f}  "
            f"三带候选：{'/'.join(map(str, mesh.edge_candidates)) or '-'}  "
            f"{'稳定' if mesh.confidence >= 0.9 else '需第二相位帧确认'}"
        ),
        (
            f"四边触点辅助 左/右/上/下：{boundary.left}/{boundary.right}/"
            f"{boundary.top}/{boundary.bottom}  约分比例："
            f"{boundary.ratio_y}:{boundary.ratio_x}  置信度：{boundary.confidence:.3f}"
        ),
        (
            f"峰值点：左红 右蓝 上紫 下青  "
            f"识别成功率：{grid_success}/{total_frames} ({success_rate:.1f}%)  "
            f"帧率：{fps:.1f} FPS  错误：{telemetry.last_error or '-'}"
        ),
    ]
    draw_text_lines_bgr(
        header, lines, origin=(12, 6), font_size=18, line_gap=2
    )
    return np.vstack((header, panels))


def run(
    config_path: str | Path,
    *,
    serial_port: str | None = None,
    serial_baudrate: int | None = None,
) -> int:
    project_dir = Path(config_path).resolve().parent.parent
    settings = RuntimeConfig.load(config_path)
    if serial_port is not None:
        settings.serial_port = serial_port.strip()
        if not settings.serial_port:
            raise ValueError("serial_port 不能为空")
    if serial_baudrate is not None:
        if serial_baudrate <= 0:
            raise ValueError("serial_baudrate 必须大于 0")
        settings.serial_baudrate = serial_baudrate
    grid_mode = settings.grid_mode
    analyzer = _make_analyzer(settings, project_dir, grid_mode)
    controller = FastSearchController(
        ack_timeout_ms=settings.ack_timeout_ms,
        max_ack_retries=settings.max_ack_retries,
        control_timeout_ms=settings.control_timeout_ms,
        hold_ms=settings.hold_ms,
        minimum_confidence=settings.minimum_confidence,
        target_span_div=settings.target_span_div,
        span_tolerance_div=settings.span_tolerance_div,
        verify_frames=settings.verify_frames,
    )
    telemetry = RuntimeTelemetry()
    link = SerialLink(
        settings.serial_port, baudrate=settings.serial_baudrate
    )
    capture = cv2.VideoCapture(parse_video_source(settings.camera_source))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, settings.camera_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.camera_height)
    capture.set(cv2.CAP_PROP_FPS, settings.camera_fps)
    if not capture.isOpened():
        link.close()
        raise RuntimeError(f"无法打开摄像头：{settings.camera_source}")

    window = "PC-F407 视觉闭环"
    snapshot_dir = project_dir / "outputs" / "snapshots"
    last_clock = time.perf_counter()
    fps = 0.0
    frame_index = 0
    grid_success = 0
    settle_frames_remaining = -1
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    try:
        while True:
            frame_index += 1
            loop_now_ms = int(time.perf_counter() * 1000.0)
            for inbound in link.poll():
                if isinstance(inbound, StartFrame):
                    action = controller.start(inbound.target_shape, loop_now_ms)
                    telemetry = RuntimeTelemetry(control_started_ms=loop_now_ms)
                    settle_frames_remaining = -1
                    _dispatch_action(link, action, telemetry, loop_now_ms)
                    print(
                        f"收到 F407 START：目标 {inbound.target_shape}，开始闭环。",
                        flush=True,
                    )
                elif isinstance(inbound, AppliedFrame):
                    if controller.on_applied(
                        inbound.sequence, inbound.probe_id, loop_now_ms
                    ):
                        telemetry.applied_ms = loop_now_ms
                        telemetry.set_to_applied_ms = (
                            None if telemetry.set_sent_ms is None
                            else loop_now_ms - telemetry.set_sent_ms
                        )
                        telemetry.applied_to_frame_ms = None
                        settle_frames_remaining = settings.settle_frames
                elif isinstance(inbound, ErrorFrame):
                    action = controller.on_error(
                        inbound.sequence, inbound.probe_id, inbound.error_code
                    )
                    if action is not None:
                        _dispatch_action(link, action, telemetry, loop_now_ms)
                        settle_frames_remaining = -1

            tick_action = controller.tick(loop_now_ms)
            if _dispatch_action(link, tick_action, telemetry, loop_now_ms):
                settle_frames_remaining = -1

            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("摄像头已打开，但读取帧失败")
            captured_ms = int(time.perf_counter() * 1000.0)
            process_started = time.perf_counter()
            result = analyzer.analyze(
                frame, timestamp_ms=float(captured_ms)
            )
            contacts = detect_boundary_contacts(
                result.raw_mask,
                edge_band_ratio=0.030,
                maximum_gap_ratio=0.003,
                minimum_run_ratio=0.010,
            )
            boundary = measure_boundary_touches(
                result.raw_mask, contacts=contacts
            )
            mesh = measure_peak_points(result.raw_mask)
            telemetry.frame_process_ms = int(
                round((time.perf_counter() - process_started) * 1000.0)
            )
            grid_success += int(_k230_grid_metrics(result)["passed"])

            if (
                not controller.waiting_for_applied
                and controller.applied_ms is not None
                and controller.stage not in (
                    ControlStage.WAIT_START,
                    ControlStage.DONE,
                    ControlStage.FAILED,
                )
            ):
                if settle_frames_remaining > 0:
                    settle_frames_remaining -= 1
                elif settle_frames_remaining == 0:
                    if telemetry.applied_to_frame_ms is None:
                        telemetry.applied_to_frame_ms = max(
                            0, captured_ms - controller.applied_ms
                        )
                    observation = _make_observation(
                        result,
                        boundary,
                        probe_grid=controller.current_probe_grid,
                    )
                    observed_ms = int(time.perf_counter() * 1000.0)
                    action = controller.observe(observation, observed_ms)
                    if _dispatch_action(link, action, telemetry, observed_ms):
                        settle_frames_remaining = -1

            now = time.perf_counter()
            instant_fps = 1.0 / max(now - last_clock, 1e-9)
            last_clock = now
            fps = instant_fps if fps == 0.0 else 0.9 * fps + 0.1 * instant_fps
            preview = _render_validation_preview(
                result,
                boundary,
                contacts,
                mesh,
                grid_mode=grid_mode,
                fps=fps,
                panel_size=settings.panel_size,
                grid_success=grid_success,
                total_frames=frame_index,
                controller=controller,
                serial_port=link.port,
                telemetry=telemetry,
                now_ms=int(now * 1000.0),
            )
            if frame_index == 1:
                cv2.resizeWindow(window, preview.shape[1], preview.shape[0])
            if frame_index % max(1, settings.print_every) == 0:
                print(_format_terminal_line(
                    frame_index, result, boundary, mesh, grid_mode, fps
                ), flush=True)
            cv2.imshow(window, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("r"), ord("R")):
                analyzer.reset(keep_calibration=False)
                grid_success = 0
                frame_index = 0
                print("已清除自动识别结果，下一帧重新寻找网格。")
            if key in (ord("a"), ord("A")):
                grid_mode = "auto"
                analyzer = _make_analyzer(settings, project_dir, grid_mode)
                grid_success = 0
                frame_index = 0
                print("已切换到自动找网格模式。")
            if key in (ord("f"), ord("F")):
                grid_mode = "fixed"
                analyzer = _make_analyzer(settings, project_dir, grid_mode)
                grid_success = 0
                frame_index = 0
                print("已切换到固定四角标定模式。")
            if key in (ord("c"), ord("C")):
                calibration_config = load_config(
                    project_dir / settings.vision_config
                )
                output_config = project_dir / settings.vision_config
                corners = interactive_calibration(
                    frame, calibration_config, output_config,
                )
                if corners is not None:
                    grid_mode = "fixed"
                    settings.grid_mode = "fixed"
                    analyzer = _make_analyzer(settings, project_dir, grid_mode)
                    grid_success = 0
                    frame_index = 0
                    print(f"手动四角已保存并立即生效：{output_config}")
            if key in (ord("s"), ord("S")):
                stamp = time.strftime("%Y%m%d_%H%M%S")
                raw_path = snapshot_dir / f"{stamp}_raw.jpg"
                debug_path = snapshot_dir / f"{stamp}_grid_debug.jpg"
                write_image(raw_path, frame)
                write_image(debug_path, preview)
                print(f"已保存原图：{raw_path}")
                print(f"已保存网格识别图：{debug_path}")
            if settings.max_frames and frame_index >= settings.max_frames:
                smoke_path = os.getenv("LISSAJOUS_SAVE_PREVIEW")
                if smoke_path:
                    preview_path = Path(smoke_path)
                    write_image(preview_path, preview)
                    write_image(
                        preview_path.with_name(preview_path.stem + "_raw.jpg"),
                        frame,
                    )
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()
        try:
            if controller.sequence != 0:
                link.send_stop(controller.sequence)
        except SerialLinkError:
            pass
        link.close()
    return 0


def main() -> int:
    project_dir = Path(__file__).resolve().parents[1]
    return run(project_dir / "configs" / "pc_fast_runtime.json")


if __name__ == "__main__":
    raise SystemExit(main())
