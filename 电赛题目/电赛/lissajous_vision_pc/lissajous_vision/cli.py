"""命令行入口，提供单图、批处理、摄像头、视频和手动标定五种模式。

此文件只负责参数解析、资源生命周期和输出格式；视觉算法本身集中在
``LissajousAnalyzer``，因此电脑端和后续 K230 移植可以共享同一处理结构。
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time
from typing import Iterable

import cv2

from .calibration import interactive_calibration
from .config import load_config
from .io_utils import iter_images, parse_video_source, read_image, write_image
from .pipeline import LissajousAnalyzer
from .visualizer import render_debug


def _add_analysis_options(parser: argparse.ArgumentParser) -> None:
    """向多个子命令添加相同的配置、目标和调试选项。"""
    parser.add_argument("--config", type=Path, help="JSON 标定与参数文件")
    parser.add_argument("--target", help="可选目标，例如 1x-8div 或 2x-8div")
    parser.add_argument(
        "--no-auto-grid",
        action="store_true",
        help="只使用配置文件中的固定角点，不再自动找屏",
    )
    parser.add_argument(
        "--include-debug", action="store_true", help="在 JSON 中包含调试信息"
    )


def _print_json(data: object) -> None:
    """以 UTF-8 中文和缩进格式打印单个 JSON 对象。"""
    print(json.dumps(data, ensure_ascii=False, indent=2))


def _make_analyzer(args: argparse.Namespace, *, stream: bool) -> LissajousAnalyzer:
    """根据命令行参数创建分析器；视频流自动启用角点缓存和时序处理。"""
    return LissajousAnalyzer(
        load_config(args.config),
        auto_grid=not args.no_auto_grid,
        lock_auto_grid=stream,
        use_temporal=stream,
        target=args.target,
    )


def command_image(args: argparse.Namespace) -> int:
    """处理单张图片，可输出 JSON、保存三联图或打开预览窗口。"""
    try:
        image = read_image(args.input)
    except (OSError, ValueError) as error:
        print(f"无法读取图像：{args.input}", file=sys.stderr)
        print(str(error), file=sys.stderr)
        return 2
    # 静态图片没有真实时间轴，统一使用 0 且关闭多帧时序。
    result = _make_analyzer(args, stream=False).analyze(image, timestamp_ms=0)
    _print_json(result.measurement.to_dict(include_debug=args.include_debug))
    if args.debug_image:
        write_image(args.debug_image, render_debug(result))
    if args.show:
        window_name = "Lissajous Vision"
        preview = render_debug(result, panel_size=args.panel_size)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(window_name, preview.shape[1], preview.shape[0])
        cv2.imshow(window_name, preview)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0 if result.measurement.valid else 3


def _write_csv(path: Path, records: list[dict[str, object]]) -> None:
    """写入 Excel 可直接打开的 UTF-8 BOM CSV 文件。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not records:
        path.write_text("", encoding="utf-8-sig")
        return
    # 字段顺序以第一条 Measurement 字典为准，保证多次导出列顺序一致。
    fields = list(records[0].keys())
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)


def command_batch(args: argparse.Namespace) -> int:
    """逐张处理目录图片，每张图独立找屏并汇总为 CSV/JSON。"""
    files = list(iter_images(args.input, recursive=args.recursive))
    if not files:
        print(f"目录中没有找到支持的图像：{args.input}", file=sys.stderr)
        return 2

    records: list[dict[str, object]] = []
    valid_count = 0
    for index, path in enumerate(files, start=1):
        try:
            image = read_image(path)
        except (OSError, ValueError) as error:
            print(f"[{index:>3}/{len(files)}] {path.name}：{error}", file=sys.stderr)
            continue
        # 不同照片可能来自不同机位，因此不能复用上一张的自动角点。
        analyzer = _make_analyzer(args, stream=False)
        result = analyzer.analyze(image, timestamp_ms=0)
        measurement = result.measurement.to_dict(include_debug=args.include_debug)
        measurement = {"file": str(path), **measurement}
        records.append(measurement)
        valid_count += int(result.measurement.valid)
        print(
            f"[{index:>3}/{len(files)}] {path.name}: "
            f"{result.measurement.shape}，"
            f"{result.measurement.span_x_div:.2f}×"
            f"{result.measurement.span_y_div:.2f} 格"
        )
        if args.debug_dir:
            relative = path.relative_to(args.input)
            output = args.debug_dir / relative.with_suffix(".debug.jpg")
            write_image(output, render_debug(result))

    if args.csv:
        _write_csv(args.csv, records)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(records, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(f"有效结果：{valid_count}/{len(files)}")
    return 0 if valid_count == len(files) else 3


def _open_capture(source: str) -> cv2.VideoCapture:
    """打开摄像头编号、视频文件或 HTTP/RTSP 地址。"""
    parsed = parse_video_source(source)
    return cv2.VideoCapture(parsed)


def _stream_frames(
    capture: cv2.VideoCapture,
    *,
    is_video: bool,
) -> Iterable[tuple[float, object]]:
    """惰性读取视频帧，并为每帧提供毫秒时间戳。

    文件使用容器内的播放位置，实时摄像头使用单调时钟，避免电脑系统时间被
    校准后发生倒退。
    """
    while True:
        ok, frame = capture.read()
        if not ok:
            return
        if is_video:
            timestamp = float(capture.get(cv2.CAP_PROP_POS_MSEC))
        else:
            timestamp = time.perf_counter() * 1000.0
        yield timestamp, frame


def command_stream(args: argparse.Namespace, *, is_video: bool) -> int:
    """持续分析摄像头/视频，直到流结束、达到帧上限或用户按退出键。"""
    capture = _open_capture(args.source)
    if not capture.isOpened():
        print(f"无法打开视频源：{args.source}", file=sys.stderr)
        return 2
    # VideoCapture 后端可能不支持指定参数；set 失败时 OpenCV 会保留默认值。
    if args.width:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.capture_fps:
        capture.set(cv2.CAP_PROP_FPS, args.capture_fps)

    analyzer = _make_analyzer(args, stream=True)
    # 输出视频在取得第一张调试图后才创建，届时才能确定实际画面尺寸。
    writer: cv2.VideoWriter | None = None
    last_clock = time.perf_counter()
    smoothed_fps = 0.0
    exit_code = 0
    try:
        for frame_index, (timestamp, frame) in enumerate(
            _stream_frames(capture, is_video=is_video), start=1
        ):
            result = analyzer.analyze(frame, timestamp_ms=timestamp)
            now = time.perf_counter()
            # 指数滑动平均让屏幕 FPS 不因单帧调度抖动剧烈跳变。
            instantaneous_fps = 1.0 / max(now - last_clock, 1e-9)
            last_clock = now
            smoothed_fps = (
                instantaneous_fps
                if smoothed_fps == 0.0
                else 0.9 * smoothed_fps + 0.1 * instantaneous_fps
            )
            debug = render_debug(
                result,
                fps=smoothed_fps,
                panel_size=args.panel_size,
            )

            # JSONL 每行一个完整对象，适合串口桥、日志程序和实时上位机消费。
            if args.jsonl and frame_index % args.print_every == 0:
                print(
                    json.dumps(
                        result.measurement.to_dict(include_debug=args.include_debug),
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
            if args.output_video:
                if writer is None:
                    args.output_video.parent.mkdir(parents=True, exist_ok=True)
                    # 网络摄像头常返回 0 或异常 FPS，无法信任时回退到 20。
                    fps = float(capture.get(cv2.CAP_PROP_FPS))
                    if not (1.0 <= fps <= 240.0):
                        fps = 20.0
                    writer = cv2.VideoWriter(
                        str(args.output_video),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        fps,
                        (debug.shape[1], debug.shape[0]),
                    )
                writer.write(debug)
            if args.show:
                window_name = (
                    "Lissajous Vision | Q/Esc: Quit | "
                    "R: Redetect | S: Snapshot"
                )
                # WINDOW_NORMAL 允许用户缩放；只在第一帧创建窗口，避免闪烁。
                if frame_index == 1:
                    cv2.namedWindow(
                        window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                    )
                    cv2.resizeWindow(window_name, debug.shape[1], debug.shape[0])
                cv2.imshow(window_name, debug)
                key = cv2.waitKey(1) & 0xFF
                # Q/Esc 退出，R 清除自动角点缓存，S 同时保存原图与调试图。
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("r"), ord("R")):
                    analyzer.reset(keep_calibration=False)
                if key in (ord("s"), ord("S")):
                    snapshot_id = time.strftime("%Y%m%d_%H%M%S")
                    raw_path = args.snapshot_dir / f"{snapshot_id}_raw.jpg"
                    debug_path = args.snapshot_dir / f"{snapshot_id}_debug.jpg"
                    write_image(raw_path, frame)
                    write_image(debug_path, debug)
                    print(f"已保存原始图和调试图：{raw_path}，{debug_path}")
            if args.max_frames and frame_index >= args.max_frames:
                break
    # 无论正常退出还是抛出异常，都释放摄像头、编码器和 OpenCV 窗口。
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
    return exit_code


def command_calibrate(args: argparse.Namespace) -> int:
    """从静态图片或视频源抓取一帧，进入四点手动标定界面。"""
    if args.image:
        try:
            frame = read_image(args.image)
        except (OSError, ValueError) as error:
            print(f"无法读取图像：{args.image}", file=sys.stderr)
            print(str(error), file=sys.stderr)
            return 2
    else:
        capture = _open_capture(args.source)
        if not capture.isOpened():
            print(f"无法打开视频源：{args.source}", file=sys.stderr)
            return 2
        # 标定只需要一张清晰帧，读取后立即释放摄像头，避免占用设备。
        ok, frame = capture.read()
        capture.release()
        if not ok:
            print("视频源已打开，但没有取得图像", file=sys.stderr)
            return 2
    config = load_config(args.config)
    corners = interactive_calibration(frame, config, args.output_config)
    if corners is None:
        print("已取消标定", file=sys.stderr)
        return 1
    print(f"标定参数已保存：{args.output_config}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """构造完整 argparse 命令树；单独成函数便于测试默认参数。"""
    parser = argparse.ArgumentParser(
        prog="lissajous-vision",
        description="DS2302A 李萨如图形电脑视觉测量程序",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    image = subparsers.add_parser("image", help="分析一张静态图片")
    image.add_argument("input", type=Path)
    _add_analysis_options(image)
    image.add_argument("--debug-image", type=Path)
    image.add_argument("--show", action="store_true")
    image.add_argument(
        "--panel-size",
        type=int,
        default=500,
        help="每个预览面板的边长，默认 500",
    )
    image.set_defaults(handler=command_image)

    batch = subparsers.add_parser("batch", help="批量分析一个图片目录")
    batch.add_argument("input", type=Path)
    _add_analysis_options(batch)
    batch.add_argument("--recursive", action="store_true")
    batch.add_argument("--csv", type=Path)
    batch.add_argument("--json", type=Path)
    batch.add_argument("--debug-dir", type=Path)
    batch.set_defaults(handler=command_batch)

    # camera 与 video 的参数完全一致，仅时间戳来源不同，因此在循环中创建。
    for name, help_text in (
        ("camera", "分析 USB、虚拟或手机摄像头"),
        ("video", "分析视频文件或网络视频流"),
    ):
        stream = subparsers.add_parser(name, help=help_text)
        stream.add_argument("source", nargs="?", default="1")
        _add_analysis_options(stream)
        stream.add_argument("--show", action=argparse.BooleanOptionalAction, default=True)
        stream.add_argument("--jsonl", action="store_true")
        stream.add_argument("--print-every", type=int, default=1)
        stream.add_argument("--output-video", type=Path)
        stream.add_argument(
            "--snapshot-dir",
            type=Path,
            default=Path("outputs/snapshots"),
            help="按 S 键时保存图片的目录",
        )
        stream.add_argument("--max-frames", type=int)
        stream.add_argument("--width", type=int)
        stream.add_argument("--height", type=int)
        stream.add_argument("--capture-fps", type=float)
        stream.add_argument(
            "--panel-size",
            type=int,
            default=500,
            help="每个预览面板的边长，默认 500；屏幕较小时可设为 420",
        )
        stream.set_defaults(
            handler=(
                (lambda options: command_stream(options, is_video=False))
                if name == "camera"
                else (lambda options: command_stream(options, is_video=True))
            )
        )

    calibrate = subparsers.add_parser(
        "calibrate", help="点击栅格四角并保存标定参数"
    )
    source = calibrate.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path)
    source.add_argument("--source", help="camera index, URL, or video source")
    calibrate.add_argument("--config", type=Path, help="基础参数文件")
    calibrate.add_argument("--output-config", type=Path, required=True)
    calibrate.set_defaults(handler=command_calibrate)
    return parser


def main(argv: list[str] | None = None) -> int:
    """解析参数并调用对应处理函数，返回适合 shell/VS Code 的退出码。"""
    parser = build_parser()
    args = parser.parse_args(argv)
    if hasattr(args, "print_every") and args.print_every < 1:
        parser.error("--print-every 必须大于或等于 1")
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
