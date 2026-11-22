"""生成中文三联调试界面：原图、透视图和 HSV 轨迹掩膜。"""

from __future__ import annotations

import cv2
import numpy as np

from .pipeline import AnalysisResult
from .text_render import draw_text_lines_bgr


SHAPE_NAMES_ZH = {
    # 协议保留稳定的英文枚举，显示层在此集中映射为中文。
    "diagonal": "对角直线",
    "ellipse": "椭圆",
    "infinity": "无穷图形",
    "unknown": "未知",
}


def _put_lines(
    image: np.ndarray,
    lines: list[str],
    *,
    origin: tuple[int, int] = (15, 30),
    scale: float = 0.65,
) -> None:
    """按统一字号、行距和描边样式在图像左上角绘制多行文字。"""
    draw_text_lines_bgr(
        image,
        lines,
        origin=origin,
        font_size=max(14, int(round(24 * scale / 0.65))),
        line_gap=max(3, int(round(6 * scale / 0.65))),
    )


def annotate_source(result: AnalysisResult) -> np.ndarray:
    """在原始摄像头画面上叠加四角、编号和标定状态。"""
    canvas = result.source.copy()
    if result.corners is not None:
        # 绿色四边形应与 8×8 栅格外框重合，黄色编号对应 TL/TR/BR/BL。
        corners = np.rint(result.corners).astype(np.int32)
        cv2.polylines(canvas, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 5)
        for index, point in enumerate(corners):
            cv2.circle(canvas, tuple(point), 10, (0, 255, 255), -1)
            cv2.putText(
                canvas,
                str(index + 1),
                (int(point[0]) + 12, int(point[1]) - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                3,
                cv2.LINE_AA,
            )
    status = "栅格标定正常" if result.corners is not None else "未找到示波器栅格"
    _put_lines(canvas, [status], scale=1.0)
    return canvas


def annotate_warped(
    result: AnalysisResult,
    *,
    fps: float | None = None,
) -> np.ndarray:
    """在标准正视图上叠加参考网格、参与测量的绿色像素和主要结果。"""
    if result.warped is None:
        canvas = np.zeros((800, 800, 3), dtype=np.uint8)
        _put_lines(canvas, [result.measurement.error or "结果无效"])
        return canvas

    canvas = result.warped.copy()
    height, width = canvas.shape[:2]
    # 手工绘制 9 条边界/网格线，使用户能快速判断透视后的分度是否均匀。
    for index in range(9):
        x = int(round(index * (width - 1) / 8.0))
        y = int(round(index * (height - 1) / 8.0))
        cv2.line(canvas, (x, 0), (x, height - 1), (80, 80, 80), 1, cv2.LINE_AA)
        cv2.line(canvas, (0, y), (width - 1, y), (80, 80, 80), 1, cv2.LINE_AA)
    cv2.line(canvas, (width // 2, 0), (width // 2, height - 1), (130, 130, 130), 2)
    cv2.line(canvas, (0, height // 2), (width - 1, height // 2), (130, 130, 130), 2)

    if result.geometry_mask is not None:
        curve = result.geometry_mask > 0
        # 绿色区域就是 geometry.py 实际使用的点，便于发现 HSV 断线或误检。
        canvas[curve] = (0, 255, 0)

    measurement = result.measurement
    if measurement.valid:
        lines = [
            (
                f"类型：{SHAPE_NAMES_ZH.get(measurement.shape, measurement.shape)}  "
                f"置信度：{measurement.confidence:.3f}"
            ),
            f"跨度 X/Y：{measurement.span_x_div:.2f}/{measurement.span_y_div:.2f} 格",
            (
                f"中心 X/Y：{measurement.center_x_div:+.2f}/"
                f"{measurement.center_y_div:+.2f} 格"
            ),
            f"评分 1:1/2:1：{measurement.score_1x:.3f}/{measurement.score_2x:.3f}",
            (
                f"运动量：{measurement.motion_score:.3f}  "
                f"稳定：{'是' if measurement.stable else '否'}"
            ),
        ]
        if measurement.flip_period_ms > 0:
            lines.append(
                f"翻转周期：{measurement.flip_period_ms:.0f} ms "
                f"（可信度 {measurement.flip_confidence:.2f}）"
            )
    else:
        lines = [f"结果无效：{measurement.error}"]
    if fps is not None:
        lines.append(f"处理帧率：{fps:.1f} FPS")
    _put_lines(canvas, lines)
    return canvas


def render_debug(
    result: AnalysisResult,
    *,
    fps: float | None = None,
    panel_size: int = 500,
) -> np.ndarray:
    """把三个不同纵横比的图像等比例装入固定方形面板后横向拼接。

    默认每个面板 500 像素，三联图总宽 1500，适合 Windows 125% DPI。
    第三个 HSV 面板额外保留黑边，避免贴边轨迹在视觉上像被窗口裁切。
    """
    source = annotate_source(result)
    warped = annotate_warped(result, fps=fps)
    # geometry_mask 在视频模式下是多帧累积结果，而 raw_mask 仅为当前帧。
    mask = (
        cv2.cvtColor(result.geometry_mask, cv2.COLOR_GRAY2BGR)
        if result.geometry_mask is not None
        else np.zeros_like(warped)
    )
    _put_lines(
        mask,
        [
            f"HSV轨迹掩膜：{result.measurement.trace_pixels} 个像素",
            "白色像素参与几何测量",
        ],
    )

    panels = []
    for index, image in enumerate((source, warped, mask)):
        height, width = image.shape[:2]
        # HSV 面板保留约 4% 黑边，另两个面板只保留最小装框边距。
        margin = max(6, panel_size // 25) if index == 2 else 2
        available = panel_size - 2 * margin
        scale = min(available / width, available / height)
        # 缩小时用 INTER_AREA 抑制锯齿，放大时用双线性插值。
        resized = cv2.resize(
            image,
            (max(1, int(width * scale)), max(1, int(height * scale))),
            interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR,
        )
        background = np.full((panel_size, panel_size, 3), 20, dtype=np.uint8)
        y0 = (panel_size - resized.shape[0]) // 2
        x0 = (panel_size - resized.shape[1]) // 2
        background[y0 : y0 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
        panels.append(background)
    return np.hstack(panels)
