"""示波器 8×8 栅格的自动检测、手动标定和透视校正。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .config import VisionConfig, save_config
from .text_render import draw_text_lines_bgr


@dataclass(slots=True)
class GridDetection:
    """一次自动找屏的结果和诊断信息。"""

    # 原图像素坐标下的 TL/TR/BR/BL 四角；失败时为 None。
    corners: np.ndarray | None
    # 置信度主要由候选四边形面积和 9×9 栅格线计数共同构成。
    confidence: float = 0.0
    reason: str | None = None
    candidates: int = 0
    debug: dict[str, float | int | list[float]] = field(default_factory=dict)


def order_quad(points: np.ndarray) -> np.ndarray:
    """把任意顺序的四点整理为左上、右上、右下、左下。

    ``x+y`` 的最小/最大值对应左上/右下，``x-y`` 的最大/最小值对应
    右上/左下。后续透视变换和配置文件都依赖这一固定顺序。
    """
    quad = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = quad.sum(axis=1)
    differences = quad[:, 0] - quad[:, 1]
    return np.array(
        [
            quad[np.argmin(sums)],
            quad[np.argmax(differences)],
            quad[np.argmax(sums)],
            quad[np.argmin(differences)],
        ],
        dtype=np.float32,
    )


def normalized_to_pixels(
    corners_norm: list[list[float]], image_shape: tuple[int, ...]
) -> np.ndarray:
    """将 0～1 归一化角点换算成当前帧的像素坐标。"""
    height, width = image_shape[:2]
    scale = np.array([width - 1, height - 1], dtype=np.float32)
    return order_quad(np.asarray(corners_norm, dtype=np.float32) * scale)


def pixels_to_normalized(
    corners: np.ndarray, image_shape: tuple[int, ...]
) -> list[list[float]]:
    """将像素角点换算成与图像分辨率无关的 0～1 坐标。"""
    height, width = image_shape[:2]
    scale = np.array([max(width - 1, 1), max(height - 1, 1)], dtype=np.float32)
    return (order_quad(corners) / scale).astype(float).tolist()


def configured_corners(
    config: VisionConfig, image_shape: tuple[int, ...]
) -> np.ndarray | None:
    """从配置中取得适用于当前图像尺寸的角点；未标定时返回 None。"""
    if config.grid.corners_norm is None:
        return None
    return normalized_to_pixels(config.grid.corners_norm, image_shape)


def perspective_matrix(corners: np.ndarray, config: VisionConfig) -> np.ndarray:
    """计算原图四边形到标准矩形画布的 3×3 单应矩阵。"""
    # 目标点落在像素索引的 0～width-1 / 0～height-1 范围内。
    destination = np.array(
        [
            [0, 0],
            [config.grid.warp_width - 1, 0],
            [config.grid.warp_width - 1, config.grid.warp_height - 1],
            [0, config.grid.warp_height - 1],
        ],
        dtype=np.float32,
    )
    return cv2.getPerspectiveTransform(order_quad(corners), destination)


def warp_grid(
    image: np.ndarray, corners: np.ndarray, config: VisionConfig
) -> np.ndarray:
    """应用透视变换，把倾斜拍摄的栅格校正为标准正视图。"""
    matrix = perspective_matrix(corners, config)
    return cv2.warpPerspective(
        image,
        matrix,
        (config.grid.warp_width, config.grid.warp_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )


def detect_grid_quad(image: np.ndarray) -> GridDetection:
    """
    尽力自动寻找 DS2302A 的 8×8 栅格四边形。

    自动找屏主要用于首次辅助标定和不同机位照片的批处理。正式比赛的固定
    摄像头应保存手动角点，因为菜单、高光和贴边波形都可能降低自动检测精度。
    """
    if image is None or image.size == 0:
        return GridDetection(None, reason="EMPTY_IMAGE")

    # 限定中央搜索区，尽量排除示波器外壳、按键和左右菜单。
    image_height, image_width = image.shape[:2]
    x0, x1 = int(0.18 * image_width), int(0.82 * image_width)
    y0, y1 = int(0.12 * image_height), int(0.92 * image_height)
    roi = image[y0:y1, x0:x1]
    if roi.size == 0:
        return GridDetection(None, reason="EMPTY_SEARCH_ROI")

    # 大图先缩至最长边不超过 1024，减少轮廓检测时间；最终角点再映回原图。
    scale = min(1.0, 1024.0 / max(roi.shape[:2]))
    resized = cv2.resize(
        roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
    )
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    # 第一方案寻找低饱和高亮的白色外框。主检测故意不使用青色，因为青色
    # 栅格可能与右侧菜单边框连通，形成比波形区更大的错误轮廓。
    bright_low_saturation = cv2.inRange(
        hsv,
        np.array((0, 0, 130), dtype=np.uint8),
        np.array((179, 105, 255), dtype=np.uint8),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    connected = cv2.morphologyEx(
        bright_low_saturation, cv2.MORPH_CLOSE, kernel, iterations=2
    )

    # 只取最外层轮廓，减少内部 8×8 网格线产生的候选数量。
    contours, _ = cv2.findContours(
        connected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    image_area = float(resized.shape[0] * resized.shape[1])
    candidates: list[tuple[float, np.ndarray, dict[str, float | int]]] = []

    for contour in contours:
        area = float(cv2.contourArea(contour))
        x, y, width, height = cv2.boundingRect(contour)
        # 面积和包围盒过小的轮廓不可能是主栅格。
        if area < 0.06 * image_area:
            continue
        if width < 0.35 * resized.shape[1] or height < 0.35 * resized.shape[0]:
            continue

        # 逐渐放宽多边形近似容差，寻找凸四边形。
        perimeter = cv2.arcLength(contour, True)
        quad = None
        for epsilon_ratio in (0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05):
            approximation = cv2.approxPolyDP(
                contour, epsilon_ratio * perimeter, True
            )
            if len(approximation) == 4 and cv2.isContourConvex(approximation):
                quad = approximation.reshape(4, 2).astype(np.float32)
                break
        if quad is None:
            continue

        quad_full = quad / scale + np.array((x0, y0), dtype=np.float32)
        quad_full = order_quad(quad_full)
        # 候选不仅要面积大，还应在透视校正后呈现接近 9 条横线和 9 条竖线。
        validation = _validate_grid_candidate(image, quad_full)
        coverage = area / image_area
        score = coverage + 0.10 * validation["line_score"]
        candidates.append((score, quad_full, validation))

    # 白框被反光打断时，退回到青色网格连通域方案。
    if not candidates:
        fallback = _detect_cyan_grid_network(image, (x0, y0, x1, y1))
        if fallback is not None:
            return fallback
        return GridDetection(None, reason="GRID_NOT_FOUND", candidates=0)

    score, corners, validation = max(candidates, key=lambda item: item[0])
    confidence = float(
        np.clip(0.55 * validation["line_score"] + 0.45 * min(score / 0.35, 1.0), 0, 1)
    )
    return GridDetection(
        corners=corners,
        confidence=confidence,
        candidates=len(candidates),
        debug={
            "search_scale": float(scale),
            "candidate_score": float(score),
            **validation,
        },
    )


def _detect_cyan_grid_network(
    image: np.ndarray,
    search_bounds: tuple[int, int, int, int],
) -> GridDetection | None:
    """使用青色网格连通域找屏，是白色外框方案失败后的回退路径。"""
    x0, y0, x1, y1 = search_bounds
    roi = image[y0:y1, x0:x1]
    scale = min(1.0, 1024.0 / max(roi.shape[:2]))
    resized = cv2.resize(roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
    # 较宽的青色范围覆盖不同曝光下的栅格线，再用闭运算和膨胀连接虚线。
    cyan = cv2.inRange(
        hsv,
        np.array((70, 28, 55), dtype=np.uint8),
        np.array((118, 255, 255), dtype=np.uint8),
    )
    cyan = cv2.morphologyEx(
        cyan,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        iterations=1,
    )
    cyan = cv2.dilate(
        cyan, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1
    )
    contours, _ = cv2.findContours(cyan, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_area = float(resized.shape[0] * resized.shape[1])
    fallback_candidates: list[tuple[float, np.ndarray, dict[str, float | int]]] = []
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        x, y, width, height = cv2.boundingRect(contour)
        # 栅格应占搜索区域较大比例，且透视后的长宽比不会极端失衡。
        if contour_area < 0.04 * roi_area:
            continue
        if width < 0.30 * resized.shape[1] or height < 0.30 * resized.shape[0]:
            continue
        aspect = width / max(float(height), 1.0)
        if not 0.65 <= aspect <= 1.55:
            continue

        # 先取凸包消除网格内部凹口，再尝试四边形近似。
        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        quad = None
        for epsilon_ratio in (0.01, 0.015, 0.02, 0.03, 0.04, 0.06, 0.08):
            approximation = cv2.approxPolyDP(hull, epsilon_ratio * perimeter, True)
            if len(approximation) == 4 and cv2.isContourConvex(approximation):
                quad = approximation.reshape(4, 2).astype(np.float32)
                break
        # 极端反光下轮廓无法化成四点时，以最小外接旋转矩形兜底。
        if quad is None:
            quad = cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float32)
        full = order_quad(quad / scale + np.array((x0, y0), dtype=np.float32))
        validation = _validate_grid_candidate(image, full)
        score = contour_area / roi_area + 0.12 * float(validation["line_score"])
        fallback_candidates.append((score, full, validation))

    if not fallback_candidates:
        return None
    score, corners, validation = max(fallback_candidates, key=lambda item: item[0])
    confidence = float(
        np.clip(0.45 + 0.45 * float(validation["line_score"]), 0.0, 0.9)
    )
    return GridDetection(
        corners=corners,
        confidence=confidence,
        candidates=len(fallback_candidates),
        debug={
            "fallback": 1,
            "search_scale": float(scale),
            "candidate_score": float(score),
            **validation,
        },
    )


def _validate_grid_candidate(
    image: np.ndarray, corners: np.ndarray
) -> dict[str, float | int]:
    """把候选四边形缩到 400×400，并按横/纵投影验证网格线数量。"""
    target_size = 400
    destination = np.array(
        [[0, 0], [399, 0], [399, 399], [0, 399]], dtype=np.float32
    )
    warped = cv2.warpPerspective(
        image,
        cv2.getPerspectiveTransform(order_quad(corners), destination),
        (target_size, target_size),
    )
    hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(
        hsv,
        np.array((0, 0, 100), dtype=np.uint8),
        np.array((179, 125, 255), dtype=np.uint8),
    )
    cyan = cv2.inRange(
        hsv,
        np.array((70, 30, 65), dtype=np.uint8),
        np.array((115, 255, 255), dtype=np.uint8),
    )
    # 外框可能是白色、内部网格可能是青色，验证阶段可安全地合并两者。
    grid_mask = cv2.bitwise_or(white, cyan)
    vertical_count = _count_projection_peaks(grid_mask.sum(axis=0), target_size)
    horizontal_count = _count_projection_peaks(grid_mask.sum(axis=1), target_size)
    vertical_score = max(0.0, 1.0 - abs(vertical_count - 9) / 9.0)
    horizontal_score = max(0.0, 1.0 - abs(horizontal_count - 9) / 9.0)
    return {
        "vertical_grid_lines": int(vertical_count),
        "horizontal_grid_lines": int(horizontal_count),
        "line_score": float((vertical_score + horizontal_score) / 2.0),
    }


def _count_projection_peaks(projection: np.ndarray, size: int) -> int:
    """在一维投影中统计相互间隔足够大的显著峰值数量。"""
    values = np.asarray(projection, dtype=np.float32)
    if values.size < 3 or float(values.max()) <= 0:
        return 0
    window = max(3, size // 80)
    # 滑动平均抑制点状刻度；阈值同时参考 75 分位和峰值的 35%。
    smooth = np.convolve(values, np.ones(window) / window, mode="same")
    threshold = max(float(np.percentile(smooth, 75)), float(smooth.max()) * 0.35)
    local_maxima = [
        index
        for index in range(1, len(smooth) - 1)
        if smooth[index] >= smooth[index - 1]
        and smooth[index] > smooth[index + 1]
        and smooth[index] >= threshold
    ]
    minimum_distance = max(10, size // 14)
    selected: list[int] = []
    # 先选择最高峰，并通过最小间距抑制同一条粗网格线的重复峰。
    for index in sorted(local_maxima, key=lambda item: smooth[item], reverse=True):
        if all(abs(index - existing) >= minimum_distance for existing in selected):
            selected.append(index)
    return len(selected)


def interactive_calibration(
    image: np.ndarray,
    config: VisionConfig,
    output_path: str | Path,
    *,
    window_name: str = "示波器栅格标定",
) -> np.ndarray | None:
    """交互采集四个栅格角点，并把归一化结果保存到配置文件。

    用户依次点击左上、右上、右下、左下；Enter 保存，R 清空重选，Esc 取消。
    为适配高分辨率照片，显示图会缩小，保存前再除以显示比例恢复原图坐标。
    """
    image_height, image_width = image.shape[:2]
    # 限制标定窗口尺寸，避免 4K 手机画面超出屏幕。
    display_scale = min(1.0, 1400.0 / image_width, 900.0 / image_height)
    display = cv2.resize(
        image,
        None,
        fx=display_scale,
        fy=display_scale,
        interpolation=cv2.INTER_AREA,
    )
    points: list[tuple[int, int]] = []

    def callback(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        """鼠标左键每次追加一个点，达到四点后停止接收。"""
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((x, y))

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, callback)

    # 循环刷新角点编号、连线和操作说明。
    while True:
        canvas = display.copy()
        for index, point in enumerate(points):
            cv2.circle(canvas, point, 7, (0, 255, 0), -1)
            cv2.putText(
                canvas,
                str(index + 1),
                (point[0] + 9, point[1] - 9),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        if len(points) > 1:
            cv2.polylines(
                canvas,
                [np.asarray(points, dtype=np.int32)],
                False,
                (0, 255, 0),
                2,
            )
        draw_text_lines_bgr(
            canvas,
            ["依次点击：左上、右上、右下、左下", "回车：保存　R：重选　Esc：取消"],
            origin=(15, 10),
            font_size=24,
            line_gap=4,
            color_bgr=(0, 220, 255),
        )
        cv2.imshow(window_name, canvas)
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("r"), ord("R")):
            points.clear()
        elif key in (13, 10) and len(points) == 4:
            break
        elif key == 27:
            cv2.destroyWindow(window_name)
            return None

    cv2.destroyWindow(window_name)
    # 恢复原图坐标后再次排序，防止轻微点选顺序错误破坏透视矩阵。
    corners = order_quad(np.asarray(points, dtype=np.float32) / display_scale)
    config.grid.corners_norm = pixels_to_normalized(corners, image.shape)
    save_config(config, output_path)
    return corners
