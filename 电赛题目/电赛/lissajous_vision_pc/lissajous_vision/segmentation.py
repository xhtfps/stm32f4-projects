"""从透视校正图中提取黄色波形，并对视频帧掩膜进行累积。"""

from __future__ import annotations

import cv2
import numpy as np

from .config import VisionConfig


def _suppress_compact_artifacts(
    mask: np.ndarray, artifact_masks_norm: list[list[float]]
) -> None:
    """Remove only compact marker blobs, preserving trace fragments.

    The old implementation cleared whole rectangles before reconstruction.
    That also removed a real waveform whenever it crossed the marker area and
    made otherwise closed cells permanently open.  Marker LEDs are compact;
    waveform fragments are elongated or connected to the main trace, so they
    can be filtered safely after continuity repair.
    """
    if not artifact_masks_norm:
        return
    height, width = mask.shape
    largest_area = 0
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count > 1:
        largest_area = int(np.max(stats[1:, cv2.CC_STAT_AREA]))
    if largest_area <= 0:
        return
    compact_limit = max(80, int(0.02 * largest_area))
    max_side = max(12, int(0.04 * min(height, width)))
    for x0, y0, x1, y1 in artifact_masks_norm:
        left = int(np.clip(x0, 0, 1) * width)
        top = int(np.clip(y0, 0, 1) * height)
        right = int(np.clip(x1, 0, 1) * width)
        bottom = int(np.clip(y1, 0, 1) * height)
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            component_left = int(stats[component, cv2.CC_STAT_LEFT])
            component_top = int(stats[component, cv2.CC_STAT_TOP])
            component_width = int(stats[component, cv2.CC_STAT_WIDTH])
            component_height = int(stats[component, cv2.CC_STAT_HEIGHT])
            component_right = component_left + component_width
            component_bottom = component_top + component_height
            inside_hint = (
                component_left >= left
                and component_top >= top
                and component_right <= right
                and component_bottom <= bottom
            )
            compact = (
                area <= compact_limit
                and component_width <= max_side
                and component_height <= max_side
            )
            if inside_hint and compact:
                mask[labels == component] = 0


def segment_yellow(
    warped_bgr: np.ndarray, config: VisionConfig
) -> tuple[np.ndarray, dict[str, int | float]]:
    """
    使用“强黄色种子 + 弱黄色约束重建”提取轨迹。

    高光或白色栅格交叉处的黄色会降低饱和度，如果只用一个严格 HSV 阈值，
    轨迹会出现断口。这里先取得可靠的强黄色像素，再允许它们沿弱黄色候选区
    生长；没有连接到强种子的白色/青色栅格不会进入最终掩膜。

    返回值为清理后的 8 位二值掩膜，以及便于现场调参的像素统计字典。
    """
    settings = config.segmentation
    # 先做轻量图像预处理。中值滤波消除孤立亮点，高斯滤波抑制摄像头
    # 高频噪声；随后用局部背景除法抵消示波器屏幕上的不均匀照明。
    median_kernel = max(3, int(settings.median_kernel))
    if median_kernel % 2 == 0:
        median_kernel += 1
    filtered_bgr = cv2.medianBlur(warped_bgr, median_kernel)
    sigma = max(0.0, float(settings.gaussian_sigma))
    if sigma > 0.0:
        filtered_bgr = cv2.GaussianBlur(
            filtered_bgr, (0, 0), sigmaX=sigma, sigmaY=sigma
        )
    hsv = cv2.cvtColor(filtered_bgr, cv2.COLOR_BGR2HSV)
    illumination_sigma = max(3.0, float(settings.illumination_sigma))
    value_background = cv2.GaussianBlur(
        hsv[:, :, 2], (0, 0), sigmaX=illumination_sigma,
        sigmaY=illumination_sigma,
    )
    hsv[:, :, 2] = cv2.divide(
        hsv[:, :, 2], np.maximum(value_background, 1), scale=180
    )

    # 第一层判定：严格 HSV 范围，得到高可信度种子。
    strong_hsv = cv2.inRange(
        hsv,
        np.array((settings.h_min, settings.s_min, settings.v_min), dtype=np.uint8),
        np.array((settings.h_max, 255, 255), dtype=np.uint8),
    )

    # 第二层判定：黄色的 R、G 分量应同时明显强于 B。使用 int16 防止
    # uint8 加减发生 0～255 回绕。
    blue, green, red = cv2.split(filtered_bgr)
    dominance = (
        red.astype(np.int16)
        + green.astype(np.int16)
        - 2 * blue.astype(np.int16)
    )
    strong_dominance = np.where(
        dominance >= settings.yellow_dominance_min, 255, 0
    ).astype(np.uint8)
    strong = cv2.bitwise_and(strong_hsv, strong_dominance)

    # 弱候选区放宽色相、饱和度和黄色优势，但仍必须同时通过两种颜色条件。
    weak_h_min = max(0, settings.h_min - settings.weak_h_padding)
    weak_h_max = min(179, settings.h_max + settings.weak_h_padding)
    weak_hsv = cv2.inRange(
        hsv,
        np.array((weak_h_min, settings.weak_s_min, settings.weak_v_min), dtype=np.uint8),
        np.array((weak_h_max, 255, 255), dtype=np.uint8),
    )
    weak_dominance = np.where(
        dominance >= settings.weak_yellow_dominance_min, 255, 0
    ).astype(np.uint8)
    weak = cv2.bitwise_and(weak_hsv, weak_dominance)
    weak = cv2.bitwise_or(weak, strong)

    # 开运算 = 腐蚀后膨胀，去除孤立噪声但保留主轨迹；再用小闭运算
    # = 膨胀后腐蚀，连接短曝光断口。
    open_size = max(1, int(settings.morphology_open_kernel))
    if open_size % 2 == 0:
        open_size += 1
    if open_size > 1:
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_size, open_size)
        )
        strong = cv2.morphologyEx(strong, cv2.MORPH_OPEN, open_kernel)
        weak = cv2.morphologyEx(weak, cv2.MORPH_OPEN, open_kernel)

    # 闭运算只连接很短的曝光断口；强制使用奇数核确保锚点位于中心。
    close_size = max(1, int(settings.weak_close_kernel))
    if close_size % 2 == 0:
        close_size += 1
    if close_size > 1:
        weak = cv2.morphologyEx(
            weak,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size)),
        )

    # 形态学重建：每轮只向相邻 1 像素扩张，并立即与 weak 相交。
    # 若一轮没有新增像素则提前结束，减少固定摄像头模式下的计算量。
    combined = strong.copy()
    growth_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    reconstruction_steps = 0
    for reconstruction_steps in range(max(0, settings.reconstruction_iterations)):
        grown = cv2.bitwise_and(cv2.dilate(combined, growth_kernel), weak)
        grown = cv2.bitwise_or(grown, strong)
        if cv2.countNonZero(cv2.bitwise_xor(grown, combined)) == 0:
            break
        combined = grown

    # 重建结束后再补齐短断口。这个核只跨越曝光丢失的小间隙，不会把
    # 相邻波瓣整段粘在一起；真正的遮挡标记留到连通域阶段再单独剔除。
    continuity_kernel = max(7, close_size + 4)
    if continuity_kernel % 2 == 0:
        continuity_kernel += 1
    combined = cv2.morphologyEx(
        combined,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (continuity_kernel, continuity_kernel)
        ),
    )
    _suppress_compact_artifacts(combined, settings.artifact_masks_norm)

    # 连通域清理不能简单地“只保留最大轮廓”，因为低刷新率示波器轨迹可能
    # 天生是多个不相连的点列，所以仅删除极小噪点和明确的边缘细条。
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        combined, connectivity=8
    )
    cleaned = np.zeros_like(combined)
    kept_components = 0
    rejected_pixels = 0
    border_artifact_pixels = 0
    largest_component_area = (
        int(np.max(stats[1:, cv2.CC_STAT_AREA])) if component_count > 1 else 0
    )
    for component in range(1, component_count):
        area = int(stats[component, cv2.CC_STAT_AREA])
        left = int(stats[component, cv2.CC_STAT_LEFT])
        top = int(stats[component, cv2.CC_STAT_TOP])
        component_width = int(stats[component, cv2.CC_STAT_WIDTH])
        component_height = int(stats[component, cv2.CC_STAT_HEIGHT])
        touches_border = (
            left <= 0
            or top <= 0
            or left + component_width >= combined.shape[1]
            or top + component_height >= combined.shape[0]
        )
        # 四角附近的黄色刻度经透视插值后可能变成宽 1～2 像素的边缘细条。
        # 它们面积虽超过普通噪点，却会严重拉坏分位数跨度，需单独排除。
        is_thin_border_artifact = (
            touches_border
            and min(component_width, component_height) <= 3
            and area < max(20, int(0.05 * largest_component_area))
        )
        if is_thin_border_artifact:
            border_artifact_pixels += area
            continue
        if area >= settings.min_component_area:
            cleaned[labels == component] = 255
            kept_components += 1
        else:
            rejected_pixels += area

    # 这些统计量只用于调试输出，不参与通信协议的核心判定。
    return cleaned, {
        "strong_yellow_pixels": int(cv2.countNonZero(strong)),
        "weak_yellow_pixels": int(cv2.countNonZero(weak)),
        "raw_yellow_pixels": int(cv2.countNonZero(combined)),
        "trace_pixels": int(cv2.countNonZero(cleaned)),
        "kept_components": kept_components,
        "rejected_pixels": rejected_pixels,
        "border_artifact_pixels": border_artifact_pixels,
        "reconstruction_steps": int(reconstruction_steps + 1),
    }


def accumulate_masks(masks: list[np.ndarray]) -> np.ndarray:
    """将连续若干帧二值掩膜按位或，补全示波器扫描形成的稀疏点列。"""
    if not masks:
        raise ValueError("masks must not be empty")
    accumulated = np.zeros_like(masks[0])
    for mask in masks:
        accumulated = cv2.bitwise_or(accumulated, mask)
    return accumulated
