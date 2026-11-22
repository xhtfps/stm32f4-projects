"""将二值轨迹点云转换为分度测量、形状分类和控制评分。

核心思路是同时使用可解释的解析残差（直线、椭圆、2:1 无穷曲线）和模板
Chamfer 距离。解析残差决定主要类别，模板距离只用于打破接近的候选，避免
示波器稀疏点列或局部断线使黑盒式模板匹配失控。
"""

from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

from .config import VisionConfig
from .models import Measurement
from .templates import (
    SHAPES,
    generate_fitted_ellipse_template,
    generate_template,
    symmetric_chamfer_distance,
)


def _safe_iou(a: np.ndarray, b: np.ndarray) -> float:
    """计算两个二值掩膜的交并比；两者同时为空时返回 0。"""
    union = np.count_nonzero((a > 0) | (b > 0))
    if union == 0:
        return 0.0
    return float(np.count_nonzero((a > 0) & (b > 0)) / union)


def _softmax_negative(errors: np.ndarray, temperature: float = 8.0) -> np.ndarray:
    """把“越小越好”的多个误差转换成总和为 1 的类别概率。

    先减去最小误差可以避免指数下溢；非有限误差会被替换成明显更差的有限值。
    ``temperature`` 越大，最优类别的概率越集中。
    """
    finite = np.isfinite(errors)
    if not finite.any():
        return np.full(len(errors), 1.0 / len(errors), dtype=np.float64)
    safe = errors.copy()
    safe[~finite] = np.max(safe[finite]) + 10.0
    logits = -temperature * (safe - np.min(safe))
    probabilities = np.exp(np.clip(logits, -60.0, 0.0))
    return probabilities / max(float(probabilities.sum()), 1e-12)


def _fit_ellipse(
    xs: np.ndarray,
    ys: np.ndarray,
    pixels_per_division: float,
) -> tuple[
    float,
    float,
    float,
    float,
    float,
    tuple[float, float],
    tuple[float, float],
]:
    """拟合任意旋转椭圆，并同时返回通信单位和模板绘制所需参数。

    OpenCV 返回的是像素中心、两个完整轴长及旋转角。本函数额外把轴长和拟合
    中心相对点云中位中心的偏差换算成 div。失败时统一返回零值，调用者可安全
    地回退到轴对齐椭圆模型。
    """
    if len(xs) < 5:
        return (0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0))
    # fitEllipse 要求形状为 N×1×2、类型为 float32，且至少有五个点。
    points = np.column_stack((xs, ys)).astype(np.float32).reshape(-1, 1, 2)
    try:
        (cx, cy), (axis_a, axis_b), angle = cv2.fitEllipse(points)
    except cv2.error:
        return (0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0))
    return (
        float(axis_a / pixels_per_division),
        float(axis_b / pixels_per_division),
        float(angle),
        float((cx - np.median(xs)) / pixels_per_division),
        float((np.median(ys) - cy) / pixels_per_division),
        (float(cx), float(cy)),
        (float(axis_a), float(axis_b)),
    )


def _symmetry_scores(
    mask: np.ndarray,
    center_px: tuple[float, float],
    dilation_radius: int,
) -> tuple[float, float]:
    """计算轨迹关于自身中心的左右、上下镜像相似度。

    先把测得中心平移到画布中心，避免中心偏差被误认为形状不对称；随后膨胀少量
    像素容忍相机抖动，最后分别与水平/垂直翻转图计算 IoU。
    """
    height, width = mask.shape
    cx, cy = center_px
    translation = np.float32(
        [[1.0, 0.0, (width - 1) * 0.5 - cx], [0.0, 1.0, (height - 1) * 0.5 - cy]]
    )
    centered = cv2.warpAffine(mask, translation, (width, height), flags=cv2.INTER_NEAREST)
    if dilation_radius > 0:
        kernel_size = 2 * dilation_radius + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        centered = cv2.dilate(centered, kernel)
    return _safe_iou(centered, cv2.flip(centered, 1)), _safe_iou(centered, cv2.flip(centered, 0))


def _target_errors(
    target: str | None,
    shape: str,
    span_x_div: float,
    span_y_div: float,
    shape_error: float,
) -> tuple[float | None, float | None, float | None]:
    """解析 ``1x-8div`` 等目标字符串并计算跨度、形状误差。

    1:1 目标允许椭圆和其退化形式对角直线；类别不匹配时额外加 1，使控制器
    能明确区分“几何误差较大”和“频率族完全错误”。
    """
    if not target:
        return None, None, None
    # 同时接受英文、中文和常见比值写法，便于命令行现场输入。
    normalized = target.strip().lower().replace("×", "x")
    target_shape: str | None = None
    if any(token in normalized for token in ("1x", "1:1", "circle", "圆", "line", "直线")):
        target_shape = "ellipse"
    elif any(token in normalized for token in ("2x", "2:1", "infinity", "∞", "8字")):
        target_shape = "infinity"

    span_target = None
    for candidate in (2.0, 4.0, 6.0, 8.0):
        if str(int(candidate)) in normalized:
            span_target = candidate
            break

    target_error_x = abs(span_x_div - span_target) if span_target is not None else None
    target_error_y = abs(span_y_div - span_target) if span_target is not None else None
    if target_shape is None:
        target_shape_error = shape_error
    elif target_shape == "ellipse" and shape in ("ellipse", "diagonal"):
        target_shape_error = shape_error
    elif target_shape == shape:
        target_shape_error = shape_error
    else:
        target_shape_error = 1.0 + shape_error
    return target_error_x, target_error_y, target_shape_error


def measure_geometry(
    mask: np.ndarray,
    config: VisionConfig,
    *,
    sequence: int = 0,
    timestamp_ms: float = 0.0,
    target: str | None = None,
    debug: dict[str, Any] | None = None,
) -> Measurement:
    """测量并分类一张已经透视校正的黄色轨迹二值掩膜。

    处理流程：

    1. 提取非零像素，以稳健分位数估计跨度和中心；
    2. 将点云归一化到约 -1～1，计算三种解析模型误差；
    3. PCA 测量直线宽度，OpenCV 拟合任意旋转椭圆；
    4. 计算中心交叉、对称性和三类模板距离；
    5. 综合误差得到类别置信度及 1:1、2:1 控制评分。
    """
    # np.nonzero 返回顺序是 y、x，与常见笛卡尔坐标顺序相反。
    ys, xs = np.nonzero(mask)
    minimum_points = int(config.measurement.min_trace_pixels)
    if len(xs) < minimum_points:
        invalid = Measurement.invalid(
            f"仅检测到 {len(xs)} 个轨迹像素，至少需要 {minimum_points} 个",
            seq=sequence,
            timestamp_ms=int(timestamp_ms),
        )
        invalid.debug = debug or {}
        return invalid

    # 将标准画布像素统一转换为示波器大格，默认 800/8=100 px/div。
    height, width = mask.shape
    grid = config.grid
    ppd_x = width / float(grid.divisions_x)
    ppd_y = height / float(grid.divisions_y)
    ppd = 0.5 * (ppd_x + ppd_y)
    low = float(config.measurement.low_percentile)
    high = float(config.measurement.high_percentile)

    # 使用 0.35%～99.65% 分位数抑制孤立颜色噪点和边缘伪影。
    x_min, x_max = np.percentile(xs, [low, high])
    y_min, y_max = np.percentile(ys, [low, high])
    center_x_px = 0.5 * float(x_min + x_max)
    center_y_px = 0.5 * float(y_min + y_max)
    half_x_px = max(0.5 * float(x_max - x_min), 1.0)
    half_y_px = max(0.5 * float(y_max - y_min), 1.0)

    # 中心以画布几何中心为零；输出坐标中 Y 向上为正。
    span_x_div = float((x_max - x_min) / ppd_x)
    span_y_div = float((y_max - y_min) / ppd_y)
    center_x_div = float((center_x_px - (width - 1) * 0.5) / ppd_x)
    center_y_div = float(((height - 1) * 0.5 - center_y_px) / ppd_y)

    # 用各自半跨度归一化 X/Y，从而使分类基本不受波形显示大小影响。
    xn = (xs.astype(np.float64) - center_x_px) / half_x_px
    yn = (center_y_px - ys.astype(np.float64)) / half_y_px
    correlation = float(np.corrcoef(xn, yn)[0, 1]) if np.std(xn) > 0 and np.std(yn) > 0 else 0.0
    diagonal_sign = 1 if correlation >= 0.0 else -1

    # 对角线模型为 y=±x，符号由相关系数决定；除以 sqrt(2) 得垂直距离。
    line_residual_norm = float(
        np.median(np.abs(yn - diagonal_sign * xn)) / math.sqrt(2.0)
    )
    # 轴对齐椭圆满足 x²+y²=1；该量保留用于调试，最终还会使用旋转拟合。
    ellipse_radius = np.sqrt(xn * xn + yn * yn)
    axis_aligned_ellipse_residual = float(np.median(np.abs(ellipse_radius - 1.0)))
    # 由 x=sin(t)、y=sin(2t) 消去 t，可得
    # |y|=2|x|sqrt(1-x²)，即标准 2:1 无穷曲线。
    infinity_model = 2.0 * np.abs(xn) * np.sqrt(
        np.maximum(0.0, 1.0 - np.minimum(xn * xn, 1.0))
    )
    infinity_residual_norm = float(np.median(np.abs(np.abs(yn) - infinity_model)))

    # X 中轴截距：圆/椭圆在 x≈0 处通常接近上下端点；直线和 ∞ 接近中心。
    center_band = float(config.measurement.axis_band_norm)
    vertical_samples = np.abs(yn[np.abs(xn) < center_band])
    vertical_intercept = (
        float(np.median(vertical_samples)) if len(vertical_samples) >= 5 else 0.0
    )
    # 对 1:1 李萨如，归一化中轴截距约为 |sin(phi)|，只能得到绝对相位。
    phase_deg = float(
        math.degrees(math.asin(np.clip(vertical_intercept, 0.0, 1.0)))
    )
    center_density = float(
        np.mean((np.abs(xn) < 2.0 * center_band) & (np.abs(yn) < 2.0 * center_band))
    )

    # PCA 在物理分度坐标中执行，最小特征向量即最佳拟合直线的法向。
    points_div = np.column_stack(
        (
            (xs.astype(np.float64) - center_x_px) / ppd_x,
            (center_y_px - ys.astype(np.float64)) / ppd_y,
        )
    )
    covariance = np.cov(points_div, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    eigenvalues = np.maximum(eigenvalues, 0.0)
    minor_axis = eigenvectors[:, 0]
    line_residual_div = float(np.median(np.abs(points_div @ minor_axis)))
    pca_minor_ratio = float(eigenvalues[0] / max(eigenvalues[1], 1e-12))

    # 填充率可用于发现轨迹过粗或把大块 UI 错分为黄色的情况。
    box_area = max(float((x_max - x_min) * (y_max - y_min)), 1.0)
    fill_ratio = float(len(xs) / box_area)
    (
        ellipse_axis_a,
        ellipse_axis_b,
        ellipse_angle,
        ellipse_center_x_div,
        ellipse_center_y_div,
        fitted_ellipse_center_px,
        fitted_ellipse_axes_px,
    ) = _fit_ellipse(
        xs, ys, ppd
    )
    # 把点云旋转到拟合椭圆自身坐标系。理想椭圆满足
    # (u/a)²+(v/b)²=1，这使细长倾斜椭圆不再被误判成对角直线。
    if min(fitted_ellipse_axes_px) >= 1.0:
        dx = xs.astype(np.float64) - fitted_ellipse_center_px[0]
        dy = ys.astype(np.float64) - fitted_ellipse_center_px[1]
        angle_rad = math.radians(ellipse_angle)
        cosine = math.cos(angle_rad)
        sine = math.sin(angle_rad)
        ellipse_u = cosine * dx + sine * dy
        ellipse_v = -sine * dx + cosine * dy
        ellipse_fit_radius = np.sqrt(
            (ellipse_u / max(0.5 * fitted_ellipse_axes_px[0], 1e-6)) ** 2
            + (ellipse_v / max(0.5 * fitted_ellipse_axes_px[1], 1e-6)) ** 2
        )
        ellipse_residual_norm = float(
            np.median(np.abs(ellipse_fit_radius - 1.0))
        )
    else:
        ellipse_residual_norm = axis_aligned_ellipse_residual
    symmetry_lr, symmetry_ud = _symmetry_scores(
        mask,
        (center_x_px, center_y_px),
        int(config.measurement.symmetry_dilate_px),
    )

    # 生成与实测中心、跨度一致的三类模板。椭圆使用拟合角度，另外两类使用
    # 解析标准形状；距离统一换算成 div。
    template_distances: dict[str, float] = {}
    for candidate in SHAPES:
        if candidate == "ellipse" and min(fitted_ellipse_axes_px) >= 1.0:
            template = generate_fitted_ellipse_template(
                mask.shape,
                fitted_ellipse_center_px,
                fitted_ellipse_axes_px,
                ellipse_angle,
                thickness=2,
            )
        else:
            template = generate_template(
                candidate,
                mask.shape,
                (center_x_px, center_y_px),
                (half_x_px, half_y_px),
                diagonal_sign=diagonal_sign,
                thickness=2,
            )
        template_distances[candidate] = symmetric_chamfer_distance(mask, template, ppd)

    # 理想直线既应具有低垂直残差，也应具有很小的 PCA 次轴能量。后一个惩罚项
    # 专门防止“细长但仍闭合的倾斜椭圆”被误标为直线。
    line_width_penalty = 0.35 * float(
        np.clip((pca_minor_ratio - 0.02) / 0.10, 0.0, 1.0)
    )
    line_adjusted = line_residual_norm + line_width_penalty
    # 椭圆若在中轴附近完全没有上下截距，更可能是直线或 ∞，因此加软惩罚。
    ellipse_adjusted = ellipse_residual_norm + 0.50 * max(
        0.0, 0.55 - vertical_intercept
    )
    infinity_crossing_penalty = 0.16 * float(
        np.clip((0.035 - center_density) / 0.035, 0.0, 1.0)
    )
    # ∞ 应具有中心交叉；中心密度不足或中轴截距过高时提高其误差。
    infinity_adjusted = (
        infinity_residual_norm
        + 0.50 * max(0.0, vertical_intercept - 0.35)
        + infinity_crossing_penalty
    )
    analytic_errors = np.array(
        [line_adjusted, ellipse_adjusted, infinity_adjusted], dtype=np.float64
    )

    # 模板距离权重仅为 0.08 且上限 1.5，只用于解析误差接近时打破平局；
    # 这样局部黄点缺失不会压倒可解释的几何公式。
    chamfer_errors = np.array([template_distances[name] for name in SHAPES])
    finite_chamfer = np.where(np.isfinite(chamfer_errors), chamfer_errors, 10.0)
    combined_errors = analytic_errors + 0.08 * np.minimum(finite_chamfer, 1.5)
    probabilities = _softmax_negative(combined_errors)
    winning_index = int(np.argmax(probabilities))
    shape = SHAPES[winning_index]
    confidence = float(probabilities[winning_index])
    shape_error = float(combined_errors[winning_index])

    # 1:1 族同时包含直线和任意旋转椭圆，取两者较小误差；中心交叉密度过大
    # 更像 2:1 图形，因此给椭圆族增加轻微惩罚。
    ellipse_family_error = ellipse_residual_norm + 2.0 * max(
        0.0, center_density - 0.025
    )
    score_1x = float(
        np.clip(
            math.exp(-5.0 * min(line_adjusted, ellipse_family_error)),
            0.0,
            1.0,
        )
    )
    score_2x = float(np.clip(math.exp(-5.0 * infinity_adjusted), 0.0, 1.0))
    target_error_x, target_error_y, target_shape_error = _target_errors(
        target, shape, span_x_div, span_y_div, shape_error
    )

    # Measurement 是电脑端 JSON 和未来 K230 串口协议的统一出口。
    return Measurement(
        valid=True,
        error="",
        seq=sequence,
        timestamp_ms=int(timestamp_ms),
        shape=shape,
        confidence=confidence,
        span_x_div=span_x_div,
        span_y_div=span_y_div,
        center_x_div=center_x_div,
        center_y_div=center_y_div,
        trace_pixels=int(len(xs)),
        fill_ratio=fill_ratio,
        corr_xy=correlation,
        phase_abs_deg=phase_deg,
        vertical_intercept_ratio=vertical_intercept,
        line_error_div=line_residual_div,
        line_error_norm=line_residual_norm,
        pca_minor_ratio=pca_minor_ratio,
        ellipse_error=ellipse_residual_norm,
        ellipse_center_x_div=ellipse_center_x_div,
        ellipse_center_y_div=ellipse_center_y_div,
        ellipse_axis_x_div=ellipse_axis_a,
        ellipse_axis_y_div=ellipse_axis_b,
        ellipse_angle_deg=ellipse_angle,
        infinity_error=infinity_residual_norm,
        center_density=center_density,
        symmetry_x=symmetry_lr,
        symmetry_y=symmetry_ud,
        chamfer_diagonal_div=template_distances["diagonal"],
        chamfer_ellipse_div=template_distances["ellipse"],
        chamfer_infinity_div=template_distances["infinity"],
        score_1x=score_1x,
        score_2x=score_2x,
        target=target,
        target_span_x_error_div=target_error_x,
        target_span_y_error_div=target_error_y,
        target_shape_error=target_shape_error,
        debug={
            **(debug or {}),
            "pixels_per_division_x": ppd_x,
            "pixels_per_division_y": ppd_y,
            "robust_bounds_px": [float(x_min), float(y_min), float(x_max), float(y_max)],
            "analytic_errors": {
                "diagonal": float(line_adjusted),
                "ellipse": float(ellipse_adjusted),
                "infinity": float(infinity_adjusted),
            },
            "line_width_penalty": line_width_penalty,
            "infinity_crossing_penalty": infinity_crossing_penalty,
            "axis_aligned_ellipse_error": axis_aligned_ellipse_residual,
            "ellipse_family_error": ellipse_family_error,
            "combined_errors": {
                name: float(combined_errors[index]) for index, name in enumerate(SHAPES)
            },
            "class_probabilities": {
                name: float(probabilities[index]) for index, name in enumerate(SHAPES)
            },
        },
    )
