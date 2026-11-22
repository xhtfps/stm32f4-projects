"""生成理想李萨如模板，并计算模板与实测掩膜的形状距离。"""

from __future__ import annotations

import math

import cv2
import numpy as np


SHAPES = ("diagonal", "ellipse", "infinity")


def generate_template(
    shape: str,
    image_shape: tuple[int, int],
    center_px: tuple[float, float],
    half_span_px: tuple[float, float],
    *,
    diagonal_sign: int = 1,
    thickness: int = 2,
) -> np.ndarray:
    """在图像坐标系中生成理想李萨如曲线模板。

    ``half_span_px`` 是 X/Y 半跨度；参数 ``t`` 取 4096 个采样点，以保证缩放到
    800×800 后曲线连续。数学坐标 Y 向上，而图像坐标 Y 向下，因此生成像素
    坐标时需要对 Y 取反。
    """
    height, width = image_shape
    cx, cy = center_px
    ax = max(float(half_span_px[0]), 1.0)
    ay = max(float(half_span_px[1]), 1.0)
    t = np.linspace(0.0, 2.0 * math.pi, 4096, endpoint=False)

    # 1:1 同相或反相退化为两种方向的对角直线。
    if shape == "diagonal":
        x = np.sin(t)
        y = float(1 if diagonal_sign >= 0 else -1) * np.sin(t)
    # 相差 90° 的 1:1 信号形成标准椭圆/圆。
    elif shape == "ellipse":
        x = np.sin(t)
        y = np.cos(t)
    elif shape == "infinity":
        # y=sin(2t) 形成具有两个叶瓣的 2:1 无穷图形。
        x = np.sin(t)
        y = np.sin(2.0 * t)
    else:
        raise ValueError(f"unsupported template shape: {shape}")

    px = np.rint(cx + ax * x).astype(np.int32)
    py = np.rint(cy - ay * y).astype(np.int32)
    points = np.column_stack((px, py))
    # 满屏波形可能恰好越过模板边界，只绘制仍在画布内的采样点。
    valid = (
        (points[:, 0] >= 0)
        & (points[:, 0] < width)
        & (points[:, 1] >= 0)
        & (points[:, 1] < height)
    )
    points = points[valid]

    canvas = np.zeros((height, width), dtype=np.uint8)
    if len(points) >= 2:
        cv2.polylines(canvas, [points.reshape(-1, 1, 2)], True, 255, thickness, cv2.LINE_AA)
    return canvas


def generate_fitted_ellipse_template(
    image_shape: tuple[int, int],
    center_px: tuple[float, float],
    axes_px: tuple[float, float],
    angle_deg: float,
    *,
    thickness: int = 2,
) -> np.ndarray:
    """按照 ``cv2.fitEllipse`` 返回的中心、整轴长和角度绘制旋转椭圆。"""
    height, width = image_shape
    cx, cy = center_px
    axis_a, axis_b = axes_px
    canvas = np.zeros((height, width), dtype=np.uint8)
    if min(axis_a, axis_b) < 1.0:
        return canvas
    cv2.ellipse(
        canvas,
        (int(round(cx)), int(round(cy))),
        (max(1, int(round(axis_a * 0.5))), max(1, int(round(axis_b * 0.5)))),
        float(angle_deg),
        0.0,
        360.0,
        255,
        thickness,
        cv2.LINE_AA,
    )
    return canvas


def symmetric_chamfer_distance(
    observed: np.ndarray,
    template: np.ndarray,
    pixels_per_division: float,
) -> float:
    """计算稳健的双向 Chamfer 距离，结果以示波器大格为单位。

    单向距离可能出现“小模板完全落在大轨迹内部仍得低误差”的问题，因此同时
    计算模板到观测、观测到模板的距离。使用第 85 百分位兼顾缺失曲段，又不会
    被少量阈值噪点完全支配。
    """
    observed_bool = observed > 0
    template_bool = template > 0
    if not np.any(observed_bool) or not np.any(template_bool):
        return float("inf")

    # distanceTransform 计算非零点到最近零点的距离，因此要把曲线像素编码为零。
    obs_distance = cv2.distanceTransform((~observed_bool).astype(np.uint8), cv2.DIST_L2, 3)
    tpl_distance = cv2.distanceTransform((~template_bool).astype(np.uint8), cv2.DIST_L2, 3)

    template_to_observed = obs_distance[template_bool]
    observed_to_template = tpl_distance[observed_bool]
    # 双向距离取平均，最后除以 px/div，消除画布分辨率的影响。
    distance_px = 0.5 * (
        float(np.percentile(template_to_observed, 85.0))
        + float(np.percentile(observed_to_template, 85.0))
    )
    return distance_px / max(float(pixels_per_division), 1e-6)
