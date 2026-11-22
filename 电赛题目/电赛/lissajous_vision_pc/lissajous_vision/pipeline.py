"""把标定、透视、分割、几何和时序模块串成统一分析流水线。"""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from .calibration import configured_corners, detect_grid_quad, warp_grid
from .config import VisionConfig, load_config
from .geometry import measure_geometry
from .models import Measurement
from .segmentation import segment_yellow
from .temporal import TemporalTracker


@dataclass(slots=True)
class AnalysisResult:
    """一次分析的完整产物。

    ``measurement`` 用于控制和 JSON 输出；其余图像用于界面显示、保存调试图
    和现场排查，不需要发送给 STM32。
    """

    measurement: Measurement
    source: np.ndarray
    warped: np.ndarray | None
    raw_mask: np.ndarray | None
    geometry_mask: np.ndarray | None
    corners: np.ndarray | None


class LissajousAnalyzer:
    """单图或单路视频流的端到端分析器。

    一个视频源应长期复用同一个实例，以保留自动角点缓存和时序历史；批处理
    不同机位照片时则应为每张图创建实例，防止复用错误角点。
    """

    def __init__(
        self,
        config: VisionConfig | None = None,
        *,
        auto_grid: bool = True,
        lock_auto_grid: bool = True,
        use_temporal: bool = False,
        target: str | None = None,
    ) -> None:
        """创建分析器。

        ``auto_grid`` 控制是否允许自动找屏；``lock_auto_grid`` 表示首次成功后
        缓存角点；``use_temporal`` 控制是否启用多帧累积和稳定判定。
        """
        self.config = config if config is not None else load_config()
        self.auto_grid = auto_grid
        self.lock_auto_grid = lock_auto_grid
        self.use_temporal = use_temporal
        self.target = target
        self._auto_corners: np.ndarray | None = None
        self._auto_grid_debug: dict[str, object] = {}
        self._sequence = 0
        self._temporal = TemporalTracker(self.config.temporal)

    def reset(self, *, keep_calibration: bool = True) -> None:
        """清零帧序号和时序状态，可选择是否保留当前自动角点。"""
        self._sequence = 0
        self._temporal.reset()
        if not keep_calibration:
            self._auto_corners = None
            self._auto_grid_debug = {}

    def analyze(
        self,
        image: np.ndarray,
        *,
        timestamp_ms: float | None = None,
    ) -> AnalysisResult:
        """分析一帧 BGR 图像并返回测量、校正图和二值掩膜。"""
        # 帧序号从 1 开始；视频未提供时间戳时使用单调时钟，避免系统时间跳变。
        self._sequence += 1
        timestamp = (
            float(timestamp_ms)
            if timestamp_ms is not None
            else time.perf_counter() * 1000.0
        )
        if image is None or image.size == 0:
            measurement = Measurement.invalid(
                "输入图像为空", seq=self._sequence, timestamp_ms=int(timestamp)
            )
            return AnalysisResult(measurement, image, None, None, None, None)

        # 角点优先级：手动配置 > 本视频缓存的自动角点 > 本帧重新自动检测。
        corners = configured_corners(self.config, image.shape)
        grid_debug: dict[str, object] = {"calibration": "configured"}
        if corners is None and self._auto_corners is not None and self.lock_auto_grid:
            corners = self._auto_corners.copy()
            grid_debug = {
                "calibration": "cached_auto",
                **self._auto_grid_debug,
            }
        elif corners is None and self.auto_grid:
            detection = detect_grid_quad(image)
            corners = detection.corners
            grid_debug = {
                "calibration": "auto",
                "grid_confidence": detection.confidence,
                "grid_reason": detection.reason,
                "grid_candidates": detection.candidates,
                **detection.debug,
            }
            if corners is not None and self.lock_auto_grid:
                self._auto_corners = corners.copy()
                self._auto_grid_debug = {
                    key: value
                    for key, value in grid_debug.items()
                    if key != "calibration"
                }

        if corners is None:
            measurement = Measurement.invalid(
                "未找到示波器栅格；请执行手动标定或启用自动找屏",
                seq=self._sequence,
                timestamp_ms=int(timestamp),
            )
            measurement.debug = grid_debug
            return AnalysisResult(measurement, image, None, None, None, None)

        # 静态处理链：透视校正 → 黄色分割。
        warped = warp_grid(image, corners, self.config)
        raw_mask, segmentation_debug = segment_yellow(warped, self.config)
        # 视频几何使用多帧累积掩膜，但运动量始终由相邻原始掩膜计算。
        if self.use_temporal:
            geometry_mask, similarity, motion = self._temporal.push_mask(raw_mask)
        else:
            geometry_mask, similarity, motion = raw_mask, 1.0, 0.0

        # 几何模块接收的 debug 字典同时包含找屏和颜色分割诊断数据。
        measurement = measure_geometry(
            geometry_mask,
            self.config,
            sequence=self._sequence,
            timestamp_ms=timestamp,
            target=self.target,
            debug={**grid_debug, **segmentation_debug},
        )
        measurement.grid_corners_px = corners.astype(float).tolist()
        measurement.frame_similarity = similarity
        measurement.motion_score = motion
        # 几何测量完成后再写入时序历史并计算 stable/flip_period。
        if self.use_temporal:
            measurement = self._temporal.update(measurement)

        return AnalysisResult(
            measurement=measurement,
            source=image,
            warped=warped,
            raw_mask=raw_mask,
            geometry_mask=geometry_mask,
            corners=corners,
        )
