"""视频时序处理：短帧累积、稳定判定和慢翻转周期估计。"""

from __future__ import annotations

from collections import deque

import cv2
import numpy as np

from .config import TemporalConfig
from .models import Measurement
from .segmentation import accumulate_masks


def _dilated_iou(first: np.ndarray, second: np.ndarray) -> float:
    """计算允许约 2 像素位置误差的相邻帧 IoU。"""
    # 先膨胀再比较，可避免相机微小抖动被放大成很大的运动量。
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    a = cv2.dilate(first, kernel) > 0
    b = cv2.dilate(second, kernel) > 0
    union = np.count_nonzero(a | b)
    if union == 0:
        return 0.0
    return float(np.count_nonzero(a & b) / union)


class TemporalTracker:
    """维护短期掩膜队列和长期几何历史。

    短队列（默认 3 帧）用于补全扫描点；长期队列（默认 6 秒）仅保存少量标量，
    用于稳定判定和相关系数翻转周期估计，不会长期占用图像内存。
    """

    def __init__(self, config: TemporalConfig) -> None:
        """根据配置创建固定长度队列，并清空上一帧状态。"""
        self.config = config
        self._masks: deque[np.ndarray] = deque(
            maxlen=max(1, int(config.geometry_accumulation_frames))
        )
        self._previous_mask: np.ndarray | None = None
        self._history: deque[tuple[float, str, float, float, float, float, float, float]] = deque()
        self._stable_since_ms: float | None = None

    def reset(self) -> None:
        """清除全部时序状态；重新标定或切换视频源时调用。"""
        self._masks.clear()
        self._previous_mask = None
        self._history.clear()
        self._stable_since_ms = None

    def push_mask(self, mask: np.ndarray) -> tuple[np.ndarray, float, float]:
        """加入原始单帧掩膜，返回累积掩膜、相似度和运动量。"""
        if self._previous_mask is None or self._previous_mask.shape != mask.shape:
            similarity = 1.0
            self._masks.clear()
        else:
            similarity = _dilated_iou(mask, self._previous_mask)
        # 运动量定义为 1-IoU；第一帧没有参照，约定 similarity=1。
        motion = float(np.clip(1.0 - similarity, 0.0, 1.0))
        self._previous_mask = mask.copy()
        self._masks.append(mask.copy())
        return accumulate_masks(list(self._masks)), similarity, motion

    def update(self, measurement: Measurement) -> Measurement:
        """把本帧几何量加入历史，并原地补充周期和稳定状态。"""
        measurement.frame_similarity = float(measurement.frame_similarity)
        measurement.motion_score = float(measurement.motion_score)
        if not measurement.valid:
            self._stable_since_ms = None
            return measurement

        now = float(measurement.timestamp_ms)
        # 历史元组依次保存：时间、类别、X/Y跨度、X/Y中心、运动量、相关系数。
        self._history.append(
            (
                now,
                measurement.shape,
                measurement.span_x_div,
                measurement.span_y_div,
                measurement.center_x_div,
                measurement.center_y_div,
                measurement.motion_score,
                measurement.corr_xy,
            )
        )
        # 丢弃周期估计窗口之外的旧记录，使内存占用有上界。
        cutoff = now - 1000.0 * float(self.config.history_seconds)
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()

        period_ms, period_confidence = self._estimate_flip_period()
        measurement.flip_period_ms = period_ms
        measurement.flip_confidence = period_confidence

        # 稳定性只看最近短窗口，并要求至少覆盖设定窗口的 80%，避免刚启动
        # 两三帧就错误输出 stable=True。
        stable_cutoff = now - 1000.0 * float(self.config.stable_window_seconds)
        recent = [record for record in self._history if record[0] >= stable_cutoff]
        enough_time = bool(recent) and recent[-1][0] - recent[0][0] >= (
            0.8 * 1000.0 * float(self.config.stable_window_seconds)
        )
        if enough_time:
            # np.ptp 返回窗口内最大值减最小值，用于检查跨度和中心抖动范围。
            values = np.asarray([record[2:7] for record in recent], dtype=np.float64)
            same_shape = len({record[1] for record in recent}) == 1
            stable_now = (
                same_shape
                and float(np.ptp(values[:, 0])) <= self.config.stable_span_range_div
                and float(np.ptp(values[:, 1])) <= self.config.stable_span_range_div
                and float(np.ptp(values[:, 2])) <= self.config.stable_center_range_div
                and float(np.ptp(values[:, 3])) <= self.config.stable_center_range_div
                and float(np.max(values[:, 4])) <= self.config.stable_motion_max
            )
        else:
            stable_now = False

        # stable_ms 从本次连续稳定区间的起点计算；任何越界都会重新计时。
        if stable_now:
            if self._stable_since_ms is None:
                self._stable_since_ms = recent[0][0]
            measurement.stable = True
            measurement.stable_ms = int(max(0.0, now - self._stable_since_ms))
        else:
            self._stable_since_ms = None
            measurement.stable = False
            measurement.stable_ms = 0
        return measurement

    def _estimate_flip_period(self) -> tuple[float, float]:
        """用相关系数序列的自相关峰估计慢翻转周期。

        频率略有失配时，李萨如图形方向会缓慢翻转，``corr_xy`` 随之周期变化。
        摄像头帧间隔并不严格相等，所以先插值到等间隔时间轴，再计算归一化
        自相关。数据不足、变化太小或没有显著峰时返回 ``(0, 0)``。
        """
        if len(self._history) < 12:
            return 0.0, 0.0
        times = np.asarray([item[0] for item in self._history], dtype=np.float64)
        values = np.asarray([item[7] for item in self._history], dtype=np.float64)
        duration = times[-1] - times[0]
        intervals = np.diff(times)
        valid_intervals = intervals[intervals > 0]
        if duration < 500.0 or len(valid_intervals) < 8:
            return 0.0, 0.0
        # 中位帧间隔对偶发卡顿比平均值更稳健。
        dt = float(np.median(valid_intervals))
        if dt <= 0.0:
            return 0.0, 0.0
        # 线性插值成均匀采样，才能把自相关峰下标可靠换算成毫秒。
        uniform_times = np.arange(times[0], times[-1] + 0.5 * dt, dt)
        uniform_values = np.interp(uniform_times, times, values)
        uniform_values -= float(np.mean(uniform_values))
        variance = float(np.dot(uniform_values, uniform_values))
        if variance < 1e-3:
            return 0.0, 0.0

        # 只保留非负延迟部分，并除以零延迟能量得到近似 0～1 置信度。
        autocorrelation = np.correlate(uniform_values, uniform_values, mode="full")
        autocorrelation = autocorrelation[len(uniform_values) - 1 :] / variance
        # 排除低于 80 ms 的抖动，并避免在历史尾部选择重叠样本过少的伪峰。
        minimum_lag = max(2, int(round(80.0 / dt)))
        maximum_lag = min(len(autocorrelation) - 2, int(round(0.7 * duration / dt)))
        if maximum_lag <= minimum_lag:
            return 0.0, 0.0

        # 选择第一个超过 0.25 的局部峰，即可得到最短基本周期。
        peaks = [
            lag
            for lag in range(minimum_lag, maximum_lag + 1)
            if autocorrelation[lag] >= autocorrelation[lag - 1]
            and autocorrelation[lag] > autocorrelation[lag + 1]
            and autocorrelation[lag] >= 0.25
        ]
        if not peaks:
            return 0.0, 0.0
        lag = peaks[0]
        confidence = float(np.clip(autocorrelation[lag], 0.0, 1.0))
        return float(lag * dt), confidence
