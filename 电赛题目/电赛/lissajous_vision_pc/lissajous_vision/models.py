"""定义一帧视觉结果的数据协议。

Measurement 既是电脑端算法内部的统一结果，也是未来 K230 向 STM32F407
发送数据时的字段依据。算法内部允许 NumPy 标量，序列化时会统一转换成标准
JSON 类型，并把 NaN/Inf 转成 null，防止通信端解析失败。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np


@dataclass(slots=True)
class Measurement:
    """一帧（或一个短时累积窗口）的完整视觉测量结果。

    字段按“协议状态、几何量、模型误差、时序量、目标误差”依次排列。
    所有带 ``_div`` 后缀的量均以示波器大格为单位。
    """

    # 协议与帧状态。
    version: int = 1
    seq: int = 0
    timestamp_ms: int = 0
    valid: bool = False
    error: str | None = None
    # 三分类结果：diagonal、ellipse、infinity 或 unknown。
    shape: str = "unknown"
    confidence: float = 0.0
    # 稳健包围盒的跨度与相对栅格中心偏移；Y 轴向上为正。
    span_x_div: float = 0.0
    span_y_div: float = 0.0
    center_x_div: float = 0.0
    center_y_div: float = 0.0
    # 掩膜统计量，用于判断轨迹是否过少、过粗或混入大面积干扰。
    trace_pixels: int = 0
    fill_ratio: float = 0.0
    # 相关系数和中心截距给出直线方向及 1:1 相位的代理量。
    corr_xy: float = 0.0
    phase_abs_deg: float = 0.0
    vertical_intercept_ratio: float = 0.0
    # 直线模型误差及 PCA 次轴/主轴能量比。
    line_error_div: float = 0.0
    line_error_norm: float = 0.0
    pca_minor_ratio: float = 0.0
    # OpenCV 椭圆拟合结果和旋转坐标系中的椭圆残差。
    ellipse_error: float = 0.0
    ellipse_center_x_div: float = 0.0
    ellipse_center_y_div: float = 0.0
    ellipse_axis_x_div: float = 0.0
    ellipse_axis_y_div: float = 0.0
    ellipse_angle_deg: float = 0.0
    # 2:1 无穷曲线解析模型误差及中心交叉区域密度。
    infinity_error: float = 0.0
    center_density: float = 0.0
    # 分别沿水平轴和垂直轴翻转后的掩膜交并比。
    symmetry_x: float = 0.0
    symmetry_y: float = 0.0
    # 实测掩膜到三种理想模板的双向 Chamfer 距离，单位为格。
    chamfer_diagonal_div: float = 0.0
    chamfer_ellipse_div: float = 0.0
    chamfer_infinity_div: float = 0.0
    # 给 STM32 搜索状态机使用的 1:1、2:1 归一化评分，越接近 1 越好。
    score_1x: float = 0.0
    score_2x: float = 0.0
    # 视频时序指标：相邻帧相似度、运动量、翻转周期及稳定状态。
    frame_similarity: float = 0.0
    motion_score: float = 0.0
    flip_period_ms: float = 0.0
    flip_confidence: float = 0.0
    stable: bool = False
    stable_ms: int = 0
    # 指定 --target 后生成的目标跨度和形状误差。
    target: str | None = None
    target_span_x_error_div: float | None = None
    target_span_y_error_div: float | None = None
    target_shape_error: float | None = None
    # 原图像素坐标中的 TL/TR/BR/BL 四角及可选调试数据。
    grid_corners_px: list[list[float]] | None = None
    debug: dict[str, Any] = field(default_factory=dict, repr=False)

    @classmethod
    def invalid(
        cls,
        error: str,
        *,
        seq: int = 0,
        timestamp_ms: int = 0,
        corners: np.ndarray | None = None,
    ) -> "Measurement":
        """构造无效结果，同时保留帧序号、时间戳和可能已有的角点。"""
        return cls(
            seq=seq,
            timestamp_ms=timestamp_ms,
            valid=False,
            error=error,
            grid_corners_px=None if corners is None else corners.astype(float).tolist(),
        )

    def to_dict(self, *, include_debug: bool = False) -> dict[str, Any]:
        """转为可直接交给 json.dumps 的字典。"""
        data = asdict(self)
        if not include_debug:
            data.pop("debug", None)
        return _json_safe(data)


def _json_safe(value: Any) -> Any:
    """递归清理 NumPy 类型、非有限浮点数和嵌套容器。"""
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, float):
        if not np.isfinite(value):
            return None
        return round(value, 6)
    return value
