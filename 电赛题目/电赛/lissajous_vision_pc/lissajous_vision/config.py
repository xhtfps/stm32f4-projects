"""视觉算法的配置数据结构，以及 JSON 配置文件的读取和保存函数。

配置按功能分成栅格、颜色分割、几何测量和时序判定四组。所有角点和遮罩区域
均使用归一化坐标保存，因此同一份标定文件可以适应分辨率相同、尺寸不同的帧。
"""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class GridConfig:
    """示波器栅格及透视校正参数。"""

    # DS2302A 的 X-Y 波形有效区域为 8×8 大格。
    divisions_x: int = 8
    divisions_y: int = 8
    # 透视校正后的标准画布；800 像素使每大格正好等于 100 像素。
    warp_width: int = 800
    warp_height: int = 800
    # 四角顺序固定为左上、右上、右下、左下，坐标范围均为 0～1。
    corners_norm: list[list[float]] | None = None


@dataclass(slots=True)
class SegmentationConfig:
    """黄色轨迹的强/弱双阈值分割参数。"""

    # 强黄色阈值：这些像素可信度高，可作为形态学重建的生长种子。
    h_min: int = 14
    h_max: int = 48
    s_min: int = 75
    v_min: int = 75
    # R+G-2B 衡量黄色相对蓝色的优势，可排除白色和青色栅格。
    yellow_dominance_min: int = 65
    # 弱黄色阈值更宽松，用于找回高光、白栅格交叉处的浅黄色轨迹。
    weak_h_padding: int = 6
    weak_s_min: int = 28
    weak_v_min: int = 90
    weak_yellow_dominance_min: int = 24
    # 从强黄色种子沿弱黄色掩膜最多生长的次数，以及弱掩膜闭运算核大小。
    reconstruction_iterations: int = 24
    weak_close_kernel: int = 5
    # 小于此面积的连通域视为相机噪点。
    min_component_area: int = 4
    # 摄像头预处理：中值去椒盐、高斯抑制高频闪烁、局部亮度归一化。
    median_kernel: int = 3
    gaussian_sigma: float = 0.8
    illumination_sigma: float = 21.0
    morphology_open_kernel: int = 3
    # 固定伪影遮罩，默认屏蔽栅格顶部中央的黄色触发位置标记。
    artifact_masks_norm: list[list[float]] = field(
        default_factory=lambda: [[0.465, 0.0, 0.535, 0.075]]
    )


@dataclass(slots=True)
class MeasurementConfig:
    """稀疏点云的稳健尺寸测量及质量判定参数。"""

    # 用分位数代替绝对最小/最大值，避免单个离群点拉坏跨度。
    low_percentile: float = 0.35
    high_percentile: float = 99.65
    # 在归一化中心轴附近取样时使用的半带宽。
    axis_band_norm: float = 0.08
    # 有效测量至少需要的轨迹像素数。
    min_trace_pixels: int = 50
    # 计算翻转对称性前的膨胀半径，用于容忍 1～2 像素抖动。
    symmetry_dilate_px: int = 2


@dataclass(slots=True)
class TemporalConfig:
    """视频模式的帧累积、稳定性和周期估计参数。"""

    # 多帧按位或可补全示波器扫描形成的稀疏点列。
    geometry_accumulation_frames: int = 3
    # 长历史用于翻转周期估计，短窗口用于稳定状态判断。
    history_seconds: float = 6.0
    stable_window_seconds: float = 0.5
    # 短窗口内跨度、中心和运动量均不越界时才输出 stable=True。
    stable_span_range_div: float = 0.15
    stable_center_range_div: float = 0.15
    stable_motion_max: float = 0.12


@dataclass(slots=True)
class VisionConfig:
    """完整视觉配置；负责在 Python 对象和 JSON 字典之间转换。"""

    schema_version: int = 1
    grid: GridConfig = field(default_factory=GridConfig)
    segmentation: SegmentationConfig = field(default_factory=SegmentationConfig)
    measurement: MeasurementConfig = field(default_factory=MeasurementConfig)
    temporal: TemporalConfig = field(default_factory=TemporalConfig)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "VisionConfig":
        """由 JSON 解码后的字典创建配置，缺失字段自动采用默认值。"""
        return cls(
            schema_version=int(data.get("schema_version", 1)),
            grid=GridConfig(**data.get("grid", {})),
            segmentation=SegmentationConfig(**data.get("segmentation", {})),
            measurement=MeasurementConfig(**data.get("measurement", {})),
            temporal=TemporalConfig(**data.get("temporal", {})),
        )

    def to_dict(self) -> dict[str, Any]:
        """转成只含 JSON 基本类型的字典，便于保存和跨设备传输。"""
        return {
            "schema_version": self.schema_version,
            "grid": {
                "divisions_x": self.grid.divisions_x,
                "divisions_y": self.grid.divisions_y,
                "warp_width": self.grid.warp_width,
                "warp_height": self.grid.warp_height,
                "corners_norm": copy.deepcopy(self.grid.corners_norm),
            },
            "segmentation": {
                "h_min": self.segmentation.h_min,
                "h_max": self.segmentation.h_max,
                "s_min": self.segmentation.s_min,
                "v_min": self.segmentation.v_min,
                "yellow_dominance_min": self.segmentation.yellow_dominance_min,
                "weak_h_padding": self.segmentation.weak_h_padding,
                "weak_s_min": self.segmentation.weak_s_min,
                "weak_v_min": self.segmentation.weak_v_min,
                "weak_yellow_dominance_min": (
                    self.segmentation.weak_yellow_dominance_min
                ),
                "reconstruction_iterations": (
                    self.segmentation.reconstruction_iterations
                ),
                "weak_close_kernel": self.segmentation.weak_close_kernel,
                "min_component_area": self.segmentation.min_component_area,
                "median_kernel": self.segmentation.median_kernel,
                "gaussian_sigma": self.segmentation.gaussian_sigma,
                "illumination_sigma": self.segmentation.illumination_sigma,
                "morphology_open_kernel": self.segmentation.morphology_open_kernel,
                "artifact_masks_norm": copy.deepcopy(
                    self.segmentation.artifact_masks_norm
                ),
            },
            "measurement": {
                "low_percentile": self.measurement.low_percentile,
                "high_percentile": self.measurement.high_percentile,
                "axis_band_norm": self.measurement.axis_band_norm,
                "min_trace_pixels": self.measurement.min_trace_pixels,
                "symmetry_dilate_px": self.measurement.symmetry_dilate_px,
            },
            "temporal": {
                "geometry_accumulation_frames": (
                    self.temporal.geometry_accumulation_frames
                ),
                "history_seconds": self.temporal.history_seconds,
                "stable_window_seconds": self.temporal.stable_window_seconds,
                "stable_span_range_div": self.temporal.stable_span_range_div,
                "stable_center_range_div": self.temporal.stable_center_range_div,
                "stable_motion_max": self.temporal.stable_motion_max,
            },
        }


def default_config_path() -> Path:
    """返回随工程提供的 DS2302A 默认参数文件路径。"""
    return Path(__file__).resolve().parents[1] / "configs" / "ds2302a_default.json"


def load_config(path: str | Path | None = None) -> VisionConfig:
    """读取配置；未指定路径时加载默认文件，文件不存在时使用内置默认值。"""
    config_path = default_config_path() if path is None else Path(path)
    if not config_path.exists():
        return VisionConfig()
    data = json.loads(config_path.read_text(encoding="utf-8"))
    return VisionConfig.from_dict(data)


def save_config(config: VisionConfig, path: str | Path) -> None:
    """以 UTF-8 和易读缩进格式保存配置，必要时自动创建父目录。"""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(config.to_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
