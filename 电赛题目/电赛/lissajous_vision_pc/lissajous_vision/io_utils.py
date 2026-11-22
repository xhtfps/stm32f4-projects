"""图像、目录和视频源的基础输入输出工具。"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def read_image(path: str | Path) -> np.ndarray:
    """读取图像。

    不直接使用 ``cv2.imread``，因为部分 Windows/OpenCV 组合不能可靠处理
    中文路径；先由 NumPy 读入字节，再调用 ``imdecode`` 可以规避此问题。
    """
    source = Path(path)
    data = np.fromfile(str(source), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"无法读取图像: {source}")
    return image


def write_image(path: str | Path, image: np.ndarray) -> None:
    """写入图像，使用 ``imencode + tofile`` 支持中文输出路径。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    suffix = target.suffix.lower() or ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise ValueError(f"无法编码图像: {target}")
    encoded.tofile(str(target))


def iter_images(root: str | Path, *, recursive: bool = True) -> list[Path]:
    """列出目录内支持的图片，并排序以保证批处理结果可复现。"""
    directory = Path(root)
    iterator = directory.rglob("*") if recursive else directory.glob("*")
    return sorted(
        path for path in iterator if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def parse_video_source(value: str) -> int | str:
    """把纯数字源转成摄像头编号，其余字符串保留为文件名或网络 URL。"""
    stripped = value.strip()
    if stripped.isdigit() or (
        stripped.startswith("-") and stripped[1:].isdigit()
    ):
        return int(stripped)
    return stripped
