"""在 OpenCV BGR 图像上绘制中文。

OpenCV 自带 Hershey 字体不包含汉字，因此先转换成 Pillow 图像，使用 Windows
中文字体绘制，再转回 BGR。字体对象使用缓存，避免视频逐帧重复读取字体文件。
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


WINDOWS_CHINESE_FONTS = (
    # 按清晰度和常见程度依次尝试：微软雅黑、微软雅黑粗体、黑体、宋体。
    Path(r"C:\Windows\Fonts\msyh.ttc"),
    Path(r"C:\Windows\Fonts\msyhbd.ttc"),
    Path(r"C:\Windows\Fonts\simhei.ttf"),
    Path(r"C:\Windows\Fonts\simsun.ttc"),
)


@lru_cache(maxsize=16)
def _font(font_size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    """取得指定字号字体；同一字号只从磁盘加载一次。"""
    for path in WINDOWS_CHINESE_FONTS:
        if path.exists():
            return ImageFont.truetype(str(path), font_size)
    return ImageFont.load_default()


def draw_text_lines_bgr(
    image: np.ndarray,
    lines: list[str],
    *,
    origin: tuple[int, int] = (15, 15),
    font_size: int = 24,
    line_gap: int = 6,
    color_bgr: tuple[int, int, int] = (255, 255, 255),
    stroke_bgr: tuple[int, int, int] = (0, 0, 0),
    stroke_width: int = 2,
) -> None:
    """在 BGR 图像上原地绘制多行 Unicode 文字。

    ``color_bgr`` 与 OpenCV 保持一致；传给 Pillow 前需要反转成 RGB。
    描边用于保证白字在明亮栅格和黄色波形上仍然清晰可读。
    """
    if not lines:
        return
    # Pillow 使用 RGB，而摄像头/OpenCV 使用 BGR。
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    canvas = Image.fromarray(rgb)
    draw = ImageDraw.Draw(canvas)
    font = _font(max(8, int(font_size)))
    fill = tuple(reversed(color_bgr))
    stroke_fill = tuple(reversed(stroke_bgr))
    x, y = origin
    for line in lines:
        draw.text(
            (x, y),
            line,
            font=font,
            fill=fill,
            stroke_width=stroke_width,
            stroke_fill=stroke_fill,
        )
        y += font_size + line_gap
    # 写回原数组，使调用者无需接收新的图像对象。
    image[:] = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)
