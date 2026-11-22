"""黄色强/弱双阈值分割的合成图测试。"""

from __future__ import annotations

import unittest

import cv2
import numpy as np

from lissajous_vision.config import VisionConfig
from lissajous_vision.segmentation import segment_yellow


class SegmentationTest(unittest.TestCase):
    """验证浅黄色补全不会沿白色网格发生错误扩散。"""

    def test_pale_yellow_gap_is_reconstructed_without_white_grid(self) -> None:
        """强黄色种子应补全 38 像素浅黄断段，同时排除远处白网格。"""
        image = np.zeros((800, 800, 3), dtype=np.uint8)
        # 先绘制与示波器相似的白色 8×8 网格。
        for coordinate in range(0, 801, 100):
            cv2.line(image, (coordinate, 0), (coordinate, 799), (210, 210, 210), 2)
            cv2.line(image, (0, coordinate), (799, coordinate), (210, 210, 210), 2)

        # 完整轨迹是低饱和浅黄色，两段强黄色从断口两侧提供重建种子。
        cv2.line(image, (100, 400), (700, 400), (170, 235, 235), 7)
        cv2.line(image, (100, 400), (361, 400), (0, 255, 255), 5)
        cv2.line(image, (399, 400), (700, 400), (0, 255, 255), 5)

        mask, debug = segment_yellow(image, VisionConfig())
        self.assertGreater(debug["weak_yellow_pixels"], debug["strong_yellow_pixels"])
        self.assertGreater(cv2.countNonZero(mask[395:406, 370:391]), 0)
        # 远离轨迹的白色网格不得进入黄色掩膜。
        self.assertEqual(cv2.countNonZero(mask[95:106, 195:206]), 0)

    def test_marker_hint_does_not_erase_crossing_trace(self) -> None:
        image = np.zeros((800, 800, 3), dtype=np.uint8)
        cv2.line(image, (100, 20), (700, 20), (0, 255, 255), 7)
        config = VisionConfig()
        mask, _ = segment_yellow(image, config)
        # The default top marker hint overlaps x=372..428.  A long trace is
        # not a compact marker and must survive that hint rectangle.
        self.assertGreater(cv2.countNonZero(mask[15:26, 380:420]), 0)


if __name__ == "__main__":
    unittest.main()
