"""几何测量的合成图回归测试。

合成模板不依赖现场照片，可快速检查三类公式、空掩膜保护和细长旋转椭圆判据。
"""

from __future__ import annotations

import unittest

import cv2
import numpy as np

from lissajous_vision.config import VisionConfig
from lissajous_vision.geometry import measure_geometry
from lissajous_vision.templates import generate_template


class GeometryTest(unittest.TestCase):
    """验证核心几何分类在典型和边界形状上的行为。"""

    def setUp(self) -> None:
        """每个测试使用一份无状态默认配置，避免测试之间相互影响。"""
        self.config = VisionConfig()

    def test_synthetic_templates_are_separable(self) -> None:
        """三种理想模板都应被分到自身类别，且跨度保持为 6 格。"""
        for shape in ("diagonal", "ellipse", "infinity"):
            with self.subTest(shape=shape):
                mask = generate_template(shape, (800, 800), (400, 400), (300, 300), thickness=3)
                # 连通域调用同时确认模板不是空图；返回标签本身无需参与断言。
                components, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
                self.assertGreaterEqual(components, 2)
                measurement = measure_geometry(mask, self.config)
                self.assertTrue(measurement.valid)
                self.assertEqual(measurement.shape, shape)
                self.assertAlmostEqual(measurement.span_x_div, 6.0, delta=0.15)
                self.assertAlmostEqual(measurement.span_y_div, 6.0, delta=0.15)

    def test_empty_mask_is_invalid(self) -> None:
        """空掩膜必须返回可解释的无效结果，而不能在协方差处崩溃。"""
        measurement = measure_geometry(np.zeros((800, 800), dtype=np.uint8), self.config)
        self.assertFalse(measurement.valid)
        self.assertIn("轨迹像素", measurement.error or "")

    def test_rotated_thin_closed_ellipse_is_not_a_diagonal(self) -> None:
        """细长但闭合的倾斜椭圆不应因相关系数较高而误判为直线。"""
        mask = np.zeros((800, 800), dtype=np.uint8)
        cv2.ellipse(mask, (400, 400), (250, 60), 135, 0, 360, 255, 5, cv2.LINE_AA)
        measurement = measure_geometry(mask, self.config)
        self.assertTrue(measurement.valid)
        self.assertEqual(measurement.shape, "ellipse")
        self.assertGreater(measurement.pca_minor_ratio, 0.02)
        self.assertGreater(measurement.line_error_div, 0.2)
        self.assertGreater(measurement.score_1x, 0.75)


if __name__ == "__main__":
    unittest.main()
