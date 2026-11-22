"""使用项目内真实 DS2302A 照片进行端到端回归验证。"""

from __future__ import annotations

from pathlib import Path
import re
import unittest

from lissajous_vision import LissajousAnalyzer
from lissajous_vision.io_utils import read_image


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def expected_shape(filename: str) -> str:
    """按照标准样本中文文件名前缀返回协议使用的英文类别。"""
    if filename.startswith("圆"):
        return "ellipse"
    if filename.startswith("对角"):
        return "diagonal"
    if filename.startswith("无穷"):
        return "infinity"
    raise AssertionError(f"unknown fixture name: {filename}")


class StandardDatasetTest(unittest.TestCase):
    """验证 12 张标准图的分类/跨度，以及 3 张带 UI 照片的抗干扰分类。"""

    @classmethod
    def setUpClass(cls) -> None:
        """定位数据集；仓库不含样本时跳过，而不是让安装测试失败。"""
        cls.files = sorted((REPOSITORY_ROOT / "100Hz").rglob("*.jpg"))
        if len(cls.files) != 12:
            raise unittest.SkipTest("the 12-image 100 Hz standard set is unavailable")

    def test_all_standard_images_are_classified_and_measured(self) -> None:
        """标准集应 12/12 分类正确，X/Y 跨度误差均不超过 0.35 格。"""
        failures: list[str] = []
        for path in self.files:
            result = LissajousAnalyzer(lock_auto_grid=False).analyze(read_image(path))
            measurement = result.measurement
            target_match = re.search(r"([2468])div", path.name)
            self.assertIsNotNone(target_match)
            target_span = float(target_match.group(1))
            if not measurement.valid:
                failures.append(f"{path.name}: invalid ({measurement.error})")
                continue
            if measurement.shape != expected_shape(path.name):
                failures.append(
                    f"{path.name}: expected {expected_shape(path.name)}, got {measurement.shape}"
                )
            if abs(measurement.span_x_div - target_span) > 0.35:
                failures.append(
                    f"{path.name}: X span {measurement.span_x_div:.3f}, target {target_span}"
                )
            if abs(measurement.span_y_div - target_span) > 0.35:
                failures.append(
                    f"{path.name}: Y span {measurement.span_y_div:.3f}, target {target_span}"
                )
        self.assertFalse(failures, "\n".join(failures))

    def test_typical_photos_classify_despite_unmasked_ui(self) -> None:
        """开发照片虽违反正式遮挡条件，仍应依次识别为椭圆、直线和无穷图。"""
        folder = REPOSITORY_ROOT / "典型照片"
        files = sorted(folder.glob("*.jpg"))
        if len(files) != 3:
            self.skipTest("the three typical photographs are unavailable")
        expected = ("ellipse", "diagonal", "infinity")
        actual = []
        for path in files:
            result = LissajousAnalyzer(lock_auto_grid=False).analyze(read_image(path))
            actual.append(result.measurement.shape)
        self.assertEqual(tuple(actual), expected)


if __name__ == "__main__":
    unittest.main()
