import unittest

import cv2
import numpy as np

from lissajous_vision.mesh import measure_mesh_holes


def synthetic_mesh(multiplier, size=800):
    """Draw visible open/closed cells with one edge peak per cell."""
    mask = np.zeros((size, size), dtype=np.uint8)
    half_height = max(18, int(0.29 * size / multiplier))
    for index in range(multiplier):
        center_y = int(round(
            0.10 * size + (index + 0.5) * 0.80 * size / multiplier
        ))
        cv2.ellipse(
            mask, (size // 2, center_y),
            (int(0.43 * size), half_height),
            0, 0, 360, 255, 9, cv2.LINE_AA,
        )
    return mask


class MeshCountTests(unittest.TestCase):
    def test_one_to_six_mesh_holes(self):
        for multiplier in range(1, 7):
            measured = measure_mesh_holes(synthetic_mesh(multiplier))
            self.assertTrue(measured.valid)
            self.assertEqual(
                multiplier, measured.hole_count,
                measured.edge_candidates,
            )

    def test_open_trace_has_no_enclosed_hole(self):
        mask = np.zeros((800, 800), dtype=np.uint8)
        cv2.line(mask, (50, 400), (750, 400), 255, 5)
        measured = measure_mesh_holes(mask)
        self.assertEqual(0, measured.hole_count)

    def test_disconnected_cell_still_keeps_multiplier(self):
        mask = synthetic_mesh(4)
        # Simulate a glare/threshold gap in the middle of one cell.  The
        # edge peak sequence remains observable, so the multiplier is still 4.
        mask[360:440, 250:550] = 0
        measured = measure_mesh_holes(mask)
        self.assertTrue(measured.valid)
        self.assertEqual(4, measured.hole_count)


if __name__ == "__main__":
    unittest.main()
