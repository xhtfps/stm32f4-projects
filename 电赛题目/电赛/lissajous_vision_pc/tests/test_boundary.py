import math
import unittest

import cv2
import numpy as np

from lissajous_vision.boundary import (
    candidate_input_grids,
    confirm_dual_phase_candidate,
    dual_phase_consistent,
    detect_boundary_contacts,
    measure_boundary_touches,
)


def synthetic_mask(ratio_y, ratio_x, phase_deg, size=800):
    t = np.linspace(0.0, 2.0 * math.pi, 12000)
    x = 0.48 * size * np.sin(ratio_x * t)
    y = 0.48 * size * np.sin(ratio_y * t + math.radians(phase_deg))
    points = np.column_stack((0.5 * size + x, 0.5 * size - y))
    points = np.rint(points).astype(np.int32).reshape(-1, 1, 2)
    mask = np.zeros((size, size), dtype=np.uint8)
    cv2.polylines(mask, [points], True, 255, 3, cv2.LINE_AA)
    return mask


class BoundaryTouchTests(unittest.TestCase):
    def test_generic_phases_recover_ratios(self):
        for ratio_x in range(1, 11):
            measured = measure_boundary_touches(
                synthetic_mask(1, ratio_x, 91.0)
            )
            self.assertTrue(measured.valid, (ratio_x, measured))
            self.assertEqual((1, ratio_x), (measured.ratio_y, measured.ratio_x))

    def test_dual_phase_confirmation(self):
        first = measure_boundary_touches(synthetic_mask(1, 7, 11.0))
        second = measure_boundary_touches(synthetic_mask(1, 7, 48.0))
        self.assertTrue(dual_phase_consistent(first, second))

    def test_probe_candidates_use_touch_tolerance(self):
        measured = measure_boundary_touches(synthetic_mask(1, 7, 48.0))
        self.assertIn(350, candidate_input_grids(50, measured))

    def test_dual_phase_returns_only_unique_frequency(self):
        first = measure_boundary_touches(synthetic_mask(1, 2, 11.0))
        second = measure_boundary_touches(synthetic_mask(1, 2, 48.0))
        self.assertEqual(100, confirm_dual_phase_candidate(50, first, second))

    def test_circle_score_prefers_circle_over_ellipse(self):
        circle = measure_boundary_touches(synthetic_mask(1, 1, 90.0))
        ellipse = synthetic_mask(1, 1, 90.0)
        ellipse = cv2.resize(ellipse, (800, 560))
        canvas = np.zeros((800, 800), dtype=np.uint8)
        canvas[120:680] = ellipse
        ellipse_measurement = measure_boundary_touches(canvas)
        self.assertGreater(circle.circle_score, ellipse_measurement.circle_score)

    def test_visualized_contacts_match_counted_contacts(self):
        mask = synthetic_mask(1, 5, 91.0)
        contacts = detect_boundary_contacts(mask)
        measured = measure_boundary_touches(mask, contacts=contacts)
        self.assertEqual(measured.left, len(contacts.left))
        self.assertEqual(measured.right, len(contacts.right))
        self.assertEqual(measured.top, len(contacts.top))
        self.assertEqual(measured.bottom, len(contacts.bottom))

    def test_isolated_edge_noise_is_not_a_peak(self):
        clean = synthetic_mask(1, 4, 91.0)
        noisy = clean.copy()
        # A bright vertical blob at the left edge passes the raw projection,
        # but it does not continue into the waveform interior.
        noisy[180:215, 0:4] = 255
        clean_contacts = detect_boundary_contacts(clean)
        noisy_contacts = detect_boundary_contacts(noisy)
        self.assertEqual(len(clean_contacts.left), len(noisy_contacts.left))


if __name__ == "__main__":
    unittest.main()
