"""Count Lissajous wave peak points from edge contacts.

The oscilloscope trace frequently has one or more arcs disconnected by glare.
Counting only enclosed connected components therefore under-counts the
waveform. The K230 fast path instead counts the vertical sequence of
left/right turning contacts.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .boundary import detect_boundary_contacts, measure_boundary_touches


@dataclass(slots=True)
class MeshMeasurement:
    valid: bool
    error: str
    hole_count: int
    confidence: float
    centers: tuple[tuple[int, int], ...]
    areas: tuple[int, ...]
    candidate_scores: tuple[float, ...]
    sealed_mask: np.ndarray | None
    hole_mask: np.ndarray | None
    edge_candidates: tuple[int, ...]
    left_peaks: int
    right_peaks: int

    @property
    def peak_count(self) -> int:
        """Primary count exposed to the runtime; not a closed-cell count."""
        return self.hole_count


def _invalid(error: str, mask: np.ndarray | None = None) -> MeshMeasurement:
    return MeshMeasurement(
        False, error, 0, 0.0, (), (), (), mask, None, (), 0, 0
    )


def _estimate_from_edge_peaks(left_count: int, right_count: int) -> int:
    """Convert paired vertical-edge peaks to a visible multiplier."""
    if left_count <= 0 or right_count <= 0:
        return 0
    return max(1, int(round(0.5 * (left_count + right_count))))


def _edge_peak_candidates(mask: np.ndarray) -> tuple[tuple[int, ...], int, int]:
    """Measure the same trace with nearby edge bands and retain stable counts.

    The 2--3 percent bands absorb antialiasing and small perspective errors,
    without merging adjacent lobes.  A real lobe count survives this small
    perturbation; a threshold fragment does not.
    """
    candidates: list[int] = []
    left_counts: list[int] = []
    right_counts: list[int] = []
    for band in (0.020, 0.025, 0.030):
        contacts = detect_boundary_contacts(
            mask,
            edge_band_ratio=band,
            maximum_gap_ratio=0.003,
            minimum_run_ratio=0.010,
        )
        if contacts.error:
            continue
        left_count = len(contacts.left)
        right_count = len(contacts.right)
        estimate = _estimate_from_edge_peaks(left_count, right_count)
        if estimate:
            candidates.append(estimate)
            left_counts.append(left_count)
            right_counts.append(right_count)
    if not candidates:
        return (), 0, 0
    return (
        tuple(candidates),
        int(round(float(np.median(left_counts)))),
        int(round(float(np.median(right_counts)))),
    )


def measure_mesh_holes(mask: np.ndarray, *, maximum_ratio: int = 10) -> MeshMeasurement:
    """Return the visible 1:N peak-point count, including broken arcs.

    This deliberately does not use a ratio/phase template bank.  A complete
    template catalogue is slow and, more importantly, tends to select dense
    high-frequency traces for sparse photographs.  The count comes from the
    observable number of left/right peak-contact rows and is clamped only to
    the supported search range.
    """
    if mask is None or mask.ndim != 2:
        return _invalid("mask_invalid")
    binary = mask > 0
    ys, xs = np.nonzero(binary)
    if len(xs) < 50:
        return _invalid("trace_too_small", mask)
    span_x = int(np.max(xs) - np.min(xs) + 1)
    span_y = int(np.max(ys) - np.min(ys) + 1)
    if span_x < 30 or span_y < 30:
        return _invalid("trace_span_too_small", mask)

    candidates, left_peaks, right_peaks = _edge_peak_candidates(mask)
    if not candidates:
        return _invalid("edge_peak_missing", mask)

    # 1:1 is a special case: the two sides of a circle naturally have two
    # contacts, but it contains one mesh cell.  Reuse the lightweight
    # correlation/span/radius test already used by the K230-compatible path.
    boundary = measure_boundary_touches(mask)
    is_circle = (
        boundary.valid
        and abs(boundary.correlation) <= 0.18
        and boundary.radial_variance <= 0.045
        and boundary.circle_score >= -0.22
    )
    if is_circle:
        ratio = 1
        agreement = 1.0
    else:
        ordered = sorted(candidates)
        # A wider band can recover a contact whose interior arc is broken by
        # glare.  Keep that recovered upper candidate; a median would turn a
        # real 4-cell trace with candidates 4/3/2 into the wrong 3-cell result.
        ratio = int(ordered[-1])
        agreement = 1.0 - (
            max(candidates) - min(candidates)
        ) / float(max(1, max(candidates)))
    ratio = int(np.clip(ratio, 1, maximum_ratio))

    left = int(np.min(xs))
    right = int(np.max(xs))
    top = int(np.min(ys))
    bottom = int(np.max(ys))
    center_x = (left + right) // 2
    centers = tuple(
        (center_x, int(round(top + (index + 0.5) * (bottom - top) / ratio)))
        for index in range(ratio)
    )
    # Candidate scores are agreement values for the three edge bands.  They
    # are retained for the PC diagnostics without reintroducing templates.
    scores = tuple(
        round(
            max(0.0, 1.0 - abs(candidate - ratio) / float(max(1, ratio))),
            4,
        )
        for candidate in candidates
    )
    confidence = float(np.clip(0.55 + 0.45 * agreement, 0.0, 1.0))
    return MeshMeasurement(
        True, "", ratio, round(confidence, 4), centers,
        tuple(0 for _ in centers), scores,
        np.where(binary, 255, 0).astype(np.uint8), None,
        candidates, left_peaks, right_peaks,
    )


# Public names for the actual decision path.  The compatibility aliases keep
# older regression scripts importable while the UI and validator use peaks.
PeakMeasurement = MeshMeasurement
measure_peak_points = measure_mesh_holes
