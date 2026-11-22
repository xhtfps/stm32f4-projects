"""Fast Lissajous ratio and circle measurements from a warped trace mask."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np


@dataclass(slots=True)
class BoundaryMeasurement:
    valid: bool
    error: str
    left: int
    right: int
    top: int
    bottom: int
    coarse_y: int
    coarse_x: int
    ratio_y: int
    ratio_x: int
    confidence: float
    span_x_div: float
    span_y_div: float
    correlation: float
    radial_variance: float
    circle_score: float

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(slots=True)
class BoundaryContacts:
    """K230-style edge-band contact positions in warped-image pixels."""

    error: str
    bbox: tuple[int, int, int, int] | None
    edge_x: int
    edge_y: int
    left: tuple[tuple[int, int], ...]
    right: tuple[tuple[int, int], ...]
    top: tuple[tuple[int, int], ...]
    bottom: tuple[tuple[int, int], ...]


def _fill_short_gaps(values: np.ndarray, maximum_gap: int) -> np.ndarray:
    result = values.astype(bool, copy=True)
    if maximum_gap <= 0 or not result.any():
        return result
    active = np.flatnonzero(result)
    for left, right in zip(active[:-1], active[1:]):
        if 1 < right - left <= maximum_gap + 1:
            result[left : right + 1] = True
    return result


def _extract_runs(
    values: np.ndarray, maximum_gap: int, minimum_length: int
) -> tuple[tuple[int, int], ...]:
    closed = _fill_short_gaps(values, maximum_gap)
    padded = np.pad(closed.astype(np.int8), (1, 1))
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    stops = np.flatnonzero(changes == -1)
    return tuple(
        (int(start), int(stop - 1))
        for start, stop in zip(starts, stops)
        if stop - start >= minimum_length
    )


def _count_runs(values: np.ndarray, maximum_gap: int, minimum_length: int) -> int:
    return len(_extract_runs(values, maximum_gap, minimum_length))


def _filter_continuous_runs(
    binary: np.ndarray,
    runs: tuple[tuple[int, int], ...],
    *,
    side: str,
    left: int,
    top: int,
    right: int,
    bottom: int,
    band_x: int,
    band_y: int,
) -> tuple[tuple[int, int], ...]:
    """Reject edge runs whose trace does not continue into the image.

    A single bright pixel or a short side blob can pass the edge projection.
    The support mask closes only short gaps, then checks that the component
    reaches at least eight percent of the perpendicular trace span.  Real
    waveform peaks satisfy this even when an interior arc is broken; isolated
    noise does not.
    """
    if not runs:
        return ()
    bridge = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    support = cv2.morphologyEx(
        binary.astype(np.uint8), cv2.MORPH_CLOSE, bridge
    )
    support = cv2.dilate(support, cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (5, 5)
    ))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(support, 8)
    span_x = right - left + 1
    span_y = bottom - top + 1
    minimum_area = max(80, int(0.00025 * binary.size))
    minimum_span = max(
        24,
        int(0.08 * (span_x if side in ("left", "right") else span_y)),
    )
    supported: list[tuple[int, int]] = []
    for start, stop in runs:
        if side == "left":
            region = labels[
                top + start : top + stop + 1,
                left : min(right + 1, left + band_x),
            ]
        elif side == "right":
            region = labels[
                top + start : top + stop + 1,
                max(left, right - band_x + 1) : right + 1,
            ]
        elif side == "top":
            region = labels[
                top : min(bottom + 1, top + band_y),
                left + start : left + stop + 1,
            ]
        else:
            region = labels[
                max(top, bottom - band_y + 1) : bottom + 1,
                left + start : left + stop + 1,
            ]
        candidates = np.unique(region)
        keep = False
        for component in candidates:
            if component <= 0 or component >= count:
                continue
            area = int(stats[component, cv2.CC_STAT_AREA])
            axis_span = int(
                stats[component, cv2.CC_STAT_WIDTH]
                if side in ("left", "right")
                else stats[component, cv2.CC_STAT_HEIGHT]
            )
            if area >= minimum_area and axis_span >= minimum_span:
                keep = True
                break
        if keep:
            supported.append((start, stop))
    return tuple(supported)


def _pair_count(first: int, second: int) -> int:
    if first <= 0 or second <= 0:
        return 0
    return max(1, int(round(0.5 * (first + second))))


def _empty(error: str) -> BoundaryMeasurement:
    return BoundaryMeasurement(
        False, error, 0, 0, 0, 0, 0, 0, 0, 0, 0.0,
        0.0, 0.0, 0.0, 1.0, -2.0,
    )


def detect_boundary_contacts(
    mask: np.ndarray,
    *,
    edge_band_ratio: float = 0.008,
    projection_pixels: int = 1,
    maximum_gap_ratio: float = 0.003,
    minimum_run_ratio: float = 0.003,
) -> BoundaryContacts:
    """Locate every contact counted by the K230-style four-edge projection."""
    empty = lambda error: BoundaryContacts(
        error, None, 0, 0, (), (), (), ()
    )
    if mask is None or mask.ndim != 2:
        return empty("mask_invalid")
    binary = mask > 0
    ys, xs = np.nonzero(binary)
    if len(xs) < 50:
        return empty("trace_too_small")
    height, width = mask.shape
    # Build a short-gap envelope before finding the bounding box.  This keeps
    # legitimate broken arcs in the envelope while preventing an isolated
    # edge blob from moving the box to x=0/y=0.
    envelope = cv2.morphologyEx(
        binary.astype(np.uint8) * 255,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
    ) > 0
    envelope_labels, envelope_stats = cv2.connectedComponentsWithStats(
        envelope.astype(np.uint8), 8
    )[1:3]
    minimum_envelope_area = max(100, int(0.0005 * binary.size))
    keep_labels = np.flatnonzero(
        envelope_stats[:, cv2.CC_STAT_AREA] >= minimum_envelope_area
    )
    keep_labels = keep_labels[keep_labels > 0]
    if len(keep_labels):
        envelope = np.isin(envelope_labels, keep_labels)
    envelope_ys, envelope_xs = np.nonzero(envelope)
    if len(envelope_xs) < 50:
        envelope_ys, envelope_xs = ys, xs
    x_low, x_high = np.percentile(envelope_xs, [0.1, 99.9])
    y_low, y_high = np.percentile(envelope_ys, [0.1, 99.9])
    left = max(0, int(math.floor(x_low)))
    right = min(width - 1, int(math.ceil(x_high)))
    top = max(0, int(math.floor(y_low)))
    bottom = min(height - 1, int(math.ceil(y_high)))
    span_x = right - left + 1
    span_y = bottom - top + 1
    if span_x < 20 or span_y < 20:
        return empty("trace_span_too_small")

    band_x = max(3, int(round(edge_band_ratio * span_x)))
    band_y = max(3, int(round(edge_band_ratio * span_y)))
    gap_y = max(1, int(round(maximum_gap_ratio * span_y)))
    gap_x = max(1, int(round(maximum_gap_ratio * span_x)))
    run_y = max(2, int(round(minimum_run_ratio * span_y)))
    run_x = max(2, int(round(minimum_run_ratio * span_x)))
    left_projection = np.count_nonzero(
        binary[top : bottom + 1, left : min(right + 1, left + band_x)], axis=1
    ) >= projection_pixels
    right_projection = np.count_nonzero(
        binary[top : bottom + 1, max(left, right - band_x + 1) : right + 1], axis=1
    ) >= projection_pixels
    top_projection = np.count_nonzero(
        binary[top : min(bottom + 1, top + band_y), left : right + 1], axis=0
    ) >= projection_pixels
    bottom_projection = np.count_nonzero(
        binary[max(top, bottom - band_y + 1) : bottom + 1, left : right + 1], axis=0
    ) >= projection_pixels

    left_runs = _filter_continuous_runs(
        binary, _extract_runs(left_projection, gap_y, run_y),
        side="left", left=left, top=top, right=right, bottom=bottom,
        band_x=band_x, band_y=band_y,
    )
    right_runs = _filter_continuous_runs(
        binary, _extract_runs(right_projection, gap_y, run_y),
        side="right", left=left, top=top, right=right, bottom=bottom,
        band_x=band_x, band_y=band_y,
    )
    top_runs = _filter_continuous_runs(
        binary, _extract_runs(top_projection, gap_x, run_x),
        side="top", left=left, top=top, right=right, bottom=bottom,
        band_x=band_x, band_y=band_y,
    )
    bottom_runs = _filter_continuous_runs(
        binary, _extract_runs(bottom_projection, gap_x, run_x),
        side="bottom", left=left, top=top, right=right, bottom=bottom,
        band_x=band_x, band_y=band_y,
    )
    return BoundaryContacts(
        "", (left, top, right, bottom), band_x, band_y,
        tuple((left, top + (start + stop) // 2) for start, stop in left_runs),
        tuple((right, top + (start + stop) // 2) for start, stop in right_runs),
        tuple((left + (start + stop) // 2, top) for start, stop in top_runs),
        tuple((left + (start + stop) // 2, bottom) for start, stop in bottom_runs),
    )


def measure_boundary_touches(
    mask: np.ndarray,
    *,
    divisions: int = 8,
    edge_band_ratio: float = 0.008,
    projection_pixels: int = 1,
    maximum_gap_ratio: float = 0.003,
    minimum_run_ratio: float = 0.003,
    contacts: BoundaryContacts | None = None,
) -> BoundaryMeasurement:
    """Count trace contacts on all four sides of its robust bounding box.

    A generic ``fy:fx = m:n`` Lissajous trace has approximately ``n``
    contacts on each vertical side and ``m`` contacts on each horizontal
    side. Two phase-separated frames are still required before accepting a
    search candidate because special phases can merge adjacent contacts.
    """
    if mask is None or mask.ndim != 2:
        return _empty("mask_invalid")
    ys, xs = np.nonzero(mask)
    contacts = contacts or detect_boundary_contacts(
        mask,
        edge_band_ratio=edge_band_ratio,
        projection_pixels=projection_pixels,
        maximum_gap_ratio=maximum_gap_ratio,
        minimum_run_ratio=minimum_run_ratio,
    )
    if contacts.error or contacts.bbox is None:
        return _empty(contacts.error or "boundary_contact_missing")
    height, width = mask.shape
    left, top, right, bottom = contacts.bbox
    span_x = right - left + 1
    span_y = bottom - top + 1
    left_count = len(contacts.left)
    right_count = len(contacts.right)
    top_count = len(contacts.top)
    bottom_count = len(contacts.bottom)
    # Screen clipping tends to lose contacts, while a fixed artifact tends to
    # add one. Keep the conservative side estimates; the search layer expands
    # them by +/-1 and resolves ambiguity with the phase-shifted frame.
    coarse_x = max(left_count, right_count)
    coarse_y = min(top_count, bottom_count)
    if coarse_x <= 0 or coarse_y <= 0:
        return BoundaryMeasurement(
            False, "boundary_contact_missing",
            left_count, right_count, top_count, bottom_count,
            coarse_y, coarse_x, 0, 0, 0.0,
            0.0, 0.0, 0.0, 1.0, -2.0,
        )
    ratio_x = coarse_x
    ratio_y = coarse_y
    divisor = math.gcd(ratio_y, ratio_x)
    ratio_y //= divisor
    ratio_x //= divisor

    pair_error = abs(left_count - right_count) + abs(top_count - bottom_count)
    total_count = max(1, left_count + right_count + top_count + bottom_count)
    confidence = float(np.clip(1.0 - pair_error / total_count, 0.0, 1.0))

    center_x = 0.5 * (left + right)
    center_y = 0.5 * (top + bottom)
    half_x = max(0.5 * (right - left), 1.0)
    half_y = max(0.5 * (bottom - top), 1.0)
    xn = (xs.astype(np.float64) - center_x) / half_x
    yn = (center_y - ys.astype(np.float64)) / half_y
    correlation = float(np.corrcoef(xn, yn)[0, 1])
    radius = np.sqrt(xn * xn + yn * yn)
    radial_variance = float(np.var(radius))
    span_x_div = divisions * (right - left) / float(width)
    span_y_div = divisions * (bottom - top) / float(height)
    circle_error = (
        abs(correlation)
        + abs(span_x_div - span_y_div) / float(divisions)
        + radial_variance
    )

    return BoundaryMeasurement(
        True, "", left_count, right_count, top_count, bottom_count,
        coarse_y, coarse_x,
        ratio_y, ratio_x, round(confidence, 4),
        round(span_x_div, 4), round(span_y_div, 4),
        round(correlation, 5), round(radial_variance, 5),
        round(-circle_error, 5),
    )


def candidate_input_grids(
    probe_grid: int,
    measurement: BoundaryMeasurement,
    *,
    tolerance: int = 1,
    minimum_grid: int = 10,
    maximum_grid: int = 1000,
    maximum_ratio: int = 20,
) -> set[int]:
    """Return 100 Hz input-grid candidates compatible with raw touch counts."""
    if not measurement.valid or probe_grid <= 0:
        return set()
    candidates: set[int] = set()
    for ratio_y in range(1, maximum_ratio + 1):
        if abs(ratio_y - measurement.coarse_y) > tolerance:
            continue
        for ratio_x in range(1, maximum_ratio + 1):
            if abs(ratio_x - measurement.coarse_x) > tolerance:
                continue
            if math.gcd(ratio_y, ratio_x) != 1:
                continue
            numerator = probe_grid * ratio_x
            if numerator % ratio_y:
                continue
            input_grid = numerator // ratio_y
            if minimum_grid <= input_grid <= maximum_grid:
                candidates.add(input_grid)
    return candidates


def confirm_dual_phase_candidate(
    probe_grid: int,
    first: BoundaryMeasurement,
    second: BoundaryMeasurement,
    *,
    tolerance: int = 1,
) -> int | None:
    """Return a candidate only when both phases report the same reduced ratio."""
    if not dual_phase_consistent(
        first, second, touch_tolerance=tolerance
    ):
        return None
    numerator = probe_grid * first.ratio_x
    if first.ratio_y <= 0 or numerator % first.ratio_y:
        return None
    input_grid = numerator // first.ratio_y
    return input_grid if 10 <= input_grid <= 1000 else None


def dual_phase_consistent(
    first: BoundaryMeasurement,
    second: BoundaryMeasurement,
    *,
    touch_tolerance: int = 1,
) -> bool:
    """Require equal reduced ratios and tolerate one raw contact per side."""
    if not first.valid or not second.valid:
        return False
    if (first.ratio_y, first.ratio_x) != (second.ratio_y, second.ratio_x):
        return False
    return all(
        abs(a - b) <= touch_tolerance
        for a, b in zip(
            (first.left, first.right, first.top, first.bottom),
            (second.left, second.right, second.top, second.bottom),
        )
    )
