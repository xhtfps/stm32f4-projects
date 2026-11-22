"""CanMV K230 requirement-5 Lissajous vision controller.

This is deliberately a single-file deployment.  Copy it to ``/sd/main.py``
on the K230.  The hardware imports are guarded so the frequency arithmetic,
shape scores and controller can also be imported by CPython regression tests.

The K230 is the master.  It never samples the unknown source electrically;
all information about the X signal comes from the yellow oscilloscope trace.
The F407 is only a DDS executor using the ASCII protocol documented below.
"""

import gc
import math
import os
import time


# CanMV-only modules are optional during host-side tests.
try:
    from machine import UART, FPIOA
    from media.sensor import Sensor
    from media.display import Display
    from media.media import MediaManager
    CANMV_AVAILABLE = True
except (ImportError, OSError):
    UART = None
    FPIOA = None
    Sensor = None
    Display = None
    MediaManager = None
    CANMV_AVAILABLE = False


# ---------------------------------------------------------------------------
# Camera, ROI and field-calibration constants
# ---------------------------------------------------------------------------

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# K230 V3.0 board display configuration.
# The camera frame is 640x480, while the onboard ST7701 panel is 800x480.
# Keep the two sizes separate: using 640x480 for ST7701 leaves the hardware
# display unselected/incorrectly configured on some CanMV firmware versions.
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DISPLAY_TARGET = "lcd"       # "lcd", "virt", or "auto"
USE_LCD = DISPLAY_TARGET == "lcd"
DISPLAY_FPS = 60              # Used by VIRT; ST7701 uses board defaults.
ROTATE_IMAGE_180 = True        # Camera is mounted upside down on the board.
ACTIVE_PREVIEW_INTERVAL_FRAMES = 3
LOG_INTERVAL_FRAMES = 90

# This is the grid rectangle from the supplied ``全部代码.txt`` camera setup.
# It must be checked once with the green overlay after moving the camera.
GRID_RECT = (89, 18, 456, 431)
GRID_DIVISIONS_X = 10.0
GRID_DIVISIONS_Y = 8.0

# LAB threshold inherited from the working yellow-trace implementation.
# CanMV LAB order is (Lmin, Lmax, Amin, Amax, Bmin, Bmax).
YELLOW_THRESHOLD = (40, 100, -45, 25, 15, 100)
TRACE_SAMPLE_STEP = 2
MIN_TRACE_POINTS = 120
MIN_TRACE_SPAN_DIV = 2.0
TRIM_FRACTION = 0.002

# Four edge bands are intentionally independent measurements.  A side is
# accepted only when at least three bands return the same peak count.
EDGE_BAND_RATIOS = (0.010, 0.015, 0.020, 0.025)
MULTISCALE_AGREE_COUNT = 3
PEAK_MIN_DISTANCE_PX = 8
MAX_TOUCH_COUNT = 5

# 471 mV is the last known command that produced about 8 divisions on the
# supplied AD9959/amplifier path.  These entries are calibration anchors, not
# assumptions about the unknown input.  Replace their command values with a
# bench measurement if the analog path changes; interpolation is automatic.
DDS_REFERENCE_SPAN_DIV = 8.0
DDS_AMPLITUDE_TABLE = (
    (10, 471),       # 1 kHz
    (50, 471),       # 5 kHz
    (100, 471),      # 10 kHz
    (200, 471),      # 20 kHz
    (500, 471),      # 50 kHz
    (1000, 471),     # 100 kHz
    (1500, 471),     # 150 kHz
    (2000, 471),     # 200 kHz
)
DDS_AMPLITUDE_MIN_MV = 20
DDS_AMPLITUDE_MAX_MV = 500


# ---------------------------------------------------------------------------
# UART2 and protocol constants
# ---------------------------------------------------------------------------

UART2_TX_PIN = 5
UART2_RX_PIN = 6
UART2_BAUDRATE = 230400
UART_ACK_TIMEOUT_MS = 80
UART_MAX_ACK_RETRIES = 1
DISCARD_FRAMES_AFTER_APPLIED = 1

CONTROL_TIMEOUT_MS = 10000
HOLD_REQUIRED_MS = 5000
LOCK_REQUIRED_FRAMES = 3
SEARCH_PHASE_DEG = 11
SECOND_PHASE_OFFSET_DEG = 37
THIRD_PHASE_OFFSET_DEG = 73

# The sign follows the specification's u_phase slope convention.  Verify it
# once on the bench by commanding DDS +1 Hz; change only this constant if the
# measured compensation direction is reversed.
FREQUENCY_TRIM_SIGN = 1.0
FREQUENCY_TRIM_LIMIT_HZ = 40.0
FREQUENCY_TRIM_STEP_HZ = 2.0
TRACK_UPDATE_MS = 150
FREQUENCY_TRIM_UPDATE_MS = 600


# ---------------------------------------------------------------------------
# Five-touch probe table (100 Hz grid units)
# ---------------------------------------------------------------------------

# ``5倍点数.md`` supplies the prime points and composite anchors for the
# low-density scheme.  Treating those 321 values as a strict four-side table
# leaves 210 legal inputs without a frame where both reduced counts are <= 5.
# The table below was completed offline with the missing composite anchors,
# redundant source points were removed, and the result was greedily reordered
# by uncovered-frequency gain.  Runtime code never generates probes from the
# unknown input and every accepted frame has at most five contacts on each
# side.
PROBE_TABLE = (
    300, 216, 165, 312, 42, 170, 190, 30, 384, 444, 492, 236,
    244, 336, 516, 552, 564, 588, 65, 87, 93, 115, 159, 176,
    134, 142, 146, 158, 166, 178, 194, 306, 462, 202, 206, 214,
    218, 226, 171, 175, 238, 242, 546, 32, 254, 262, 274, 278,
    298, 302, 314, 326, 243, 266, 286, 322, 378, 396, 696, 744,
    169, 185, 187, 334, 346, 358, 362, 382, 386, 394, 398, 82,
    86, 94, 203, 209, 217, 221, 247, 422, 446, 454, 458, 466,
    478, 482, 98, 234, 250, 456, 27, 145, 155, 253, 259, 287,
    289, 299, 301, 319, 323, 329, 502, 514, 526, 538, 542, 554,
    562, 566, 586, 614, 622, 626, 634, 662, 408, 480, 636, 22,
    34, 77, 91, 177, 183, 201, 213, 219, 237, 249, 341, 361,
    371, 377, 391, 403, 407, 413, 427, 437, 451, 469, 473, 481,
    493, 497, 674, 694, 698, 706, 718, 734, 746, 758, 766, 778,
    794, 802, 818, 838, 842, 862, 866, 878, 886, 898, 914, 922,
    926, 934, 958, 974, 982, 998, 207, 224, 325, 343, 10, 38,
    63, 106, 267, 291, 303, 309, 321, 327, 503, 509, 511, 517,
    521, 523, 527, 529, 533, 541, 547, 551, 553, 557, 559, 563,
    569, 571, 577, 581, 583, 587, 589, 593, 599, 601, 607, 611,
    613, 617, 619, 623, 629, 631, 641, 643, 647, 649, 653, 659,
    661, 667, 673, 677, 679, 683, 689, 691, 697, 701, 703, 707,
    709, 713, 719, 727, 733, 739, 743, 749, 751, 757, 761, 769,
    773, 787, 797, 809, 811, 821, 823, 827, 829, 839, 853, 857,
    859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941,
    947, 953, 967, 971, 977, 983, 991, 997, 135, 208, 261, 279,
    297, 324, 333, 440, 539, 637, 671, 721, 731, 737, 763, 767,
    779, 781, 791, 793, 799, 803, 817, 833, 841, 847, 851, 869,
    871, 889, 893, 899, 901, 913, 917, 923, 931, 943, 949, 959,
    961, 973, 979, 989,
)
SUPPORTED_INPUT_GRIDS = tuple(range(10, 1001))

if len(PROBE_TABLE) != 328 or len(set(PROBE_TABLE)) != 328:
    raise RuntimeError("fixed five-touch probe table must have 328 entries")


# ---------------------------------------------------------------------------
# Small MicroPython-safe helpers
# ---------------------------------------------------------------------------

def ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.monotonic() * 1000.0)


def ticks_diff(newer, older):
    try:
        return time.ticks_diff(int(newer), int(older))
    except AttributeError:
        return int(newer) - int(older)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def integer_gcd(left, right):
    left = abs(int(left))
    right = abs(int(right))
    while right:
        left, right = right, left % right
    return left


def median(values):
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) & 1:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = int((len(ordered) - 1) * float(fraction) + 0.5)
    index = int(clamp(index, 0, len(ordered) - 1))
    return ordered[index]


def wrap_phase_deg(value):
    return float(value) % 360.0


def wrap_phase_millideg(value):
    return int(value) % 360000


def phase_delta_deg(new_phase, old_phase):
    """Shortest signed delta from old_phase to new_phase in [-180, 180)."""
    return (float(new_phase) - float(old_phase) + 180.0) % 360.0 - 180.0


def unwrap_phase_degrees(values):
    if not values:
        return []
    result = [float(values[0])]
    previous_wrapped = float(values[0])
    for value in values[1:]:
        wrapped = float(value)
        result.append(result[-1] + phase_delta_deg(wrapped, previous_wrapped))
        previous_wrapped = wrapped
    return result


def phase_drift_frequency_trim_hz(history):
    """Fit phase command slope and return a signed, per-update Hz trim.

    ``history`` is ``[(timestamp_ms, wrapped_phase_deg), ...]``.  At least
    0.5 s of data is required, preventing a one-frame phase fluctuation from
    changing frequency.
    """
    if history is None or len(history) < 3:
        return 0.0
    if ticks_diff(history[-1][0], history[0][0]) < 500:
        return 0.0
    times = []
    wrapped = []
    start_ms = history[0][0]
    for stamp, phase in history:
        times.append(ticks_diff(stamp, start_ms) / 1000.0)
        wrapped.append(float(phase))
    phases = unwrap_phase_degrees(wrapped)
    mean_t = sum(times) / len(times)
    mean_p = sum(phases) / len(phases)
    numerator = 0.0
    denominator = 0.0
    for index in range(len(times)):
        dt = times[index] - mean_t
        numerator += dt * (phases[index] - mean_p)
        denominator += dt * dt
    if denominator <= 1.0e-9:
        return 0.0
    slope_deg_per_s = numerator / denominator
    trim = FREQUENCY_TRIM_SIGN * slope_deg_per_s / 360.0
    return clamp(trim, -FREQUENCY_TRIM_STEP_HZ, FREQUENCY_TRIM_STEP_HZ)


def reduced_ratio(count_x, count_y):
    count_x = int(count_x)
    count_y = int(count_y)
    if count_x <= 0 or count_y <= 0:
        return (0, 0)
    divisor = integer_gcd(count_x, count_y)
    return (count_x // divisor, count_y // divisor)


def strict_ratio_from_counts(left, right, top, bottom):
    """Return (valid, reason, ratio_y, ratio_x) without count repair."""
    left = int(left)
    right = int(right)
    top = int(top)
    bottom = int(bottom)
    if left <= 0 or top <= 0:
        return (False, "NO_CONTACT", 0, 0)
    if left != right:
        return (False, "LEFT_RIGHT_MISMATCH", 0, 0)
    if top != bottom:
        return (False, "TOP_BOTTOM_MISMATCH", 0, 0)
    if left > MAX_TOUCH_COUNT or top > MAX_TOUCH_COUNT:
        return (False, "TOUCH_COUNT_OVERFLOW", 0, 0)
    ratio_x, ratio_y = reduced_ratio(left, top)
    return (True, None, ratio_y, ratio_x)


def exact_frequency_candidate(probe_grid, left, right, top, bottom):
    """Infer fx in 100 Hz units from one strictly paired frame."""
    valid, unused, ratio_y, ratio_x = strict_ratio_from_counts(
        left, right, top, bottom
    )
    if not valid or ratio_x <= 0 or ratio_y <= 0:
        return None
    numerator = int(probe_grid) * ratio_x
    if numerator % ratio_y:
        return None
    candidate = numerator // ratio_y
    if candidate < 10 or candidate > 1000:
        return None
    return candidate


def candidate_input_grids(probe_grid, left, right, top, bottom):
    """Compatibility helper: a strict frame has zero or one exact candidate."""
    candidate = exact_frequency_candidate(
        probe_grid, left, right, top, bottom
    )
    return set() if candidate is None else {candidate}


def expected_touch_counts(input_grid, probe_grid):
    divisor = integer_gcd(input_grid, probe_grid)
    if divisor <= 0:
        return (0, 0)
    return (int(input_grid) // divisor, int(probe_grid) // divisor)


def choose_second_probe(input_grid, first_probe_grid, preferred_max_count=10):
    """Choose a distinct, always-low-density cross-check frequency.

    The 328-entry table is fixed for *search*.  Once a candidate ``fx`` has
    been inferred optically, verifying at ``fy=fx`` (or ``2*fx`` when the
    first probe already equals ``fx``) is stronger and keeps both predicted
    counts at 1 or 2.  Restricting this step to the search table makes an
    independent <=10-touch verification mathematically impossible for large
    prime input grids such as 997.
    """
    input_grid = int(input_grid)
    first_probe_grid = int(first_probe_grid)
    if input_grid < 10 or input_grid > 1000:
        return None
    candidates = (input_grid, 2 * input_grid)
    ranked = []
    for probe in candidates:
        if probe == first_probe_grid or probe < 1 or probe > 2000:
            continue
        count_x, count_y = expected_touch_counts(input_grid, probe)
        if max(count_x, count_y) <= int(preferred_max_count):
            ranked.append((max(count_x, count_y), count_x + count_y, probe))
    if not ranked:
        return None
    ranked.sort()
    return ranked[0][2]


def probe_table_coverage(max_touch_count=MAX_TOUCH_COUNT):
    """Return legal fx grids not covered by any fixed search probe."""
    missing = []
    for input_grid in SUPPORTED_INPUT_GRIDS:
        covered = False
        for probe_grid in PROBE_TABLE:
            count_x, count_y = expected_touch_counts(input_grid, probe_grid)
            if max(count_x, count_y) <= int(max_touch_count):
                covered = True
                break
        if not covered:
            missing.append(input_grid)
    return missing


def amplitude_feedforward(frequency_grid, target_span_div):
    """Interpolate the calibrated command and scale it for requested span."""
    frequency_grid = int(frequency_grid)
    if frequency_grid <= DDS_AMPLITUDE_TABLE[0][0]:
        reference_mv = float(DDS_AMPLITUDE_TABLE[0][1])
    elif frequency_grid >= DDS_AMPLITUDE_TABLE[-1][0]:
        reference_mv = float(DDS_AMPLITUDE_TABLE[-1][1])
    else:
        reference_mv = float(DDS_AMPLITUDE_TABLE[-1][1])
        for index in range(1, len(DDS_AMPLITUDE_TABLE)):
            low_grid, low_mv = DDS_AMPLITUDE_TABLE[index - 1]
            high_grid, high_mv = DDS_AMPLITUDE_TABLE[index]
            if frequency_grid <= high_grid:
                fraction = (
                    float(frequency_grid - low_grid)
                    / float(high_grid - low_grid)
                )
                reference_mv = low_mv + fraction * (high_mv - low_mv)
                break
    command = reference_mv * float(target_span_div) / DDS_REFERENCE_SPAN_DIV
    return int(clamp(round(command), DDS_AMPLITUDE_MIN_MV,
                     DDS_AMPLITUDE_MAX_MV))


# ---------------------------------------------------------------------------
# Real, phase-sensitive shape scores
# ---------------------------------------------------------------------------

def _point_moments(points):
    if points is None or len(points) < 3:
        return None
    mean_x = sum(point[0] for point in points) / float(len(points))
    mean_y = sum(point[1] for point in points) / float(len(points))
    cxx = 0.0
    cyy = 0.0
    cxy = 0.0
    for x, y in points:
        dx = x - mean_x
        dy = y - mean_y
        cxx += dx * dx
        cyy += dy * dy
        cxy += dx * dy
    scale = float(len(points))
    cxx /= scale
    cyy /= scale
    cxy /= scale
    root = math.sqrt(max(0.0, (cxx - cyy) * (cxx - cyy)
                             + 4.0 * cxy * cxy))
    large = 0.5 * (cxx + cyy + root)
    small = max(0.0, 0.5 * (cxx + cyy - root))
    theta = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)
    return (mean_x, mean_y, cxx, cyy, cxy, large, small, theta)


def diagonal_shape_metrics(points):
    """PCA/TLS line error in division coordinates; either slope is valid."""
    moments = _point_moments(points)
    if moments is None:
        return {"error": 999.0, "score": 0.0, "pass": False}
    mean_x, mean_y, cxx, cyy, cxy, large, small, theta = moments
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    eigen_ratio = small / max(large, 1.0e-9)
    orthogonal_rms = math.sqrt(small)
    ux = math.cos(theta)
    uy = math.sin(theta)
    # Correct TLS corner pair is selected from the covariance sign.
    if cxy >= 0.0:
        corners = ((min(xs), min(ys)), (max(xs), max(ys)))
    else:
        corners = ((min(xs), max(ys)), (max(xs), min(ys)))
    corner_error = 0.0
    for corner_x, corner_y in corners:
        dx = corner_x - mean_x
        dy = corner_y - mean_y
        corner_error += abs(-uy * dx + ux * dy)
    corner_error *= 0.5
    span_error = abs(span_x - span_y)
    error = (
        4.0 * eigen_ratio + 2.0 * orthogonal_rms
        + 0.5 * corner_error + 0.5 * span_error
    )
    passed = bool(
        eigen_ratio <= 0.03 and orthogonal_rms <= 0.12
        and span_error <= 0.12
    )
    return {
        "error": error,
        "score": 1.0 / (1.0 + error),
        "pass": passed,
        "eigen_ratio": eigen_ratio,
        "orthogonal_rms_div": orthogonal_rms,
        "corner_error_div": corner_error,
    }


def circle_shape_metrics(points, scope_center=(0.0, 0.0)):
    """Actual normalized radial CV plus correlation/span/center errors."""
    moments = _point_moments(points)
    if moments is None:
        return {"error": 999.0, "score": 0.0, "pass": False,
                "radial_cv": 999.0}
    mean_x, mean_y, cxx, cyy, cxy, large, small, theta = moments
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    low_x = percentile(xs, 0.01)
    high_x = percentile(xs, 0.99)
    low_y = percentile(ys, 0.01)
    high_y = percentile(ys, 0.99)
    span_x = max(1.0e-6, high_x - low_x)
    span_y = max(1.0e-6, high_y - low_y)
    center_x = 0.5 * (low_x + high_x)
    center_y = 0.5 * (low_y + high_y)
    radii = []
    for x, y in points:
        nx = (x - center_x) / (0.5 * span_x)
        ny = (y - center_y) / (0.5 * span_y)
        radii.append(math.sqrt(nx * nx + ny * ny))
    radius_mean = sum(radii) / max(1, len(radii))
    radius_variance = 0.0
    for radius in radii:
        radius_variance += (radius - radius_mean) * (radius - radius_mean)
    radius_variance /= max(1, len(radii))
    radial_cv = math.sqrt(radius_variance) / max(radius_mean, 1.0e-9)
    correlation = cxy / math.sqrt(max(cxx * cyy, 1.0e-12))
    span_difference = abs(span_x - span_y)
    span_relative = span_difference / max(span_x, 1.0e-9)
    center_offset = math.sqrt(
        (center_x - scope_center[0]) ** 2
        + (center_y - scope_center[1]) ** 2
    )
    error = (
        2.0 * abs(correlation) + 2.0 * span_relative
        + 4.0 * radial_cv + center_offset
    )
    passed = bool(
        abs(correlation) <= 0.08 and span_difference <= 0.12
        and radial_cv <= 0.06 and center_offset <= 0.15
    )
    return {
        "error": error,
        "score": 1.0 / (1.0 + error),
        "pass": passed,
        "correlation": correlation,
        "radial_cv": radial_cv,
        "span_difference_div": span_difference,
        "center_offset_div": center_offset,
    }


def _expanded_cells(cells, bins):
    expanded = set()
    for cell in cells:
        x = cell % bins
        y = cell // bins
        for dy in (-1, 0, 1):
            ny = y + dy
            if ny < 0 or ny >= bins:
                continue
            for dx in (-1, 0, 1):
                nx = x + dx
                if 0 <= nx < bins:
                    expanded.add(ny * bins + nx)
    return expanded


def _set_iou(first, second):
    if not first and not second:
        return 1.0
    union = first | second
    if not union:
        return 0.0
    return len(first & second) / float(len(union))


def infinity_shape_metrics(points, bins=64, scope_center=(0.0, 0.0)):
    """Occupancy mirror symmetry, centre crossing and lobe balance score."""
    if points is None or len(points) < 20:
        return {"error": 999.0, "score": 0.0, "pass": False}
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    low_x = percentile(xs, 0.01)
    high_x = percentile(xs, 0.99)
    low_y = percentile(ys, 0.01)
    high_y = percentile(ys, 0.99)
    span_x = max(1.0e-6, high_x - low_x)
    span_y = max(1.0e-6, high_y - low_y)
    center_x = 0.5 * (low_x + high_x)
    center_y = 0.5 * (low_y + high_y)
    cells = set()
    left_points = 0
    right_points = 0
    center_cross_distance = 999.0
    for x, y in points:
        bx = int(clamp(round((x - low_x) * (bins - 1) / span_x),
                       0, bins - 1))
        by = int(clamp(round((y - low_y) * (bins - 1) / span_y),
                       0, bins - 1))
        cells.add(by * bins + bx)
        if x < center_x:
            left_points += 1
        elif x > center_x:
            right_points += 1
        distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
        if distance < center_cross_distance:
            center_cross_distance = distance
    cells = _expanded_cells(cells, bins)
    mirror_lr = set()
    mirror_tb = set()
    for cell in cells:
        x = cell % bins
        y = cell // bins
        mirror_lr.add(y * bins + (bins - 1 - x))
        mirror_tb.add((bins - 1 - y) * bins + x)
    iou_lr = _set_iou(cells, mirror_lr)
    iou_tb = _set_iou(cells, mirror_tb)
    lobe_balance = abs(left_points - right_points) / float(
        max(1, left_points + right_points)
    )
    center_offset = math.sqrt(
        (center_x - scope_center[0]) ** 2
        + (center_y - scope_center[1]) ** 2
    )
    span_difference = abs(span_x - span_y)
    error = (
        3.0 * center_cross_distance
        + 2.0 * (1.0 - iou_lr)
        + 2.0 * (1.0 - iou_tb)
        + 2.0 * lobe_balance + center_offset
    )
    passed = bool(
        center_cross_distance <= 0.15
        and iou_lr >= 0.90 and iou_tb >= 0.90
        and lobe_balance <= 0.08 and span_difference <= 0.12
        and center_offset <= 0.15
    )
    return {
        "error": error,
        "score": 1.0 / (1.0 + error),
        "pass": passed,
        "center_cross_distance_div": center_cross_distance,
        "mirror_iou_lr": iou_lr,
        "mirror_iou_tb": iou_tb,
        "lobe_area_imbalance": lobe_balance,
        "center_offset_div": center_offset,
        "span_difference_div": span_difference,
    }


# ---------------------------------------------------------------------------
# Yellow trace extraction and strict four-side multiscale counting
# ---------------------------------------------------------------------------

def pixel_is_white(pixel):
    """Accept CanMV binary pixels in integer, RGB565 or RGB tuple form."""
    if isinstance(pixel, tuple):
        if len(pixel) >= 3:
            return pixel[0] + pixel[1] + pixel[2] > 300
        return bool(pixel[0])
    return pixel != 0


def collect_yellow_points(img, roi=GRID_RECT):
    """Use the supplied script's native LAB-copy/binary camera path."""
    roi_x, roi_y, roi_w, roi_h = roi
    mask = None
    points = []
    try:
        mask = img.copy(roi=roi)
        mask.binary([YELLOW_THRESHOLD])
        for y in range(0, roi_h, TRACE_SAMPLE_STEP):
            for x in range(0, roi_w, TRACE_SAMPLE_STEP):
                if pixel_is_white(mask.get_pixel(x, y)):
                    points.append((x, y))
    except Exception as error:
        print("yellow mask error: %s" % error)
        points = []
    mask = None
    return points


def robust_trace_bounds(points):
    if points is None or len(points) < MIN_TRACE_POINTS:
        return None
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    low_x = percentile(xs, TRIM_FRACTION)
    high_x = percentile(xs, 1.0 - TRIM_FRACTION)
    low_y = percentile(ys, TRIM_FRACTION)
    high_y = percentile(ys, 1.0 - TRIM_FRACTION)
    if high_x <= low_x or high_y < low_y:
        return None
    return (int(low_x), int(high_x), int(low_y), int(high_y))


def _median_filter(values, radius=1):
    result = [0.0] * len(values)
    for index in range(len(values)):
        low = max(0, index - radius)
        high = min(len(values), index + radius + 1)
        result[index] = float(median(values[low:high]))
    return result


def _smooth_projection(values):
    filtered = _median_filter(values, 1)
    result = [0.0] * len(filtered)
    for index in range(len(filtered)):
        low = max(0, index - 2)
        high = min(len(filtered), index + 3)
        result[index] = sum(filtered[low:high]) / float(high - low)
    return result


def projection_peak_positions(projection, min_distance=PEAK_MIN_DISTANCE_PX):
    """Find one prominent run per boundary contact.

    A thick oscilloscope trace often produces two nearby local maxima around
    one true tangent.  Counting local maxima therefore splits an ideal circle
    into several false contacts.  Thresholded runs preserve the requested
    prominence gate while merging sub-pixel/sampling gaps inside one tangent.
    """
    if projection is None or len(projection) < 3:
        return []
    values = _smooth_projection(projection)
    positive = [value for value in values if value > 0.0]
    if not positive:
        return []
    p95 = percentile(positive, 0.95)
    threshold = max(0.25, 0.12 * float(p95))
    merge_gap = max(2, int(TRACE_SAMPLE_STEP) + 2)
    runs = []
    start = None
    previous = None
    for index in range(len(values)):
        if values[index] < threshold:
            continue
        if start is None:
            start = index
        elif index - previous > merge_gap:
            runs.append((start, previous))
            start = index
        previous = index
    if start is not None:
        runs.append((start, previous))

    candidates = []
    for start, end in runs:
        weight = 0.0
        weighted_position = 0.0
        height = 0.0
        for index in range(start, end + 1):
            value = values[index]
            weight += value
            weighted_position += index * value
            height = max(height, value)
        if weight <= 0.0:
            continue
        position = int(weighted_position / weight + 0.5)
        candidates.append((height, position))

    # Contacts closer than the optical resolution are deliberately merged;
    # dual-phase and second-probe confirmation then prevent a false frequency.
    candidates.sort(reverse=True)
    accepted = []
    for height, position in candidates:
        separated = True
        for other in accepted:
            if abs(position - other) < int(min_distance):
                separated = False
                break
        if separated:
            accepted.append(position)
    accepted.sort()
    return accepted


def _projection_for_side(points, bounds, side, band_ratio):
    low_x, high_x, low_y, high_y = bounds
    span_x = max(1, high_x - low_x + 1)
    span_y = max(1, high_y - low_y + 1)
    if side in ("left", "right"):
        projection = [0] * (high_y + 1)
        band = max(3, int(round(span_x * float(band_ratio))))
        if side == "left":
            limit_low = low_x
            limit_high = low_x + band
        else:
            limit_low = high_x - band
            limit_high = high_x
        for x, y in points:
            if limit_low <= x <= limit_high and low_y <= y <= high_y:
                projection[y] += 1
        return projection
    projection = [0] * (high_x + 1)
    band = max(3, int(round(span_y * float(band_ratio))))
    if side == "top":
        limit_low = low_y
        limit_high = low_y + band
    else:
        limit_low = high_y - band
        limit_high = high_y
    for x, y in points:
        if limit_low <= y <= limit_high and low_x <= x <= high_x:
            projection[x] += 1
    return projection


def multiscale_side_count(points, bounds, side):
    counts = []
    positions_by_scale = []
    for ratio in EDGE_BAND_RATIOS:
        projection = _projection_for_side(points, bounds, side, ratio)
        positions = projection_peak_positions(projection)
        counts.append(len(positions))
        positions_by_scale.append(positions)
    best_count = 0
    best_votes = 0
    for count in counts:
        votes = counts.count(count)
        if votes > best_votes or (votes == best_votes and count < best_count):
            best_count = count
            best_votes = votes
    valid = bool(
        best_votes >= MULTISCALE_AGREE_COUNT
        and 0 < best_count <= MAX_TOUCH_COUNT
    )
    return {
        "valid": valid,
        "count": best_count if valid else 0,
        "scale_counts": tuple(counts),
        "positions": tuple(positions_by_scale),
        "reason": None if valid else "PEAK_MERGED",
    }


def points_to_divisions(points, roi_width, roi_height):
    pixels_per_div_x = float(roi_width) / GRID_DIVISIONS_X
    pixels_per_div_y = float(roi_height) / GRID_DIVISIONS_Y
    center_x = 0.5 * (roi_width - 1)
    center_y = 0.5 * (roi_height - 1)
    converted = []
    for x, y in points:
        converted.append((
            (x - center_x) / pixels_per_div_x,
            (center_y - y) / pixels_per_div_y,
        ))
    return converted


def empty_measurement(reason="NO_TRACE"):
    return {
        "valid": False,
        "strict_pair_valid": False,
        "invalid_reason": reason,
        "left": 0,
        "right": 0,
        "top": 0,
        "bottom": 0,
        "ratio_x": 0,
        "ratio_y": 0,
        "trace_pixels": 0,
        "span_x_div": 0.0,
        "span_y_div": 0.0,
        "center_x_div": 0.0,
        "center_y_div": 0.0,
        "bounds_div": None,
        "score_diagonal": 0.0,
        "score_circle": 0.0,
        "score_infinity": 0.0,
        "error_diagonal": 999.0,
        "error_circle": 999.0,
        "error_infinity": 999.0,
        "diagonal_pass": False,
        "circle_pass": False,
        "infinity_pass": False,
        "shape": "unknown",
        "shape_confidence": 0.0,
    }


def analyze_trace_points(points, roi_width=None, roi_height=None):
    """Pure trace analysis used by both CanMV frames and CPython tests."""
    if roi_width is None:
        roi_width = GRID_RECT[2]
    if roi_height is None:
        roi_height = GRID_RECT[3]
    result = empty_measurement()
    if points is None or len(points) < MIN_TRACE_POINTS:
        return result
    result["trace_pixels"] = len(points) * TRACE_SAMPLE_STEP * TRACE_SAMPLE_STEP
    bounds = robust_trace_bounds(points)
    if bounds is None:
        result["invalid_reason"] = "NO_TRACE"
        return result
    low_x, high_x, low_y, high_y = bounds
    px_per_div_x = float(roi_width) / GRID_DIVISIONS_X
    px_per_div_y = float(roi_height) / GRID_DIVISIONS_Y
    span_x_div = (high_x - low_x) / px_per_div_x
    span_y_div = (high_y - low_y) / px_per_div_y
    center_x_div = (
        0.5 * (low_x + high_x) - 0.5 * (roi_width - 1)
    ) / px_per_div_x
    center_y_div = (
        0.5 * (roi_height - 1) - 0.5 * (low_y + high_y)
    ) / px_per_div_y
    bounds_div = (
        (low_x - 0.5 * (roi_width - 1)) / px_per_div_x,
        (high_x - 0.5 * (roi_width - 1)) / px_per_div_x,
        (0.5 * (roi_height - 1) - high_y) / px_per_div_y,
        (0.5 * (roi_height - 1) - low_y) / px_per_div_y,
    )
    result.update({
        "span_x_div": span_x_div,
        "span_y_div": span_y_div,
        "center_x_div": center_x_div,
        "center_y_div": center_y_div,
        "bounds_div": bounds_div,
    })

    # Horizontal X-only precheck intentionally has very little Y span.  It
    # still returns its robust X span, but it is not a search-valid trace.
    if span_x_div < MIN_TRACE_SPAN_DIV or span_y_div < MIN_TRACE_SPAN_DIV:
        result["invalid_reason"] = "TRACE_SPAN_TOO_SMALL"
    else:
        sides = {}
        for side in ("left", "right", "top", "bottom"):
            sides[side] = multiscale_side_count(points, bounds, side)
            result[side + "_scale_counts"] = sides[side]["scale_counts"]
        if not all(sides[side]["valid"] for side in sides):
            result["invalid_reason"] = "PEAK_MERGED"
        else:
            left = sides["left"]["count"]
            right = sides["right"]["count"]
            top = sides["top"]["count"]
            bottom = sides["bottom"]["count"]
            valid, reason, ratio_y, ratio_x = strict_ratio_from_counts(
                left, right, top, bottom
            )
            result.update({
                "valid": valid,
                "strict_pair_valid": valid,
                "invalid_reason": reason,
                "left": left,
                "right": right,
                "top": top,
                "bottom": bottom,
                "ratio_x": ratio_x,
                "ratio_y": ratio_y,
            })

    div_points = points_to_divisions(points, roi_width, roi_height)
    diagonal = diagonal_shape_metrics(div_points)
    circle = circle_shape_metrics(div_points)
    infinity = infinity_shape_metrics(div_points)
    result.update({
        "score_diagonal": diagonal["score"],
        "score_circle": circle["score"],
        "score_infinity": infinity["score"],
        "error_diagonal": diagonal["error"],
        "error_circle": circle["error"],
        "error_infinity": infinity["error"],
        "diagonal_pass": diagonal["pass"],
        "circle_pass": circle["pass"],
        "infinity_pass": infinity["pass"],
        "radial_cv": circle.get("radial_cv", 999.0),
        "correlation": circle.get("correlation", 0.0),
        "mirror_iou_lr": infinity.get("mirror_iou_lr", 0.0),
        "mirror_iou_tb": infinity.get("mirror_iou_tb", 0.0),
        "center_cross_distance_div": infinity.get(
            "center_cross_distance_div", 999.0
        ),
        "lobe_area_imbalance": infinity.get(
            "lobe_area_imbalance", 1.0
        ),
    })
    candidates = (
        (diagonal["score"], "diagonal", diagonal["pass"]),
        (circle["score"], "ellipse", circle["pass"]),
        (infinity["score"], "infinity", infinity["pass"]),
    )
    best = max(candidates)
    result["shape"] = best[1]
    result["shape_confidence"] = best[0]
    return result


def measure_image(img):
    points = collect_yellow_points(img, GRID_RECT)
    measurement = analyze_trace_points(points, GRID_RECT[2], GRID_RECT[3])
    points = None
    return measurement


def shape_error_for_target(measurement, target_shape):
    if not measurement:
        return 999.0
    key = {
        1: "error_diagonal",
        2: "error_circle",
        3: "error_infinity",
    }[int(target_shape)]
    if key in measurement:
        return float(measurement.get(key, 999.0))
    score_key = {
        1: "score_diagonal",
        2: "score_circle",
        3: "score_infinity",
    }[int(target_shape)]
    return max(0.0, 1.0 - float(measurement.get(score_key, 0.0)))


def target_ratio_ok(measurement, target_shape):
    if not measurement or not measurement.get("strict_pair_valid",
                                               measurement.get("valid", False)):
        return False
    ratio_x = int(measurement.get("ratio_x", 0))
    ratio_y = int(measurement.get("ratio_y", 0))
    if int(target_shape) in (1, 2):
        return ratio_x == 1 and ratio_y == 1
    return ratio_x == 1 and ratio_y == 2


def target_shape_ok(measurement, target_shape):
    if not measurement:
        return False
    explicit_key = {
        1: "diagonal_pass",
        2: "circle_pass",
        3: "infinity_pass",
    }[int(target_shape)]
    if explicit_key in measurement:
        return bool(measurement.get(explicit_key))
    expected_name = {1: "diagonal", 2: "ellipse", 3: "infinity"}[
        int(target_shape)
    ]
    return bool(
        measurement.get("shape") == expected_name
        and float(measurement.get("shape_confidence", 0.0)) >= 0.75
    )


def bounds_drift_div(current, reference):
    if current is None or reference is None or len(current) != 4:
        return 999.0
    return max(abs(float(current[index]) - float(reference[index]))
               for index in range(4))


# ---------------------------------------------------------------------------
# Event objects and complete requirement-5 controller
# ---------------------------------------------------------------------------

class LockedEvent:
    def __init__(self, sequence, input_frequency_millihz):
        self.sequence = int(sequence)
        self.input_frequency_millihz = int(input_frequency_millihz)


class DoneEvent:
    def __init__(self, sequence, input_frequency_millihz):
        self.sequence = int(sequence)
        self.input_frequency_millihz = int(input_frequency_millihz)


class FailedEvent:
    def __init__(self, sequence, error_code, reason):
        self.sequence = int(sequence)
        self.error_code = int(error_code)
        self.reason = str(reason)


class Requirement5Controller:
    WAIT_START = "WAIT_START"
    MEASURE_X_AMPLITUDE = "MEASURE_X_AMPLITUDE"
    SEARCH = "SEARCH"
    DUAL_PHASE = "DUAL_PHASE"
    DUAL_PHASE_RETRY = "DUAL_PHASE_RETRY"
    SECOND_PROBE = "SECOND_PROBE"
    PHASE_COARSE = "PHASE_COARSE"
    PHASE_FINE = "PHASE_FINE"
    AMPLITUDE_FINE = "AMPLITUDE_FINE"
    LOCK_TRACK = "LOCK_TRACK"
    HOLD = "HOLD"
    DONE = "DONE"
    FAILED = "FAILED"

    ERROR_UART_TIMEOUT = 1
    ERROR_CONTROL_TIMEOUT = 2
    ERROR_BAD_SHAPE = 3
    ERROR_FREQUENCY_NOT_FOUND = 4
    ERROR_AMPLITUDE = 5
    ERROR_SHAPE_NOT_FOUND = 6
    ERROR_F407 = 10

    def __init__(self, hold_ms=HOLD_REQUIRED_MS,
                 control_timeout_ms=CONTROL_TIMEOUT_MS,
                 ack_timeout_ms=UART_ACK_TIMEOUT_MS,
                 max_ack_retries=UART_MAX_ACK_RETRIES,
                 verify_frames=LOCK_REQUIRED_FRAMES,
                 track_update_ms=TRACK_UPDATE_MS):
        self.hold_ms = int(hold_ms)
        self.control_timeout_ms = int(control_timeout_ms)
        self.ack_timeout_ms = int(ack_timeout_ms)
        self.max_ack_retries = int(max_ack_retries)
        self.verify_frames = int(verify_frames)
        self.track_update_ms = int(track_update_ms)
        self.sequence = 0
        self._reset_cycle()

    def _reset_cycle(self):
        self.state = self.WAIT_START
        self.target_shape = 0
        self.started_ms = None
        self.last_command = None
        self.waiting_for_applied = False
        self.ack_started_ms = None
        self.ack_retries = 0
        self.applied_ms = None
        self.probe_cursor = 0
        self.current_probe_grid = 0
        self.current_probe_id = 0
        self.first_counts = None
        self.first_ratio = None
        self.input_grid = 0
        self.second_probe_grid = 0
        self.second_expected_counts = (0, 0)
        self.control_probe_id = 0x7FFF
        self.target_span_div = 0.0
        self.x_span_samples = []
        self.nominal_grid = 0
        self.nominal_frequency_millihz = 0
        self.locked_frequency_millihz = 0
        self.phase_deg = 0.0
        self.amplitude_mv = amplitude_feedforward(100, 8.0)
        self.coarse_phase_index = 0
        self.phase_samples = []
        self.best_phase_deg = 0.0
        self.best_shape_error = 999.0
        self.best_measurement = None
        self.fine_steps = (10, 5, 2)
        self.fine_step_index = 0
        self.fine_candidates = []
        self.fine_candidate_index = 0
        self.amplitude_samples = []
        self.amplitude_corrections = 0
        self.amplitude_invalid_frames = 0
        self.lock_frames = 0
        self.hold_since_ms = None
        self.lock_reference_bounds = None
        self.previous_bounds = None
        self.hard_fail_frames = 0
        self.last_track_ms = None
        self.last_trim_ms = None
        self.last_slow_amplitude_ms = None
        self.track_direction = 1
        self.track_worse_frames = 0
        self.track_best_error = 999.0
        self.track_error_samples = []
        self.track_previous_phase_error = None
        self.phase_history = []
        self.phase_moves = []

    def _next_sequence(self):
        self.sequence = (self.sequence + 1) & 0xFFFF
        if self.sequence == 0:
            self.sequence = 1
        return self.sequence

    def _next_control_probe_id(self):
        self.control_probe_id += 1
        if self.control_probe_id > 0xFFFF:
            self.control_probe_id = 0x8000
        return self.control_probe_id

    def _issue_millihz(self, frequency_millihz, phase_deg, amplitude_mv,
                       probe_id, now_ms):
        frequency_millihz = int(clamp(
            round(frequency_millihz), 1000, 200000000
        ))
        self.phase_deg = wrap_phase_deg(phase_deg)
        self.amplitude_mv = int(clamp(
            round(amplitude_mv), 0, DDS_AMPLITUDE_MAX_MV
        ))
        self.last_command = {
            "kind": "SET",
            "sequence": self._next_sequence(),
            "probe_id": int(probe_id),
            "frequency_grid": int(round(frequency_millihz / 100000.0)),
            "frequency_millihz": frequency_millihz,
            "phase_millideg": wrap_phase_millideg(
                round(self.phase_deg * 1000.0)
            ),
            "amplitude_mv": self.amplitude_mv,
            "shape": self.target_shape,
        }
        self.waiting_for_applied = True
        self.ack_started_ms = int(now_ms)
        self.ack_retries = 0
        self.applied_ms = None
        return self.last_command

    def _issue_grid(self, frequency_grid, phase_deg, amplitude_mv,
                    probe_id, now_ms):
        return self._issue_millihz(
            int(frequency_grid) * 100000, phase_deg, amplitude_mv,
            probe_id, now_ms
        )

    def start(self, target_shape, now_ms):
        if int(target_shape) not in (1, 2, 3):
            return self._fail(
                self.ERROR_BAD_SHAPE, "target shape must be 1, 2, or 3"
            )
        saved_sequence = self.sequence
        self._reset_cycle()
        self.sequence = saved_sequence
        self.target_shape = int(target_shape)
        self.started_ms = int(now_ms)
        self.state = self.MEASURE_X_AMPLITUDE
        # Zero DDS amplitude leaves the unknown X trace as a horizontal line.
        return self._issue_grid(
            10, 0.0, 0, self._next_control_probe_id(), now_ms
        )

    def on_applied(self, sequence, probe_id, now_ms):
        if (
            not self.waiting_for_applied or self.last_command is None
            or int(sequence) != self.last_command["sequence"]
            or int(probe_id) != self.last_command["probe_id"]
        ):
            return False
        self.waiting_for_applied = False
        self.applied_ms = int(now_ms)
        return True

    def on_error(self, sequence, probe_id, error_code):
        if (
            self.last_command is None
            or int(sequence) != self.last_command["sequence"]
            or int(probe_id) != self.last_command["probe_id"]
        ):
            return None
        code = self.ERROR_F407 + min(max(int(error_code), 1), 245)
        return self._fail(code, "F407 rejected SET")

    def _issue_search_probe(self, now_ms):
        self.first_counts = None
        self.first_ratio = None
        if self.probe_cursor >= len(PROBE_TABLE):
            return self._fail(
                self.ERROR_FREQUENCY_NOT_FOUND,
                "all 328 five-touch probes were exhausted",
            )
        self.state = self.SEARCH
        self.current_probe_grid = int(PROBE_TABLE[self.probe_cursor])
        self.current_probe_id = self.probe_cursor + 1
        self.probe_cursor += 1
        amplitude = amplitude_feedforward(
            self.current_probe_grid, self.target_span_div
        )
        return self._issue_grid(
            self.current_probe_grid, SEARCH_PHASE_DEG, amplitude,
            self.current_probe_id, now_ms
        )

    @staticmethod
    def _raw_counts(measurement):
        return tuple(int(measurement.get(key, 0)) for key in (
            "left", "right", "top", "bottom"
        ))

    @staticmethod
    def _strict_measurement(measurement):
        if not measurement:
            return False
        counts = Requirement5Controller._raw_counts(measurement)
        return strict_ratio_from_counts(*counts)[0]

    def _same_dual_phase(self, measurement):
        if not self._strict_measurement(measurement):
            return False
        counts = self._raw_counts(measurement)
        valid, unused, ratio_y, ratio_x = strict_ratio_from_counts(*counts)
        return bool(
            valid and counts == self.first_counts
            and (ratio_x, ratio_y) == self.first_ratio
        )

    def _issue_second_probe(self, now_ms):
        self.second_probe_grid = choose_second_probe(
            self.input_grid, self.current_probe_grid
        )
        if self.second_probe_grid is None:
            return self._issue_search_probe(now_ms)
        self.second_expected_counts = expected_touch_counts(
            self.input_grid, self.second_probe_grid
        )
        self.state = self.SECOND_PROBE
        amplitude = amplitude_feedforward(
            self.second_probe_grid, self.target_span_div
        )
        return self._issue_grid(
            self.second_probe_grid,
            SEARCH_PHASE_DEG + THIRD_PHASE_OFFSET_DEG,
            amplitude, self._next_control_probe_id(), now_ms
        )

    def _issue_control(self, phase_deg, now_ms):
        return self._issue_millihz(
            self.locked_frequency_millihz, phase_deg, self.amplitude_mv,
            self._next_control_probe_id(), now_ms
        )

    def _begin_coarse(self, now_ms):
        self.state = self.PHASE_COARSE
        self.coarse_phase_index = 0
        self.phase_samples = []
        self.best_phase_deg = 0.0
        self.best_shape_error = 999.0
        self.best_measurement = None
        self.phase_deg = 0.0
        return self._issue_control(0.0, now_ms)

    def _phase_observation_error(self, measurement):
        if not target_ratio_ok(measurement, self.target_shape):
            return 999.0
        return shape_error_for_target(measurement, self.target_shape)

    def _record_phase_result(self, phase_deg, errors, measurement):
        phase_error = median(errors)
        if phase_error is None:
            phase_error = 999.0
        if phase_error < self.best_shape_error:
            self.best_shape_error = float(phase_error)
            self.best_phase_deg = wrap_phase_deg(phase_deg)
            self.best_measurement = dict(measurement)

    def _begin_fine_round(self, now_ms):
        if self.fine_step_index >= len(self.fine_steps):
            if (
                self.best_measurement is None
                or not target_ratio_ok(self.best_measurement,
                                       self.target_shape)
                or not target_shape_ok(self.best_measurement,
                                       self.target_shape)
            ):
                return self._fail(
                    self.ERROR_SHAPE_NOT_FOUND,
                    "phase search did not pass target shape and ratio gates",
                )
            self.state = self.AMPLITUDE_FINE
            self.amplitude_samples = []
            self.amplitude_corrections = 0
            self.amplitude_invalid_frames = 0
            return self._issue_control(self.best_phase_deg, now_ms)
        step = self.fine_steps[self.fine_step_index]
        self.fine_candidates = (
            wrap_phase_deg(self.best_phase_deg - step),
            wrap_phase_deg(self.best_phase_deg + step),
        )
        self.fine_candidate_index = 0
        self.phase_samples = []
        self.state = self.PHASE_FINE
        return self._issue_control(self.fine_candidates[0], now_ms)

    def _target_good(self, measurement, check_adjacent=True):
        if not target_ratio_ok(measurement, self.target_shape):
            return False
        if not target_shape_ok(measurement, self.target_shape):
            return False
        span_x = float(measurement.get("span_x_div", self.target_span_div))
        span_y = float(measurement.get("span_y_div", 0.0))
        if abs(span_y - span_x) > 0.12:
            return False
        if abs(float(measurement.get("center_x_div", 0.0))) > 0.15:
            return False
        if abs(float(measurement.get("center_y_div", 0.0))) > 0.15:
            return False
        current_bounds = measurement.get("bounds_div")
        if (
            check_adjacent and current_bounds is not None
            and self.previous_bounds is not None
            and bounds_drift_div(current_bounds, self.previous_bounds) >= 0.25
        ):
            return False
        return True

    def _record_tracking_phase(self, now_ms):
        self.phase_history.append((int(now_ms), self.phase_deg))
        while (
            len(self.phase_history) > 3
            and ticks_diff(now_ms, self.phase_history[0][0]) > 1000
        ):
            self.phase_history.pop(0)

    def _tracking_command(self, measurement, now_ms):
        """Bounded phase hill climb, clock trim and slow amplitude loop."""
        self._record_tracking_phase(now_ms)
        if self.last_track_ms is None:
            self.last_track_ms = int(now_ms)
        if self.last_trim_ms is None:
            self.last_trim_ms = int(now_ms)
        if self.last_slow_amplitude_ms is None:
            self.last_slow_amplitude_ms = int(now_ms)

        current_error = self._phase_observation_error(measurement)
        self.track_error_samples.append(current_error)
        if len(self.track_error_samples) > 12:
            self.track_error_samples.pop(0)

        changed = False
        new_phase = self.phase_deg
        new_frequency = self.locked_frequency_millihz
        new_amplitude = self.amplitude_mv

        # Keep no more than 20 commanded phase degrees in any one-second span.
        while (
            self.phase_moves
            and ticks_diff(now_ms, self.phase_moves[0][0]) > 1000
        ):
            self.phase_moves.pop(0)
        moved_recently = sum(item[1] for item in self.phase_moves)
        if ticks_diff(now_ms, self.last_track_ms) >= self.track_update_ms:
            # Compare complete perturbation points, not adjacent camera
            # frames.  Several frames normally arrive at the same phase; a
            # per-frame comparison would clear the "worse" observation before
            # the next phase command and could make the search walk forever in
            # one direction.  The median also rejects one noisy trace frame.
            observed_error = median(self.track_error_samples)
            if observed_error is None:
                observed_error = current_error
            if self.track_previous_phase_error is not None:
                if observed_error > self.track_previous_phase_error + 0.01:
                    self.track_direction = -self.track_direction
                    self.track_worse_frames += 1
                else:
                    self.track_worse_frames = 0
            self.track_previous_phase_error = float(observed_error)
            self.track_best_error = float(observed_error)
            self.track_error_samples = []
            step = 1 if self._target_good(measurement, False) else 2
            if moved_recently + step <= 20:
                new_phase = wrap_phase_deg(
                    self.phase_deg + self.track_direction * step
                )
                self.phase_moves.append((int(now_ms), step))
                changed = True
            self.last_track_ms = int(now_ms)

        if ticks_diff(now_ms, self.last_trim_ms) >= FREQUENCY_TRIM_UPDATE_MS:
            trim_hz = phase_drift_frequency_trim_hz(self.phase_history)
            nominal = self.nominal_frequency_millihz
            low = nominal - int(FREQUENCY_TRIM_LIMIT_HZ * 1000.0)
            high = nominal + int(FREQUENCY_TRIM_LIMIT_HZ * 1000.0)
            trimmed = int(clamp(
                self.locked_frequency_millihz + round(trim_hz * 1000.0),
                low, high
            ))
            if trimmed != self.locked_frequency_millihz:
                new_frequency = trimmed
                changed = True
            self.last_trim_ms = int(now_ms)

        if (
            ticks_diff(now_ms, self.last_slow_amplitude_ms) >= 1000
            and target_ratio_ok(measurement, self.target_shape)
        ):
            span_x = float(measurement.get(
                "span_x_div", self.target_span_div
            ))
            span_y = float(measurement.get("span_y_div", 0.0))
            if span_y > 0.0 and abs(span_y - span_x) > 0.12:
                scale = clamp(span_x / span_y, 0.95, 1.05)
                adjusted = int(clamp(
                    round(self.amplitude_mv * scale),
                    DDS_AMPLITUDE_MIN_MV, DDS_AMPLITUDE_MAX_MV
                ))
                if adjusted != self.amplitude_mv:
                    new_amplitude = adjusted
                    changed = True
            self.last_slow_amplitude_ms = int(now_ms)

        if not changed:
            return None
        self.locked_frequency_millihz = int(new_frequency)
        self.amplitude_mv = int(new_amplitude)
        return self._issue_millihz(
            self.locked_frequency_millihz, new_phase, self.amplitude_mv,
            self._next_control_probe_id(), now_ms
        )

    def on_measurement(self, measurement, now_ms):
        if self.waiting_for_applied or self.state in (
            self.WAIT_START, self.FAILED
        ):
            return None
        timeout = self._control_timeout(now_ms)
        if timeout is not None:
            return timeout

        if self.state == self.MEASURE_X_AMPLITUDE:
            span_x = float(measurement.get("span_x_div", 0.0))
            if span_x <= 0.0 or int(measurement.get("trace_pixels", 0)) <= 0:
                return None
            self.x_span_samples.append(span_x)
            if len(self.x_span_samples) > 5:
                self.x_span_samples.pop(0)
            if len(self.x_span_samples) < 5:
                return None
            if max(self.x_span_samples) - min(self.x_span_samples) > 0.10:
                return None
            self.target_span_div = float(median(self.x_span_samples))
            x_vpp = self.target_span_div * 0.5
            if x_vpp < 0.9 or x_vpp > 4.1:
                return self._fail(
                    self.ERROR_AMPLITUDE,
                    "X amplitude %.3f Vpp is outside 1..4 Vpp" % x_vpp,
                )
            return self._issue_search_probe(now_ms)

        if self.state == self.SEARCH:
            if not self._strict_measurement(measurement):
                return self._issue_search_probe(now_ms)
            counts = self._raw_counts(measurement)
            candidate = exact_frequency_candidate(
                self.current_probe_grid, *counts
            )
            if candidate is None:
                return self._issue_search_probe(now_ms)
            valid, unused, ratio_y, ratio_x = strict_ratio_from_counts(*counts)
            self.first_counts = counts
            self.first_ratio = (ratio_x, ratio_y)
            self.input_grid = candidate
            self.state = self.DUAL_PHASE
            return self._issue_grid(
                self.current_probe_grid,
                SEARCH_PHASE_DEG + SECOND_PHASE_OFFSET_DEG,
                self.amplitude_mv, self.current_probe_id, now_ms
            )

        if self.state in (self.DUAL_PHASE, self.DUAL_PHASE_RETRY):
            if self._same_dual_phase(measurement):
                counts = self._raw_counts(measurement)
                candidate = exact_frequency_candidate(
                    self.current_probe_grid, *counts
                )
                if candidate == self.input_grid:
                    return self._issue_second_probe(now_ms)
            if self.state == self.DUAL_PHASE:
                self.state = self.DUAL_PHASE_RETRY
                return self._issue_grid(
                    self.current_probe_grid,
                    SEARCH_PHASE_DEG + THIRD_PHASE_OFFSET_DEG,
                    self.amplitude_mv, self.current_probe_id, now_ms
                )
            return self._issue_search_probe(now_ms)

        if self.state == self.SECOND_PROBE:
            counts = self._raw_counts(measurement)
            expected_x, expected_y = self.second_expected_counts
            expected = (expected_x, expected_x, expected_y, expected_y)
            candidate = exact_frequency_candidate(
                self.second_probe_grid, *counts
            )
            if (
                not self._strict_measurement(measurement)
                or counts != expected or candidate != self.input_grid
            ):
                self.input_grid = 0
                return self._issue_search_probe(now_ms)
            self.nominal_grid = self.input_grid * (
                2 if self.target_shape == 3 else 1
            )
            self.nominal_frequency_millihz = self.nominal_grid * 100000
            self.locked_frequency_millihz = self.nominal_frequency_millihz
            self.amplitude_mv = amplitude_feedforward(
                self.nominal_grid, self.target_span_div
            )
            return self._begin_coarse(now_ms)

        if self.state == self.PHASE_COARSE:
            self.phase_samples.append(
                self._phase_observation_error(measurement)
            )
            if len(self.phase_samples) < 2:
                return None
            phase = self.coarse_phase_index * 30
            self._record_phase_result(phase, self.phase_samples, measurement)
            self.phase_samples = []
            self.coarse_phase_index += 1
            if self.coarse_phase_index < 12:
                return self._issue_control(
                    self.coarse_phase_index * 30, now_ms
                )
            if self.best_shape_error >= 999.0:
                return self._fail(
                    self.ERROR_SHAPE_NOT_FOUND,
                    "all 12 coarse phase points failed the ratio gate",
                )
            self.fine_step_index = 0
            return self._begin_fine_round(now_ms)

        if self.state == self.PHASE_FINE:
            self.phase_samples.append(
                self._phase_observation_error(measurement)
            )
            if len(self.phase_samples) < 2:
                return None
            candidate_phase = self.fine_candidates[self.fine_candidate_index]
            self._record_phase_result(
                candidate_phase, self.phase_samples, measurement
            )
            self.phase_samples = []
            self.fine_candidate_index += 1
            if self.fine_candidate_index < len(self.fine_candidates):
                return self._issue_control(
                    self.fine_candidates[self.fine_candidate_index], now_ms
                )
            self.fine_step_index += 1
            return self._begin_fine_round(now_ms)

        if self.state == self.AMPLITUDE_FINE:
            if (
                not target_ratio_ok(measurement, self.target_shape)
                or not target_shape_ok(measurement, self.target_shape)
                or float(measurement.get("span_y_div", 0.0)) <= 0.0
            ):
                self.amplitude_invalid_frames += 1
                if self.amplitude_invalid_frames >= 6:
                    return self._fail(
                        self.ERROR_AMPLITUDE,
                        "shape was invalid during amplitude adjustment",
                    )
                return None
            self.amplitude_invalid_frames = 0
            self.amplitude_samples.append(
                float(measurement.get("span_y_div", 0.0))
            )
            if len(self.amplitude_samples) < 3:
                return None
            span_y = float(median(self.amplitude_samples))
            self.amplitude_samples = []
            error = self.target_span_div - span_y
            if abs(error) <= 0.12:
                self.state = self.LOCK_TRACK
                self.lock_frames = 0
                self.previous_bounds = None
                self.phase_deg = self.best_phase_deg
                self.track_best_error = self.best_shape_error
                self.track_worse_frames = 0
                self.track_error_samples = []
                self.track_previous_phase_error = None
                self.phase_history = []
                self.phase_moves = []
                self.last_track_ms = int(now_ms)
                self.last_trim_ms = int(now_ms)
                self.last_slow_amplitude_ms = int(now_ms)
                return self._issue_control(self.best_phase_deg, now_ms)
            if self.amplitude_corrections >= 4:
                return self._fail(
                    self.ERROR_AMPLITUDE,
                    "Y amplitude did not converge in four corrections",
                )
            scale = clamp(self.target_span_div / span_y, 0.90, 1.10)
            adjusted = int(clamp(
                round(self.amplitude_mv * scale),
                DDS_AMPLITUDE_MIN_MV, DDS_AMPLITUDE_MAX_MV
            ))
            if adjusted == self.amplitude_mv:
                adjusted += 1 if error > 0.0 else -1
                adjusted = int(clamp(
                    adjusted, DDS_AMPLITUDE_MIN_MV,
                    DDS_AMPLITUDE_MAX_MV
                ))
            self.amplitude_mv = adjusted
            self.amplitude_corrections += 1
            return self._issue_control(self.best_phase_deg, now_ms)

        if self.state == self.LOCK_TRACK:
            good = self._target_good(measurement, True)
            current_bounds = measurement.get("bounds_div")
            if good:
                self.lock_frames += 1
            else:
                self.lock_frames = 0
            if current_bounds is not None:
                self.previous_bounds = current_bounds
            if self.lock_frames >= self.verify_frames:
                self.state = self.HOLD
                self.hold_since_ms = int(now_ms)
                self.lock_reference_bounds = current_bounds
                self.hard_fail_frames = 0
                return LockedEvent(
                    self.sequence, self.input_grid * 100000
                )
            return self._tracking_command(measurement, now_ms)

        if self.state == self.HOLD:
            current_bounds = measurement.get("bounds_div")
            good = self._target_good(measurement, False)
            if (
                current_bounds is not None
                and self.lock_reference_bounds is not None
                and bounds_drift_div(
                    current_bounds, self.lock_reference_bounds
                ) >= 0.5
            ):
                good = False
            if good:
                self.hard_fail_frames = 0
            else:
                self.hard_fail_frames += 1
                # The completion interval is genuinely continuous: even one
                # rejected frame restarts the 5 s timer.  Three rejects also
                # leave HOLD and require a new three-frame lock.
                self.hold_since_ms = int(now_ms)
                if self.hard_fail_frames >= 3:
                    self.state = self.LOCK_TRACK
                    self.lock_frames = 0
                    self.previous_bounds = current_bounds
                    self.hold_since_ms = None
                    self.track_error_samples = []
                    self.track_previous_phase_error = None
                    return None
            if (
                good and self.hold_since_ms is not None
                and ticks_diff(now_ms, self.hold_since_ms) >= self.hold_ms
            ):
                self.state = self.DONE
                return DoneEvent(
                    self.sequence, self.input_grid * 100000
                )
            return self._tracking_command(measurement, now_ms)

        if self.state == self.DONE:
            # DONE is an event, not an instruction to stop stabilizing.
            return self._tracking_command(measurement, now_ms)
        return None

    def _control_timeout(self, now_ms):
        if (
            self.started_ms is not None
            and self.state not in (
                self.HOLD, self.DONE, self.FAILED, self.WAIT_START
            )
            and ticks_diff(now_ms, self.started_ms) >= self.control_timeout_ms
        ):
            return self._fail(
                self.ERROR_CONTROL_TIMEOUT,
                "LOCKED was not reached before the 10 s deadline",
            )
        return None

    def tick(self, now_ms):
        if self.state in (self.WAIT_START, self.FAILED):
            return None
        timeout = self._control_timeout(now_ms)
        if timeout is not None:
            return timeout
        if (
            not self.waiting_for_applied or self.last_command is None
            or self.ack_started_ms is None
            or ticks_diff(now_ms, self.ack_started_ms) < self.ack_timeout_ms
        ):
            return None
        if self.ack_retries < self.max_ack_retries:
            self.ack_retries += 1
            self.ack_started_ms = int(now_ms)
            return self.last_command
        return self._fail(
            self.ERROR_UART_TIMEOUT, "APPLIED acknowledgement timed out"
        )

    def _fail(self, error_code, reason):
        self.state = self.FAILED
        self.waiting_for_applied = False
        return FailedEvent(self.sequence, error_code, reason)

    def stop(self):
        saved_sequence = self.sequence
        self._reset_cycle()
        self.sequence = saved_sequence

    def hold_elapsed_ms(self, now_ms):
        if self.state not in (self.HOLD, self.DONE):
            return 0
        if self.hold_since_ms is None:
            return 0
        return max(0, ticks_diff(now_ms, self.hold_since_ms))


# Backward-compatible name used by the earlier K230 host tests.
FastSearchController = Requirement5Controller


# ---------------------------------------------------------------------------
# ASCII UART framing
# ---------------------------------------------------------------------------

class UartFrameParser:
    def __init__(self):
        self.collecting = False
        self.buffer = ""

    def feed(self, data):
        if not data:
            return []
        if isinstance(data, bytes):
            data = data.decode("ascii", "ignore")
        frames = []
        for character in str(data):
            if character == "$":
                self.collecting = True
                self.buffer = ""
            elif character == "\r" or character == "\n":
                if self.collecting and self.buffer:
                    frames.append(self.buffer)
                self.collecting = False
                self.buffer = ""
            elif self.collecting:
                if len(self.buffer) < 127:
                    self.buffer += character
                else:
                    self.collecting = False
                    self.buffer = ""
        return frames


def parse_uart_frame(frame):
    """Return a tuple for START/APPLIED/ERROR, otherwise None."""
    fields = str(frame).split(",")
    try:
        if len(fields) == 2 and fields[0] == "START":
            shape = int(fields[1])
            if shape in (1, 2, 3):
                return ("START", shape)
        elif len(fields) == 3 and fields[0] == "APPLIED":
            return ("APPLIED", int(fields[1]), int(fields[2]))
        elif len(fields) == 4 and fields[0] == "ERROR":
            return (
                "ERROR", int(fields[1]), int(fields[2]), int(fields[3])
            )
    except (TypeError, ValueError):
        return None
    return None


def format_action(action):
    if isinstance(action, dict) and action.get("kind") == "SET":
        return "$SET,%d,%d,%d,%d,%d,%d\r\n" % (
            action["sequence"], action["probe_id"],
            action["frequency_millihz"], action["phase_millideg"],
            action["amplitude_mv"], action["shape"],
        )
    if isinstance(action, LockedEvent):
        return "$LOCKED,%d,%d\r\n" % (
            action.sequence, action.input_frequency_millihz
        )
    if isinstance(action, DoneEvent):
        return "$DONE,%d,%d\r\n" % (
            action.sequence, action.input_frequency_millihz
        )
    if isinstance(action, FailedEvent):
        return "$FAILED,%d,%d\r\n" % (
            action.sequence, action.error_code
        )
    return None


def send_action(uart, action):
    message = format_action(action)
    if message is None:
        return False
    uart.write(message)
    if isinstance(action, FailedEvent):
        print("control failed: %s" % action.reason)
    return isinstance(action, dict)


def consume_uart(uart, parser, controller, now_ms):
    """Return (actions, matching_applied_seen)."""
    actions = []
    applied_seen = False
    try:
        pending = uart.any()
    except Exception:
        pending = 0
    if not pending:
        return actions, applied_seen
    data = uart.read(pending)
    for raw_frame in parser.feed(data):
        frame = parse_uart_frame(raw_frame)
        if frame is None:
            print("ignored UART2 frame: %s" % raw_frame)
            continue
        if frame[0] == "START":
            # Ignore only a very recent retransmission of the same START.  A
            # later START is an explicit new contest run and resets the cycle.
            duplicate = bool(
                controller.target_shape == frame[1]
                and controller.started_ms is not None
                and controller.state not in (
                    controller.WAIT_START, controller.DONE,
                    controller.FAILED,
                )
                and ticks_diff(now_ms, controller.started_ms) < 1000
            )
            if not duplicate:
                actions.append(controller.start(frame[1], now_ms))
        elif frame[0] == "APPLIED":
            if controller.on_applied(frame[1], frame[2], now_ms):
                applied_seen = True
        elif frame[0] == "ERROR":
            action = controller.on_error(frame[1], frame[2], frame[3])
            if action is not None:
                actions.append(action)
    return actions, applied_seen


# ---------------------------------------------------------------------------
# CanMV hardware runtime (camera init/deinit preserved from supplied script)
# ---------------------------------------------------------------------------

sensor = None


def _sleep_ms(milliseconds):
    try:
        time.sleep_ms(int(milliseconds))
    except AttributeError:
        time.sleep(float(milliseconds) / 1000.0)


def init_uart2():
    fpioa = FPIOA()
    fpioa.set_function(UART2_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART2_RX_PIN, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=UART2_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def camera_init():
    global sensor
    sensor = Sensor(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    # The K230 customer examples use both transforms for a 180-degree image
    # rotation.  Apply this at the sensor so recognition and LCD preview see
    # the same orientation.
    if ROTATE_IMAGE_180:
        sensor.set_hmirror(True)
        sensor.set_vflip(True)

    # Fixed controls prevent exposure/AWB motion from looking like trace drift.
    # Not every CanMV sensor exposes all calls, so each optional calibration is
    # guarded while the proven Sensor/Display/Media sequence stays unchanged.
    try:
        sensor.set_auto_exposure(False, exposure_us=4000)
    except Exception:
        pass
    try:
        sensor.set_auto_gain(False, gain_db=0.0)
    except Exception:
        pass
    try:
        sensor.set_auto_whitebal(False)
    except Exception:
        pass

    # Match the board-level ST7701 setup used by the working waveform demo.
    # ``to_ide=True`` keeps the CanMV IDE preview available while the physical
    # LCD is active.  ``auto`` is useful when the same image is used both on
    # hardware and in the IDE; it never hides an LCD init exception.
    target = DISPLAY_TARGET.lower()
    if target not in ("lcd", "virt", "auto"):
        raise ValueError("DISPLAY_TARGET must be lcd, virt, or auto")

    if target in ("lcd", "auto"):
        try:
            # The customer ST7701 examples intentionally omit width/height
            # and fps.  Some K230 firmware revisions block during LCD init
            # when those optional arguments are supplied.
            print("initializing ST7701 LCD...")
            Display.init(Display.ST7701, to_ide=True)
            print("ST7701 LCD display initialized (%dx%d)" % (
                DISPLAY_WIDTH, DISPLAY_HEIGHT
            ))
        except Exception as error:
            if target == "lcd":
                print("ST7701 LCD init failed: %s" % error)
                raise
            print("ST7701 LCD init failed, falling back to VIRT: %s" % error)
            Display.init(
                Display.VIRT, width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
                fps=DISPLAY_FPS, to_ide=True
            )
    else:
        Display.init(
            Display.VIRT, width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
            fps=DISPLAY_FPS, to_ide=True
        )
        print("VIRT display initialized (%dx%d)" % (
            IMAGE_WIDTH, IMAGE_HEIGHT
        ))
    print("initializing media manager...")
    MediaManager.init()
    print("starting camera stream...")
    sensor.run()
    print("camera stream started")
    _sleep_ms(1000)
    return sensor


def camera_deinit(uart=None):
    global sensor
    try:
        if sensor is not None:
            sensor.stop()
    except Exception:
        pass
    try:
        if uart is not None:
            uart.deinit()
    except Exception:
        pass
    try:
        Display.deinit()
    except Exception:
        pass
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except Exception:
        pass
    _sleep_ms(100)
    try:
        MediaManager.deinit()
    except Exception:
        pass


def _draw_text(img, x, y, text_value, color, scale=1):
    try:
        img.draw_string(
            int(x), int(y), str(text_value), color=color, scale=scale
        )
    except Exception:
        try:
            font_size = 16 if scale <= 1 else 22
            img.draw_string_advanced(
                int(x), int(y), font_size, str(text_value), color=color
            )
        except Exception:
            pass


def draw_runtime_overlay(img, controller, measurement, fps=0.0):
    img.draw_rectangle(GRID_RECT, color=(0, 255, 255), thickness=2)
    _draw_text(
        img, 8, 8, controller.state,
        (0, 255, 0) if controller.state in (
            controller.HOLD, controller.DONE
        ) else (255, 255, 0), 1
    )
    if measurement is not None:
        _draw_text(img, 8, 28, "L%d R%d T%d B%d" % (
            measurement.get("left", 0), measurement.get("right", 0),
            measurement.get("top", 0), measurement.get("bottom", 0),
        ), (255, 255, 255), 1)
        _draw_text(img, 8, 48, "X%.2f Y%.2f div" % (
            measurement.get("span_x_div", 0.0),
            measurement.get("span_y_div", 0.0),
        ), (255, 255, 255), 1)
    _draw_text(img, 8, IMAGE_HEIGHT - 24, "FPS %.1f" % fps,
               (255, 255, 255), 1)


def main_loop(uart):
    controller = Requirement5Controller()
    parser = UartFrameParser()
    discard_frames = 0
    measurement = None
    frame_id = 0
    try:
        clock = time.clock()
    except Exception:
        clock = None

    while True:
        try:
            os.exitpoint()
        except Exception:
            pass
        if clock is not None:
            clock.tick()
        now = ticks_ms()

        actions, applied_seen = consume_uart(
            uart, parser, controller, now
        )
        for action in actions:
            send_action(uart, action)
        if applied_seen:
            # This first snapshot can still have started exposing before the
            # DDS IO_UPDATE.  It is displayed but never analyzed.
            discard_frames = max(
                discard_frames, DISCARD_FRAMES_AFTER_APPLIED
            )

        timed_action = controller.tick(now)
        if timed_action is not None:
            send_action(uart, timed_action)

        img = sensor.snapshot()
        if discard_frames > 0:
            discard_frames -= 1
        elif (
            not controller.waiting_for_applied
            and controller.state not in (
                controller.WAIT_START, controller.FAILED
            )
        ):
            frame_started = ticks_ms()
            measurement = measure_image(img)
            measurement["frame_id"] = frame_id
            measurement["timestamp_ms"] = frame_started
            measurement["frame_process_ms"] = ticks_diff(
                ticks_ms(), frame_started
            )
            action = controller.on_measurement(measurement, ticks_ms())
            if action is not None:
                send_action(uart, action)

        fps = clock.fps() if clock is not None else 0.0
        full_rate_preview = controller.state in (
            controller.WAIT_START, controller.HOLD, controller.DONE,
            controller.FAILED,
        )
        if (
            full_rate_preview
            or frame_id % ACTIVE_PREVIEW_INTERVAL_FRAMES == 0
        ):
            draw_runtime_overlay(img, controller, measurement, fps)
            Display.show_image(img)

        if frame_id % LOG_INTERVAL_FRAMES == 0:
            print(
                "state=%s probe=%d fx_grid=%d fy_mHz=%d phase=%.1f "
                "amp=%d hold_ms=%d fps=%.1f" % (
                    controller.state, controller.current_probe_grid,
                    controller.input_grid,
                    controller.locked_frequency_millihz,
                    controller.phase_deg, controller.amplitude_mv,
                    controller.hold_elapsed_ms(now), fps,
                )
            )
        frame_id += 1
        img = None
        if frame_id % LOG_INTERVAL_FRAMES == 0:
            gc.collect()


def main():
    if not CANMV_AVAILABLE:
        print("CanMV hardware modules unavailable; pure helpers are importable")
        return 0
    uart = None
    try:
        camera_init()
        uart = init_uart2()
        print(
            "UART2 TX=Pin%d RX=Pin%d %d 8N1; probes=%d" % (
                UART2_TX_PIN, UART2_RX_PIN, UART2_BAUDRATE,
                len(PROBE_TABLE),
            )
        )
        main_loop(uart)
    except KeyboardInterrupt:
        print("Program stopped by user")
    except BaseException as error:
        print("Program exception: %s" % error)
    finally:
        if uart is not None:
            try:
                uart.write("$STOP,0\r\n")
            except Exception:
                pass
        camera_deinit(uart)
    return 0


if __name__ == "__main__":
    main()
