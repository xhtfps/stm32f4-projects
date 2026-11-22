"""K230 / CanMV 实时示波器黄色波形识别。

在 CanMV IDE 中直接运行。默认使用 IDE 虚拟显示；若使用 HDMI/LCD，请按板卡
型号修改 Display.init。算法不需要 KPU 模型，使用 K230 固件自带的 OpenMV API。
"""

import math
import os
import time

from machine import FPIOA, UART
from media.display import *
from media.media import *
from media.sensor import *

from lissajous_core import fit_lissajous


# 关闭整帧软件旋转，直接处理摄像头原始 640x480 画面。
# CanMV IDE 的 Rotate 也应关闭；需要正向画面时请物理调整摄像头。
CAPTURE_W = 640
CAPTURE_H = 480
TARGET_FPS = 60
FRAME_W = CAPTURE_W
FRAME_H = CAPTURE_H

# 参考 K230 例程中的 ST7701 LCD 配置，同时保留 IDE 预览。
DISPLAY_W = 800
DISPLAY_H = 480
DISPLAY_X = (DISPLAY_W - FRAME_W) // 2

# CanMV V3.0 排针：IO5/物理 17 脚为 UART2_TX，IO6/物理 20 脚为 UART2_RX。
UART2_TX_PIN = 5
UART2_RX_PIN = 6
UART2_BAUDRATE = 115200
UART_SEND_INTERVAL_MS = 200


STATUS_BAR_H = 72
STATUS_UPDATE_MS = 1000
_status_overlay = None
_status_last_ms = time.ticks_ms()
_status_frames = 0
_status_fps = 0.0
_status_last_label = None
_status_last_detail = None


def draw_status_text(img, label, detail):
    """每秒更新一次缓存横幅，避免每帧旋转文字图层。"""
    global _status_overlay, _status_last_ms, _status_frames, _status_fps
    global _status_last_label, _status_last_detail
    _status_frames += 1
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, _status_last_ms)
    refresh = (
        _status_overlay is None
        or label != _status_last_label
        or detail != _status_last_detail
    )
    if elapsed >= STATUS_UPDATE_MS:
        _status_fps = _status_frames * 1000.0 / max(elapsed, 1)
        _status_frames = 0
        _status_last_ms = now
        refresh = True

    if _status_overlay is None:
        _status_overlay = image.Image(FRAME_W, STATUS_BAR_H, image.RGB565)

    if refresh:
        _status_last_label = label
        _status_last_detail = detail
        _status_overlay.clear()
        _status_overlay.draw_string(
            8, 4, label, color=(0, 255, 0), scale=3, mono_space=False
        )
        fps_text = "FPS --" if _status_fps <= 0.0 else "FPS %.1f" % _status_fps
        detail_text = "%s  %s" % (detail, fps_text)
        _status_overlay.draw_string(
            8, 42, detail_text,
            color=(0, 255, 0), scale=2, mono_space=False
        )

    img.draw_image(_status_overlay, 0, 0)


def make_uart_message(ratio, fit, vpp_x, vpp_y, fps):
    """将屏幕上的测量值编码为一行 ASCII，便于 STM32/串口助手解析。"""
    phase_text = "--" if fit is None else "%.1f" % fit["phase_deg"]
    vpp_x_text = "--" if vpp_x is None else "%.2f" % vpp_x
    vpp_y_text = "--" if vpp_y is None else "%.2f" % vpp_y
    return (
        "ratio=%s,phase_deg=%s,VppX=%s,VppY=%s,FPS=%.1f\r\n"
        % (ratio, phase_text, vpp_x_text, vpp_y_text, fps)
    )

# OpenMV/CanMV 使用 LAB 阈值：(L_min, L_max, A_min, A_max, B_min, B_max)。
# 该值由本目录三张样图统计得到；换摄像头后可按 README 的方法微调。
# LAB 的 B 正值表示偏黄；范围比单台相机的标定值略宽，空间形状过滤负责排除噪声。
YELLOW_LAB = (25, 100, -45, 25, 10, 100)
ADAPT_COLOR_EVERY = 120
MODEL_FIT_EVERY = 20
# 摄像头横纵坐标与 IDE 旋转后的观察坐标互换。
# 不旋转整帧，只在拟合和结果显示时交换频率轴。
SWAP_RATIO_AXES = True
UNKNOWN_LABEL = "unkonw"
# 当前只判断 1:1；所有非 1:1 图形统一显示 unkonw。
# 72 档对应 5° 的相位分辨率。
FIT_PHASE_STEPS = 72
FIT_MODEL_SAMPLES = 96
FIT_FREQUENCY_PAIRS = ((1, 1),)
GRID_COLUMNS = 8
GRID_ROWS = 8
X_VOLTS_PER_DIV = 0.5
Y_VOLTS_PER_DIV = 0.5

# 当前固定机位下的 8x8 网格区域，避免自动检测误选整块显示屏。
# 如果移动了摄像头，需重新微调 (x, y, w, h)，或改回 None 自动检测。
MANUAL_GRID_ROI = (254, 188, 164, 156)
AUTO_RECT_THRESHOLD = 16000
AUTO_REDETECT_GRID = False
GRID_DETECT_EVERY = 120
ANALYZE_EVERY = 4
# 同时进行竖条和横条扫描，避免近竖直/近水平分支只剩一个质心而造成频率比翻转。
POINT_STEP = 12
MIN_WAVE_PIXELS = 25
# 点集经协方差归一化后，真实椭圆上各点到中心的半径应近似一致。
# 双环 2:1 的归一化半径起伏明显更大，不会命中该阈值。
ELLIPSE_RADIAL_SPREAD_MAX = 1.10
ELLIPSE_MIN_POINTS = 24
# 单个椭圆偶尔会在网格交点处碎裂，不能用单条扫描带直接拒绝。
# 至少 3 条扫描带都出现 4 个独立分支，才认为是稳定的多瓣曲线。
MULTI_CROSSING_MIN = 4
DOUBLE_LOOP_MIN_STRIPS = 3
CROSSING_MERGE_GAP_RATIO = 0.05
CROSSING_MERGE_GAP_MIN = 6
VPP_ZERO_SPAN_RATIO = 0.08
PHASE_FIT_SHAPES = (
    "curve", "ellipse", "single_loop", "diagonal_line"
)


def clamp_roi(roi):
    x, y, w, h = roi
    x = max(0, min(FRAME_W - 2, int(x)))
    y = max(0, min(FRAME_H - 2, int(y)))
    w = max(2, min(FRAME_W - x, int(w)))
    h = max(2, min(FRAME_H - y, int(h)))
    return (x, y, w, h)


def inner_roi(roi, ratio=0.018):
    x, y, w, h = roi
    mx = max(2, int(w * ratio))
    my = max(2, int(h * ratio))
    return clamp_roi((x + mx, y + my, w - 2 * mx, h - 2 * my))


def smooth_roi(old_roi, new_roi, alpha=0.28):
    if old_roi is None:
        return clamp_roi(new_roi)
    # 单帧突跳通常是把某一个小网格误认为外框，先拒绝。
    ox, oy, ow, oh = old_roi
    nx, ny, nw, nh = new_roi
    if abs(nx - ox) > 0.25 * ow or abs(ny - oy) > 0.25 * oh:
        return old_roi
    values = []
    for old, new in zip(old_roi, new_roi):
        values.append(int(round(old * (1.0 - alpha) + new * alpha)))
    return clamp_roi(tuple(values))


def find_grid(img, waveform_roi=None):
    """返回网格外接 ROI 和四角；自动检测失败则返回 (None, None)。"""
    best = None
    best_score = -1
    try:
        rects = img.find_rects(threshold=AUTO_RECT_THRESHOLD)
    except Exception as exc:
        print("find_rects error:", exc)
        return None, None

    frame_area = FRAME_W * FRAME_H
    for rect in rects:
        x, y, w, h = rect.rect()
        area_ratio = (w * h) / frame_area
        aspect = w / max(h, 1)
        # 排除单个网格、文字框、整幅宽屏边框。
        if area_ratio < 0.025 or area_ratio > 0.78:
            continue
        if aspect < 0.75 or aspect > 1.65:
            continue
        cx = x + w * 0.5
        cy = y + h * 0.5
        center_penalty = abs(cx / FRAME_W - 0.5) + abs(cy / FRAME_H - 0.5)
        aspect_penalty = abs(aspect - 1.0)
        # 外层显示区和 8x8 网格都可能近似方形。有波形边界时，
        # 优先选择中心一致且最紧密包住波形的矩形，避免选到整块屏幕。
        waveform_penalty = 0.0
        if waveform_roi is not None:
            wx, wy, ww, wh = waveform_roi
            wave_cx = wx + ww * 0.5
            wave_cy = wy + wh * 0.5
            if ww > w * 1.15 or wh > h * 1.15:
                continue
            waveform_penalty = (
                abs(wave_cx - cx) / max(w, 1)
                + abs(wave_cy - cy) / max(h, 1)
                + max(0.0, (w - ww) / max(w, 1))
                + max(0.0, (h - wh) / max(h, 1))
            )
        score = (
            -20.0 * aspect_penalty
            - 2.0 * center_penalty
            - 8.0 * waveform_penalty
            - area_ratio
        )
        if score > best_score:
            best_score = score
            best = rect

    if best is None:
        return None, None
    return clamp_roi(best.rect()), best.corners()


def find_grid_from_waveform(img, yellow_threshold=YELLOW_LAB):
    """白色外框不完整时，用大范围黄色李萨如轨迹反推绘图区 ROI。"""
    try:
        blobs = img.find_blobs(
            [yellow_threshold],
            roi=(0, 0, FRAME_W, FRAME_H),
            x_stride=2,
            y_stride=2,
            pixels_threshold=70,
            area_threshold=100,
            merge=True,
            margin=8,
        )
    except Exception as exc:
        print("waveform grid fallback:", exc)
        return None
    best = None
    best_score = -1
    for blob in blobs:
        x, y, w, h = blob.rect()
        area_ratio = (w * h) / float(FRAME_W * FRAME_H)
        # 李萨如曲线应同时具有明显的水平和垂直跨度；小文字和黄色状态灯会被排除。
        if not 0.025 <= area_ratio <= 0.70:
            continue
        if w < 0.16 * FRAME_W or h < 0.16 * FRAME_H:
            continue
        score = blob.pixels() + 0.08 * w * h
        if score > best_score:
            best_score = score
            best = blob
    if best is None:
        return None
    x, y, w, h = best.rect()
    pad_x = max(3, int(w * 0.025))
    pad_y = max(3, int(h * 0.025))
    return clamp_roi((x - pad_x, y - pad_y, w + 2 * pad_x, h + 2 * pad_y))


def adaptive_yellow_threshold(img, roi, previous):
    """利用网格内 LAB-B 直方图的 Otsu 分界自适应不同摄像头的黄色偏移。"""
    try:
        threshold = img.get_histogram(roi=inner_roi(roi)).get_threshold()
        b_cut = max(8, min(45, threshold.b_value()))
        # 阈值只缓慢变化，避免某一帧屏幕刷新条纹造成跳变。
        if previous is not None:
            b_cut = int(round(previous[4] * 0.7 + b_cut * 0.3))
        return (20, 100, -50, 30, b_cut, 127)
    except Exception as exc:
        print("adaptive color fallback:", exc)
        return previous if previous is not None else YELLOW_LAB


def find_wave_blobs(img, roi, yellow_threshold):
    blobs = img.find_blobs(
        [yellow_threshold],
        roi=inner_roi(roi),
        x_stride=1,
        y_stride=1,
        pixels_threshold=MIN_WAVE_PIXELS,
        area_threshold=MIN_WAVE_PIXELS,
        merge=True,
        margin=4,
    )
    if not blobs:
        return []
    largest = max(blob.pixels() for blob in blobs)
    return [blob for blob in blobs if blob.pixels() >= max(MIN_WAVE_PIXELS, int(largest * 0.015))]


def count_distinct_crossings(positions, axis_span):
    """合并同一条扫描线上因网格遮挡或反光断裂而产生的相邻交点。"""
    if not positions:
        return 0
    ordered = sorted(positions)
    merge_gap = max(
        CROSSING_MERGE_GAP_MIN,
        int(axis_span * CROSSING_MERGE_GAP_RATIO),
    )
    count = 1
    cluster_end = ordered[0]
    for position in ordered[1:]:
        if position - cluster_end > merge_gap:
            count += 1
        cluster_end = position
    return count


def extract_scan_points(img, roi, yellow_threshold):
    """从竖条和横条双向提取黄色中心，兼顾近水平与近竖直分支。"""
    x0, y0, w, h = inner_roi(roi)
    points = []
    vertical_multi_strips = 0
    horizontal_multi_strips = 0
    maximum_crossings = 0
    strip_w = min(4, POINT_STEP)
    x = x0
    while x < x0 + w:
        current_w = min(strip_w, x0 + w - x)
        segments = img.find_blobs(
            [yellow_threshold],
            roi=(x, y0, current_w, h),
            x_stride=1,
            y_stride=1,
            pixels_threshold=2,
            area_threshold=2,
            merge=False,
        )
        crossing_positions = []
        for blob in segments:
            # 去掉偶发黄色噪点，同时保留细线。
            if blob.pixels() >= 2 and blob.h() <= int(h * 0.18):
                points.append((blob.cx(), blob.cy()))
                crossing_positions.append(blob.cy())
        valid_count = count_distinct_crossings(crossing_positions, h)
        maximum_crossings = max(maximum_crossings, valid_count)
        if valid_count >= MULTI_CROSSING_MIN:
            vertical_multi_strips += 1
        x += POINT_STEP
    strip_h = min(4, POINT_STEP)
    y = y0
    while y < y0 + h:
        current_h = min(strip_h, y0 + h - y)
        segments = img.find_blobs(
            [yellow_threshold],
            roi=(x0, y, w, current_h),
            x_stride=1,
            y_stride=1,
            pixels_threshold=2,
            area_threshold=2,
            merge=False,
        )
        crossing_positions = []
        for blob in segments:
            if blob.pixels() >= 2 and blob.w() <= int(w * 0.18):
                points.append((blob.cx(), blob.cy()))
                crossing_positions.append(blob.cx())
        valid_count = count_distinct_crossings(crossing_positions, w)
        maximum_crossings = max(maximum_crossings, valid_count)
        if valid_count >= MULTI_CROSSING_MIN:
            horizontal_multi_strips += 1
        y += POINT_STEP
    if len(points) < 8:
        loop_count = 0
    elif max(vertical_multi_strips, horizontal_multi_strips) >= DOUBLE_LOOP_MIN_STRIPS:
        loop_count = 2
    else:
        loop_count = 1
    return points, loop_count


def extract_bright_scan_points(img, roi):
    """黄色退化为白色时，跳过 8x8 网格位置并提取高亮波形中心。"""
    x0, y0, w, h = inner_roi(roi)
    try:
        otsu = img.get_histogram(roi=(x0, y0, w, h)).get_threshold()
        l_cut = max(35, min(85, otsu.l_value()))
    except Exception as exc:
        print("bright fallback threshold:", exc)
        l_cut = 55
    bright_lab = (l_cut, 100, -128, 127, -128, 127)
    points = []
    vertical_multi_strips = 0
    horizontal_multi_strips = 0
    maximum_crossings = 0
    strip_w = min(4, POINT_STEP)
    x = x0
    while x < x0 + w:
        on_vertical_grid = False
        for column in range(GRID_COLUMNS + 1):
            grid_x = x0 + column * w / float(GRID_COLUMNS)
            if abs((x + strip_w * 0.5) - grid_x) <= max(3, w * 0.008):
                on_vertical_grid = True
                break
        if not on_vertical_grid:
            segments = img.find_blobs(
                [bright_lab],
                roi=(x, y0, min(strip_w, x0 + w - x), h),
                x_stride=1,
                y_stride=1,
                pixels_threshold=2,
                area_threshold=2,
                merge=False,
            )
            crossing_positions = []
            for blob in segments:
                on_horizontal_grid = False
                for row in range(GRID_ROWS + 1):
                    grid_y = y0 + row * h / float(GRID_ROWS)
                    # 横网格会在细条扫描中变成高而亮的 blob，不能再按 blob 高度判断。
                    # 无条件挖掉规则行，交点处缺失的少量波形由李萨如模型补回。
                    if abs(blob.cy() - grid_y) <= max(4, h * 0.012):
                        on_horizontal_grid = True
                        break
                if not on_horizontal_grid and blob.pixels() >= 2:
                    points.append((blob.cx(), blob.cy()))
                    crossing_positions.append(blob.cy())
            valid_count = count_distinct_crossings(crossing_positions, h)
            maximum_crossings = max(maximum_crossings, valid_count)
            if valid_count >= MULTI_CROSSING_MIN:
                vertical_multi_strips += 1
        x += POINT_STEP
    # 与黄色通道保持双向扫描；这里对称地跳过横网格扫描行和竖网格 blob。
    strip_h = min(4, POINT_STEP)
    y = y0
    while y < y0 + h:
        on_horizontal_grid = False
        for row in range(GRID_ROWS + 1):
            grid_y = y0 + row * h / float(GRID_ROWS)
            if abs((y + strip_h * 0.5) - grid_y) <= max(3, h * 0.008):
                on_horizontal_grid = True
                break
        if not on_horizontal_grid:
            segments = img.find_blobs(
                [bright_lab],
                roi=(x0, y, w, min(strip_h, y0 + h - y)),
                x_stride=1,
                y_stride=1,
                pixels_threshold=2,
                area_threshold=2,
                merge=False,
            )
            crossing_positions = []
            for blob in segments:
                on_vertical_grid = False
                for column in range(GRID_COLUMNS + 1):
                    grid_x = x0 + column * w / float(GRID_COLUMNS)
                    if abs(blob.cx() - grid_x) <= max(4, w * 0.012):
                        on_vertical_grid = True
                        break
                if (not on_vertical_grid and blob.pixels() >= 2
                        and blob.w() <= int(w * 0.18)):
                    points.append((blob.cx(), blob.cy()))
                    crossing_positions.append(blob.cx())
            valid_count = count_distinct_crossings(crossing_positions, w)
            maximum_crossings = max(maximum_crossings, valid_count)
            if valid_count >= MULTI_CROSSING_MIN:
                horizontal_multi_strips += 1
        y += POINT_STEP
    if len(points) < 8:
        loop_count = 0
    elif max(vertical_multi_strips, horizontal_multi_strips) >= DOUBLE_LOOP_MIN_STRIPS:
        loop_count = 2
    else:
        loop_count = 1
    return points, loop_count


def classify_points(points, roi):
    if len(points) < 8:
        return "no_waveform", 0.0, 1.0

    x0, y0, w, h = roi
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    span_x = (max_x - min_x + 1) / max(w, 1)
    span_y = (max_y - min_y + 1) / max(h, 1)

    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    cxx = sum((point[0] - mean_x) ** 2 for point in points) / len(points)
    cyy = sum((point[1] - mean_y) ** 2 for point in points) / len(points)
    cxy = sum((point[0] - mean_x) * (point[1] - mean_y) for point in points) / len(points)
    root = math.sqrt(max(0.0, (cxx - cyy) ** 2 + 4.0 * cxy * cxy))
    major = 0.5 * (cxx + cyy + root)
    minor = 0.5 * (cxx + cyy - root)
    residual = minor / max(major, 1e-9)
    angle = -0.5 * math.atan2(2.0 * cxy, cxx - cyy) * 180.0 / math.pi

    if residual < 0.008:
        if span_y < 0.08:
            shape = "horizontal_line"
        elif span_x < 0.08:
            shape = "vertical_line"
        else:
            shape = "diagonal_line"
    else:
        # 对点集做旋转/缩放无关的椭圆检测。椭圆点的马氏半径近似恒定，
        # 2:1 双环曲线则会从中心穿过，半径分布的起伏明显更大。
        determinant = cxx * cyy - cxy * cxy
        ellipse_like = False
        if (len(points) >= ELLIPSE_MIN_POINTS
                and span_x >= 0.16 and span_y >= 0.16
                and determinant > 1e-9):
            radii = []
            for point in points:
                dx = point[0] - mean_x
                dy = point[1] - mean_y
                radius = (
                    cyy * dx * dx - 2.0 * cxy * dx * dy + cxx * dy * dy
                ) / determinant
                radii.append(radius)
            radii.sort()
            last_index = len(radii) - 1
            radius_p10 = radii[int(last_index * 0.10)]
            radius_p50 = radii[int(last_index * 0.50)]
            radius_p90 = radii[int(last_index * 0.90)]
            radial_spread = (
                (radius_p90 - radius_p10) / max(radius_p50, 1e-9)
            )
            ellipse_like = radial_spread <= ELLIPSE_RADIAL_SPREAD_MAX
        shape = "ellipse" if ellipse_like else "curve"
    return shape, angle, residual


def majority_vote(history, new_value, length=5):
    history.append(new_value)
    if len(history) > length:
        history.pop(0)
    best = new_value
    best_count = 0
    for value in history:
        count = history.count(value)
        if count > best_count:
            best_count = count
            best = value
    return best


def competition_ratio(fit):
    """当前只接受 1:1 模型，其他结果一律拒绝。"""
    if not fit or not fit.get("valid"):
        return UNKNOWN_LABEL
    freq_x = fit["frequency_x"]
    freq_y = fit["frequency_y"]
    if SWAP_RATIO_AXES:
        freq_x, freq_y = freq_y, freq_x
    if (freq_x, freq_y) == (1, 1):
        return "1:1"
    return UNKNOWN_LABEL


def recognition_label(shape, fit):
    """屏幕只显示 1:1 或用户指定的 unkonw。"""
    # 对角线退化时模型相位可能不稳定，但按用户要求始终属于 1:1。
    if shape == "diagonal_line":
        return "1:1"
    # 圆、椭圆和普通宽幅曲线统一由真正的 1:1 模型分数确认。
    if shape in ("ellipse", "single_loop", "curve"):
        return competition_ratio(fit)
    return UNKNOWN_LABEL


def waveform_bounds(blobs):
    """合并黄色波形块的外接范围，不使用亮度网格点。"""
    if not blobs:
        return None
    left = FRAME_W
    top = FRAME_H
    right = 0
    bottom = 0
    for blob in blobs:
        x, y, w, h = blob.rect()
        left = min(left, x)
        top = min(top, y)
        right = max(right, x + w - 1)
        bottom = max(bottom, y + h - 1)
    return (left, top, right - left + 1, bottom - top + 1)


def axis_vpp_volts(bounds, roi, shape):
    """根据当前黄色轨迹范围计算 X/Y 轴的峰峰值电压。"""
    if bounds is None or roi is None:
        return None, None
    pixels_per_div_x = roi[2] / float(GRID_COLUMNS)
    pixels_per_div_y = roi[3] / float(GRID_ROWS)
    if pixels_per_div_x <= 0.0 or pixels_per_div_y <= 0.0:
        return None, None
    span_x = bounds[2]
    span_y = bounds[3]
    vpp_x = span_x / pixels_per_div_x * X_VOLTS_PER_DIV
    vpp_y = span_y / pixels_per_div_y * Y_VOLTS_PER_DIV
    # 线宽、反光会产生少量假跨度；单轴小于网格的 8% 时归零。
    if shape == "vertical_line" or span_x < roi[2] * VPP_ZERO_SPAN_RATIO:
        vpp_x = 0.0
    if shape == "horizontal_line" or span_y < roi[3] * VPP_ZERO_SPAN_RATIO:
        vpp_y = 0.0
    return vpp_x, vpp_y


sensor = None
uart2 = None
try:
    fpioa = FPIOA()
    fpioa.set_function(UART2_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART2_RX_PIN, FPIOA.UART2_RXD)
    uart2 = UART(
        UART.UART2,
        baudrate=UART2_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )

    sensor = Sensor(width=CAPTURE_W, height=CAPTURE_H, fps=TARGET_FPS)
    sensor.reset()
    sensor.set_framesize(width=CAPTURE_W, height=CAPTURE_H)
    sensor.set_pixformat(Sensor.RGB565)
    # 水平镜像和垂直翻转同时开启，等效于画面旋转 180°。
    sensor.set_hmirror(True)
    sensor.set_vflip(True)

    Display.init(
        Display.ST7701, width=DISPLAY_W, height=DISPLAY_H,
        fps=TARGET_FPS, to_ide=True
    )
    MediaManager.init()
    sensor.run()

    clock = time.clock()
    frame_id = 0
    last_uart_send_ms = time.ticks_ms()
    grid_roi = clamp_roi(MANUAL_GRID_ROI) if MANUAL_GRID_ROI else None
    grid_corners = None
    last_points = []
    last_wave_bounds = None
    last_lissajous_fit = None
    last_shape = "searching_grid"
    last_angle = 0.0
    last_residual = 1.0
    last_loop_count = 0
    shape_history = []
    last_ratio_label = UNKNOWN_LABEL
    current_yellow_threshold = YELLOW_LAB

    while True:
        clock.tick()
        img = sensor.snapshot()

        should_detect_grid = (
            grid_roi is None
            or (AUTO_REDETECT_GRID and frame_id % GRID_DETECT_EVERY == 0)
        )
        if MANUAL_GRID_ROI is None and should_detect_grid:
            waveform_roi = find_grid_from_waveform(img, YELLOW_LAB)
            detected_roi, detected_corners = find_grid(img, waveform_roi)
            # K230 实拍中网格可能贴住画面边缘，find_rects 会因外框不闭合而失败。
            # 幅值换算必须以真实网格尺寸为基准；只在外框检测失败时
            # 才使用波形范围估计 ROI。
            if detected_roi is None:
                if waveform_roi is not None:
                    detected_roi = waveform_roi
                    detected_corners = None
            if detected_roi is not None:
                grid_roi = smooth_roi(grid_roi, detected_roi)
                grid_corners = detected_corners

        if grid_roi is not None and frame_id % ANALYZE_EVERY == 0:
            if frame_id % ADAPT_COLOR_EVERY == 0:
                current_yellow_threshold = adaptive_yellow_threshold(
                    img, grid_roi, current_yellow_threshold
                )
            wave_blobs = find_wave_blobs(img, grid_roi, current_yellow_threshold)
            if wave_blobs:
                last_wave_bounds = waveform_bounds(wave_blobs)
                last_points, last_loop_count = extract_scan_points(
                    img, grid_roi, current_yellow_threshold
                )
                raw_shape, last_angle, last_residual = classify_points(last_points, grid_roi)
            else:
                # 二次拍屏可能让黄色完全变白；此时利用亮度并跳过规则网格线。
                # 亮度兜底可用于形状识别，但不可用于 Vpp，否则会把网格当成波形。
                last_wave_bounds = None
                last_points, last_loop_count = extract_bright_scan_points(
                    img, grid_roi
                )
                raw_shape, last_angle, last_residual = classify_points(last_points, grid_roi)
            # 不再用扫描交点数覆盖几何类型。网格会把单椭圆切成碎片，
            # 最终 1:1/unkonw 只由 1:1 模型分数确认。
            should_fit = (
                raw_shape in PHASE_FIT_SHAPES
                and (
                    last_lissajous_fit is None
                    or frame_id % MODEL_FIT_EVERY == 0
                )
            )
            if should_fit:
                local_points = [
                    (
                        point[0] - grid_roi[0],
                        grid_roi[3] - 1 - (point[1] - grid_roi[1]),
                    )
                    for point in last_points
                ]
                last_lissajous_fit = fit_lissajous(
                    local_points,
                    grid_roi[2],
                    grid_roi[3],
                    samples=FIT_MODEL_SAMPLES,
                    bins_x=32,
                    bins_y=32,
                    phase_steps=FIT_PHASE_STEPS,
                    frequency_pairs=FIT_FREQUENCY_PAIRS,
                    neighbor_radius=1,
                    minimum_score=0.56,
                )
            elif raw_shape not in PHASE_FIT_SHAPES:
                # 没有有效波形时不能沿用上一帧的相位结果。
                last_lissajous_fit = None
            if raw_shape in (
                "complex_waveform",
                "horizontal_line", "vertical_line"
            ):
                # 明确的非 1:1 图形不经历史投票，避免旧结果继续停留。
                shape_history = []
                last_shape = raw_shape
            else:
                last_shape = majority_vote(shape_history, raw_shape)

            candidate_ratio = recognition_label(raw_shape, last_lissajous_fit)
            last_ratio_label = candidate_ratio

        if grid_roi is not None:
            img.draw_rectangle(grid_roi, color=(0, 255, 0), thickness=2)
            if grid_corners is not None:
                for corner in grid_corners:
                    img.draw_cross(corner[0], corner[1], color=(255, 0, 0), size=6, thickness=2)
        ratio_label = last_ratio_label
        vpp_x, vpp_y = axis_vpp_volts(
            last_wave_bounds, grid_roi, last_shape
        )
        display_fit = last_lissajous_fit if ratio_label == "1:1" else None
        if display_fit is not None:
            label = "%s phase_deg=%.1f" % (
                ratio_label, display_fit["phase_deg"]
            )
        else:
            label = "%s phase_deg=--" % ratio_label
        if vpp_x is None or vpp_y is None:
            amplitude_text = "VppX=--  VppY=--"
        else:
            amplitude_text = "VppX=%.2fV  VppY=%.2fV" % (
                vpp_x, vpp_y
            )
        draw_status_text(img, label, amplitude_text)
        Display.show_image(img, x=DISPLAY_X, y=0)

        uart_now = time.ticks_ms()
        if time.ticks_diff(uart_now, last_uart_send_ms) >= UART_SEND_INTERVAL_MS:
            uart2.write(make_uart_message(
                ratio_label,
                display_fit,
                vpp_x,
                vpp_y,
                _status_fps,
            ))
            last_uart_send_ms = uart_now

        if frame_id % 30 == 0:
            ratio = last_ratio_label
            score = last_lissajous_fit["score"] if last_lissajous_fit else 0.0
            phase = last_lissajous_fit["phase_deg"] if last_lissajous_fit else 0.0
            print("shape=%s loops=%d ratio=%s phase=%.1f score=%.3f angle=%.2f points=%d fps=%.1f" %
                  (last_shape, last_loop_count, ratio, phase, score, last_angle,
                   len(last_points), clock.fps()))
        frame_id += 1
        os.exitpoint()

except KeyboardInterrupt:
    print("user stopped")
except BaseException as exc:
    print("fatal:", exc)
finally:
    if sensor is not None:
        sensor.stop()
    if uart2 is not None:
        uart2.deinit()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
