"""李萨如曲线模型拟合核心；仅使用 Python 标准语法，兼容 CPython 和 MicroPython。"""

import math


# 模型只与频率、相位和占据网格参数有关，缓存后不再在每次视频帧中重复计算 sin()。
_MODEL_CACHE = {}


def _gcd(a, b):
    while b:
        a, b = b, a % b
    return a


def _percentile(values, fraction):
    ordered = sorted(values)
    index = int((len(ordered) - 1) * fraction)
    return ordered[max(0, min(len(ordered) - 1, index))]


def _model_cells(freq_x, freq_y, phase, samples, bins_x, bins_y):
    cells = set()
    for index in range(samples):
        t = 2.0 * math.pi * index / samples
        nx = math.sin(freq_x * t + phase)
        ny = math.sin(freq_y * t)
        x = int(round((nx + 1.0) * 0.5 * (bins_x - 1)))
        y = int(round((ny + 1.0) * 0.5 * (bins_y - 1)))
        cells.add(y * bins_x + x)
    return cells


def _expand_cells(cells, radius, bins_x, bins_y):
    """一次性扩张占据格，后续可用集合交集代替逐点九宫格搜索。"""
    if radius <= 0:
        return cells
    expanded = set()
    for cell in cells:
        x = cell % bins_x
        y = cell // bins_x
        for dx in range(-radius, radius + 1):
            nx = x + dx
            if nx < 0 or nx >= bins_x:
                continue
            for dy in range(-radius, radius + 1):
                ny = y + dy
                if 0 <= ny < bins_y:
                    expanded.add(ny * bins_x + nx)
    return expanded


def _get_cached_models(
    max_frequency, frequency_pairs, phase_steps, samples,
    bins_x, bins_y, neighbor_radius
):
    if frequency_pairs is None:
        pairs = []
        for freq_x in range(1, max_frequency + 1):
            for freq_y in range(1, max_frequency + 1):
                if _gcd(freq_x, freq_y) == 1:
                    pairs.append((freq_x, freq_y))
    else:
        pairs = [(int(pair[0]), int(pair[1])) for pair in frequency_pairs]
    pairs = tuple(pairs)
    key = (pairs, phase_steps, samples, bins_x, bins_y, neighbor_radius)
    cached = _MODEL_CACHE.get(key)
    if cached is not None:
        return cached

    cached = []
    for freq_x, freq_y in pairs:
        for phase_index in range(phase_steps):
            phase = 2.0 * math.pi * phase_index / phase_steps
            model = _model_cells(
                freq_x, freq_y, phase, samples, bins_x, bins_y
            )
            expanded_model = _expand_cells(
                model, neighbor_radius, bins_x, bins_y
            )
            cached.append((
                freq_x, freq_y, phase * 180.0 / math.pi,
                model, expanded_model
            ))
    _MODEL_CACHE[key] = cached
    return cached


def generate_model_points(fit, samples=240):
    """根据拟合结果生成干净、连续的像素坐标。"""
    if not fit or not fit.get("valid"):
        return []
    points = []
    phase = fit["phase_deg"] * math.pi / 180.0
    for index in range(samples):
        t = 2.0 * math.pi * index / samples
        x = fit["center_x"] + fit["amplitude_x"] * math.sin(fit["frequency_x"] * t + phase)
        y = fit["center_y"] + fit["amplitude_y"] * math.sin(fit["frequency_y"] * t)
        points.append((int(round(x)), int(round(y))))
    return points


def fit_lissajous(
    points,
    width,
    height,
    max_frequency=4,
    phase_steps=16,
    samples=160,
    bins_x=48,
    bins_y=40,
    neighbor_radius=1,
    minimum_score=0.52,
    complexity_penalty=0.015,
    frequency_pairs=None,
):
    """拟合 x=A*sin(m*t+delta), y=B*sin(n*t)，返回频率比、相位和置信度。"""
    empty = {
        "valid": False,
        "score": 0.0,
        "frequency_x": 0,
        "frequency_y": 0,
        "ratio": "unkown",
        "phase_deg": 0.0,
        "center_x": 0.0,
        "center_y": 0.0,
        "amplitude_x": 0.0,
        "amplitude_y": 0.0,
    }
    if points is None or len(points) < 20:
        return empty

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    low_x = _percentile(xs, 0.01)
    high_x = _percentile(xs, 0.99)
    low_y = _percentile(ys, 0.01)
    high_y = _percentile(ys, 0.99)
    span_x = high_x - low_x
    span_y = high_y - low_y
    if span_x < width * 0.16 or span_y < height * 0.16:
        return empty

    center_x = (low_x + high_x) * 0.5
    center_y = (low_y + high_y) * 0.5
    amplitude_x = max(1.0, span_x * 0.5)
    amplitude_y = max(1.0, span_y * 0.5)
    observed = set()
    for x, y in points:
        nx = (x - center_x) / amplitude_x
        ny = (y - center_y) / amplitude_y
        if abs(nx) <= 1.20 and abs(ny) <= 1.20:
            bx = int(round((nx + 1.0) * 0.5 * (bins_x - 1)))
            by = int(round((ny + 1.0) * 0.5 * (bins_y - 1)))
            observed.add(by * bins_x + bx)
    if len(observed) < 12:
        return empty

    best_score = -1.0
    best_selection_score = -1.0
    best_x = 0
    best_y = 0
    best_phase = 0.0
    expanded_observed = _expand_cells(
        observed, neighbor_radius, bins_x, bins_y
    )
    models = _get_cached_models(
        max_frequency, frequency_pairs, phase_steps, samples,
        bins_x, bins_y, neighbor_radius
    )
    for freq_x, freq_y, phase_deg, model, expanded_model in models:
        recall = len(model & expanded_observed) / len(model)
        precision = len(observed & expanded_model) / len(observed)
        score = 2.0 * recall * precision / max(recall + precision, 1e-9)
        # 实拍曲线越界被裁切时，高阶模型可能用多余分支“解释”边缘，从而只比
        # 正确的简单频率比高一丁点。采用很弱的奥卡姆惩罚，仅在原始得分接近时
        # 优先更简单模型。
        selection_score = score - complexity_penalty * (freq_x + freq_y - 2)
        if selection_score > best_selection_score:
            best_selection_score = selection_score
            best_score = score
            best_x = freq_x
            best_y = freq_y
            best_phase = phase_deg

    is_valid = best_score >= minimum_score
    return {
        "valid": is_valid,
        "score": round(best_score, 5),
        "frequency_x": best_x,
        "frequency_y": best_y,
        "ratio": "%d:%d" % (best_x, best_y) if is_valid else "unkown",
        "phase_deg": round(best_phase, 2),
        "center_x": round(center_x, 3),
        "center_y": round(center_y, 3),
        "amplitude_x": round(amplitude_x, 3),
        "amplitude_y": round(amplitude_y, 3),
    }
