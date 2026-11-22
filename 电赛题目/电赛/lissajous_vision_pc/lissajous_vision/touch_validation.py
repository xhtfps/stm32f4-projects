"""Batch validation for K230 photographs of 1:1 through 1:10 traces."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import re

from .boundary import measure_boundary_touches
from .calibration import configured_corners, warp_grid
from .config import load_config
from .io_utils import read_image
from .mesh import measure_peak_points
from .segmentation import segment_yellow


def _expected_ratio(path: Path) -> int | None:
    match = re.fullmatch(r"(\d+)倍频", path.parent.name)
    return int(match.group(1)) if match else None


def main(argv: list[str] | None = None) -> int:
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="K230 波峰点主判与四边触点辅助回归")
    parser.add_argument(
        "root", nargs="?", type=Path,
        default=project.parent / "26电赛F题拍照" / "网孔图",
    )
    parser.add_argument(
        "--config", type=Path,
        default=project / "configs" / "k230_mesh_dataset.json",
    )
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument(
        "--strict", action="store_true",
        help="要求每张实拍图单帧都精确计数；默认仅输出诊断报告",
    )
    parser.add_argument("--edge-band", type=float, default=0.008)
    parser.add_argument("--max-gap", type=float, default=0.003)
    parser.add_argument("--min-run", type=float, default=0.003)
    args = parser.parse_args(argv)

    config = load_config(args.config)
    records: list[dict[str, object]] = []
    per_class: dict[int, list[dict[str, object]]] = {}
    counts: dict[int, int] = {}
    files = sorted(args.root.glob("*倍频/*.jpg"))
    for path in files:
        expected = _expected_ratio(path)
        if expected is None:
            continue
        counts.setdefault(expected, 0)
        if args.limit and counts[expected] >= args.limit:
            continue
        counts[expected] += 1
        image = read_image(path)
        corners = configured_corners(config, image.shape)
        if corners is None:
            raise RuntimeError("K230 dataset calibration is missing")
        warped = warp_grid(image, corners, config)
        mask, _ = segment_yellow(warped, config)
        measured = measure_boundary_touches(
            mask, edge_band_ratio=args.edge_band,
            maximum_gap_ratio=args.max_gap,
            minimum_run_ratio=args.min_run,
        )
        peaks = measure_peak_points(mask)
        correct = peaks.valid and peaks.peak_count == expected
        within_tolerance = (
            peaks.valid and abs(peaks.peak_count - expected) <= 1
        )
        boundary_correct = (
            measured.valid
            and measured.ratio_y == 1
            and measured.ratio_x == expected
        )
        record = {
            "file": str(path), "expected_y": 1, "expected_x": expected,
            "correct": correct, "within_tolerance": within_tolerance,
            "peak_count": peaks.peak_count,
            "peak_confidence": peaks.confidence,
            "peak_candidates": str(peaks.edge_candidates),
            "boundary_correct": boundary_correct,
            **measured.to_dict(),
        }
        records.append(record)
        per_class.setdefault(expected, []).append(record)

    print("倍数 图片 波峰点有效 波峰点精确 ±1容差 波峰点中值 四边触点中值(L/R/T/B)")
    total_exact = 0
    total_valid = 0
    for expected in sorted(per_class):
        group = per_class[expected]
        valid = [item for item in group if item["peak_confidence"] > 0]
        exact = sum(bool(item["correct"]) for item in group)
        tolerant = sum(bool(item["within_tolerance"]) for item in group)
        total_valid += len(valid)
        total_exact += exact
        medians = []
        for key in ("left", "right", "top", "bottom"):
            values = sorted(int(item[key]) for item in group)
            medians.append(values[len(values) // 2])
        peak_values = sorted(int(item["peak_count"]) for item in group)
        peak_median = peak_values[len(peak_values) // 2]
        print(
            f"{expected:>2}倍 {len(group):>3} {len(valid):>3} {exact:>3} "
            f"{100.0 * exact / max(1, len(group)):>6.1f}% "
            f"{100.0 * tolerant / max(1, len(group)):>6.1f}% "
            f"{peak_median:>3} {tuple(medians)}"
        )
    print(
        f"总图像={len(records)} 波峰点有效={total_valid} "
        f"精确={total_exact} 准确率={100.0 * total_exact / max(1, len(records)):.1f}%"
    )

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", encoding="utf-8-sig", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(records, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    return 0 if records and (not args.strict or total_exact == len(records)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
