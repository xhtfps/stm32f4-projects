"""Evaluate a folder tree whose directory names encode the expected multiplier."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import re

from lissajous_vision import LissajousAnalyzer
from lissajous_vision.config import load_config
from lissajous_vision.io_utils import read_image
from lissajous_vision.mesh import measure_peak_points
from lissajous_vision.boundary import measure_boundary_touches


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def expected_multiplier(folder: Path) -> int:
    match = re.search(r"(\d+)", folder.name)
    if match is None:
        raise ValueError(f"folder name has no multiplier: {folder}")
    return int(match.group(1))


def predicted_multiplier(span_x: float, span_y: float) -> int:
    return int(round((span_x + span_y) * 0.5))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()
    config = load_config(args.config)

    total = correct = 0
    for folder in sorted(
        (path for path in args.root.iterdir() if path.is_dir()),
        key=expected_multiplier,
    ):
        expected = expected_multiplier(folder)
        counts: Counter[str] = Counter()
        failures: list[str] = []
        files = sorted(
            path for path in folder.rglob("*") if path.suffix.lower() in IMAGE_SUFFIXES
        )
        if args.limit:
            files = files[: args.limit]
        for path in files:
            total += 1
            try:
                result = LissajousAnalyzer(
                    config=config, lock_auto_grid=False
                ).analyze(read_image(path))
                measurement = result.measurement
            except Exception as error:  # noqa: BLE001 - keep one bad image from stopping a batch
                counts["exception"] += 1
                failures.append(f"{path.name}: exception={type(error).__name__}: {error}")
                continue
            if not measurement.valid:
                counts["invalid"] += 1
                failures.append(f"{path.name}: invalid={measurement.error}")
                continue
            mesh = measure_peak_points(result.raw_mask, use_template=True)
            boundary = measure_boundary_touches(result.raw_mask)
            if mesh.valid:
                predicted = mesh.peak_count
            else:
                predicted = predicted_multiplier(
                    measurement.span_x_div, measurement.span_y_div
                )
            counts[f"pred_{predicted}"] += 1
            if predicted == expected:
                correct += 1
            else:
                failures.append(
                    f"{path.name}: pred={predicted}, shape={measurement.shape}, "
                    f"span={measurement.span_x_div:.2f}x{measurement.span_y_div:.2f}, "
                    f"confidence={measurement.confidence:.2f}, "
                    f"mesh={mesh.peak_count}/{mesh.edge_candidates}, "
                    f"boundary={boundary.ratio_y}:{boundary.ratio_x} "
                    f"({boundary.top}/{boundary.bottom}/{boundary.left}/{boundary.right})"
                )
        folder_total = sum(counts.values())
        folder_correct = counts.get(f"pred_{expected}", 0)
        print(
            f"{folder.name}: {folder_correct}/{folder_total} "
            f"({folder_correct / folder_total:.1%}) {dict(counts)}"
        )
        for failure in failures[:10]:
            print(f"  {failure}")
        if len(failures) > 10:
            print(f"  ... {len(failures) - 10} more failures")

    print(f"TOTAL: {correct}/{total} ({correct / total:.1%})")
    return 0 if correct == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
