"""PC 版 K230 快速视觉主控的一键启动入口。"""

from __future__ import annotations

from pathlib import Path

from lissajous_vision.desktop_launcher import launch


PROJECT_DIR = Path(__file__).resolve().parent
RUNTIME_CONFIG = PROJECT_DIR / "configs" / "pc_fast_runtime.json"


def run() -> int:
    """Choose a serial port, then start the PC-side F407 controller."""
    return int(launch(RUNTIME_CONFIG))


if __name__ == "__main__":
    raise SystemExit(run())
