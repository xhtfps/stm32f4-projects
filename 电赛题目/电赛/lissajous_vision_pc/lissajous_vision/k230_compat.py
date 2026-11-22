"""Compatibility exports for scripts that used the former K230 controller."""

from .fast_controller import (
    FastSearchController,
    PROBE_TABLE,
    SUPPORTED_INPUT_GRIDS,
)

__all__ = ["FastSearchController", "PROBE_TABLE", "SUPPORTED_INPUT_GRIDS"]
