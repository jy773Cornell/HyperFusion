"""Decode projector (u, v) from one HDMI PSP burst.

26 frames = u + v. Legacy 14-frame folders decode u only.
Maps are NaN outside the lit patch. Decode only — no hardware I/O.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .capture import U_ONLY_STEP_COUNT, STEP_COUNT, FppBurst
from .psp import decode_psp_burst


@dataclass
class DecodeResult:
    mask: np.ndarray
    projector_u: np.ndarray
    projector_v: np.ndarray
    wrapped_phase: np.ndarray
    wrapped_phase_v: np.ndarray
    modulation: np.ndarray
    phase_quality: np.ndarray


def decode_burst(
    burst: FppBurst,
    *,
    min_modulation: float = 0.08,
    min_phase_quality: float = 0.04,
) -> DecodeResult:
    n = len(burst.frames)
    if n not in (U_ONLY_STEP_COUNT, STEP_COUNT):
        raise ValueError(
            f"FPP decode expects {STEP_COUNT} (u+v) or {U_ONLY_STEP_COUNT} (u) frames, got {n}."
        )
    return decode_psp_burst(
        burst,
        min_modulation=min_modulation,
        min_phase_quality=min_phase_quality,
    )
