"""Hierarchical sine PSP (adapter). 1 / 8 / 80 periods, 4 shifts, u then v.

No hardware I/O.
"""

from __future__ import annotations

import numpy as np

from .capture import PROJECTOR_HEIGHT_PX, PROJECTOR_WIDTH_PX, U_ONLY_STEP_COUNT, FppBurst
from .mask import illumination_mask, normalize_in_mask

PSP_FREQUENCIES = (1.0, 8.0, 80.0)
PSP_SHIFTS_DEG = (0, 90, 180, 270)
PSP_U_FIRST_SINE_STEP = 2
PSP_V_FIRST_SINE_STEP = 14


def _four_step_phase(
    n0: np.ndarray,
    n90: np.ndarray,
    n180: np.ndarray,
    n270: np.ndarray,
    mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    d_cos = n0 - n180
    d_sin = n90 - n270
    phase = np.full(n0.shape, np.nan, dtype=np.float32)
    quality = np.zeros(n0.shape, dtype=np.float32)
    valid = mask & np.isfinite(d_cos) & np.isfinite(d_sin)
    phase[valid] = np.arctan2(d_sin[valid], d_cos[valid]).astype(np.float32)
    quality[valid] = np.hypot(d_cos[valid], d_sin[valid]).astype(np.float32)
    return phase, quality


def temporal_unwrap(
    phase_l: np.ndarray,
    phase_h: np.ndarray,
    lamb_l: float,
    lamb_h: float,
) -> np.ndarray:
    """Zuo TPU: k = round((λl/λh) φl − φh) / 2π."""
    k = np.round(((lamb_l / lamb_h) * phase_l - phase_h) / (2.0 * np.pi))
    return (phase_h + 2.0 * np.pi * k).astype(np.float32)


def _monotonic_one_period(phase: np.ndarray) -> np.ndarray:
    """Lift atan2 [−π, π] to [0, 2π). Cosine PNGs have φ=0 at axis origin."""
    out = phase.astype(np.float32, copy=True)
    neg = np.isfinite(out) & (out < 0.0)
    out[neg] = out[neg] + np.float32(2.0 * np.pi)
    return out


def _decode_axis(
    norm,
    mask: np.ndarray,
    first_sine_step: int,
    length_px: float,
    min_phase_quality: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    phases: list[np.ndarray] = []
    quality_last = np.zeros(mask.shape, dtype=np.float32)
    for fi, _freq in enumerate(PSP_FREQUENCIES):
        base = first_sine_step + fi * 4
        phase, quality = _four_step_phase(
            norm(base), norm(base + 1), norm(base + 2), norm(base + 3), mask
        )
        phases.append(phase)
        quality_last = quality

    unwrapped = [_monotonic_one_period(phases[0])]
    for i in range(1, len(PSP_FREQUENCIES)):
        unwrapped.append(
            temporal_unwrap(
                unwrapped[i - 1],
                phases[i],
                1.0 / PSP_FREQUENCIES[i - 1],
                1.0 / PSP_FREQUENCIES[i],
            )
        )

    coord = np.full(mask.shape, np.nan, dtype=np.float32)
    phi = unwrapped[-1]
    valid = mask & np.isfinite(phi)
    freq_h = float(PSP_FREQUENCIES[-1])
    coord[valid] = (phi[valid] / (2.0 * np.pi) * (length_px / freq_h)).astype(np.float32)
    coord[valid] = np.clip(coord[valid], 0.0, length_px - 1.0)
    good = valid & (quality_last >= min_phase_quality)
    coord = np.where(good, coord, np.nan).astype(np.float32)
    phase_out = np.where(good, phases[-1], np.nan).astype(np.float32)
    return coord, phase_out, quality_last


def decode_psp_burst(
    burst: FppBurst,
    *,
    min_modulation: float = 0.15,
    min_phase_quality: float = 0.04,
):
    from .decode import DecodeResult

    mask = illumination_mask(burst, min_modulation=min_modulation)
    black = burst.image(0)
    white = burst.image(1)
    modulation = np.clip(white - black, 0.0, 1.0).astype(np.float32)

    def norm(step: int) -> np.ndarray:
        return normalize_in_mask(burst.image(step), black, white, mask)

    u, phase_u, quality_u = _decode_axis(
        norm, mask, PSP_U_FIRST_SINE_STEP, float(PROJECTOR_WIDTH_PX), min_phase_quality
    )
    v = np.full(mask.shape, np.nan, dtype=np.float32)
    phase_v = np.full(mask.shape, np.nan, dtype=np.float32)
    quality = quality_u
    if len(burst.frames) > U_ONLY_STEP_COUNT:
        v, phase_v, quality_v = _decode_axis(
            norm, mask, PSP_V_FIRST_SINE_STEP, float(PROJECTOR_HEIGHT_PX), min_phase_quality
        )
        quality = np.minimum(quality_u, quality_v)
        both = np.isfinite(u) & np.isfinite(v)
        u = np.where(both, u, np.nan).astype(np.float32)
        v = np.where(both, v, np.nan).astype(np.float32)
        mask = both
    else:
        mask = np.isfinite(u)

    return DecodeResult(
        mask=mask,
        projector_u=u,
        projector_v=v,
        wrapped_phase=phase_u,
        wrapped_phase_v=phase_v,
        modulation=modulation,
        phase_quality=quality,
    )
