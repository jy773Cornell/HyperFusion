# Wavelength / band-index helpers for dual-camera HSI fusion (backend/offline pipeline).
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BandPair:
    fx_band: int
    sw_band: int
    fx_nm: float
    sw_nm: float
    delta_nm: float


def bands_in_range(wavelengths_nm: list[float], wl_min: float, wl_max: float) -> list[int]:
    return [index for index, value in enumerate(wavelengths_nm) if wl_min <= value <= wl_max]


def nearest_band_index(wavelengths_nm: list[float], target_nm: float) -> int:
    if not wavelengths_nm:
        raise ValueError("Wavelength list is empty")
    return min(range(len(wavelengths_nm)), key=lambda index: abs(wavelengths_nm[index] - target_nm))


def build_overlap_band_pairs(
    fx_wavelengths_nm: list[float],
    sw_wavelengths_nm: list[float],
    wl_min: float,
    wl_max: float,
    max_pair_delta_nm: float,
) -> list[BandPair]:
    """Pair each FX band in range with the closest SWIR band; skip poor matches."""
    pairs: list[BandPair] = []
    for fx_band in bands_in_range(fx_wavelengths_nm, wl_min, wl_max):
        fx_nm = fx_wavelengths_nm[fx_band]
        sw_band = nearest_band_index(sw_wavelengths_nm, fx_nm)
        sw_nm = sw_wavelengths_nm[sw_band]
        if not (wl_min <= sw_nm <= wl_max):
            continue
        delta = abs(fx_nm - sw_nm)
        if delta > max_pair_delta_nm:
            continue
        pairs.append(BandPair(fx_band=fx_band, sw_band=sw_band, fx_nm=fx_nm, sw_nm=sw_nm, delta_nm=delta))
    return pairs
