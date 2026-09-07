# Phase 3 spectral stitching of aligned FX10e + SWIR3 ROI cubes (backend/offline).
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.coarse_alignment import CoarseRoiState, SceneContext
from src.utils.envi import write_bil_cube, write_envi_hdr
from src.utils.roi import fused_cube_basename


@dataclass
class SpectralFusionResult:
    fused_cube: np.ndarray
    fused_wavelengths: list[float]
    fused_bands: int
    wavelength_range_nm: dict[str, float]
    fused_raw: Path
    fused_hdr: Path


def fuse_fx_swir_cubes(
    fx_cube: np.ndarray,
    fx_wavelengths_nm: list[float],
    sw_cube: np.ndarray,
    sw_wavelengths_nm: list[float],
    split_nm: float,
) -> tuple[np.ndarray, list[float]]:
    if fx_cube.shape[:2] != sw_cube.shape[:2]:
        raise ValueError(
            f"Spatial shape mismatch: FX {fx_cube.shape[:2]} vs SWIR {sw_cube.shape[:2]}"
        )
    if len(fx_wavelengths_nm) != fx_cube.shape[2]:
        raise ValueError("FX wavelength count does not match cube bands")
    if len(sw_wavelengths_nm) != sw_cube.shape[2]:
        raise ValueError("SWIR wavelength count does not match cube bands")

    fx_indices = [index for index, wl in enumerate(fx_wavelengths_nm) if wl <= split_nm]
    sw_indices = [index for index, wl in enumerate(sw_wavelengths_nm) if wl > split_nm]
    if not fx_indices:
        raise ValueError(f"No FX bands at or below split {split_nm} nm")
    if not sw_indices:
        raise ValueError(f"No SWIR bands above split {split_nm} nm")

    fx_part = fx_cube[:, :, fx_indices]
    sw_part = sw_cube[:, :, sw_indices]
    fused = np.concatenate([fx_part, sw_part], axis=2)
    wavelengths = [fx_wavelengths_nm[index] for index in fx_indices] + [
        sw_wavelengths_nm[index] for index in sw_indices
    ]
    return fused.astype(np.float32, copy=False), wavelengths


def export_fused_roi(
    scene: SceneContext,
    coarse: CoarseRoiState,
    sw_refined: np.ndarray,
    spectral_split_nm: float,
) -> SpectralFusionResult:
    """Fuse FX + refined SWIR and write ENVI BIL for one ROI."""
    if coarse.fx_cube is None:
        raise ValueError("FX cube required for spectral fusion")
    if scene.fx_meta is None or scene.sw_meta is None:
        raise ValueError("ENVI metadata required for spectral fusion")

    fused_cube, fused_wavelengths = fuse_fx_swir_cubes(
        coarse.fx_cube,
        scene.fx_meta.wavelengths_nm,
        sw_refined,
        scene.sw_meta.wavelengths_nm,
        spectral_split_nm,
    )
    fused_raw = coarse.roi_dir / fused_cube_basename(coarse.fx_roi)
    fused_hdr = fused_raw.with_suffix(".hdr")
    write_bil_cube(fused_raw, fused_cube)
    write_envi_hdr(
        fused_hdr,
        scene.fx_meta,
        samples=fused_cube.shape[1],
        lines=fused_cube.shape[0],
        bands=fused_cube.shape[2],
        raw_basename=fused_raw.name,
        description=(
            f"HyperFusion fused FX10e+SWIR3 ({coarse.roi_folder}, split={spectral_split_nm}nm)"
        ),
        wavelengths_nm=fused_wavelengths,
    )
    return SpectralFusionResult(
        fused_cube=fused_cube,
        fused_wavelengths=fused_wavelengths,
        fused_bands=fused_cube.shape[2],
        wavelength_range_nm={
            "min": round(min(fused_wavelengths), 2),
            "max": round(max(fused_wavelengths), 2),
        },
        fused_raw=fused_raw,
        fused_hdr=fused_hdr,
    )
