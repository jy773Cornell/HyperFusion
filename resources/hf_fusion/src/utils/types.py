# Shared datatypes for the HF fusion pipeline (backend/offline).
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from src.utils.resample import RgbInterpolation


@dataclass
class FusionPipelineParams:
    """Inputs for FX10e + SWIR3 coarse align, phase correction, and spectral fusion."""

    session: Path
    mode: str = "reflectance"
    cfg: Path | None = None
    margin_mm: float = 5.0
    max_pair_distance_mm: float = 50.0
    rgb_upsample_method: RgbInterpolation = RgbInterpolation.BICUBIC
    overlay_alpha: float = 0.45
    process_hsi: bool = True
    wl_min: float = 977.0
    wl_max: float = 990.0
    max_pair_delta_nm: float = 8.0
    spectral_split_nm: float = 1000.0
    pc_response_min: float = 0.25
    refine_cap_px: float = 3.0
    mask_dilate_px: int = 8
    min_masked_pixels: int = 500
    scale_refine: bool = True
    scale_search_min: float = 0.985
    scale_search_max: float = 1.015
    scale_search_steps: int = 15
    scale_cap: float = 0.02
    roi_mask_mode: str = "intersection"


@dataclass
class RoiFusionResult:
    roi_folder: str
    fx_roi: int
    sw_roi: int
    shift_mm: dict[str, float]
    shift_px: dict[str, float]
    crop_px: dict[str, int]
    crop_size_px: dict[str, int]
    crop_size_mm: dict[str, float]
    overlap_coarse: dict[str, float]
    refine: dict | None = None
    overlap_after: dict[str, float] | None = None
    fused_bands: int | None = None
    wavelength_range_nm: dict[str, float] | None = None
    outputs: dict[str, str] = field(default_factory=dict)


@dataclass
class FusionPipelineResult:
    session: Path
    mode: str
    strategy: str
    reference_camera: str
    grid_mm_per_pixel: float
    upsample_scale: float
    margin_mm: float
    fov_intersection_px: dict[str, int]
    pairs: list[RoiFusionResult]
    alignment_json: Path
    pipeline_complete: bool
