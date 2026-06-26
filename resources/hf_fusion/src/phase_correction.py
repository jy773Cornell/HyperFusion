# Phase 2 overlap-band phase correlation + uniform scale search (backend/offline).
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from src.coarse_alignment import CoarseRoiState, SceneContext
from src.utils.masks import pc_chip_mask, sw_mask_after_refine
from src.utils.types import FusionPipelineParams
from src.utils.resample import shift_hsi_cube, similarity_warp_hsi_cube
from src.utils.viz import mask_overlap_metrics
from src.utils.wavelength import BandPair, build_overlap_band_pairs


@dataclass
class IntensityRefineResult:
    dx_px: float
    dy_px: float
    response: float
    accepted: bool
    pair_count: int
    shifts: list[dict]
    reason: str = ""
    scale: float = 1.0


@dataclass
class PhaseCorrectionResult:
    sw_refined: np.ndarray
    sw_mask_after: np.ndarray
    refine: IntensityRefineResult
    overlap_after: dict[str, float]
    refine_center_x: float
    refine_center_y: float


def mask_centroid(mask: np.ndarray) -> tuple[float, float]:
    moments = cv2.moments(mask.astype(np.uint8))
    if moments["m00"] < 1e-6:
        height, width = mask.shape
        return width / 2.0, height / 2.0
    return float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"])


def _scale_band_about(band: np.ndarray, scale: float, center_x: float, center_y: float) -> np.ndarray:
    matrix = cv2.getRotationMatrix2D((center_x, center_y), 0.0, scale)
    return cv2.warpAffine(
        band.astype(np.float32),
        matrix,
        (band.shape[1], band.shape[0]),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0.0,
    )


def _scale_translation_search(
    reference: np.ndarray,
    moving: np.ndarray,
    window: np.ndarray,
    center_x: float,
    center_y: float,
    scale_min: float,
    scale_max: float,
    scale_steps: int,
) -> tuple[float, float, float, float]:
    best_scale = 1.0
    best_dx = 0.0
    best_dy = 0.0
    best_response = -1.0
    for scale in np.linspace(scale_min, scale_max, scale_steps):
        mov_scaled = _scale_band_about(moving, float(scale), center_x, center_y)
        try:
            dx, dy, response = phase_correlation_shift(reference, mov_scaled, window)
        except cv2.error:
            continue
        if response > best_response:
            best_scale = float(scale)
            best_dx = dx
            best_dy = dy
            best_response = response
    if best_response < 0:
        raise cv2.error("scale_translation_search failed for all scales")
    return best_scale, best_dx, best_dy, best_response


def build_refine_mask(
    fx_band: np.ndarray,
    sw_band: np.ndarray,
    dilate_px: int = 8,
) -> np.ndarray:
    valid = (fx_band > 0) & (sw_band > 0) & np.isfinite(fx_band) & np.isfinite(sw_band)
    mask = valid.astype(np.uint8)
    if dilate_px > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px * 2 + 1, dilate_px * 2 + 1))
        mask = cv2.dilate(mask, kernel, iterations=1)
    return mask


def normalize_band_in_mask(band: np.ndarray, mask: np.ndarray) -> np.ndarray:
    region = mask > 0
    if int(np.count_nonzero(region)) < 16:
        raise ValueError("Mask too small for band normalization")
    values = band[region].astype(np.float64)
    mean = float(values.mean())
    std = float(values.std())
    if std < 1e-6:
        std = 1.0
    out = np.zeros_like(band, dtype=np.float32)
    out[region] = ((band[region].astype(np.float64) - mean) / std).astype(np.float32)
    return out


def phase_correlation_shift(
    reference: np.ndarray,
    moving: np.ndarray,
    window: np.ndarray | None = None,
) -> tuple[float, float, float]:
    ref = reference.astype(np.float32)
    mov = moving.astype(np.float32)
    win = None
    if window is not None:
        win = window.astype(np.float32)
        if not np.any(win > 0):
            raise ValueError("Correlation window is empty")
    shift, response = cv2.phaseCorrelate(ref, mov, win)
    return float(shift[0]), float(shift[1]), float(response)


def median_intensity_refine(
    fx_cube: np.ndarray,
    sw_cube: np.ndarray,
    band_pairs: list[BandPair],
    *,
    chip_mask: np.ndarray | None = None,
    mask_dilate_px: int = 8,
    pc_response_min: float = 0.25,
    refine_cap_px: float = 3.0,
    min_masked_pixels: int = 500,
) -> IntensityRefineResult:
    if not band_pairs:
        return IntensityRefineResult(0.0, 0.0, 0.0, False, 0, [], "no_overlap_band_pairs")

    shifts: list[dict] = []
    dx_values: list[float] = []
    dy_values: list[float] = []
    responses: list[float] = []

    for pair in band_pairs:
        fx_band = fx_cube[:, :, pair.fx_band]
        sw_band = sw_cube[:, :, pair.sw_band]
        if chip_mask is not None:
            mask = chip_mask.astype(np.uint8)
            if mask_dilate_px > 0:
                kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (mask_dilate_px * 2 + 1, mask_dilate_px * 2 + 1)
                )
                mask = cv2.dilate(mask, kernel, iterations=1)
        else:
            mask = build_refine_mask(fx_band, sw_band, dilate_px=mask_dilate_px)
        if int(np.count_nonzero(mask)) < min_masked_pixels:
            continue
        ref = normalize_band_in_mask(fx_band, mask)
        mov = normalize_band_in_mask(sw_band, mask)
        try:
            dx, dy, response = phase_correlation_shift(ref, mov, mask)
        except cv2.error:
            continue
        shifts.append(
            {
                "fx_band": pair.fx_band,
                "sw_band": pair.sw_band,
                "fx_nm": round(pair.fx_nm, 2),
                "sw_nm": round(pair.sw_nm, 2),
                "dx_px": round(dx, 4),
                "dy_px": round(dy, 4),
                "response": round(response, 4),
            }
        )
        dx_values.append(dx)
        dy_values.append(dy)
        responses.append(response)

    if not dx_values:
        return IntensityRefineResult(0.0, 0.0, 0.0, False, 0, shifts, "all_band_pairs_failed")

    dx = float(np.median(dx_values))
    dy = float(np.median(dy_values))
    response = float(np.median(responses))

    if response < pc_response_min:
        return IntensityRefineResult(
            0.0, 0.0, response, False, len(dx_values), shifts, f"response_below_{pc_response_min}"
        )

    magnitude = (dx**2 + dy**2) ** 0.5
    if magnitude > refine_cap_px:
        scale = refine_cap_px / magnitude
        dx *= scale
        dy *= scale
        reason = f"capped_at_{refine_cap_px}px"
    else:
        reason = "accepted"

    return IntensityRefineResult(dx, dy, response, True, len(dx_values), shifts, reason)


def median_scale_translation_refine(
    fx_cube: np.ndarray,
    sw_cube: np.ndarray,
    band_pairs: list[BandPair],
    *,
    chip_mask: np.ndarray | None = None,
    mask_dilate_px: int = 8,
    pc_response_min: float = 0.25,
    refine_cap_px: float = 3.0,
    scale_search_min: float = 0.985,
    scale_search_max: float = 1.015,
    scale_search_steps: int = 15,
    scale_cap: float = 0.02,
    min_masked_pixels: int = 500,
) -> IntensityRefineResult:
    if not band_pairs:
        return IntensityRefineResult(0.0, 0.0, 0.0, False, 0, [], "no_overlap_band_pairs")

    if chip_mask is None:
        return median_intensity_refine(
            fx_cube,
            sw_cube,
            band_pairs,
            chip_mask=None,
            mask_dilate_px=mask_dilate_px,
            pc_response_min=pc_response_min,
            refine_cap_px=refine_cap_px,
            min_masked_pixels=min_masked_pixels,
        )

    mask = chip_mask.astype(np.uint8)
    if mask_dilate_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (mask_dilate_px * 2 + 1, mask_dilate_px * 2 + 1)
        )
        mask = cv2.dilate(mask, kernel, iterations=1)
    if int(np.count_nonzero(mask)) < min_masked_pixels:
        return IntensityRefineResult(0.0, 0.0, 0.0, False, 0, [], "mask_too_small")

    center_x, center_y = mask_centroid(mask)
    shifts: list[dict] = []
    scale_values: list[float] = []
    dx_values: list[float] = []
    dy_values: list[float] = []
    responses: list[float] = []

    for pair in band_pairs:
        fx_band = fx_cube[:, :, pair.fx_band]
        sw_band = sw_cube[:, :, pair.sw_band]
        ref = normalize_band_in_mask(fx_band, mask)
        mov = normalize_band_in_mask(sw_band, mask)
        try:
            scale, dx, dy, response = _scale_translation_search(
                ref,
                mov,
                mask,
                center_x,
                center_y,
                scale_search_min,
                scale_search_max,
                scale_search_steps,
            )
        except cv2.error:
            continue
        shifts.append(
            {
                "fx_band": pair.fx_band,
                "sw_band": pair.sw_band,
                "fx_nm": round(pair.fx_nm, 2),
                "sw_nm": round(pair.sw_nm, 2),
                "scale": round(scale, 6),
                "dx_px": round(dx, 4),
                "dy_px": round(dy, 4),
                "response": round(response, 4),
            }
        )
        scale_values.append(scale)
        dx_values.append(dx)
        dy_values.append(dy)
        responses.append(response)

    if not dx_values:
        return IntensityRefineResult(0.0, 0.0, 0.0, False, 0, shifts, "all_band_pairs_failed")

    scale = float(np.median(scale_values))
    dx = float(np.median(dx_values))
    dy = float(np.median(dy_values))
    response = float(np.median(responses))

    if response < pc_response_min:
        return IntensityRefineResult(
            0.0, 0.0, response, False, len(dx_values), shifts, f"response_below_{pc_response_min}", scale=1.0
        )

    if abs(scale - 1.0) > scale_cap:
        scale = 1.0 + scale_cap if scale > 1.0 else 1.0 - scale_cap
        reason = f"scale_capped_at_{scale_cap}"
    else:
        reason = "accepted"

    magnitude = (dx**2 + dy**2) ** 0.5
    if magnitude > refine_cap_px:
        factor = refine_cap_px / magnitude
        dx *= factor
        dy *= factor
        reason = f"shift_capped_at_{refine_cap_px}px"

    if abs(scale - 1.0) < 1e-6 and magnitude < 1e-6:
        reason = "identity"

    return IntensityRefineResult(dx, dy, response, True, len(dx_values), shifts, reason, scale=scale)


def _run_refine(
    fx_cube: np.ndarray,
    sw_cube: np.ndarray,
    pc_mask: np.ndarray,
    band_pairs: list[BandPair],
    params: FusionPipelineParams,
) -> IntensityRefineResult:
    if params.scale_refine:
        return median_scale_translation_refine(
            fx_cube,
            sw_cube,
            band_pairs,
            chip_mask=pc_mask,
            mask_dilate_px=0,
            pc_response_min=params.pc_response_min,
            refine_cap_px=params.refine_cap_px,
            scale_search_min=params.scale_search_min,
            scale_search_max=params.scale_search_max,
            scale_search_steps=params.scale_search_steps,
            scale_cap=params.scale_cap,
            min_masked_pixels=params.min_masked_pixels,
        )
    return median_intensity_refine(
        fx_cube,
        sw_cube,
        band_pairs,
        chip_mask=pc_mask,
        mask_dilate_px=0,
        pc_response_min=params.pc_response_min,
        refine_cap_px=params.refine_cap_px,
        min_masked_pixels=params.min_masked_pixels,
    )


def apply_phase_correction(
    scene: SceneContext,
    coarse: CoarseRoiState,
    params: FusionPipelineParams,
) -> PhaseCorrectionResult:
    """Overlap-band PC (+ optional scale search) and warp SWIR cube on the ROI crop."""
    if coarse.fx_cube is None or coarse.sw_cube is None:
        raise ValueError("HSI cubes required for phase correction")
    if scene.fx_meta is None or scene.sw_meta is None:
        raise ValueError("ENVI metadata required for phase correction")

    pc_mask = pc_chip_mask(coarse.fx_mask_crop, coarse.sw_mask_crop, dilate_px=params.mask_dilate_px)
    band_pairs = build_overlap_band_pairs(
        scene.fx_meta.wavelengths_nm,
        scene.sw_meta.wavelengths_nm,
        params.wl_min,
        params.wl_max,
        params.max_pair_delta_nm,
    )
    refine_center_x, refine_center_y = mask_centroid(pc_mask)
    refine = _run_refine(coarse.fx_cube, coarse.sw_cube, pc_mask, band_pairs, params)

    sw_refined = coarse.sw_cube
    if refine.accepted:
        has_scale = params.scale_refine and abs(refine.scale - 1.0) > 1e-6
        has_shift = abs(refine.dx_px) > 1e-6 or abs(refine.dy_px) > 1e-6
        if has_scale or has_shift:
            if params.scale_refine:
                sw_refined = similarity_warp_hsi_cube(
                    coarse.sw_cube,
                    refine.scale,
                    refine.dx_px,
                    refine.dy_px,
                    refine_center_x,
                    refine_center_y,
                )
            else:
                sw_refined = shift_hsi_cube(coarse.sw_cube, refine.dx_px, refine.dy_px)

    sw_mask_after = sw_mask_after_refine(
        coarse.sw_mask_crop,
        refine.dx_px,
        refine.dy_px,
        scale=refine.scale if params.scale_refine and refine.accepted else 1.0,
        center_x=refine_center_x,
        center_y=refine_center_y,
    )
    overlap_after = mask_overlap_metrics(coarse.fx_mask_crop, sw_mask_after, scene.cal.fx10e_mm_per_pixel)
    return PhaseCorrectionResult(
        sw_refined=sw_refined,
        sw_mask_after=sw_mask_after,
        refine=refine,
        overlap_after=overlap_after,
        refine_center_x=refine_center_x,
        refine_center_y=refine_center_y,
    )
