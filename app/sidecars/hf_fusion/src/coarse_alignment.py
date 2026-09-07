# Phase 1 coarse spatial alignment: SWIR upsample, centroid match, per-ROI shift and crop (backend/offline).
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from src.utils.types import FusionPipelineParams
from src.utils.config import SpatialCalibration, find_hyperfusion_cfg, load_spatial_calibration
from src.utils.crop import CropRect, compute_roi_crop, crop_image, fov_intersection_rect
from src.utils.dataset import (
    LoadedCameraRgb,
    centroids_from_masks,
    fusion_dir,
    fusion_metadata_dir,
    load_camera_rgb,
    load_fx_segmentation_mask,
    upsample_swir_to_fx_grid,
)
from src.utils.envi import EnviMetadata, parse_envi_hdr, read_bil_crop, read_bil_cube, resolve_ffc_hdr
from src.utils.match import (
    PerRoiShift,
    match_by_sorted_order,
    per_roi_shifts,
    records_from_table,
    shift_spread_mm,
)
from src.utils.resample import (
    crop_hsi_cube,
    shift_hsi_to_canvas,
    shift_mask_to_canvas,
    shift_rgb_to_canvas,
    upsample_hsi_cube,
)
from src.utils.roi import roi_folder_name, roi_rgb_overlay_name
from src.utils.viz import blend_rgb_overlay, draw_centroids, mask_overlap_metrics, save_centroid_table


@dataclass
class SceneContext:
    session: Path
    mode: str
    fusion_out: Path
    fusion_metadata: Path
    cal: SpatialCalibration
    fx: LoadedCameraRgb
    sw: LoadedCameraRgb
    sw_masks_up: dict[int, np.ndarray]
    sw_rgb_up: np.ndarray
    target_w: int
    target_h: int
    canvas_w: int
    canvas_h: int
    fov_rect: CropRect
    shifts: list[PerRoiShift]
    shift_spread: dict[str, float]
    sw_cube_up: np.ndarray | None
    fx_meta: EnviMetadata | None
    sw_meta: EnviMetadata | None


@dataclass
class CoarseRoiState:
    fx_roi: int
    sw_roi: int
    roi_folder: str
    roi_dir: Path
    shift: PerRoiShift
    crop_rect: CropRect
    fx_mask_crop: np.ndarray
    sw_mask_crop: np.ndarray
    overlay_rgb: np.ndarray
    overlap_coarse: dict[str, float]
    fx_cube: np.ndarray | None
    sw_cube: np.ndarray | None


def prepare_scene(params: FusionPipelineParams) -> SceneContext:
    """Upsample SWIR, match centroids, and optionally load upsampled SWIR HSI."""
    session = params.session.resolve()
    if not session.is_dir():
        raise FileNotFoundError(f"Session not found: {session}")

    cfg_path = params.cfg or find_hyperfusion_cfg()
    cal = load_spatial_calibration(cfg_path)
    fusion_out = fusion_dir(session, params.mode)
    fusion_metadata = fusion_metadata_dir(session, params.mode)

    fx = load_camera_rgb(session, params.mode, "fx10e", cal.fx10e_mm_per_pixel)
    sw_rgb_up, target_w, target_h, sw_masks_up = upsample_swir_to_fx_grid(
        session, params.mode, cal, params.rgb_upsample_method
    )
    sw_centroids = centroids_from_masks(
        session, params.mode, "swir3", cal.fx10e_mm_per_pixel, sw_masks_up
    )
    sw = LoadedCameraRgb(
        camera="swir3",
        rgb=sw_rgb_up,
        width=target_w,
        height=target_h,
        mm_per_pixel=cal.fx10e_mm_per_pixel,
        centroids=sw_centroids,
    )

    draw_centroids(fx, f"FX10e {fx.width}x{fx.height} @ {cal.fx10e_mm_per_pixel} mm/px").save(
        fusion_metadata / "fx10e_centroids.png"
    )
    draw_centroids(
        sw,
        f"SWIR3 upsampled {sw.width}x{sw.height} @ {cal.fx10e_mm_per_pixel} mm/px (masks on FX grid)",
    ).save(fusion_metadata / "swir3_centroids.png")

    fx_records = records_from_table(save_centroid_table(fx.centroids))
    sw_records = records_from_table(save_centroid_table(sw.centroids))
    matched = match_by_sorted_order(fx_records, sw_records, max_pair_distance_mm=params.max_pair_distance_mm)
    shifts = per_roi_shifts(matched, cal.fx10e_mm_per_pixel)
    spread = shift_spread_mm(matched)

    canvas_w, canvas_h = fx.width, fx.height
    fov_rect = fov_intersection_rect(canvas_w, canvas_h, target_w, target_h, 0.0, 0.0)
    if not fov_rect.is_valid():
        raise ValueError("FX/SWIR FOV intersection is empty")

    sw_cube_up = None
    fx_meta = None
    sw_meta = None
    if params.process_hsi:
        fx_hdr_path = resolve_ffc_hdr(session, params.mode, "fx10e")
        sw_hdr_path = resolve_ffc_hdr(session, params.mode, "swir3")
        fx_meta = parse_envi_hdr(fx_hdr_path)
        sw_meta = parse_envi_hdr(sw_hdr_path)
        if not fx_meta.wavelengths_nm or not sw_meta.wavelengths_nm:
            raise ValueError("FFC HDR files must include wavelength metadata")
        sw_cube = read_bil_cube(sw_meta)
        sw_cube_up = upsample_hsi_cube(sw_cube, target_w, target_h)

    return SceneContext(
        session=session,
        mode=params.mode,
        fusion_out=fusion_out,
        fusion_metadata=fusion_metadata,
        cal=cal,
        fx=fx,
        sw=sw,
        sw_masks_up=sw_masks_up,
        sw_rgb_up=sw_rgb_up,
        target_w=target_w,
        target_h=target_h,
        canvas_w=canvas_w,
        canvas_h=canvas_h,
        fov_rect=fov_rect,
        shifts=shifts,
        shift_spread=spread,
        sw_cube_up=sw_cube_up,
        fx_meta=fx_meta,
        sw_meta=sw_meta,
    )


def coarse_align_roi(scene: SceneContext, shift: PerRoiShift, params: FusionPipelineParams) -> CoarseRoiState:
    """Per-ROI centroid shift, crop, and optional in-memory HSI extraction."""
    pair = shift.pair
    folder = roi_folder_name(pair.fx_roi)
    roi_dir = scene.fusion_out / folder
    roi_dir.mkdir(parents=True, exist_ok=True)

    fx_mask = load_fx_segmentation_mask(scene.session, scene.mode, pair.fx_roi)
    sw_mask = shift_mask_to_canvas(
        scene.sw_masks_up[pair.sw_roi],
        shift.dx_px,
        shift.dy_px,
        scene.canvas_w,
        scene.canvas_h,
    )
    crop_rect = compute_roi_crop(
        [fx_mask, sw_mask],
        scene.fov_rect,
        margin_mm=params.margin_mm,
        mm_per_pixel=scene.cal.fx10e_mm_per_pixel,
        canvas_width=scene.canvas_w,
        canvas_height=scene.canvas_h,
    )

    sw_rgb_shifted = shift_rgb_to_canvas(
        scene.sw_rgb_up, shift.dx_px, shift.dy_px, scene.canvas_w, scene.canvas_h
    )
    fx_rgb_crop = crop_image(scene.fx.rgb, crop_rect)
    sw_rgb_crop = crop_image(sw_rgb_shifted, crop_rect)
    fx_mask_crop = crop_image(fx_mask, crop_rect)
    sw_mask_crop = crop_image(sw_mask, crop_rect)
    overlay_rgb = blend_rgb_overlay(fx_rgb_crop, sw_rgb_crop, params.overlay_alpha)
    overlap_coarse = mask_overlap_metrics(fx_mask_crop, sw_mask_crop, scene.cal.fx10e_mm_per_pixel)

    rgb_overlay_path = roi_dir / roi_rgb_overlay_name(pair.fx_roi)
    Image.fromarray(overlay_rgb).save(rgb_overlay_path)

    fx_cube = None
    sw_cube = None
    if (
        params.process_hsi
        and scene.sw_cube_up is not None
        and scene.fx_meta is not None
        and scene.sw_meta is not None
    ):
        fx_cube = read_bil_crop(scene.fx_meta, crop_rect.x0, crop_rect.y0, crop_rect.x1, crop_rect.y1)
        sw_on_canvas = shift_hsi_to_canvas(
            scene.sw_cube_up, shift.dx_px, shift.dy_px, scene.canvas_w, scene.canvas_h
        )
        sw_cube = crop_hsi_cube(sw_on_canvas, crop_rect.x0, crop_rect.y0, crop_rect.x1, crop_rect.y1)

    return CoarseRoiState(
        fx_roi=pair.fx_roi,
        sw_roi=pair.sw_roi,
        roi_folder=folder,
        roi_dir=roi_dir,
        shift=shift,
        crop_rect=crop_rect,
        fx_mask_crop=fx_mask_crop,
        sw_mask_crop=sw_mask_crop,
        overlay_rgb=overlay_rgb,
        overlap_coarse=overlap_coarse,
        fx_cube=fx_cube,
        sw_cube=sw_cube,
    )
