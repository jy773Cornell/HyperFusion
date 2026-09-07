# HF fusion pipeline integrator: coarse alignment → phase correction → spectral fusion (backend/offline).
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from src.coarse_alignment import coarse_align_roi, prepare_scene
from src.phase_correction import IntensityRefineResult, apply_phase_correction
from src.spectral_fusion import export_fused_roi
from src.utils.masks import fused_chip_mask, save_roi_mask
from src.utils.types import FusionPipelineParams, FusionPipelineResult, RoiFusionResult
from src.utils.roi import (
    roi_mask_npy_name,
    roi_mask_overlay_outline_name,
    roi_mask_png_name,
    roi_rgb_overlay_name,
    roi_spectra_csv_name,
    roi_spectra_plot_name,
)
from src.utils.roi_spectra import (
    RoiSpectraRow,
    export_fusion_roi_spectra,
    write_combined_roi_spectra_artifacts,
)
from src.utils.viz import draw_mask_overlap_outline

__all__ = [
    "FusionPipelineParams",
    "FusionPipelineResult",
    "RoiFusionResult",
    "run_fusion_pipeline",
]


def _refine_result_to_dict(
    result: IntensityRefineResult,
    *,
    wl_min: float,
    wl_max: float,
    spectral_split_nm: float,
    total_shift_px: dict[str, float],
    scale_refine: bool,
) -> dict:
    method = "scale_translation_pc" if scale_refine else "phase_correlation"
    payload = {
        "method": method,
        "dx_px": round(result.dx_px, 4),
        "dy_px": round(result.dy_px, 4),
        "response": round(result.response, 4),
        "accepted": result.accepted,
        "pair_count": result.pair_count,
        "reason": result.reason,
        "band_shifts": result.shifts,
        "wl_window_nm": [wl_min, wl_max],
        "spectral_split_nm": spectral_split_nm,
        "pc_region": "chip_segmentation_mask",
        "total_shift_px": total_shift_px,
    }
    if scale_refine:
        payload["scale"] = round(result.scale, 6)
        payload["scale_percent"] = round(100.0 * result.scale, 4)
    return payload


def run_fusion_pipeline(params: FusionPipelineParams) -> FusionPipelineResult:
    scene = prepare_scene(params)
    roi_results: list[RoiFusionResult] = []
    alignment_pairs: list[dict] = []
    spectrum_rows: list[RoiSpectraRow] = []

    for shift in scene.shifts:
        coarse = coarse_align_roi(scene, shift, params)
        outputs: dict[str, str] = {
            "rgb_overlay": str(coarse.roi_dir / roi_rgb_overlay_name(coarse.fx_roi)),
        }

        refine_dict: dict | None = None
        overlap_after: dict[str, float] | None = None
        fused_bands: int | None = None
        wavelength_range: dict[str, float] | None = None

        if params.process_hsi and coarse.fx_cube is not None and coarse.sw_cube is not None:
            corrected = apply_phase_correction(scene, coarse, params)
            overlap_after = corrected.overlap_after

            mask_outline_path = coarse.roi_dir / roi_mask_overlay_outline_name(coarse.fx_roi)
            draw_mask_overlap_outline(
                coarse.overlay_rgb,
                coarse.fx_mask_crop,
                corrected.sw_mask_after,
                overlap_after,
                title=coarse.roi_folder,
            ).save(mask_outline_path)
            outputs["maskoverlay_outline"] = str(mask_outline_path)

            roi_mask = fused_chip_mask(
                coarse.fx_mask_crop, corrected.sw_mask_after, mode=params.roi_mask_mode
            )
            roi_mask_npy_path = coarse.roi_dir / roi_mask_npy_name(coarse.fx_roi)
            roi_mask_png_path = coarse.roi_dir / roi_mask_png_name(coarse.fx_roi)
            save_roi_mask(roi_mask, roi_mask_npy_path, roi_mask_png_path)
            outputs["roi_mask_npy"] = str(roi_mask_npy_path)
            outputs["roi_mask_png"] = str(roi_mask_png_path)

            fused = export_fused_roi(scene, coarse, corrected.sw_refined, params.spectral_split_nm)
            outputs["fused_hdr"] = str(fused.fused_hdr)
            outputs["fused_raw"] = str(fused.fused_raw)
            fused_bands = fused.fused_bands
            wavelength_range = fused.wavelength_range_nm

            spectra_row = export_fusion_roi_spectra(
                session=scene.session,
                mode=scene.mode,
                roi_dir=coarse.roi_dir,
                fx_roi=coarse.fx_roi,
                fused_cube=fused.fused_cube,
                wavelengths_nm=fused.fused_wavelengths,
                mask=roi_mask,
            )
            spectrum_rows.append(spectra_row)
            outputs["roi_spectra_csv"] = str(coarse.roi_dir / roi_spectra_csv_name())
            outputs["roi_spectra_plot"] = str(coarse.roi_dir / roi_spectra_plot_name())

            total_shift_px = {
                "dx": round(coarse.shift.dx_px + corrected.refine.dx_px, 4),
                "dy": round(coarse.shift.dy_px + corrected.refine.dy_px, 4),
            }
            refine_dict = _refine_result_to_dict(
                corrected.refine,
                wl_min=params.wl_min,
                wl_max=params.wl_max,
                spectral_split_nm=params.spectral_split_nm,
                total_shift_px=total_shift_px,
                scale_refine=params.scale_refine,
            )

        elif not params.process_hsi:
            sw_mask_final = coarse.sw_mask_crop
            mask_outline_path = coarse.roi_dir / roi_mask_overlay_outline_name(coarse.fx_roi)
            draw_mask_overlap_outline(
                coarse.overlay_rgb,
                coarse.fx_mask_crop,
                sw_mask_final,
                coarse.overlap_coarse,
                title=coarse.roi_folder,
            ).save(mask_outline_path)
            outputs["maskoverlay_outline"] = str(mask_outline_path)

            roi_mask = fused_chip_mask(coarse.fx_mask_crop, sw_mask_final, mode=params.roi_mask_mode)
            roi_mask_npy_path = coarse.roi_dir / roi_mask_npy_name(coarse.fx_roi)
            roi_mask_png_path = coarse.roi_dir / roi_mask_png_name(coarse.fx_roi)
            save_roi_mask(roi_mask, roi_mask_npy_path, roi_mask_png_path)
            outputs["roi_mask_npy"] = str(roi_mask_npy_path)
            outputs["roi_mask_png"] = str(roi_mask_png_path)

        roi_result = RoiFusionResult(
            roi_folder=coarse.roi_folder,
            fx_roi=coarse.fx_roi,
            sw_roi=coarse.sw_roi,
            shift_mm={"dx": round(coarse.shift.dx_mm, 3), "dy": round(coarse.shift.dy_mm, 3)},
            shift_px={"dx": round(coarse.shift.dx_px, 3), "dy": round(coarse.shift.dy_px, 3)},
            crop_px=coarse.crop_rect.as_dict(),
            crop_size_px={"width": coarse.crop_rect.width, "height": coarse.crop_rect.height},
            crop_size_mm=coarse.crop_rect.size_mm(scene.cal.fx10e_mm_per_pixel),
            overlap_coarse=coarse.overlap_coarse,
            refine=refine_dict,
            overlap_after=overlap_after,
            fused_bands=fused_bands,
            wavelength_range_nm=wavelength_range,
            outputs=outputs,
        )
        roi_results.append(roi_result)

        pair_entry: dict[str, Any] = {
            "roi_folder": coarse.roi_folder,
            "fx_roi": coarse.fx_roi,
            "sw_roi": coarse.sw_roi,
            "shift_mm": roi_result.shift_mm,
            "shift_px": roi_result.shift_px,
            "margin_mm": params.margin_mm,
            "crop_px": roi_result.crop_px,
            "crop_size_px": roi_result.crop_size_px,
            "crop_size_mm": roi_result.crop_size_mm,
            "overlap_coarse": coarse.overlap_coarse,
            "outputs": outputs,
        }
        if refine_dict is not None:
            pair_entry["refine"] = refine_dict
            pair_entry["overlap_after_refine"] = overlap_after
            pair_entry["fused"] = {
                "bands": fused_bands,
                "split_nm": params.spectral_split_nm,
                "wavelength_range_nm": wavelength_range,
                "outputs": {
                    "hdr": outputs.get("fused_hdr"),
                    "raw": outputs.get("fused_raw"),
                },
            }
        alignment_pairs.append(pair_entry)

    combined_spectra_csv: Path | None = None
    combined_spectra_plot: Path | None = None
    if spectrum_rows:
        combined_spectra_csv, combined_spectra_plot = write_combined_roi_spectra_artifacts(
            scene.fusion_out, scene.mode, spectrum_rows
        )

    alignment_path = scene.fusion_metadata / "alignment.json"
    alignment = {
        "version": 6,
        "pipeline": "coarse_alignment_phase_correction_spectral_fusion",
        "strategy": "per_roi",
        "centroid_source": {
            "fx10e": "native_masks",
            "swir3": "upsampled_masks_on_fx_grid",
        },
        "session": str(scene.session),
        "mode": scene.mode,
        "reference_camera": "fx10e",
        "grid_mm_per_pixel": scene.cal.fx10e_mm_per_pixel,
        "upsample_scale": round(scene.cal.upsample_scale, 6),
        "margin_mm": params.margin_mm,
        "fov_intersection_px": scene.fov_rect.as_dict(),
        "fov_intersection_mm": scene.fov_rect.size_mm(scene.cal.fx10e_mm_per_pixel),
        "shift_spread_mm": {k: round(v, 3) for k, v in scene.shift_spread.items()},
        "refine": {
            "method": "scale_translation_pc" if params.scale_refine else "phase_correlation",
            "wl_window_nm": [params.wl_min, params.wl_max],
            "spectral_split_nm": params.spectral_split_nm,
            "pc_region": "chip_segmentation_mask",
            "pc_response_min": params.pc_response_min,
            "refine_cap_px": params.refine_cap_px,
            "scale_refine": params.scale_refine,
            "scale_search": [params.scale_search_min, params.scale_search_max],
            "scale_search_steps": params.scale_search_steps,
            "scale_cap": params.scale_cap,
            "roi_mask_mode": params.roi_mask_mode,
        },
        "pairs": alignment_pairs,
        "pipeline_complete": params.process_hsi,
        "outputs": {
            "fusion_dir": str(scene.fusion_out),
            "metadata_dir": str(scene.fusion_metadata),
            "alignment_json": str(alignment_path),
            "fx10e_centroids_png": str(scene.fusion_metadata / "fx10e_centroids.png"),
            "swir3_centroids_png": str(scene.fusion_metadata / "swir3_centroids.png"),
            **(
                {
                    "roi_spectra_combined_csv": str(combined_spectra_csv),
                    "roi_spectra_combined_plot": str(combined_spectra_plot),
                }
                if combined_spectra_csv is not None and combined_spectra_plot is not None
                else {}
            ),
        },
    }
    alignment_path.write_text(json.dumps(alignment, indent=2), encoding="utf-8")

    return FusionPipelineResult(
        session=scene.session,
        mode=scene.mode,
        strategy="per_roi",
        reference_camera="fx10e",
        grid_mm_per_pixel=scene.cal.fx10e_mm_per_pixel,
        upsample_scale=scene.cal.upsample_scale,
        margin_mm=params.margin_mm,
        fov_intersection_px=scene.fov_rect.as_dict(),
        pairs=roi_results,
        alignment_json=alignment_path,
        pipeline_complete=params.process_hsi,
    )
