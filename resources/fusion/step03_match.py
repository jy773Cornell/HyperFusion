#!/usr/bin/env python3
# Step 3: match objects in mm (left-to-right) and estimate median SWIR->FX shift.
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hf_fusion.config import find_hyperfusion_cfg, load_spatial_calibration
from hf_fusion.dataset import fusion_step_dir, load_camera_rgb
from hf_fusion.match import estimate_median_shift, match_by_sorted_order, records_from_table
from hf_fusion.resample import shift_mask_to_canvas
from hf_fusion.viz import draw_matched_shift_qa


def _load_step01_report(session: Path, mode: str) -> dict:
    path = session / mode / "fusion" / "step01" / "step01_report.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run step01 first: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_step02_report(session: Path, mode: str) -> dict:
    path = session / mode / "fusion" / "step02" / "step02_report.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run step02 first: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_fx_mask(session: Path, mode: str, roi: int) -> np.ndarray:
    path = session / mode / "fx10e" / "preprocessed" / "segmentation" / "masks" / f"mask_{roi:03d}.npy"
    return (np.load(path) > 0).astype(np.uint8)


def _load_sw_upsampled_mask(step02: dict, roi: int) -> np.ndarray:
    for entry in step02["masks"]:
        if int(entry["roi"]) == roi:
            return (np.load(entry["upsampled_path_npy"]) > 0).astype(np.uint8)
    raise FileNotFoundError(f"Upsampled SWIR mask for roi {roi} not found in step02 report")


def _shifted_sw_masks(
    pairs,
    step02: dict,
    dx_px: float,
    dy_px: float,
    canvas_w: int,
    canvas_h: int,
) -> dict[int, np.ndarray]:
    out: dict[int, np.ndarray] = {}
    for pair in pairs:
        sw_up = _load_sw_upsampled_mask(step02, pair.sw_roi)
        out[pair.sw_roi] = shift_mask_to_canvas(sw_up, dx_px, dy_px, canvas_w, canvas_h)
    return out


def _pair_to_dict(pair) -> dict:
    return {
        "pair": pair.pair_index,
        "fx_roi": pair.fx_roi,
        "sw_roi": pair.sw_roi,
        "fx_mm": [round(pair.fx_mm[0], 2), round(pair.fx_mm[1], 2)],
        "sw_mm": [round(pair.sw_mm[0], 2), round(pair.sw_mm[1], 2)],
        "delta_mm": [round(pair.delta_mm[0], 2), round(pair.delta_mm[1], 2)],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion step 3: match objects and median shift")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mode", choices=["reflectance", "transmittance"], default="reflectance")
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument(
        "--max-pair-distance-mm",
        type=float,
        default=50.0,
        help="Reject sorted pairs farther apart than this (mm)",
    )
    args = parser.parse_args()

    session = args.session.resolve()
    if not session.is_dir():
        print(f"ERROR: session not found: {session}", file=sys.stderr)
        return 1

    step02 = _load_step02_report(session, args.mode)
    cfg_path = args.cfg or find_hyperfusion_cfg()
    cal = load_spatial_calibration(cfg_path)
    step01 = _load_step01_report(session, args.mode)

    fx_records = records_from_table(step01["fx10e"]["centroids_sorted_left_to_right"])
    sw_records = records_from_table(step01["swir3"]["centroids_sorted_left_to_right"])

    try:
        pairs = match_by_sorted_order(fx_records, sw_records, max_pair_distance_mm=args.max_pair_distance_mm)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    shift = estimate_median_shift(pairs, cal.fx10e_mm_per_pixel)
    fx = load_camera_rgb(session, args.mode, "fx10e", cal.fx10e_mm_per_pixel)
    out_dir = fusion_step_dir(session, args.mode, "step03")

    fx_masks_by_roi: dict[int, np.ndarray] = {}
    for pair in shift.pairs:
        fx_masks_by_roi[pair.fx_roi] = _load_fx_mask(session, args.mode, pair.fx_roi)

    sw_shifted_masks = _shifted_sw_masks(
        shift.pairs, step02, shift.dx_px, shift.dy_px, fx.width, fx.height
    )

    qa_path = out_dir / "matched_shift_qa.png"
    draw_matched_shift_qa(
        fx.rgb,
        shift.pairs,
        shift.dx_px,
        shift.dy_px,
        cal.fx10e_mm_per_pixel,
        "Step 3: median centroid shift",
        fx_masks_by_roi=fx_masks_by_roi,
        sw_shifted_masks_by_roi=sw_shifted_masks,
    ).save(qa_path)

    report = {
        "step": 3,
        "name": "match_objects_median_shift",
        "session": str(session),
        "mode": args.mode,
        "reference": "fx10e",
        "grid_mm_per_pixel": cal.fx10e_mm_per_pixel,
        "matching": {
            "method": "sort_by_cx_mm_left_to_right",
            "max_pair_distance_mm": args.max_pair_distance_mm,
            "matched_objects": [_pair_to_dict(pair) for pair in shift.pairs],
        },
        "shift_mm": {"dx": round(shift.dx_mm, 3), "dy": round(shift.dy_mm, 3)},
        "shift_px": {"dx": round(shift.dx_px, 3), "dy": round(shift.dy_px, 3)},
        "residual_std_mm": {
            "dx": round(shift.residual_std_mm[0], 3),
            "dy": round(shift.residual_std_mm[1], 3),
        },
        "flags": {
            "low_confidence": shift.low_confidence,
            "review_required": shift.review_required,
        },
        "interpretation": (
            "Apply (+shift_px.dx, +shift_px.dy) to upsampled SWIR to align with fixed FX10e. "
            "shift = median(FX_mm - SWIR_mm) per matched pair."
        ),
        "outputs": {"matched_shift_qa": str(qa_path)},
    }

    report_path = out_dir / "step03_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Step 3 complete -> {out_dir}")
    print(f"  Matched objects: {shift.matched_count}")
    for pair in shift.pairs:
        print(
            f"    pair {pair.pair_index}: fx roi{pair.fx_roi} <-> sw roi{pair.sw_roi}  "
            f"delta=({pair.delta_mm[0]:.2f}, {pair.delta_mm[1]:.2f}) mm"
        )
    print(f"  Shift: ({shift.dx_mm:.2f}, {shift.dy_mm:.2f}) mm = ({shift.dx_px:.1f}, {shift.dy_px:.1f}) px")
    print(f"  Residual std: dx={shift.residual_std_mm[0]:.2f} mm, dy={shift.residual_std_mm[1]:.2f} mm")
    if shift.review_required:
        print("  WARNING: review_required=true (residual spread > 5 mm)")
    print(f"  Report: {report_path}")
    print(f"  QA: {qa_path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
