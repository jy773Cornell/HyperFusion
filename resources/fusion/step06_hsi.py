#!/usr/bin/env python3
# Step 6: apply phase-1 coarse transforms to FFC HSI cubes (upsample, shift, FOV crop).
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hf_fusion.config import find_hyperfusion_cfg, load_spatial_calibration
from hf_fusion.crop import CropRect
from hf_fusion.dataset import fusion_step_dir, load_camera_rgb
from hf_fusion.envi import (
    crop_bil_memmap_to_file,
    parse_envi_hdr,
    read_bil_cube,
    resolve_ffc_hdr,
    write_bil_cube,
    write_envi_hdr,
)
from hf_fusion.resample import crop_hsi_cube, shift_hsi_to_canvas, upsample_hsi_cube


def _load_step02_report(session: Path, mode: str) -> dict:
    path = session / mode / "fusion" / "step02" / "step02_report.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run step02 first: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_step05_report(session: Path, mode: str) -> dict:
    path = session / mode / "fusion" / "step05" / "step05_report.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run step05 first: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_alignment(session: Path, mode: str) -> dict:
    path = session / mode / "fusion" / "alignment.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run step05 first (alignment.json): {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_step03_shift(session: Path, mode: str) -> tuple[float, float] | None:
    path = session / mode / "fusion" / "step03" / "step03_report.json"
    if not path.is_file():
        return None
    report = json.loads(path.read_text(encoding="utf-8"))
    return float(report["shift_px"]["dx"]), float(report["shift_px"]["dy"])


def _crop_rect_from_alignment(alignment: dict) -> CropRect:
    fov = alignment["fov_intersection_px"]
    return CropRect(int(fov["x0"]), int(fov["y0"]), int(fov["x1"]), int(fov["y1"]))


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion step 6: coarse HSI alignment on FFC cubes")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mode", choices=["reflectance", "transmittance"], default="reflectance")
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument(
        "--no-shift",
        action="store_true",
        help="Skip step-3 shift even if step03_report.json exists",
    )
    args = parser.parse_args()

    session = args.session.resolve()
    if not session.is_dir():
        print(f"ERROR: session not found: {session}", file=sys.stderr)
        return 1

    step02 = _load_step02_report(session, args.mode)
    step05 = _load_step05_report(session, args.mode)
    alignment = _load_alignment(session, args.mode)
    cfg_path = args.cfg or find_hyperfusion_cfg()
    cal = load_spatial_calibration(cfg_path)
    fx_rgb = load_camera_rgb(session, args.mode, "fx10e", cal.fx10e_mm_per_pixel)

    crop_rect = _crop_rect_from_alignment(alignment)
    if not crop_rect.is_valid():
        print("ERROR: invalid FOV crop rectangle", file=sys.stderr)
        return 1

    shift = None if args.no_shift else _load_step03_shift(session, args.mode)
    dx_px, dy_px = shift if shift is not None else (0.0, 0.0)

    target_w = int(step02["target_size_px"]["width"])
    target_h = int(step02["target_size_px"]["height"])
    canvas_w, canvas_h = fx_rgb.width, fx_rgb.height

    fx_hdr_path = resolve_ffc_hdr(session, args.mode, "fx10e")
    sw_hdr_path = resolve_ffc_hdr(session, args.mode, "swir3")
    fx_meta = parse_envi_hdr(fx_hdr_path)
    sw_meta = parse_envi_hdr(sw_hdr_path)

    out_dir = fusion_step_dir(session, args.mode, "step06")

    fx_raw_out = out_dir / "fx10e_ffc_cropped.raw"
    fx_hdr_out = out_dir / "fx10e_ffc_cropped.hdr"
    fx_samples, fx_lines, fx_bands = crop_bil_memmap_to_file(
        fx_meta,
        fx_raw_out,
        crop_rect.x0,
        crop_rect.y0,
        crop_rect.x1,
        crop_rect.y1,
    )
    write_envi_hdr(
        fx_hdr_out,
        fx_meta,
        samples=fx_samples,
        lines=fx_lines,
        bands=fx_bands,
        raw_basename=fx_raw_out.name,
        description="HyperFusion phase-1 FX10e FFC cropped to FOV intersection",
    )

    sw_cube = read_bil_cube(sw_meta)
    if sw_cube.shape[0] != sw_meta.lines or sw_cube.shape[1] != sw_meta.samples:
        print(
            f"ERROR: SWIR cube shape {sw_cube.shape[:2]} does not match header "
            f"{sw_meta.lines}x{sw_meta.samples}",
            file=sys.stderr,
        )
        return 1

    sw_up = upsample_hsi_cube(sw_cube, target_w, target_h)
    sw_on_canvas = shift_hsi_to_canvas(sw_up, dx_px, dy_px, canvas_w, canvas_h)
    sw_crop = crop_hsi_cube(sw_on_canvas, crop_rect.x0, crop_rect.y0, crop_rect.x1, crop_rect.y1)

    sw_raw_out = out_dir / "swir3_ffc_cropped.raw"
    sw_hdr_out = out_dir / "swir3_ffc_cropped.hdr"
    write_bil_cube(sw_raw_out, sw_crop)
    write_envi_hdr(
        sw_hdr_out,
        sw_meta,
        samples=sw_crop.shape[1],
        lines=sw_crop.shape[0],
        bands=sw_crop.shape[2],
        raw_basename=sw_raw_out.name,
        description="HyperFusion phase-1 SWIR3 FFC upsampled, shifted, cropped",
    )

    fusion_dir = session / args.mode / "fusion"
    alignment_path = fusion_dir / "alignment.json"
    alignment.update(
        {
            "phase1_complete": True,
            "shift_px": {"dx": round(dx_px, 3), "dy": round(dy_px, 3)},
            "shift_applied_to_hsi": shift is not None,
            "hsi_outputs": {
                "fx10e_ffc_cropped_hdr": str(fx_hdr_out),
                "fx10e_ffc_cropped_raw": str(fx_raw_out),
                "swir3_ffc_cropped_hdr": str(sw_hdr_out),
                "swir3_ffc_cropped_raw": str(sw_raw_out),
            },
            "hsi_size_px": {
                "fx10e": {"samples": fx_samples, "lines": fx_lines, "bands": fx_bands},
                "swir3": {
                    "samples": int(sw_crop.shape[1]),
                    "lines": int(sw_crop.shape[0]),
                    "bands": int(sw_crop.shape[2]),
                },
            },
            "source_reports": {
                **alignment.get("source_reports", {}),
                "step06": str(out_dir / "step06_report.json"),
            },
        }
    )
    alignment_path.write_text(json.dumps(alignment, indent=2), encoding="utf-8")

    report = {
        "step": 6,
        "name": "hsi_coarse_alignment",
        "session": str(session),
        "mode": args.mode,
        "reference": "fx10e",
        "grid_mm_per_pixel": cal.fx10e_mm_per_pixel,
        "inputs": {
            "fx10e_ffc_hdr": str(fx_hdr_path),
            "swir3_ffc_hdr": str(sw_hdr_path),
            "step02_report": str(session / args.mode / "fusion" / "step02" / "step02_report.json"),
            "step05_report": str(step05.get("outputs", {}).get("alignment_json", "")),
        },
        "native_size_px": {
            "fx10e": {"samples": fx_meta.samples, "lines": fx_meta.lines, "bands": fx_meta.bands},
            "swir3": {"samples": sw_meta.samples, "lines": sw_meta.lines, "bands": sw_meta.bands},
        },
        "upsample_target_px": {"width": target_w, "height": target_h},
        "shift_px": {"dx": round(dx_px, 3), "dy": round(dy_px, 3)},
        "shift_applied": shift is not None,
        "fov_intersection_px": crop_rect.as_dict(),
        "output_size_px": {
            "fx10e": {"samples": fx_samples, "lines": fx_lines, "bands": fx_bands},
            "swir3": {
                "samples": int(sw_crop.shape[1]),
                "lines": int(sw_crop.shape[0]),
                "bands": int(sw_crop.shape[2]),
            },
        },
        "output_size_mm": {
            "cross_track": round(crop_rect.width * cal.fx10e_mm_per_pixel, 2),
            "along_scan": round(crop_rect.height * cal.fx10e_mm_per_pixel, 2),
        },
        "wavelength_nm": {
            "fx10e": {
                "first": fx_meta.wavelengths_nm[0] if fx_meta.wavelengths_nm else None,
                "last": fx_meta.wavelengths_nm[-1] if fx_meta.wavelengths_nm else None,
                "count": fx_bands,
            },
            "swir3": {
                "first": sw_meta.wavelengths_nm[0] if sw_meta.wavelengths_nm else None,
                "last": sw_meta.wavelengths_nm[-1] if sw_meta.wavelengths_nm else None,
                "count": int(sw_crop.shape[2]),
            },
        },
        "outputs": {
            "fx10e_ffc_cropped_hdr": str(fx_hdr_out),
            "fx10e_ffc_cropped_raw": str(fx_raw_out),
            "swir3_ffc_cropped_hdr": str(sw_hdr_out),
            "swir3_ffc_cropped_raw": str(sw_raw_out),
            "alignment_json": str(alignment_path),
        },
        "notes": [
            "FX10e: FOV crop only (reference grid).",
            "SWIR3: bilinear spatial upsample, optional step-3 shift, then same FOV crop.",
            "Phase 1 coarse RGB+HSI alignment complete.",
        ],
    }

    report_path = out_dir / "step06_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Step 6 complete -> {out_dir}")
    print(f"  FX10e FFC: {fx_meta.samples}x{fx_meta.lines}x{fx_meta.bands} -> {fx_samples}x{fx_lines}x{fx_bands}")
    print(
        f"  SWIR3 FFC: {sw_meta.samples}x{sw_meta.lines}x{sw_meta.bands} -> "
        f"{sw_crop.shape[1]}x{sw_crop.shape[0]}x{sw_crop.shape[2]}"
    )
    print(f"  Shift applied: {shift is not None} ({dx_px:.1f}, {dy_px:.1f}) px")
    print(f"  Crop: {crop_rect.width} x {crop_rect.height} px")
    if fx_meta.wavelengths_nm and sw_meta.wavelengths_nm:
        print(
            f"  Wavelength: FX {fx_meta.wavelengths_nm[0]:.0f}-{fx_meta.wavelengths_nm[-1]:.0f} nm, "
            f"SWIR {sw_meta.wavelengths_nm[0]:.0f}-{sw_meta.wavelengths_nm[-1]:.0f} nm"
        )
    print(f"  Outputs: {fx_hdr_out.name}, {sw_hdr_out.name}")
    print(f"  Phase 1 coarse alignment complete -> {alignment_path}")
    print(f"  Report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
