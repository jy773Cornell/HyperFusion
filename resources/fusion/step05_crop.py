#!/usr/bin/env python3
# Step 5: crop FX and upsampled SWIR to their FOV intersection (step 2 only, no object crop box).
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hf_fusion.config import find_hyperfusion_cfg, load_spatial_calibration
from hf_fusion.crop import CropRect, crop_image, fov_intersection_rect, union_masks
from hf_fusion.dataset import fusion_step_dir, load_camera_rgb, load_segmentation
from hf_fusion.resample import shift_mask_to_canvas, shift_rgb_to_canvas


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


def _blend_overlay(fx_rgb: np.ndarray, sw_rgb: np.ndarray, alpha: float = 0.45) -> np.ndarray:
    blended = fx_rgb.astype(np.float32) * (1.0 - alpha) + sw_rgb.astype(np.float32) * alpha
    return np.clip(blended, 0, 255).astype(np.uint8)


def _draw_fov_qa(fx_rgb: np.ndarray, fov: CropRect, title: str) -> Image.Image:
    canvas = cv2.cvtColor(fx_rgb.copy(), cv2.COLOR_RGB2BGR)
    cv2.rectangle(canvas, (fov.x0, fov.y0), (fov.x1 - 1, fov.y1 - 1), (64, 255, 64), 3, lineType=cv2.LINE_AA)
    img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    draw.text(
        (8, 8),
        f"{title}\ngreen=FOV intersection crop ({fov.width}x{fov.height} px)",
        fill=(255, 255, 0),
    )
    return img


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion step 5: crop to FX/SWIR FOV intersection")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mode", choices=["reflectance", "transmittance"], default="reflectance")
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument(
        "--overlay-alpha",
        type=float,
        default=0.45,
        help="SWIR weight in cropped overlay QA (default: 0.45)",
    )
    args = parser.parse_args()

    session = args.session.resolve()
    if not session.is_dir():
        print(f"ERROR: session not found: {session}", file=sys.stderr)
        return 1

    step02 = _load_step02_report(session, args.mode)
    cfg_path = args.cfg or find_hyperfusion_cfg()
    cal = load_spatial_calibration(cfg_path)
    fx = load_camera_rgb(session, args.mode, "fx10e", cal.fx10e_mm_per_pixel)

    sw_w = int(step02["target_size_px"]["width"])
    sw_h = int(step02["target_size_px"]["height"])
    canvas_w, canvas_h = fx.width, fx.height

    fov_rect = fov_intersection_rect(canvas_w, canvas_h, sw_w, sw_h, shift_dx_px=0.0, shift_dy_px=0.0)
    if not fov_rect.is_valid():
        print("ERROR: FX/SWIR FOV intersection is empty", file=sys.stderr)
        return 1

    upsampled_rgb_path = Path(step02["outputs"]["swir3_rgb_upsampled"])
    sw_rgb_up = np.asarray(Image.open(upsampled_rgb_path).convert("RGB"))
    sw_rgb_on_canvas = shift_rgb_to_canvas(sw_rgb_up, 0.0, 0.0, canvas_w, canvas_h)

    fx_rgb_crop = crop_image(fx.rgb, fov_rect)
    sw_rgb_crop = crop_image(sw_rgb_on_canvas, fov_rect)

    fx_seg = load_segmentation(session, args.mode, "fx10e")
    fx_rois = [int(det["roi"]) for det in fx_seg.get("detections", [])]
    sw_rois = [int(entry["roi"]) for entry in step02["masks"]]

    out_dir = fusion_step_dir(session, args.mode, "step05")
    masks_out = out_dir / "masks"
    masks_out.mkdir(exist_ok=True)

    fx_rgb_out = out_dir / "fx10e_rgb_cropped.png"
    sw_rgb_out = out_dir / "swir3_rgb_cropped.png"
    Image.fromarray(fx_rgb_crop).save(fx_rgb_out)
    Image.fromarray(sw_rgb_crop).save(sw_rgb_out)

    fx_mask_entries: list[dict] = []
    sw_mask_entries: list[dict] = []
    fx_cropped_masks: list[np.ndarray] = []
    sw_cropped_masks: list[np.ndarray] = []
    for roi in fx_rois:
        fx_crop = crop_image(_load_fx_mask(session, args.mode, roi), fov_rect)
        fx_cropped_masks.append(fx_crop)
        fx_npy = masks_out / f"fx_mask_{roi:03d}_cropped.npy"
        np.save(fx_npy, fx_crop)
        fx_mask_entries.append({"fx_roi": roi, "path_npy": str(fx_npy)})

    for roi in sw_rois:
        sw_on_canvas = shift_mask_to_canvas(
            _load_sw_upsampled_mask(step02, roi), 0.0, 0.0, canvas_w, canvas_h
        )
        sw_crop = crop_image(sw_on_canvas, fov_rect)
        sw_cropped_masks.append(sw_crop)
        sw_npy = masks_out / f"sw_mask_{roi:03d}_cropped.npy"
        np.save(sw_npy, sw_crop)
        sw_mask_entries.append({"sw_roi": roi, "path_npy": str(sw_npy)})

    fx_stack = union_masks(fx_cropped_masks) if fx_cropped_masks else np.zeros((fov_rect.height, fov_rect.width), dtype=np.uint8)
    sw_stack = union_masks(sw_cropped_masks) if sw_cropped_masks else np.zeros((fov_rect.height, fov_rect.width), dtype=np.uint8)
    object_union = union_masks(fx_cropped_masks + sw_cropped_masks) if fx_cropped_masks and sw_cropped_masks else fx_stack | sw_stack

    fx_stack_path = out_dir / "stack_fx_mask_cropped.png"
    sw_stack_path = out_dir / "stack_sw_mask_cropped.png"
    union_path = out_dir / "stack_object_union_cropped.png"
    Image.fromarray(fx_stack * 255).save(fx_stack_path)
    Image.fromarray(sw_stack * 255).save(sw_stack_path)
    Image.fromarray(object_union * 255).save(union_path)

    overlay_crop = _blend_overlay(fx_rgb_crop, sw_rgb_crop, alpha=args.overlay_alpha)
    overlay_path = out_dir / "overlay_fx_swir_cropped.png"
    qa_path = out_dir / "fov_intersection_qa.png"
    Image.fromarray(overlay_crop).save(overlay_path)
    _draw_fov_qa(fx.rgb, fov_rect, "Step 5: FOV intersection crop").save(qa_path)

    fusion_dir = session / args.mode / "fusion"
    alignment_path = fusion_dir / "alignment.json"
    alignment = {
        "version": 1,
        "session": str(session),
        "mode": args.mode,
        "reference_camera": "fx10e",
        "grid_mm_per_pixel": cal.fx10e_mm_per_pixel,
        "upsample_scale": round(cal.upsample_scale, 6),
        "fov_intersection_px": fov_rect.as_dict(),
        "fov_intersection_mm": fov_rect.size_mm(cal.fx10e_mm_per_pixel),
        "fx_rois": fx_rois,
        "swir_rois": sw_rois,
        "source_reports": {
            "step02": str(session / args.mode / "fusion" / "step02" / "step02_report.json"),
            "step05": str(out_dir / "step05_report.json"),
        },
        "notes": [
            "Crop = FOV intersection of FX canvas and upsampled SWIR at origin (step 2).",
            "Alignment shift (steps 3-4) is separate; apply shift_px before this crop on HSI cubes.",
        ],
    }
    alignment_path.write_text(json.dumps(alignment, indent=2), encoding="utf-8")

    report = {
        "step": 5,
        "name": "crop_to_fov_intersection",
        "session": str(session),
        "mode": args.mode,
        "reference": "fx10e",
        "grid_mm_per_pixel": cal.fx10e_mm_per_pixel,
        "canvas_size_px": {"width": canvas_w, "height": canvas_h},
        "swir_upsampled_size_px": {"width": sw_w, "height": sw_h},
        "fov_intersection_px": fov_rect.as_dict(),
        "fov_intersection_mm": fov_rect.size_mm(cal.fx10e_mm_per_pixel),
        "fx_rois": fx_rois,
        "swir_rois": sw_rois,
        "outputs": {
            "fx10e_rgb_cropped": str(fx_rgb_out),
            "swir3_rgb_cropped": str(sw_rgb_out),
            "stack_fx_mask_cropped": str(fx_stack_path),
            "stack_sw_mask_cropped": str(sw_stack_path),
            "stack_object_union_cropped": str(union_path),
            "overlay_cropped": str(overlay_path),
            "fov_intersection_qa": str(qa_path),
            "alignment_json": str(alignment_path),
            "masks_dir": str(masks_out),
        },
        "fx_masks": fx_mask_entries,
        "swir_masks": sw_mask_entries,
    }

    report_path = out_dir / "step05_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    size_mm = fov_rect.size_mm(cal.fx10e_mm_per_pixel)
    print(f"Step 5 complete -> {out_dir}")
    print(f"  FOV crop: {fov_rect.width} x {fov_rect.height} px @ {cal.fx10e_mm_per_pixel} mm/px")
    print(f"    = {size_mm['cross_track']:.1f} x {size_mm['along_scan']:.1f} mm")
    print(f"  RGB: {fx_rgb_out.name}, {sw_rgb_out.name}")
    print(f"  QA: {qa_path.name}, {overlay_path.name}")
    print(f"  Alignment: {alignment_path}")
    print(f"  Report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
