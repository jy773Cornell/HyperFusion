#!/usr/bin/env python3
# Step 2: upsample SWIR3 RGB and masks to the FX10e ground grid (mm/pixel).
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hf_fusion.config import find_hyperfusion_cfg, load_spatial_calibration
from hf_fusion.dataset import fusion_step_dir, load_camera_rgb, load_segmentation
from hf_fusion.resample import RgbInterpolation, target_size_px, upsample_mask, upsample_rgb


def _resolve_mask_path(session: Path, mode: str, det: dict) -> Path:
    mask_file = Path(det["mask_npy"])
    if mask_file.is_file():
        return mask_file
    roi = int(det["roi"])
    return session / mode / "swir3" / "preprocessed" / "segmentation" / "masks" / f"mask_{roi:03d}.npy"


def _side_by_side(native: np.ndarray, upsampled: np.ndarray, gap: int = 8) -> np.ndarray:
    h = max(native.shape[0], upsampled.shape[0])
    pad_native = np.full((h, native.shape[1], 3), 32, dtype=np.uint8)
    pad_native[: native.shape[0], : native.shape[1]] = native
    pad_up = np.full((h, upsampled.shape[1], 3), 32, dtype=np.uint8)
    pad_up[: upsampled.shape[0], : upsampled.shape[1]] = upsampled
    separator = np.full((h, gap, 3), 255, dtype=np.uint8)
    return np.concatenate([pad_native, separator, pad_up], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion step 2: upsample SWIR to FX10e ground grid")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mode", choices=["reflectance", "transmittance"], default="reflectance")
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument(
        "--rgb-method",
        choices=[m.value for m in RgbInterpolation],
        default=RgbInterpolation.BICUBIC.value,
        help="Interpolation for SWIR RGB (default: bicubic)",
    )
    parser.add_argument(
        "--compare-lanczos",
        action="store_true",
        help="Also write an extra RGB upsample using Lanczos for visual comparison",
    )
    args = parser.parse_args()

    session = args.session.resolve()
    if not session.is_dir():
        print(f"ERROR: session not found: {session}", file=sys.stderr)
        return 1

    cfg_path = args.cfg or find_hyperfusion_cfg()
    cal = load_spatial_calibration(cfg_path)
    rgb_method = RgbInterpolation(args.rgb_method)

    fx = load_camera_rgb(session, args.mode, "fx10e", cal.fx10e_mm_per_pixel)
    sw = load_camera_rgb(session, args.mode, "swir3", cal.swir3_mm_per_pixel)

    target_w, target_h = target_size_px(
        sw.width,
        sw.height,
        cal.swir3_mm_per_pixel,
        cal.fx10e_mm_per_pixel,
    )

    out_dir = fusion_step_dir(session, args.mode, "step02")
    masks_out = out_dir / "masks"
    masks_out.mkdir(exist_ok=True)

    sw_rgb_up = upsample_rgb(sw.rgb, target_w, target_h, method=rgb_method)
    rgb_out = out_dir / f"swir3_rgb_upsampled_{rgb_method.value}.png"
    Image.fromarray(sw_rgb_up).save(rgb_out)

    seg = load_segmentation(session, args.mode, "swir3")
    mask_entries: list[dict] = []
    for det in seg.get("detections", []):
        roi = int(det["roi"])
        mask_path = _resolve_mask_path(session, args.mode, det)
        mask = np.load(mask_path)
        mask_up = upsample_mask(mask, target_w, target_h)
        mask_png = masks_out / f"mask_{roi:03d}_upsampled.png"
        Image.fromarray(mask_up * 255).save(mask_png)
        mask_npy = masks_out / f"mask_{roi:03d}_upsampled.npy"
        np.save(mask_npy, mask_up)
        mask_entries.append(
            {
                "roi": roi,
                "source_path": str(mask_path),
                "native_size_px": {"width": int(mask.shape[1]), "height": int(mask.shape[0])},
                "upsampled_path_png": str(mask_png),
                "upsampled_path_npy": str(mask_npy),
                "upsample_method": "nearest",
            }
        )

    stack_up = np.max(
        np.stack([np.load(masks_out / f"mask_{int(d['roi']):03d}_upsampled.npy") for d in seg["detections"]]),
        axis=0,
    )
    stack_png = out_dir / "stack_mask_upsampled.png"
    Image.fromarray(stack_up * 255).save(stack_png)

    side_by_side = _side_by_side(sw.rgb, sw_rgb_up)
    side_by_side_path = out_dir / "side_by_side_native_vs_upsampled.png"
    Image.fromarray(side_by_side).save(side_by_side_path)

    compare_paths: dict[str, str] = {}
    if args.compare_lanczos and rgb_method != RgbInterpolation.LANCZOS:
        lanczos_rgb = upsample_rgb(sw.rgb, target_w, target_h, method=RgbInterpolation.LANCZOS)
        lanczos_path = out_dir / "swir3_rgb_upsampled_lanczos.png"
        Image.fromarray(lanczos_rgb).save(lanczos_path)
        compare_paths["lanczos_rgb"] = str(lanczos_path)

    report = {
        "step": 2,
        "name": "upsample_swir_to_fx_grid",
        "session": str(session),
        "mode": args.mode,
        "cfg": str(cfg_path),
        "reference_grid": {
            "camera": "fx10e",
            "mm_per_pixel": cal.fx10e_mm_per_pixel,
        },
        "source": {
            "camera": "swir3",
            "mm_per_pixel": cal.swir3_mm_per_pixel,
            "native_size_px": {"width": sw.width, "height": sw.height},
        },
        "upsample_scale": round(cal.upsample_scale, 6),
        "target_size_px": {"width": target_w, "height": target_h},
        "target_size_mm": {
            "cross_track": round(target_w * cal.fx10e_mm_per_pixel, 2),
            "along_scan": round(target_h * cal.fx10e_mm_per_pixel, 2),
        },
        "fx10e_reference_size_px": {"width": fx.width, "height": fx.height},
        "rgb_method": rgb_method.value,
        "mask_method": "nearest",
        "outputs": {
            "swir3_rgb_upsampled": str(rgb_out),
            "stack_mask_upsampled": str(stack_png),
            "side_by_side": str(side_by_side_path),
            "masks_dir": str(masks_out),
            **compare_paths,
        },
        "masks": mask_entries,
        "notes": [
            "Upsampled SWIR is on the FX10e mm/pixel grid but may differ in line/sample count from FX10e.",
            "Alignment shift and crop happen in steps 3-5.",
        ],
    }

    report_path = out_dir / "step02_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Step 2 complete -> {out_dir}")
    print(f"  SWIR native:  {sw.width} x {sw.height} px @ {cal.swir3_mm_per_pixel} mm/px")
    print(f"  SWIR upsampled: {target_w} x {target_h} px @ {cal.fx10e_mm_per_pixel} mm/px")
    print(f"  FX10e reference: {fx.width} x {fx.height} px")
    print(f"  Scale: {cal.upsample_scale:.4f}, RGB method: {rgb_method.value}, masks: nearest")
    print(f"  Report: {report_path}")
    print(f"  QA: {rgb_out.name}, {side_by_side_path.name}, {len(mask_entries)} masks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
