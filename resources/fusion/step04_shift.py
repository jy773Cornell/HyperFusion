#!/usr/bin/env python3
# Step 4: apply step-3 translation to upsampled SWIR RGB and masks on the FX10e canvas.
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hf_fusion.config import find_hyperfusion_cfg, load_spatial_calibration
from hf_fusion.dataset import fusion_step_dir, load_camera_rgb
from hf_fusion.resample import shift_mask_to_canvas, shift_rgb_to_canvas


def _load_step02_report(session: Path, mode: str) -> dict:
    path = session / mode / "fusion" / "step02" / "step02_report.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run step02 first: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _load_step03_report(session: Path, mode: str) -> dict:
    path = session / mode / "fusion" / "step03" / "step03_report.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run step03 first: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def _blend_overlay(fx_rgb: np.ndarray, sw_rgb: np.ndarray, alpha: float = 0.45) -> np.ndarray:
    """Semi-transparent SWIR over FX for alignment QA."""
    fx = fx_rgb.astype(np.float32)
    sw = sw_rgb.astype(np.float32)
    blended = fx * (1.0 - alpha) + sw * alpha
    return np.clip(blended, 0, 255).astype(np.uint8)


def _side_by_side(left: np.ndarray, right: np.ndarray, gap: int = 8) -> np.ndarray:
    h = max(left.shape[0], right.shape[0])
    w = max(left.shape[1], right.shape[1])
    pad_left = np.zeros((h, w, 3), dtype=np.uint8)
    pad_right = np.zeros((h, w, 3), dtype=np.uint8)
    pad_left[: left.shape[0], : left.shape[1]] = left
    pad_right[: right.shape[0], : right.shape[1]] = right
    separator = np.full((h, gap, 3), 255, dtype=np.uint8)
    return np.concatenate([pad_left, separator, pad_right], axis=1)


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion step 4: apply shift to upsampled SWIR")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--mode", choices=["reflectance", "transmittance"], default="reflectance")
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument(
        "--overlay-alpha",
        type=float,
        default=0.45,
        help="SWIR weight in before/after FX overlay QA (default: 0.45)",
    )
    args = parser.parse_args()

    session = args.session.resolve()
    if not session.is_dir():
        print(f"ERROR: session not found: {session}", file=sys.stderr)
        return 1

    step02 = _load_step02_report(session, args.mode)
    step03 = _load_step03_report(session, args.mode)
    cfg_path = args.cfg or find_hyperfusion_cfg()
    cal = load_spatial_calibration(cfg_path)
    fx = load_camera_rgb(session, args.mode, "fx10e", cal.fx10e_mm_per_pixel)

    dx_px = float(step03["shift_px"]["dx"])
    dy_px = float(step03["shift_px"]["dy"])
    dx_mm = float(step03["shift_mm"]["dx"])
    dy_mm = float(step03["shift_mm"]["dy"])

    upsampled_rgb_path = Path(step02["outputs"]["swir3_rgb_upsampled"])
    if not upsampled_rgb_path.is_file():
        print(f"ERROR: upsampled RGB not found: {upsampled_rgb_path}", file=sys.stderr)
        return 1

    sw_rgb_up = np.asarray(Image.open(upsampled_rgb_path).convert("RGB"))
    canvas_w, canvas_h = fx.width, fx.height

    sw_rgb_before = shift_rgb_to_canvas(sw_rgb_up, 0.0, 0.0, canvas_w, canvas_h)
    sw_rgb_shifted = shift_rgb_to_canvas(sw_rgb_up, dx_px, dy_px, canvas_w, canvas_h)

    out_dir = fusion_step_dir(session, args.mode, "step04")
    masks_out = out_dir / "masks"
    masks_out.mkdir(exist_ok=True)

    rgb_before_path = out_dir / "swir3_rgb_on_fx_canvas_before_shift.png"
    rgb_shifted_path = out_dir / "swir3_rgb_shifted.png"
    Image.fromarray(sw_rgb_before).save(rgb_before_path)
    Image.fromarray(sw_rgb_shifted).save(rgb_shifted_path)

    mask_entries: list[dict] = []
    shifted_masks: list[np.ndarray] = []
    for entry in step02["masks"]:
        roi = int(entry["roi"])
        mask_up = (np.load(entry["upsampled_path_npy"]) > 0).astype(np.uint8)
        mask_before = shift_mask_to_canvas(mask_up, 0.0, 0.0, canvas_w, canvas_h)
        mask_shifted = shift_mask_to_canvas(mask_up, dx_px, dy_px, canvas_w, canvas_h)
        shifted_masks.append(mask_shifted)

        before_png = masks_out / f"mask_{roi:03d}_on_canvas_before_shift.png"
        shifted_png = masks_out / f"mask_{roi:03d}_shifted.png"
        shifted_npy = masks_out / f"mask_{roi:03d}_shifted.npy"
        Image.fromarray(mask_before * 255).save(before_png)
        Image.fromarray(mask_shifted * 255).save(shifted_png)
        np.save(shifted_npy, mask_shifted)

        mask_entries.append(
            {
                "roi": roi,
                "source_upsampled_npy": entry["upsampled_path_npy"],
                "before_shift_path_png": str(before_png),
                "shifted_path_png": str(shifted_png),
                "shifted_path_npy": str(shifted_npy),
            }
        )

    stack_shifted = np.max(np.stack(shifted_masks, axis=0), axis=0) if shifted_masks else np.zeros((canvas_h, canvas_w), dtype=np.uint8)
    stack_path = out_dir / "stack_mask_shifted.png"
    Image.fromarray(stack_shifted * 255).save(stack_path)

    overlay_before = _blend_overlay(fx.rgb, sw_rgb_before, alpha=args.overlay_alpha)
    overlay_after = _blend_overlay(fx.rgb, sw_rgb_shifted, alpha=args.overlay_alpha)
    overlay_before_path = out_dir / "overlay_fx_swir_before_shift.png"
    overlay_after_path = out_dir / "overlay_fx_swir_after_shift.png"
    side_by_side_path = out_dir / "overlay_before_after_side_by_side.png"
    Image.fromarray(overlay_before).save(overlay_before_path)
    Image.fromarray(overlay_after).save(overlay_after_path)
    Image.fromarray(_side_by_side(overlay_before, overlay_after)).save(side_by_side_path)

    report = {
        "step": 4,
        "name": "apply_shift_to_upsampled_swir",
        "session": str(session),
        "mode": args.mode,
        "reference": "fx10e",
        "grid_mm_per_pixel": cal.fx10e_mm_per_pixel,
        "canvas_size_px": {"width": canvas_w, "height": canvas_h},
        "shift_source": str(session / args.mode / "fusion" / "step03" / "step03_report.json"),
        "shift_mm": {"dx": round(dx_mm, 3), "dy": round(dy_mm, 3)},
        "shift_px": {"dx": round(dx_px, 3), "dy": round(dy_px, 3)},
        "inputs": {
            "upsampled_rgb": str(upsampled_rgb_path),
            "upsampled_masks_dir": step02["outputs"]["masks_dir"],
        },
        "outputs": {
            "swir3_rgb_on_fx_canvas_before_shift": str(rgb_before_path),
            "swir3_rgb_shifted": str(rgb_shifted_path),
            "stack_mask_shifted": str(stack_path),
            "overlay_before_shift": str(overlay_before_path),
            "overlay_after_shift": str(overlay_after_path),
            "overlay_before_after_side_by_side": str(side_by_side_path),
            "masks_dir": str(masks_out),
        },
        "masks": mask_entries,
        "notes": [
            "Shifted SWIR RGB and masks are warped onto the FX10e pixel grid (fixed reference).",
            "Black border regions appear where shifted content falls outside the upsampled SWIR extent.",
        ],
    }

    report_path = out_dir / "step04_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Step 4 complete -> {out_dir}")
    print(f"  Canvas: {canvas_w} x {canvas_h} px (FX10e reference)")
    print(f"  Shift applied: ({dx_mm:.2f}, {dy_mm:.2f}) mm = ({dx_px:.1f}, {dy_px:.1f}) px")
    print(f"  RGB: {rgb_shifted_path.name}")
    print(f"  Masks shifted: {len(mask_entries)}")
    print(f"  QA: {overlay_before_path.name}, {overlay_after_path.name}, {side_by_side_path.name}")
    print(f"  Report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
