#!/usr/bin/env python3
# Step 1: load RGB + masks, compute object centroids in px and mm.
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hf_fusion.config import find_hyperfusion_cfg, load_spatial_calibration
from hf_fusion.dataset import fusion_step_dir, load_camera_rgb, resolve_rgb_path
from hf_fusion.viz import draw_centroids, save_centroid_table


def main() -> int:
    parser = argparse.ArgumentParser(description="Fusion step 1: load RGB and centroids")
    parser.add_argument("--session", type=Path, required=True, help="Capture session root, e.g. E:\\chiptest")
    parser.add_argument("--mode", choices=["reflectance", "transmittance"], default="reflectance")
    parser.add_argument("--cfg", type=Path, default=None, help="Optional hyperfusion.cfg path")
    args = parser.parse_args()

    session = args.session.resolve()
    if not session.is_dir():
        print(f"ERROR: session not found: {session}", file=sys.stderr)
        return 1

    cfg_path = args.cfg or find_hyperfusion_cfg()
    cal = load_spatial_calibration(cfg_path)
    out_dir = fusion_step_dir(session, args.mode, "step01")

    fx = load_camera_rgb(session, args.mode, "fx10e", cal.fx10e_mm_per_pixel)
    sw = load_camera_rgb(session, args.mode, "swir3", cal.swir3_mm_per_pixel)

    report = {
        "step": 1,
        "name": "load_rgb_and_centroids",
        "session": str(session),
        "mode": args.mode,
        "cfg": str(cfg_path),
        "calibration": {
            "fx10e_mm_per_pixel": cal.fx10e_mm_per_pixel,
            "swir3_mm_per_pixel": cal.swir3_mm_per_pixel,
            "upsample_scale": round(cal.upsample_scale, 6),
        },
        "fx10e": {
            "rgb_path": str(resolve_rgb_path(session, args.mode, "fx10e")),
            "size_px": {"width": fx.width, "height": fx.height},
            "size_mm": {
                "cross_track": round(fx.width * cal.fx10e_mm_per_pixel, 2),
                "along_scan": round(fx.height * cal.fx10e_mm_per_pixel, 2),
            },
            "object_count": len(fx.centroids),
            "centroids_sorted_left_to_right": save_centroid_table(fx.centroids),
        },
        "swir3": {
            "rgb_path": str(resolve_rgb_path(session, args.mode, "swir3")),
            "size_px": {"width": sw.width, "height": sw.height},
            "size_mm": {
                "cross_track": round(sw.width * cal.swir3_mm_per_pixel, 2),
                "along_scan": round(sw.height * cal.swir3_mm_per_pixel, 2),
            },
            "object_count": len(sw.centroids),
            "centroids_sorted_left_to_right": save_centroid_table(sw.centroids),
        },
    }

    fx_png = out_dir / "fx10e_centroids.png"
    sw_png = out_dir / "swir3_centroids.png"
    draw_centroids(fx, f"FX10e {fx.width}x{fx.height} @ {cal.fx10e_mm_per_pixel} mm/px").save(fx_png)
    draw_centroids(sw, f"SWIR3 {sw.width}x{sw.height} @ {cal.swir3_mm_per_pixel} mm/px").save(sw_png)

    report_path = out_dir / "step01_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"Step 1 complete -> {out_dir}")
    print(f"  FX10e RGB: {fx.width} x {fx.height} px, {len(fx.centroids)} objects")
    print(f"  SWIR3 RGB: {sw.width} x {sw.height} px, {len(sw.centroids)} objects")
    print(f"  Upsample scale (SWIR->FX grid): {cal.upsample_scale:.4f}")
    print(f"  Report: {report_path}")
    print(f"  QA images: {fx_png.name}, {sw_png.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
