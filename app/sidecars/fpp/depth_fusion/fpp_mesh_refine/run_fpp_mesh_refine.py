#!/usr/bin/env python3
# Classical FPP grape refine CLI (sidecar / fpp / depth_fusion / fpp_mesh_refine).
# No robot motion. Writes under <burst>/fusion/fpp_mesh_refine/.
"""
Per-view decode depth → filter → confidence → pose refine → consistency
→ densify → ROI/SOR/ROR cleanup → dense_point_cloud.ply
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DF = HERE.parent
for p in (HERE, DF):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from hyperfusion_depth_fusion.poses import load_flange_T_camera_yaml  # noqa: E402
from hyperfusion_fpp_mesh.pipeline import run_pipeline  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--burst", type=Path, required=True)
    p.add_argument("--decode", type=Path, default=None)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--hand-eye", type=Path, default=None)
    p.add_argument(
        "--pose-mode",
        choices=("auto", "flange_camera", "json"),
        default="flange_camera",
    )
    p.add_argument("--mode", choices=("sweep", "apex", "all"), default="sweep")
    p.add_argument("--poses", type=str, default=None)
    p.add_argument("--stride", type=int, default=2)
    p.add_argument("--z-min-mm", type=float, default=350.0)
    p.add_argument("--z-max-mm", type=float, default=600.0)
    p.add_argument("--min-modulation", type=float, default=0.12)
    p.add_argument("--voxel-mm", type=float, default=1.5)
    p.add_argument("--trunc-mm", type=float, default=8.0)
    p.add_argument("--conf-min", type=float, default=0.12)
    p.add_argument("--keep-tray", action="store_true")
    p.add_argument("--tray-band-mm", type=float, default=12.0)
    p.add_argument("--no-pose-refine", action="store_true")
    p.add_argument("--no-consistency", action="store_true")
    p.add_argument("--consistency-min-views", type=int, default=1)
    p.add_argument("--consistency-max-dz-mm", type=float, default=6.0)
    p.add_argument("--no-densify", action="store_true", help="Skip dense back-project cloud")
    p.add_argument(
        "--dense-voxel-mm",
        type=float,
        default=0.5,
        help="Voxel merge for dense cloud (0 = keep every pixel)",
    )
    p.add_argument("--dense-pixel-stride", type=int, default=1)
    p.add_argument(
        "--no-cleanup",
        action="store_true",
        help="Skip ROI/SOR/ROR cleanup on dense back-projected cloud",
    )
    p.add_argument("--sor-k", type=int, default=25, help="SOR neighbor count (k)")
    p.add_argument("--sor-std", type=float, default=1.75, help="SOR std ratio α")
    p.add_argument("--ror-min-neighbors", type=int, default=12)
    p.add_argument(
        "--ror-radius-mm",
        type=float,
        default=None,
        help="ROR radius mm (default: auto ~2.5× mean spacing)",
    )
    p.add_argument("--component-min-points", type=int, default=800)
    p.add_argument(
        "--light-smooth",
        action="store_true",
        help="Mild MLS-style smooth after ROR (off by default)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    burst = Path(args.burst).resolve()
    decode = Path(args.decode).resolve() if args.decode else burst / "decode"
    out = Path(args.out).resolve() if args.out else burst / "fusion" / "fpp_mesh_refine"

    tool0 = None
    if args.pose_mode != "json":
        he = args.hand_eye
        if he is None:
            camera_cal = burst.parents[1] / "camera_cal"
            candidates = (
                camera_cal / "bfs_cal" / "results" / "flange_T_camera.yaml",
                camera_cal / "bfs_cal" / "ba" / "results" / "flange_T_camera.yaml",
                camera_cal / "ba" / "results" / "flange_T_camera.yaml",
            )
            he = next((path for path in candidates if path.is_file()), None)
        if he is None or not Path(he).is_file():
            raise SystemExit("Need --hand-eye flange_T_camera.yaml (or --pose-mode json)")
        tool0 = load_flange_T_camera_yaml(Path(he))
        print(f"hand-eye: {he}", flush=True)

    poses = (
        [x.strip() for x in args.poses.split(",") if x.strip()] if args.poses else None
    )
    summary = run_pipeline(
        burst,
        decode,
        out,
        tool0_T_camera=tool0,
        pose_mode=args.pose_mode,
        mode=args.mode,
        poses=poses,
        z_min_m=float(args.z_min_mm) * 1.0e-3,
        z_max_m=float(args.z_max_mm) * 1.0e-3,
        min_modulation=float(args.min_modulation),
        stride=int(args.stride),
        remove_tray_plane=not bool(args.keep_tray),
        tray_band_m=float(args.tray_band_mm) * 1.0e-3,
        do_pose_refine=not bool(args.no_pose_refine),
        do_consistency=not bool(args.no_consistency),
        consistency_min_views=int(args.consistency_min_views),
        consistency_max_dz_m=float(args.consistency_max_dz_mm) * 1.0e-3,
        voxel_m=float(args.voxel_mm) * 1.0e-3,
        trunc_m=float(args.trunc_mm) * 1.0e-3,
        conf_min=float(args.conf_min),
        densify=not bool(args.no_densify),
        dense_voxel_m=float(args.dense_voxel_mm) * 1.0e-3,
        dense_pixel_stride=int(args.dense_pixel_stride),
        cleanup_dense=not bool(args.no_cleanup),
        sor_k=int(args.sor_k),
        sor_std=float(args.sor_std),
        ror_min_neighbors=int(args.ror_min_neighbors),
        ror_radius_mm=float(args.ror_radius_mm) if args.ror_radius_mm is not None else None,
        component_min_points=int(args.component_min_points),
        light_smooth=bool(args.light_smooth),
    )
    slim = {k: summary[k] for k in summary if k != "stages"}
    slim["stages"] = summary.get("stages")
    print(json.dumps(slim, indent=2))
    return 0 if summary.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
