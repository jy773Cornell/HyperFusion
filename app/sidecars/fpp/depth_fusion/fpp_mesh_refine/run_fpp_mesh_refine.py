#!/usr/bin/env python3
# Classical FPP grape refine CLI (sidecar / fpp / depth_fusion / fpp_mesh_refine).
# No robot motion. Writes under <burst>/fusion/fpp_mesh_refine/.
"""
Per-view decode depth -> filter -> confidence -> pose refine -> consistency
-> weighted densify -> physical ROI/SOR/ROR/surface cleanup -> dense_point_cloud.ply
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
    p.add_argument(
        "--remove-tray",
        action="store_true",
        help=(
            "RANSAC-remove a near-horizontal tray plane before fuse. "
            "Default off: ambient object-only FPP has no tray; ROI uses "
            "base_link +Z object workspace instead."
        ),
    )
    p.add_argument(
        "--keep-tray",
        action="store_true",
        help="Deprecated alias: tray is kept by default (same as omitting --remove-tray).",
    )
    p.add_argument("--tray-band-mm", type=float, default=12.0)
    p.add_argument("--no-pose-refine", action="store_true")
    p.add_argument("--no-consistency", action="store_true")
    p.add_argument("--consistency-min-views", type=int, default=1)
    p.add_argument("--consistency-max-dz-mm", type=float, default=6.0)
    p.add_argument(
        "--fusion-mode",
        choices=("intersection", "union"),
        default="intersection",
        help=(
            "intersection: hard multi-view agree + densify ≥2 views. "
            "union: soft score keep + densify 1-view cells."
        ),
    )
    p.add_argument(
        "--score-lambda-agree",
        type=float,
        default=0.15,
        help="Soft keep: score bonus per AGREE view (union mode).",
    )
    p.add_argument(
        "--score-lambda-contradict",
        type=float,
        default=0.40,
        help="Soft keep: score penalty per CONTRADICT view (union mode).",
    )
    p.add_argument(
        "--score-keep-threshold",
        type=float,
        default=0.12,
        help="Soft keep: minimum Score = C_FPP + λ_a·N_agree − λ_c·N_contradict.",
    )
    p.add_argument("--no-densify", action="store_true", help="Skip dense back-project cloud")
    p.add_argument(
        "--dense-voxel-mm",
        type=float,
        default=0.5,
        help="Voxel merge for dense cloud (0 = keep every pixel)",
    )
    p.add_argument("--dense-pixel-stride", type=int, default=1)
    p.add_argument("--support-voxel-mm", type=float, default=2.0)
    p.add_argument(
        "--support-min-views",
        type=int,
        default=None,
        help=(
            "Min distinct views per support cell (default: 2 for intersection, "
            "1 for union). Explicit value overrides fusion-mode default."
        ),
    )
    p.add_argument(
        "--no-cleanup",
        action="store_true",
        help="Skip ROI/SOR/ROR cleanup on dense back-projected cloud",
    )
    p.add_argument(
        "--sor-k",
        type=int,
        default=None,
        help="SOR neighbor count (default: mode preset)",
    )
    p.add_argument(
        "--sor-std",
        type=float,
        default=None,
        help="SOR std ratio α (default: mode preset; union is milder)",
    )
    p.add_argument(
        "--ror-min-neighbors",
        type=int,
        default=None,
        help="ROR min neighbors (default: mode preset; union is milder)",
    )
    p.add_argument(
        "--ror-radius-mm",
        type=float,
        default=None,
        help="ROR radius mm (default: auto from mode radius scale × spacing)",
    )
    p.add_argument(
        "--component-min-points",
        type=int,
        default=None,
        help="Min points per ROI component (default: mode preset)",
    )
    p.add_argument("--workspace-width-mm", type=float, default=300.0)
    p.add_argument(
        "--workspace-depth-mm",
        type=float,
        default=300.0,
        help="Workspace height along base_link +Z (mm). With width, forms a W×W×H box.",
    )
    p.add_argument(
        "--workspace-height-mm",
        type=float,
        default=None,
        help="Alias for --workspace-depth-mm (height of the W×W×H box).",
    )
    p.add_argument(
        "--final-component-eps-mm",
        type=float,
        default=None,
        help="Final DBSCAN eps mm (default: mode preset)",
    )
    p.add_argument(
        "--final-component-min-points",
        type=int,
        default=None,
        help="Final component min points (default: mode preset)",
    )
    p.add_argument(
        "--enable-tsdf",
        action="store_true",
        help=(
            "Also write score-gated TSDF mesh/cloud (secondary; densify remains primary). "
            "Low-score pixels are unknown (depth=0), not free space."
        ),
    )
    p.add_argument(
        "--tsdf-voxel-mm",
        type=float,
        default=None,
        help="TSDF voxel mm (default: --voxel-mm)",
    )
    p.add_argument(
        "--tsdf-trunc-mm",
        type=float,
        default=None,
        help="TSDF truncation mm (default: --trunc-mm)",
    )
    p.add_argument("--no-top-view", action="store_true")
    p.add_argument("--top-view-resolution-mm", type=float, default=0.5)
    p.add_argument(
        "--no-surface-filter",
        action="store_true",
        help="Skip multi-scale normal consistency filtering",
    )
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
        remove_tray_plane=bool(args.remove_tray) and not bool(args.keep_tray),
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
        support_voxel_m=float(args.support_voxel_mm) * 1.0e-3,
        support_min_views=(
            int(args.support_min_views) if args.support_min_views is not None else None
        ),
        fusion_mode=str(args.fusion_mode),
        score_lambda_agree=float(args.score_lambda_agree),
        score_lambda_contradict=float(args.score_lambda_contradict),
        score_keep_threshold=float(args.score_keep_threshold),
        enable_tsdf=bool(args.enable_tsdf),
        tsdf_voxel_m=(
            float(args.tsdf_voxel_mm) * 1.0e-3 if args.tsdf_voxel_mm is not None else None
        ),
        tsdf_trunc_m=(
            float(args.tsdf_trunc_mm) * 1.0e-3 if args.tsdf_trunc_mm is not None else None
        ),
        cleanup_dense=not bool(args.no_cleanup),
        sor_k=int(args.sor_k) if args.sor_k is not None else None,
        sor_std=float(args.sor_std) if args.sor_std is not None else None,
        ror_min_neighbors=(
            int(args.ror_min_neighbors) if args.ror_min_neighbors is not None else None
        ),
        ror_radius_mm=float(args.ror_radius_mm) if args.ror_radius_mm is not None else None,
        component_min_points=(
            int(args.component_min_points)
            if args.component_min_points is not None
            else None
        ),
        workspace_width_m=float(args.workspace_width_mm) * 1.0e-3,
        workspace_depth_m=float(
            args.workspace_height_mm
            if args.workspace_height_mm is not None
            else args.workspace_depth_mm
        )
        * 1.0e-3,
        surface_filter=not bool(args.no_surface_filter),
        final_component_eps_m=(
            float(args.final_component_eps_mm) * 1.0e-3
            if args.final_component_eps_mm is not None
            else None
        ),
        final_component_min_points=(
            int(args.final_component_min_points)
            if args.final_component_min_points is not None
            else None
        ),
        top_view=not bool(args.no_top_view),
        top_view_resolution_mm=float(args.top_view_resolution_mm),
        light_smooth=bool(args.light_smooth),
    )
    slim = {k: summary[k] for k in summary if k != "stages"}
    slim["stages"] = summary.get("stages")
    print(json.dumps(slim, indent=2))
    return 0 if summary.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
