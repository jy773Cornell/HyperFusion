#!/usr/bin/env python3
# FPP MVS CLI: decode → dense clean cloud under multiview/processed/ (sidecar / fpp).
# No robot motion. Auto-skips when the multiview folder has no FPP burst metadata.
"""
Process an FPP MVS dataset::

  {datafolder}/multiview/          raw #####.tif + .json
    processed/
      metadata.json                timings + paths
      decode/<stem>/fpp_*.npy
      fusion/dense_point_cloud.ply

Typical::

  .\\.venv\\Scripts\\python.exe fpp_mvs_cli.py ^
    --input D:\\Data_JY\\...\\Nagara_DM_Cluster_1_T
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from fpp_mvs.run import run_fpp_mvs_pipeline  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Dataset/cluster root (…/Cluster_X) or its multiview/ folder",
    )
    p.add_argument("--stereo", type=Path, default=None)
    p.add_argument("--no-stereo", action="store_true")
    p.add_argument("--hand-eye", type=Path, default=None)
    p.add_argument(
        "--pose-mode",
        choices=("auto", "flange_camera", "json"),
        default="auto",
        help=(
            "auto: use nearby camera_cal hand-eye when present; otherwise use "
            "the scan JSON pose written from app parameters"
        ),
    )
    p.add_argument(
        "--mode",
        choices=("sweep",),
        default="sweep",
        help="Fusion uses all FPP sweep/ring pins (no apex). 00000 is a normal pin.",
    )
    p.add_argument("--channel", default="auto")
    p.add_argument("--min-modulation", type=float, default=0.15)
    p.add_argument("--min-phase-quality", type=float, default=0.04)
    p.add_argument("--skip-decode", action="store_true")
    p.add_argument("--skip-fusion", action="store_true")
    p.add_argument(
        "--fusion-mode",
        choices=("intersection", "union"),
        default="union",
        help=(
            "union (default): soft score keep + densify 1-view cells — keeps one "
            "object together when views only partially overlap. "
            "intersection: hard multi-view agree + densify ≥2 views."
        ),
    )
    p.add_argument(
        "--support-min-views",
        type=int,
        default=None,
        help="Override densify multi-view support (default depends on --fusion-mode).",
    )
    p.add_argument("--score-lambda-agree", type=float, default=0.15)
    p.add_argument("--score-lambda-contradict", type=float, default=0.40)
    p.add_argument("--score-keep-threshold", type=float, default=0.12)
    p.add_argument(
        "--enable-tsdf",
        action="store_true",
        help="Also write score-gated TSDF mesh/cloud (secondary to dense_point_cloud.ply).",
    )
    p.add_argument(
        "--z-min-mm",
        type=float,
        default=150.0,
        help="Keep depth ≥ this (camera Z, mm). Default 150 for ~250 mm working distance.",
    )
    p.add_argument(
        "--z-max-mm",
        type=float,
        default=500.0,
        help="Keep depth ≤ this (camera Z, mm). Default 500.",
    )
    p.add_argument(
        "--allow-non-fpp",
        action="store_true",
        help="Run even if pose JSON lacks fpp_pattern (default: skip)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = run_fpp_mvs_pipeline(
            Path(args.input),
            stereo=args.stereo,
            no_stereo=bool(args.no_stereo),
            hand_eye=args.hand_eye,
            pose_mode=args.pose_mode,
            mode=args.mode,
            channel=args.channel,
            min_modulation=float(args.min_modulation),
            min_phase_quality=float(args.min_phase_quality),
            skip_decode=bool(args.skip_decode),
            skip_fusion=bool(args.skip_fusion),
            require_fpp=not bool(args.allow_non_fpp),
            fusion_mode=str(args.fusion_mode),
            support_min_views=(
                int(args.support_min_views) if args.support_min_views is not None else None
            ),
            score_lambda_agree=float(args.score_lambda_agree),
            score_lambda_contradict=float(args.score_lambda_contradict),
            score_keep_threshold=float(args.score_keep_threshold),
            enable_tsdf=bool(args.enable_tsdf),
            z_min_m=float(args.z_min_mm) * 1.0e-3,
            z_max_m=float(args.z_max_mm) * 1.0e-3,
        )
    except Exception as exc:  # noqa: BLE001 — CLI
        print(json.dumps({"ok": False, "error": str(exc)}))
        return 1

    # Final JSON line for C++ / tooling to parse
    slim = {
        "ok": bool(result.get("ok")),
        "skipped": bool(result.get("skipped", False)),
        "reason": result.get("reason"),
        "datafolder": result.get("datafolder"),
        "processed": result.get("processed"),
        "metadata": result.get("metadata"),
        "primary_cloud": result.get("primary_cloud"),
        "elapsed_s": result.get("elapsed_s"),
        "stages": {
            "decode_s": (result.get("stages") or {}).get("decode", {}).get("elapsed_s"),
            "fusion_s": (result.get("stages") or {}).get("fusion", {}).get("elapsed_s"),
        },
    }
    # Keep the final summary on one line so the C++ background runner can parse it.
    print(json.dumps(slim))
    if result.get("skipped"):
        return 0
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
