#!/usr/bin/env python3
# Offline TSDF fusion CLI (sidecar / fpp / depth_fusion). Prefer fpp_mvs_cli.py.
# No robot motion.
"""Optional TSDF fuse of decoded FPP depth.

Preferred layout (from ``fpp_mvs_cli.py``)::

  <cluster>/multiview/processed/decode/
  <cluster>/multiview/processed/fusion/

Legacy defaults still accept ``multiview/decode`` → ``multiview/fusion/tsdf``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hyperfusion_depth_fusion.poses import load_flange_T_camera_yaml  # noqa: E402
from hyperfusion_depth_fusion.tsdf import fuse_tsdf  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--burst",
        type=Path,
        default=None,
        help="One cluster multiview/ capture folder",
    )
    p.add_argument(
        "--decode",
        type=Path,
        default=None,
        help="Decode tree (default: <burst>/decode)",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output dir (default: <burst>/fusion/tsdf)",
    )
    p.add_argument(
        "--clusters-root",
        type=Path,
        default=None,
        help="Batch: each child/*/multiview with a decode/ subfolder (skips camera_cal)",
    )
    p.add_argument(
        "--hand-eye",
        type=Path,
        default=None,
        help="flange_T_camera.yaml from bfs_cal/results",
    )
    p.add_argument(
        "--pose-mode",
        choices=("auto", "flange_camera", "json"),
        default="auto",
        help="auto/flange_camera use hand-eye; json uses pose extrinsics only",
    )
    p.add_argument("--voxel-mm", type=float, default=2.0)
    p.add_argument("--sdf-trunc-mm", type=float, default=12.0)
    p.add_argument("--stride", type=int, default=2)
    p.add_argument("--z-min-mm", type=float, default=350.0)
    p.add_argument("--z-max-mm", type=float, default=600.0)
    p.add_argument("--min-modulation", type=float, default=0.15)
    return p.parse_args()


def _one(
    burst: Path,
    decode: Path | None,
    out: Path | None,
    *,
    hand_eye: Path | None,
    pose_mode: str,
    voxel_mm: float,
    sdf_trunc_mm: float,
    stride: int,
    z_min_mm: float,
    z_max_mm: float,
    min_modulation: float,
) -> dict:
    decode_root = Path(decode) if decode is not None else burst / "decode"
    out_dir = Path(out) if out is not None else burst / "fusion" / "tsdf"
    tool0_T_camera = None
    if pose_mode != "json":
        if hand_eye is None or not Path(hand_eye).is_file():
            raise SystemExit(
                "Need --hand-eye flange_T_camera.yaml (or --pose-mode json)"
            )
        tool0_T_camera = load_flange_T_camera_yaml(Path(hand_eye))
    return fuse_tsdf(
        burst,
        decode_root,
        out_dir,
        tool0_T_camera=tool0_T_camera,
        pose_mode=pose_mode,
        voxel_m=float(voxel_mm) * 1.0e-3,
        sdf_trunc_m=float(sdf_trunc_mm) * 1.0e-3,
        stride=max(1, int(stride)),
        z_min_m=float(z_min_mm) * 1.0e-3,
        z_max_m=float(z_max_mm) * 1.0e-3,
        min_modulation=float(min_modulation),
    )


def main() -> int:
    args = parse_args()
    if args.clusters_root is not None:
        root = Path(args.clusters_root)
        hand = args.hand_eye
        if hand is None:
            cand = root / "camera_cal" / "bfs_cal" / "results" / "flange_T_camera.yaml"
            if cand.is_file():
                hand = cand
        rows = []
        for cluster in sorted(root.iterdir()):
            if not cluster.is_dir() or cluster.name == "camera_cal":
                continue
            burst = cluster / "multiview"
            decode = burst / "decode"
            if not burst.is_dir() or not decode.is_dir():
                continue
            print(f"==== {cluster.name} ====", flush=True)
            try:
                summary = _one(
                    burst,
                    decode,
                    burst / "fusion" / "tsdf",
                    hand_eye=hand,
                    pose_mode=args.pose_mode,
                    voxel_mm=args.voxel_mm,
                    sdf_trunc_mm=args.sdf_trunc_mm,
                    stride=args.stride,
                    z_min_mm=args.z_min_mm,
                    z_max_mm=args.z_max_mm,
                    min_modulation=args.min_modulation,
                )
                rows.append(
                    {
                        "cluster": cluster.name,
                        "ok": True,
                        "n_poses": summary["n_poses"],
                        "mesh_vertices": summary["mesh_vertices"],
                        "cloud_points": summary["cloud_points"],
                        "out": summary["mesh"],
                    }
                )
                print(
                    f"  OK poses={summary['n_poses']} "
                    f"verts={summary['mesh_vertices']} pts={summary['cloud_points']}",
                    flush=True,
                )
            except Exception as exc:  # noqa: BLE001
                rows.append({"cluster": cluster.name, "ok": False, "error": str(exc)})
                print(f"  FAIL {exc}", flush=True)
        batch = {"ok": all(r.get("ok") for r in rows), "n": len(rows), "clusters": rows}
        out_sum = root / "camera_cal" / "cluster_tsdf_summary.json"
        out_sum.parent.mkdir(parents=True, exist_ok=True)
        out_sum.write_text(json.dumps(batch, indent=2), encoding="utf-8")
        print(json.dumps(batch, indent=2))
        return 0 if batch["ok"] else 1

    if args.burst is None:
        raise SystemExit("Pass --burst or --clusters-root")
    summary = _one(
        Path(args.burst),
        args.decode,
        args.out,
        hand_eye=args.hand_eye,
        pose_mode=args.pose_mode,
        voxel_mm=args.voxel_mm,
        sdf_trunc_mm=args.sdf_trunc_mm,
        stride=args.stride,
        z_min_mm=args.z_min_mm,
        z_max_mm=args.z_max_mm,
        min_modulation=args.min_modulation,
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
