#!/usr/bin/env python3
# Offline FPP decode + metric depth (sidecar). Not wired into the GUI yet.
"""Decode an FPP burst. Default: camera Z (mm) via camera_projector_stereo.yaml."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from hyperfusion_fpp.capture import list_burst_jobs, load_burst
from hyperfusion_fpp.decode import decode_burst
from hyperfusion_fpp.depth import depth_from_decode
from hyperfusion_fpp.geometry import StereoGeometry
from hyperfusion_fpp.io_maps import write_decode_maps, write_depth_maps
from hyperfusion_fpp.paths import DEFAULT_STEREO_YAML, resolve_stereo_yaml
from hyperfusion_fpp.undistort import undistort_burst


def _process_burst(
    burst_dir: Path,
    start: Path,
    out_dir: Path,
    stereo_arg: Path | None,
    min_modulation: float,
) -> dict:
    burst = load_burst(burst_dir, start=start)
    undistort_burst(burst)
    decoded = decode_burst(burst, min_modulation=min_modulation)
    stereo = StereoGeometry.load(Path(stereo_arg)) if stereo_arg is not None else None
    depth = depth_from_decode(decoded, stereo=stereo)
    dest = out_dir / burst_dir.name / start.stem
    write_decode_maps(dest, decoded, stem="fpp")
    write_depth_maps(dest, depth, stem="fpp")
    summary: dict = {
        "start": start.name,
        "burst_dir": str(burst_dir),
        "valid_px": int(depth.mask.sum()),
        "mode": depth.mode,
        "stereo": str(stereo_arg) if stereo_arg else None,
        "out": str(dest),
    }
    if depth.units == "mm":
        finite = depth.depth[np.isfinite(depth.depth)]
        if finite.size:
            summary["z_mm"] = {
                "med": float(np.median(finite)),
                "p05": float(np.percentile(finite, 5)),
                "p95": float(np.percentile(finite, 95)),
            }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="One Execute burst folder, or a parent of numbered/multiview_* bursts",
    )
    parser.add_argument(
        "--stereo",
        type=Path,
        default=None,
        help=f"Stereo YAML (default: {DEFAULT_STEREO_YAML} if present)",
    )
    parser.add_argument(
        "--no-stereo",
        action="store_true",
        help="Emit projector u only (skip metric triangulation)",
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--min-modulation",
        type=float,
        default=0.08,
        help="White-Black threshold. Unlit FOV is dropped.",
    )
    args = parser.parse_args()

    src = Path(args.input)
    jobs = list_burst_jobs(src)
    if not jobs:
        raise FileNotFoundError(f"No FPP burst in {src}")

    stereo_path: Path | None = None
    if not args.no_stereo:
        stereo_path = resolve_stereo_yaml(args.stereo)
        if stereo_path is None and args.stereo is not None:
            raise FileNotFoundError(f"Stereo YAML not found: {args.stereo}")
        if stereo_path is None:
            raise FileNotFoundError(
                f"No stereo YAML at {DEFAULT_STEREO_YAML}. "
                "Run calibrate_fpp_geometry.py, or pass --stereo PATH, or --no-stereo."
            )

    results = [
        _process_burst(
            burst_dir,
            start,
            Path(args.out),
            stereo_path,
            args.min_modulation,
        )
        for burst_dir, start in jobs
    ]
    print(
        json.dumps(
            {
                "ok": True,
                "stereo": str(stereo_path) if stereo_path else None,
                "n_bursts": len(results),
                "bursts": results,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 — CLI
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise SystemExit(1)
