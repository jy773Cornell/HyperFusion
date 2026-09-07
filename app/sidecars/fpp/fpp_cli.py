#!/usr/bin/env python3
"""FPP measurement decode (offline). Plane calib is scripts/calibrate_fpp.py."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from hyperfusion_fpp.calibrate import resolve_calibration
from hyperfusion_fpp.capture import list_burst_starts, load_burst
from hyperfusion_fpp.decode import decode_burst
from hyperfusion_fpp.depth import depth_from_decode, remove_plane_background
from hyperfusion_fpp.io_maps import write_decode_maps, write_depth_maps
from hyperfusion_fpp.undistort import undistort_burst


def _process_burst(
    burst_dir: Path,
    start: Path,
    out_dir: Path,
    calib_arg: Path | None,
    min_modulation: float,
    bg_min_disparity_px: float,
) -> dict:
    burst = load_burst(burst_dir, start=start)
    undistort_burst(burst)
    decoded = decode_burst(burst, min_modulation=min_modulation)
    calib = None
    calib_used = None
    match = None
    if calib_arg is not None:
        calib, path, match = resolve_calibration(calib_arg, start, burst.first_meta())
        calib_used = str(path)
    depth = depth_from_decode(decoded, calib, burst.first_meta())
    if calib is not None:
        depth = remove_plane_background(depth, decoded, min_abs_disparity_px=bg_min_disparity_px)
    dest = out_dir / start.stem
    write_decode_maps(dest, decoded, stem="fpp")
    write_depth_maps(dest, depth, stem="fpp")
    return {
        "start": start.name,
        "valid_px": int(decoded.mask.sum()),
        "mode": depth.mode,
        "calib": calib_used,
        "calib_match": match,
        "out": str(dest),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Folder with one or more FPP bursts")
    parser.add_argument(
        "--calib",
        type=Path,
        default=None,
        help="One .npz, or a folder of {stem}_fpp_calib.npz from calibrate_fpp.py",
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--min-modulation",
        type=float,
        default=0.08,
        help="White-Black threshold. Unlit FOV is dropped.",
    )
    parser.add_argument(
        "--bg-min-disparity-px",
        type=float,
        default=0.0,
        help="If >0, drop the larger |u-plane| blob (card). 0 keeps the full lit patch.",
    )
    args = parser.parse_args()

    src = Path(args.input)
    starts = list_burst_starts(src)
    if not starts:
        raise FileNotFoundError(f"No FPP burst in {src}")
    results = [
        _process_burst(
            src,
            start,
            Path(args.out),
            args.calib,
            args.min_modulation,
            args.bg_min_disparity_px,
        )
        for start in starts
    ]
    print(json.dumps({"ok": True, "bursts": results}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 — CLI
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise SystemExit(1)
