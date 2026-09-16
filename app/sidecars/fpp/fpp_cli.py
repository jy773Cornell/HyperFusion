#!/usr/bin/env python3
# Offline FPP decode + metric depth (sidecar). Not wired into the GUI yet.
"""Decode an FPP burst. Default: camera Z (mm) via camera_projector_stereo.yaml.

Uses ``dlp_led`` / ``decode_channel`` from pose JSON (or ``--channel``) so red-LED
bursts decode on R instead of Rec.601 luma.
"""

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


def _parse_led_ma(text: str) -> tuple[int, int, int]:
    parts = [p.strip() for p in str(text).split(",")]
    if len(parts) != 3:
        raise ValueError("--set-led needs red,green,blue mA (e.g. 2400,0,0)")
    return int(parts[0]), int(parts[1]), int(parts[2])


def _backfill_led_json(
    burst_dir: Path,
    start: Path,
    *,
    red_ma: int,
    green_ma: int,
    blue_ma: int,
    channel: str,
) -> int:
    """Write dlp_led + decode_channel into pose JSONs for this burst (offline repair)."""
    from hyperfusion_fpp.capture import STEP_COUNT, U_ONLY_STEP_COUNT, _burst_stride, _stem_sort_key

    tiffs = sorted(burst_dir.glob("*.tif"), key=_stem_sort_key)
    tiffs += sorted(p for p in burst_dir.glob("*.tiff") if p not in set(tiffs))
    offset = next((i for i, p in enumerate(tiffs) if p.name == start.name), None)
    if offset is None:
        return 0
    stride = _burst_stride(tiffs, offset)
    stride = min(stride, STEP_COUNT if stride >= STEP_COUNT else U_ONLY_STEP_COUNT)
    n = 0
    led = {
        "red_ma": int(red_ma),
        "green_ma": int(green_ma),
        "blue_ma": int(blue_ma),
        "decode_channel": channel,
    }
    for path in tiffs[offset : offset + stride]:
        jpath = path.with_suffix(".json")
        if not jpath.is_file():
            continue
        meta = json.loads(jpath.read_text(encoding="utf-8"))
        meta["dlp_led"] = led
        meta["decode_channel"] = channel
        jpath.write_text(json.dumps(meta, indent=4) + "\n", encoding="utf-8")
        n += 1
    return n


def _process_burst(
    burst_dir: Path,
    start: Path,
    out_dir: Path,
    stereo_arg: Path | None,
    min_modulation: float,
    min_phase_quality: float,
    channel: str,
) -> dict:
    burst = load_burst(burst_dir, start=start, channel=channel)
    undistort_burst(burst)
    decoded = decode_burst(
        burst,
        min_modulation=min_modulation,
        min_phase_quality=min_phase_quality,
    )
    stereo = StereoGeometry.load(Path(stereo_arg)) if stereo_arg is not None else None
    depth = depth_from_decode(decoded, stereo=stereo)
    dest = out_dir / burst_dir.name / start.stem
    write_decode_maps(dest, decoded, stem="fpp")
    write_depth_maps(dest, depth, stem="fpp")
    summary: dict = {
        "start": start.name,
        "burst_dir": str(burst_dir),
        "decode_channel": burst.decode_channel,
        "min_modulation": float(min_modulation),
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
        "--channel",
        default="auto",
        help="Decode gray channel: auto|r|g|b|luma (default auto from dlp_led / image).",
    )
    parser.add_argument(
        "--min-modulation",
        type=float,
        default=0.15,
        help="White−Black threshold (default 0.15). Use ~0.08–0.12 if still too sparse.",
    )
    parser.add_argument(
        "--min-phase-quality",
        type=float,
        default=0.04,
        help="Fine-frequency phase quality gate (hypot of quadrature).",
    )
    parser.add_argument(
        "--set-led",
        default=None,
        help="Backfill pose JSON dlp_led as red,green,blue mA (e.g. 2400,0,0) then decode.",
    )
    args = parser.parse_args()

    src = Path(args.input)
    jobs = list_burst_jobs(src)
    if not jobs:
        raise FileNotFoundError(f"No FPP burst in {src}")

    if args.set_led is not None:
        red_ma, green_ma, blue_ma = _parse_led_ma(args.set_led)
        from hyperfusion_fpp.capture import decode_channel_from_led_ma, _normalize_channel

        ch = _normalize_channel(args.channel)
        if ch == "auto":
            ch = decode_channel_from_led_ma(red_ma, green_ma, blue_ma)
        for burst_dir, start in jobs:
            n = _backfill_led_json(
                burst_dir,
                start,
                red_ma=red_ma,
                green_ma=green_ma,
                blue_ma=blue_ma,
                channel=ch,
            )
            print(json.dumps({"backfill_led": True, "n_json": n, "channel": ch, "burst": str(burst_dir)}))

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
            args.min_phase_quality,
            args.channel,
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
