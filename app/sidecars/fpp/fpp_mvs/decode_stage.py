# Decode stage for FPP MVS (sidecar / fpp / fpp_mvs). Writes processed/decode/<stem>/.
"""Per-pin FPP decode → metric depth under a flat decode root."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from fpp_depth.capture import list_burst_jobs
from fpp_depth.decode_pin import decode_pin
from fpp_depth.geometry import StereoGeometry


def decode_mvs_burst(
    burst: Path,
    decode_out: Path,
    *,
    stereo_yaml: Path | None,
    channel: str = "auto",
    min_modulation: float = 0.15,
    min_phase_quality: float = 0.04,
) -> dict[str, Any]:
    """Decode every FPP pin burst into ``decode_out/<start_stem>/``."""
    t0 = time.perf_counter()
    burst = Path(burst).resolve()
    decode_out = Path(decode_out).resolve()
    decode_out.mkdir(parents=True, exist_ok=True)

    jobs = list_burst_jobs(burst)
    if not jobs:
        raise FileNotFoundError(f"No FPP pin bursts in {burst}")

    stereo = StereoGeometry.load(Path(stereo_yaml)) if stereo_yaml is not None else None
    pin_results: list[dict[str, Any]] = []
    pin_timings: list[dict[str, Any]] = []

    for burst_dir, start in jobs:
        dest = decode_out / start.stem
        summary = decode_pin(
            burst_dir,
            start,
            dest,
            stereo=stereo,
            channel=channel,
            min_modulation=min_modulation,
            min_phase_quality=min_phase_quality,
        )
        elapsed = float(summary["elapsed_s"])
        pin_results.append(summary)
        pin_timings.append({"stem": start.stem, "elapsed_s": elapsed})
        print(
            f"  decode {start.stem}: valid={summary['valid_px']:,} px  t={elapsed:.2f}s",
            flush=True,
        )

    total = round(time.perf_counter() - t0, 3)
    return {
        "ok": True,
        "n_pins": len(pin_results),
        "stereo": str(stereo_yaml) if stereo_yaml else None,
        "decode_root": str(decode_out),
        "pins": pin_results,
        "elapsed_s": total,
        "pin_timings_s": pin_timings,
    }
