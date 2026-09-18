# Camera–projector stereo from checkerboard HDMI FPP bursts.
# dlp_cal layer. No robot motion. Last --holdout bursts are not used in the fit.
"""Fit metric projector geometry from tilted-board bursts (no tray-plane calib)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SIDECAR_FPP = ROOT.parents[2] / "sidecars" / "fpp"
DEFAULT_BOARD = ROOT.parent / "bfs_cal" / "board.yaml"
DEFAULT_INPUT = ROOT / "checkerboard"
DEFAULT_OUT = ROOT / "results"

if str(SIDECAR_FPP) not in sys.path:
    sys.path.insert(0, str(SIDECAR_FPP))

from fpp_depth.board_detect import load_board  # noqa: E402
from fpp_depth.capture import list_burst_jobs, load_burst  # noqa: E402
from fpp_depth.decode import decode_burst  # noqa: E402
from fpp_depth.geometry import evaluate_holdout, fit_stereo, observe_burst  # noqa: E402
from fpp_depth.undistort import undistort_burst  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help="fpp_cal/checkerboard (00000..), or a parent of multiview_* bursts",
    )
    p.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument(
        "--holdout",
        type=int,
        default=3,
        help="Last N bursts in collect order (shots 14–16). Not used in the fit.",
    )
    p.add_argument("--min-modulation", type=float, default=0.15)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    board = load_board(args.board)
    jobs = list_burst_jobs(args.input)
    if not jobs:
        raise FileNotFoundError(f"No FPP burst in {args.input}")
    views: list[dict] = []
    skipped: list[dict] = []
    camera_k = None
    camera_d = None
    camera_size = None
    for burst_dir, start in jobs:
        rec = {"dir": str(burst_dir), "start": start.name, "ok": False}
        try:
            burst = load_burst(burst_dir, start=start)
            undistort_burst(burst)
            decoded = decode_burst(burst, min_modulation=args.min_modulation)
            view, reason = observe_burst(burst, decoded, board, require_all_in_patch=True)
            rec["reason"] = reason
            rec["decode_px"] = int(decoded.mask.sum())
            if view is None:
                skipped.append(rec)
                print(f"SKIP {burst_dir.name}/{start.name} {reason}", flush=True)
                continue
            view["label"] = f"{burst_dir.name}/{start.name}"
            view.pop("white", None)
            views.append(view)
            rec["ok"] = True
            rec["n_corners"] = view["n_corners"]
            camera_k = view["camera_k"]
            camera_d = view["camera_d"]
            camera_size = view["camera_size"]
            print(
                f"OK   {view['label']}  corners={view['n_corners']}  decode_px={rec['decode_px']}",
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001
            rec["reason"] = str(exc)
            skipped.append(rec)
            print(f"SKIP {start.name} {exc}", flush=True)
    n_hold = min(int(args.holdout), max(0, len(views) - 4))
    fit_views = views[:-n_hold] if n_hold else views
    hold_views = views[-n_hold:] if n_hold else []
    if len(fit_views) < 4 or camera_k is None or camera_size is None:
        raise RuntimeError(
            f"Need >= 4 GOOD bursts for the fit (got {len(fit_views)}). "
            "Collect the 13 tilted/shifted shots first."
        )
    geom, stats = fit_stereo(
        fit_views,
        camera_k,
        camera_d,
        camera_size,
        undistorted=True,
    )
    dest = Path(args.out)
    dest.mkdir(parents=True, exist_ok=True)
    yaml_path = dest / "camera_projector_stereo.yaml"
    geom.save(yaml_path)
    hold = evaluate_holdout(geom, hold_views) if hold_views else {"n_views": 0}
    train = evaluate_holdout(geom, fit_views)
    report = {
        "ok": True,
        "input": str(args.input),
        "n_ok": len(views),
        "n_skipped": len(skipped),
        "n_fit": len(fit_views),
        "n_holdout": len(hold_views),
        "fit_stems": [v.get("label") or v.get("stem") for v in fit_views],
        "holdout_stems": [v.get("label") or v.get("stem") for v in hold_views],
        "skipped": skipped,
        "fit": stats,
        "train_mm": {k: train[k] for k in ("rms_mm", "med_mm", "p95_mm", "max_mm", "reproj_rms_uv")},
        "holdout_mm": {k: hold[k] for k in ("rms_mm", "med_mm", "p95_mm", "max_mm", "reproj_rms_uv")},
        "holdout_views": hold.get("views"),
        "stereo": str(yaml_path),
    }
    (dest / "geometry_validation.yaml").write_text(
        __import__("yaml").safe_dump(report, sort_keys=False),
        encoding="utf-8",
    )
    (dest / "geometry_validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise SystemExit(1)
