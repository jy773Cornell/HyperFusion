# Camera–projector stereo from checkerboard HDMI FPP bursts.
# dlp_cal layer. No robot motion. Smart (or last-N) hold-outs are not used in the fit.
"""Fit metric projector geometry from tilted-board bursts (no tray-plane calib)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

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


def _holdout_feature(view: dict[str, Any]) -> np.ndarray:
    """Board image + projector UV coverage for diversity hold-out picking."""
    cam = np.asarray(view["camera_xy"], dtype=np.float64).reshape(-1, 2)
    c = cam.mean(axis=0)
    bb = cam.max(axis=0) - cam.min(axis=0)
    feat = [c[0] / 1000.0, c[1] / 1000.0, bb[0] / 1000.0, bb[1] / 1000.0]
    uv = view.get("projector_uv")
    if uv is not None:
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
        finite = np.isfinite(uv).all(axis=1)
        if int(finite.sum()) >= 4:
            u = uv[finite]
            uc = u.mean(axis=0)
            ub = u.max(axis=0) - u.min(axis=0)
            # Projector coords are typically hundreds–thousands of pixels.
            feat.extend((uc / 500.0).tolist())
            feat.extend((ub / 500.0).tolist())
        else:
            feat.extend([0.0] * 4)
    else:
        feat.extend([0.0] * 4)
    return np.asarray(feat, dtype=np.float64)


def split_holdout(
    views: list[dict[str, Any]],
    holdout: int,
    *,
    mode: str = "smart",
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Split fit / hold-out. ``smart`` = diversity sample; ``last`` = chronological tail."""
    if holdout <= 0 or holdout >= len(views):
        return views, []
    if mode == "last":
        return views[:-holdout], views[-holdout:]

    n = len(views)
    n_hold = min(int(holdout), max(1, n // 5), max(0, n - 4))
    if n_hold <= 0:
        return views, []
    X = np.stack([_holdout_feature(v) for v in views], axis=0)
    std = X.std(axis=0)
    std[std < 1e-9] = 1.0
    Xn = (X - X.mean(axis=0)) / std
    d2 = ((Xn[:, None, :] - Xn[None, :, :]) ** 2).sum(axis=2)
    D = np.sqrt(np.maximum(d2, 0.0))
    np.fill_diagonal(D, np.inf)
    nn = D.min(axis=1)
    med_nn = float(np.median(nn[np.isfinite(nn)])) if np.isfinite(nn).any() else 1.0
    candidates = [i for i in range(n) if nn[i] <= 1.5 * med_nn]
    if len(candidates) < n_hold:
        candidates = list(range(n))

    hold_idx: list[int] = [min(candidates, key=lambda i: float(nn[i]))]
    while len(hold_idx) < n_hold:
        best_i = None
        best_score = -1.0
        for i in candidates:
            if i in hold_idx:
                continue
            md = float(min(D[i, j] for j in hold_idx))
            if md > best_score:
                best_score = md
                best_i = i
        if best_i is None:
            break
        hold_idx.append(best_i)

    hold_set = set(hold_idx)
    fit = [v for i, v in enumerate(views) if i not in hold_set]
    hold = [views[i] for i in sorted(hold_idx)]
    return fit, hold


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
        help="Number of bursts reserved for hold-out validation (not used in fit).",
    )
    p.add_argument(
        "--holdout-mode",
        choices=("smart", "last"),
        default="smart",
        help="smart = diversity across camera/projector board coverage; last = chronological tail",
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
    fit_views, hold_views = split_holdout(views, args.holdout, mode=args.holdout_mode)
    print(
        f"Fit {len(fit_views)}  hold-out {len(hold_views)}  mode={args.holdout_mode}",
        flush=True,
    )
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
        "holdout_mode": args.holdout_mode,
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
