# Live BFS checkerboard quality monitor for eye-in-hand / paper BA capture.
# bfs_cal layer. No robot motion. Watches a folder; grades each still for fixed-board BA.
"""Watch checkerboard stills while collecting; report GOOD / WEAK / REJECT.

Targets the *paper* fixed-board BA path (one shared base_T_board), not soft
per-view board BA. A still is useful only if corners lock, flange pose exists,
and the set spans distance / tilt / image coverage.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

# Reuse detect helpers from calibrate_bfs in the same folder.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from calibrate_bfs import (  # noqa: E402
    detect_with_swap,
    load_board,
    load_flange_T,
    object_points_for,
    read_bgr,
)


IMAGE_EXTS = {".tif", ".tiff", ".png", ".jpg", ".jpeg"}


@dataclass
class FrameGrade:
    path: str
    status: str  # GOOD | WEAK | REJECT | WAIT_JSON
    reasons: list[str] = field(default_factory=list)
    pattern: list[int] | None = None
    coverage: dict[str, float] | None = None
    tilt_deg: float | None = None
    distance_mm: float | None = None
    flange_ok: bool = False
    laplacian_var: float | None = None
    pnp_rms_px: float | None = None


def list_images(folder: Path) -> list[Path]:
    out: list[Path] = []
    for p in sorted(folder.rglob("*")):
        if not p.is_file() or p.suffix.lower() not in IMAGE_EXTS:
            continue
        parts = {x.lower() for x in p.parts}
        if parts & {"decode", "fusion", "rejected_no_corners", "dropped_worst", "results"}:
            continue
        out.append(p)
    return out


def wait_stable(path: Path, settle_s: float = 0.4, tries: int = 8) -> bool:
    last = -1
    for _ in range(tries):
        try:
            size = path.stat().st_size
        except OSError:
            time.sleep(settle_s)
            continue
        if size > 0 and size == last:
            return True
        last = size
        time.sleep(settle_s)
    return path.is_file() and path.stat().st_size > 0


def board_coverage(corners: np.ndarray, wh: tuple[int, int]) -> dict[str, float]:
    w, h = float(wh[0]), float(wh[1])
    xy = corners.reshape(-1, 2)
    xmin, ymin = float(xy[:, 0].min()), float(xy[:, 1].min())
    xmax, ymax = float(xy[:, 0].max()), float(xy[:, 1].max())
    # Fraction of image spanned by board bbox.
    span_x = (xmax - xmin) / max(w, 1.0)
    span_y = (ymax - ymin) / max(h, 1.0)
    # Distance of bbox center from image center (0 = center, ~0.5 = near edge).
    cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
    offset = math.hypot((cx - 0.5 * w) / w, (cy - 0.5 * h) / h)
    margin = min(xmin, ymin, w - xmax, h - ymax) / min(w, h)
    return {
        "span_x": span_x,
        "span_y": span_y,
        "center_offset": offset,
        "edge_margin": margin,
    }


def _isotropic_K(wh: tuple[int, int], f: float | None = None) -> np.ndarray:
    w, h = wh
    if f is None:
        f = 0.9 * max(w, h)
    return np.array(
        [[f, 0.0, w * 0.5], [0.0, f, h * 0.5], [0.0, 0.0, 1.0]], dtype=np.float64
    )


def _pnp_rms(
    corners: np.ndarray, obj: np.ndarray, K: np.ndarray
) -> tuple[float, np.ndarray, np.ndarray] | None:
    D = np.zeros((5, 1), dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        obj.astype(np.float64),
        corners.reshape(-1, 1, 2).astype(np.float64),
        K,
        D,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj.astype(np.float64), rvec, tvec, K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    return float(np.sqrt(np.mean(err * err))), rvec, tvec


def rough_pose(
    corners: np.ndarray,
    obj: np.ndarray,
    wh: tuple[int, int],
    K_seed: np.ndarray | None,
) -> tuple[float | None, float | None, float | None]:
    """Return (tilt_deg, distance_mm, pnp_rms_px).

    Always pick the K (seed or isotropic grid) with lowest board reprojection.
    A stale seed K can make healthy corners look like 8–10 px RMS.
    """
    w, h = wh
    candidates: list[np.ndarray] = [
        _isotropic_K((w, h), f)
        for f in (1800.0, 2500.0, 3500.0, 4500.0, 5200.0, 0.9 * max(w, h))
    ]
    if K_seed is not None:
        candidates.append(np.asarray(K_seed, dtype=np.float64))

    best: tuple[float, np.ndarray, np.ndarray] | None = None
    for K in candidates:
        got = _pnp_rms(corners, obj, K)
        if got is None:
            continue
        if best is None or got[0] < best[0]:
            best = got
    if best is None:
        return None, None, None
    rms, rvec, tvec = best
    R, _ = cv2.Rodrigues(rvec)
    n_cam = R[:, 2]
    n_cam = n_cam / max(np.linalg.norm(n_cam), 1e-9)
    cosang = float(np.clip(abs(n_cam[2]), 0.0, 1.0))
    tilt_deg = float(math.degrees(math.acos(cosang)))
    dist_mm = float(np.linalg.norm(tvec) * 1000.0)
    return tilt_deg, dist_mm, rms


def load_seed_K(results_dir: Path) -> np.ndarray | None:
    path = results_dir / "camera_intrinsics.yaml"
    if not path.is_file():
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    K = data.get("K")
    if K is None:
        return None
    return np.asarray(K, dtype=np.float64)


def grade_image(
    path: Path,
    board: dict[str, Any],
    K_seed: np.ndarray | None,
    *,
    json_wait_s: float = 2.0,
) -> FrameGrade:
    if not wait_stable(path):
        return FrameGrade(str(path), "REJECT", reasons=["file unstable / empty"])

    jpath = path.with_suffix(".json")
    deadline = time.time() + json_wait_s
    while time.time() < deadline and not jpath.is_file():
        time.sleep(0.2)

    try:
        bgr = read_bgr(path)
    except Exception as exc:  # noqa: BLE001
        return FrameGrade(str(path), "REJECT", reasons=[f"read failed: {exc}"])

    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    lap = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    found = detect_with_swap(gray, board["pattern_size"])
    if found is None:
        return FrameGrade(
            str(path),
            "REJECT",
            reasons=["no chessboard corners (10x7 / swap)"],
            laplacian_var=lap,
        )

    pattern, corners = found
    cov = board_coverage(corners, (w, h))
    obj = object_points_for(board, pattern)
    tilt, dist, pnp_rms = rough_pose(corners, obj, (w, h), K_seed)
    flange = load_flange_T(jpath)
    flange_ok = flange is not None

    reasons: list[str] = []
    status = "GOOD"

    if not flange_ok:
        if not jpath.is_file():
            return FrameGrade(
                str(path),
                "WAIT_JSON",
                reasons=["corners OK; waiting for pose JSON with base_T_flange"],
                pattern=list(pattern),
                coverage=cov,
                tilt_deg=tilt,
                distance_mm=dist,
                flange_ok=False,
                laplacian_var=lap,
                pnp_rms_px=pnp_rms,
            )
        reasons.append("JSON missing base_T_flange — no hand-eye / paper BA")
        status = "REJECT"

    # 4K BFS boards often sit ~30–80 laplacian; only flag clearly soft frames.
    if lap < 25.0:
        reasons.append(f"soft/blurry (laplacian={lap:.1f})")
        status = "WEAK" if status == "GOOD" else status

    if cov["span_x"] < 0.12 or cov["span_y"] < 0.12:
        reasons.append("board too small in FOV (move closer)")
        status = "WEAK" if status == "GOOD" else status

    if cov["edge_margin"] < 0.02:
        reasons.append("board clipped near image border")
        status = "WEAK" if status == "GOOD" else status

    # Paper fixed-board BA wants some tilt + edge coverage, but extreme tilt is fragile.
    if tilt is not None and tilt > 45.0:
        reasons.append(f"extreme tilt {tilt:.1f} deg (paper BA prefers <=~35)")
        status = "WEAK" if status == "GOOD" else status

    # With a temporary isotropic K, ~3 px is normal; >5 px usually means bad corners.
    if pnp_rms is not None and pnp_rms > 5.0:
        reasons.append(f"rough PnP RMS {pnp_rms:.2f} px (corners noisy?)")
        status = "WEAK" if status == "GOOD" else status

    if status == "GOOD":
        tips = []
        if cov["center_offset"] < 0.08:
            tips.append("centered — also collect near edges")
        if tilt is not None and tilt < 5.0:
            tips.append("near-nadir — add tilted views")
        if tips:
            reasons.extend(tips)

    return FrameGrade(
        path=str(path),
        status=status,
        reasons=reasons,
        pattern=list(pattern),
        coverage=cov,
        tilt_deg=tilt,
        distance_mm=dist,
        flange_ok=flange_ok,
        laplacian_var=lap,
        pnp_rms_px=pnp_rms,
    )


def set_summary(grades: list[FrameGrade]) -> dict[str, Any]:
    good = [g for g in grades if g.status == "GOOD"]
    weak = [g for g in grades if g.status == "WEAK"]
    reject = [g for g in grades if g.status == "REJECT"]
    wait = [g for g in grades if g.status == "WAIT_JSON"]

    tilts = [g.tilt_deg for g in good + weak if g.tilt_deg is not None]
    dists = [g.distance_mm for g in good + weak if g.distance_mm is not None]
    offsets = [
        g.coverage["center_offset"]
        for g in good + weak
        if g.coverage is not None
    ]

    advice: list[str] = []
    n_usable = len(good) + len(weak)
    if n_usable < 20:
        advice.append(f"need more poses (usable={n_usable}; aim >=25 fit + 6 holdout)")
    if tilts and (max(tilts) - min(tilts) < 10.0):
        advice.append("tilt range small — add 10–30 deg board/camera tilt")
    if dists and (max(dists) - min(dists) < 40.0):
        advice.append("distance range small — move closer AND farther")
    if offsets and max(offsets) < 0.12:
        advice.append("all boards centered — put board near image edges")
    if len(good) >= 20 and not advice:
        advice.append("set looks paper-BA ready (fixed board); keep collecting holdouts")

    return {
        "n_total": len(grades),
        "n_good": len(good),
        "n_weak": len(weak),
        "n_reject": len(reject),
        "n_wait_json": len(wait),
        "tilt_deg_min_max": [min(tilts), max(tilts)] if tilts else None,
        "distance_mm_min_max": [min(dists), max(dists)] if dists else None,
        "advice": advice,
        "paper_ba_note": (
            "Paper BA = one shared base_T_board (run_bundle_adjustment.py). "
            "Soft per-view board BA (v2) is NOT the target for this capture."
        ),
    }


def print_grade(g: FrameGrade) -> None:
    name = Path(g.path).name
    bit = (
        f"tilt={g.tilt_deg:.1f}deg dist={g.distance_mm:.0f}mm "
        if g.tilt_deg is not None and g.distance_mm is not None
        else ""
    )
    rms = f"pnp={g.pnp_rms_px:.2f}px " if g.pnp_rms_px is not None else ""
    print(f"[{g.status}] {name}  {bit}{rms}flange={'Y' if g.flange_ok else 'N'}")
    for r in g.reasons:
        print(f"         - {r}")
    sys.stdout.flush()


def write_state(path: Path, grades: list[FrameGrade], summary: dict[str, Any]) -> None:
    payload = {
        "updated_unix": time.time(),
        "summary": summary,
        "frames": [asdict(g) for g in grades],
        "good_files": [Path(g.path).name for g in grades if g.status == "GOOD"],
        "weak_files": [Path(g.path).name for g in grades if g.status == "WEAK"],
        "reject_files": [Path(g.path).name for g in grades if g.status == "REJECT"],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    board = load_board(args.board)
    images_dir = Path(args.images)
    images_dir.mkdir(parents=True, exist_ok=True)
    state_path = Path(args.state)
    K_seed = load_seed_K(Path(args.seed_results)) if args.seed_results else None

    print("=== BFS checkerboard monitor (paper fixed-board BA) ===")
    print(f"watch:  {images_dir}")
    print(f"board:  {board['pattern_size']} @ {board['square_size_mm']} mm")
    print(f"state:  {state_path}")
    print(f"seed K: {'yes' if K_seed is not None else 'none (rough focal guess)'}")
    print("Collect. Ctrl+C to stop.\n")
    sys.stdout.flush()

    grades: dict[str, FrameGrade] = {}
    seen_mtime: dict[str, float] = {}

    last_set_sig: str | None = None
    while True:
        changed = False
        live_keys: set[str] = set()
        for img in list_images(images_dir):
            key = str(img.resolve())
            live_keys.add(key)
            try:
                mtime = img.stat().st_mtime
            except OSError:
                continue
            prev = seen_mtime.get(key)
            if prev is not None and abs(prev - mtime) < 1e-6 and key in grades:
                # Re-check WAIT_JSON periodically.
                if grades[key].status != "WAIT_JSON":
                    continue
            seen_mtime[key] = mtime
            g = grade_image(img, board, K_seed, json_wait_s=float(args.json_wait))
            old = grades.get(key)
            grades[key] = g
            if old is None or old.status != g.status or old.reasons != g.reasons:
                print_grade(g)
                changed = True

        # Drop grades for files removed from the folder (fresh redo).
        for gone in [k for k in grades if k not in live_keys]:
            del grades[gone]
            seen_mtime.pop(gone, None)
            changed = True

        ordered = [grades[k] for k in sorted(grades.keys())]
        summary = set_summary(ordered)
        set_sig = (
            f"{summary['n_good']}|{summary['n_weak']}|{summary['n_reject']}|"
            f"{summary['n_wait_json']}|{len(ordered)}"
        )
        if changed or not state_path.is_file() or set_sig != last_set_sig:
            write_state(state_path, ordered, summary)
            last_set_sig = set_sig
            print(
                f"-- SET  good={summary['n_good']} weak={summary['n_weak']} "
                f"reject={summary['n_reject']} wait={summary['n_wait_json']}"
            )
            for a in summary["advice"]:
                print(f"   tip: {a}")
            print()
            sys.stdout.flush()

        time.sleep(float(args.poll))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Live BFS checkerboard quality monitor")
    p.add_argument("--board", type=Path, required=True)
    p.add_argument("--images", type=Path, required=True)
    p.add_argument("--state", type=Path, required=True)
    p.add_argument(
        "--seed-results",
        type=Path,
        default=None,
        help="Optional bfs_cal/results for seed K (better tilt/distance)",
    )
    p.add_argument("--poll", type=float, default=2.0)
    p.add_argument("--json-wait", type=float, default=3.0)
    return p.parse_args()


if __name__ == "__main__":
    try:
        raise SystemExit(run(parse_args()))
    except KeyboardInterrupt:
        print("\nmonitor stopped")
        raise SystemExit(0)
