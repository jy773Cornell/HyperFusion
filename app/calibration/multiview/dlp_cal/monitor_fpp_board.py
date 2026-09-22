# Live DLP / FPP checkerboard burst monitor for camera–projector stereo capture.
# dlp_cal layer. No robot motion. Watches a folder; grades complete HDMI PSP bursts.
"""Watch FPP Execute bursts; report GOOD / WEAK / REJECT and copy keepers.

A burst is GOOD only when all 70 inner corners sit inside the lit DLP patch
(same rule as ``check_fpp_board.py``). Prefer capturing after the classical BFS
``K``/``D`` are in ``hyperfusion.cfg``; pass ``--camera-results`` so grading
uses that classical camera model even if pose JSON still has an old lens.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
SIDECAR_FPP = ROOT.parents[2] / "sidecars" / "fpp"
DEFAULT_BOARD = ROOT.parent / "bfs_cal" / "board.yaml"

if str(SIDECAR_FPP) not in sys.path:
    sys.path.insert(0, str(SIDECAR_FPP))

from fpp_depth.board_detect import gray_u8, load_board  # noqa: E402
from fpp_depth.capture import (  # noqa: E402
    STEP_COUNT,
    U_ONLY_STEP_COUNT,
    _burst_stride,
    _read_json,
    _stem_sort_key,
    camera_intrinsics,
    list_burst_jobs,
    load_burst,
)
from fpp_depth.decode import decode_burst  # noqa: E402
from fpp_depth.geometry import observe_burst  # noqa: E402
from fpp_depth.undistort import undistort_burst  # noqa: E402

SKIP_DIR_NAMES = {
    "good",
    "reject",
    "rejects",
    "board_check",
    "results",
    "patterns",
    "decode",
    "fusion",
    "undistort_preview",
}


@dataclass
class BurstGrade:
    key: str
    dir: str
    start: str
    status: str  # GOOD | WEAK | REJECT | WAIT
    reasons: list[str] = field(default_factory=list)
    n_corners: int | None = None
    n_in_patch: int | None = None
    need: int | None = None
    decode_px: int | None = None
    fx_json: float | None = None
    overlay: str | None = None


def load_camera_kd(results_dir: Path) -> tuple[np.ndarray, np.ndarray] | None:
    path = Path(results_dir) / "camera_intrinsics.yaml"
    if not path.is_file():
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    K = np.asarray(data["K"], dtype=np.float64).reshape(3, 3)
    D = np.asarray(data["D"], dtype=np.float64).reshape(-1)
    return K, D


def patch_burst_intrinsics(burst: Any, K: np.ndarray, D: np.ndarray) -> None:
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    dist = [float(x) for x in np.asarray(D).reshape(-1)[:5]]
    for fr in burst.frames:
        meta = fr.meta
        if "intrinsics" not in meta or not isinstance(meta["intrinsics"], dict):
            meta["intrinsics"] = {}
        meta["intrinsics"].update(
            {
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy,
                "distortion": dist,
                "source": "bfs_cal/results (monitor inject)",
            }
        )


def write_intrinsics_into_jsons(
    paths: list[Path], K: np.ndarray, D: np.ndarray, *, source: str
) -> int:
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    dist = [float(x) for x in np.asarray(D).reshape(-1)[:5]]
    n = 0
    for jpath in paths:
        if jpath.suffix.lower() != ".json" or not jpath.is_file():
            continue
        meta = json.loads(jpath.read_text(encoding="utf-8-sig"))
        if "intrinsics" not in meta or not isinstance(meta["intrinsics"], dict):
            meta["intrinsics"] = {}
        meta["intrinsics"].update(
            {
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy,
                "distortion": dist,
                "source": source,
            }
        )
        jpath.write_text(json.dumps(meta, indent=4) + "\n", encoding="utf-8")
        n += 1
    return n


def burst_chunk_paths(burst_dir: Path, start: Path) -> list[Path]:
    """TIFF (+ matching JSON) that belong to one complete burst starting at *start*."""
    tiffs = sorted(burst_dir.glob("*.tif"), key=_stem_sort_key)
    tiffs += sorted(p for p in burst_dir.glob("*.tiff") if p not in set(tiffs))
    start_name = Path(start).name
    offset = next((i for i, p in enumerate(tiffs) if p.name == start_name), None)
    if offset is None:
        return []
    stride = _burst_stride(tiffs, offset)
    if offset + stride > len(tiffs):
        return []
    out: list[Path] = []
    for path in tiffs[offset : offset + stride]:
        out.append(path)
        j = path.with_suffix(".json")
        if j.is_file():
            out.append(j)
    return out


def _overlay(
    white: np.ndarray, corners: np.ndarray, in_patch: np.ndarray, mask: np.ndarray
) -> np.ndarray:
    gray = gray_u8(white)
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    contour = (mask.astype(np.uint8)) * 255
    edges = cv2.Canny(contour, 40, 120)
    bgr[edges > 0] = (255, 220, 0)
    pts = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    for i, (x, y) in enumerate(pts):
        color = (0, 220, 0) if bool(in_patch[i]) else (0, 0, 255)
        cv2.circle(bgr, (int(round(x)), int(round(y))), 8, color, 2, cv2.LINE_AA)
    return bgr


def grade_burst(
    burst_dir: Path,
    start: Path,
    board: dict[str, Any],
    *,
    camera_kd: tuple[np.ndarray, np.ndarray] | None,
    overlay_dir: Path | None,
    min_modulation: float,
) -> BurstGrade:
    key = f"{burst_dir.resolve()}::{start.name}"
    need = int(board["n_corners"])
    g = BurstGrade(
        key=key,
        dir=str(burst_dir),
        start=start.name,
        status="REJECT",
        need=need,
    )
    try:
        first_json = start.with_suffix(".json")
        if first_json.is_file():
            k_json = camera_intrinsics(_read_json(first_json))
            if k_json is not None:
                g.fx_json = float(k_json[0][0, 0])
                if g.fx_json < 3000.0:
                    g.reasons.append(
                        f"JSON fx={g.fx_json:.0f} looks like old short lens; "
                        "restart app with new cfg or rely on --camera-results inject"
                    )

        burst = load_burst(burst_dir, start=start)
        if camera_kd is not None:
            patch_burst_intrinsics(burst, camera_kd[0], camera_kd[1])
        undistort_burst(burst)
        decoded = decode_burst(burst, min_modulation=min_modulation)
        g.decode_px = int(decoded.mask.sum())
        view, reason = observe_burst(burst, decoded, board, require_all_in_patch=False)
        if view is None:
            g.status = "REJECT"
            g.reasons.append(reason or "observe_failed")
            return g
        g.n_corners = int(view["n_corners"])
        g.n_in_patch = int(view["n_in_patch"])
        if overlay_dir is not None:
            overlay_dir.mkdir(parents=True, exist_ok=True)
            label = (
                "good"
                if (g.n_corners == need and g.n_in_patch == need and reason == "ok")
                else "bad"
            )
            name = f"{burst_dir.name}_{start.stem}_{label}.jpg"
            ov = _overlay(view["white"], view["camera_xy"], view["in_patch"], decoded.mask)
            cv2.imwrite(str(overlay_dir / name), ov, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            g.overlay = name
        if g.n_corners == need and g.n_in_patch == need and reason == "ok":
            g.status = "GOOD"
            g.reasons.append("all corners in DLP patch")
        elif g.n_corners == need and g.n_in_patch >= int(0.9 * need):
            g.status = "WEAK"
            g.reasons.append(
                f"corners_in_dark={need - g.n_in_patch} (need 0 for GOOD)"
            )
        else:
            g.status = "REJECT"
            g.reasons.append(
                f"corners={g.n_corners}/{need} in_patch={g.n_in_patch} ({reason})"
            )
    except Exception as exc:  # noqa: BLE001
        g.status = "REJECT"
        g.reasons.append(str(exc))
    return g


def keep_burst(
    burst_dir: Path,
    start: Path,
    dest_root: Path,
    *,
    camera_kd: tuple[np.ndarray, np.ndarray] | None,
) -> Path:
    dest_root.mkdir(parents=True, exist_ok=True)
    dest = dest_root / f"{burst_dir.name}__{start.stem}"
    dest.mkdir(parents=True, exist_ok=True)
    paths = burst_chunk_paths(burst_dir, start)
    for src in paths:
        target = dest / src.name
        if not target.is_file() or target.stat().st_size != src.stat().st_size:
            shutil.copy2(src, target)
    if camera_kd is not None:
        write_intrinsics_into_jsons(
            list(dest.glob("*.json")),
            camera_kd[0],
            camera_kd[1],
            source="bfs_cal/results (classical)",
        )
    return dest


def set_summary(grades: list[BurstGrade]) -> dict[str, Any]:
    n_good = sum(1 for g in grades if g.status == "GOOD")
    n_weak = sum(1 for g in grades if g.status == "WEAK")
    n_reject = sum(1 for g in grades if g.status == "REJECT")
    advice: list[str] = []
    if n_good < 13:
        advice.append(
            f"need more GOOD bursts (have {n_good}; aim >=13 fit + 3 holdout)"
        )
    elif n_good < 16:
        advice.append(
            f"have {n_good} GOOD — add ~3 tilted hold-outs then run calibrate_fpp_geometry"
        )
    else:
        advice.append("set looks stereo-ready; run calibrate_fpp_geometry.py on good/")
    advice.append("whole board must sit inside the bright DLP rectangle")
    advice.append("tilt + shift board between Execute shots; keep robot tip / stage locked")
    return {
        "n_good": n_good,
        "n_weak": n_weak,
        "n_reject": n_reject,
        "n_total": len(grades),
        "advice": advice,
        "need_for_fit": 13,
        "need_holdout": 3,
    }


def print_grade(g: BurstGrade) -> None:
    bits = []
    if g.n_corners is not None:
        bits.append(f"corners={g.n_corners}/{g.need}")
    if g.n_in_patch is not None:
        bits.append(f"in_patch={g.n_in_patch}")
    if g.decode_px is not None:
        bits.append(f"decode_px={g.decode_px}")
    if g.fx_json is not None:
        bits.append(f"fx_json={g.fx_json:.0f}")
    print(f"[{g.status}] {Path(g.dir).name}/{g.start}  {' '.join(bits)}")
    for r in g.reasons:
        print(f"         - {r}")
    sys.stdout.flush()


def write_state(path: Path, grades: list[BurstGrade], summary: dict[str, Any]) -> None:
    payload = {
        "updated_unix": time.time(),
        "summary": summary,
        "bursts": [asdict(g) for g in grades],
        "good": [g.key for g in grades if g.status == "GOOD"],
        "weak": [g.key for g in grades if g.status == "WEAK"],
        "reject": [g.key for g in grades if g.status == "REJECT"],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def filter_jobs(root: Path) -> list[tuple[Path, Path]]:
    jobs = []
    for burst_dir, start in list_burst_jobs(root):
        parts = {x.lower() for x in burst_dir.parts}
        if parts & SKIP_DIR_NAMES:
            continue
        jobs.append((burst_dir, start))
    return jobs


def run(args: argparse.Namespace) -> int:
    board = load_board(args.board)
    watch = Path(args.input)
    watch.mkdir(parents=True, exist_ok=True)
    keep_dir = Path(args.keep_dir) if args.keep_dir else None
    reject_dir = Path(args.reject_dir) if args.reject_dir else None
    overlay_dir = Path(args.overlay_dir) if args.overlay_dir else (watch / "board_check")
    state_path = Path(args.state)
    camera_kd = load_camera_kd(Path(args.camera_results)) if args.camera_results else None

    print("=== DLP / FPP checkerboard burst monitor ===")
    print(f"watch:     {watch}")
    print(f"board:     {board['pattern_size']} @ {board['square_size_mm']} mm")
    print(f"state:     {state_path}")
    print(f"overlay:   {overlay_dir}")
    if camera_kd is not None:
        print(f"camera K:  injected from {args.camera_results}  fx={camera_kd[0][0,0]:.1f}")
    else:
        print("camera K:  from each burst JSON (restart app after classical cfg write)")
    print(f"keep GOOD: {keep_dir if keep_dir else '(off)'}")
    if reject_dir:
        print(f"keep REJECT copies: {reject_dir}")
    print(
        f"Expect complete visual+{STEP_COUNT} or {STEP_COUNT}-frame PSP bursts "
        f"(legacy {U_ONLY_STEP_COUNT} also ok). Ctrl+C to stop.\n"
    )
    sys.stdout.flush()

    grades: dict[str, BurstGrade] = {}
    seen_sig: dict[str, tuple[float, int]] = {}
    last_set_sig: str | None = None

    while True:
        changed = False
        live: set[str] = set()
        for burst_dir, start in filter_jobs(watch):
            key = f"{burst_dir.resolve()}::{start.name}"
            live.add(key)
            paths = burst_chunk_paths(burst_dir, start)
            if not paths:
                continue
            try:
                mtime = max(p.stat().st_mtime for p in paths)
                nbytes = sum(p.stat().st_size for p in paths)
            except OSError:
                continue
            sig = (mtime, nbytes)
            prev = seen_sig.get(key)
            if prev is not None and prev == sig and key in grades:
                continue
            time.sleep(0.35)
            try:
                mtime2 = max(p.stat().st_mtime for p in paths)
                nbytes2 = sum(p.stat().st_size for p in paths)
            except OSError:
                continue
            if (mtime2, nbytes2) != sig:
                continue
            seen_sig[key] = sig
            g = grade_burst(
                burst_dir,
                start,
                board,
                camera_kd=camera_kd,
                overlay_dir=overlay_dir,
                min_modulation=float(args.min_modulation),
            )
            old = grades.get(key)
            grades[key] = g
            if old is None or old.status != g.status or old.reasons != g.reasons:
                print_grade(g)
                changed = True
            if keep_dir is not None and g.status == "GOOD":
                keep_burst(burst_dir, start, keep_dir, camera_kd=camera_kd)
            if reject_dir is not None and g.status == "REJECT":
                keep_burst(burst_dir, start, reject_dir, camera_kd=camera_kd)

        for gone in [k for k in grades if k not in live]:
            del grades[gone]
            seen_sig.pop(gone, None)
            changed = True

        ordered = [grades[k] for k in sorted(grades.keys())]
        summary = set_summary(ordered)
        if keep_dir is not None:
            summary["keep_dir"] = str(keep_dir)
            summary["n_kept_bursts"] = (
                len([p for p in keep_dir.iterdir() if p.is_dir()])
                if keep_dir.is_dir()
                else 0
            )
        set_sig = (
            f"{summary['n_good']}|{summary['n_weak']}|"
            f"{summary['n_reject']}|{len(ordered)}"
        )
        if changed or not state_path.is_file() or set_sig != last_set_sig:
            write_state(state_path, ordered, summary)
            last_set_sig = set_sig
            print(
                f"-- SET  good={summary['n_good']} weak={summary['n_weak']} "
                f"reject={summary['n_reject']}"
            )
            for a in summary["advice"]:
                print(f"   tip: {a}")
            print()
            sys.stdout.flush()

        time.sleep(float(args.poll))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True, help="Folder to watch (checkerboard/)")
    p.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    p.add_argument("--state", type=Path, required=True)
    p.add_argument("--keep-dir", type=Path, default=None)
    p.add_argument("--reject-dir", type=Path, default=None)
    p.add_argument("--overlay-dir", type=Path, default=None)
    p.add_argument(
        "--camera-results",
        type=Path,
        default=None,
        help="bfs_cal/results with classical camera_intrinsics.yaml (inject K/D)",
    )
    p.add_argument("--poll", type=float, default=3.0)
    p.add_argument("--min-modulation", type=float, default=0.15)
    return p.parse_args()


if __name__ == "__main__":
    try:
        raise SystemExit(run(parse_args()))
    except KeyboardInterrupt:
        print("\nmonitor stopped")
        raise SystemExit(0)
