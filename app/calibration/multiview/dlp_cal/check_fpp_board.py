# QA one checkerboard FPP Execute burst (white frame + lit patch).
# dlp_cal layer. No robot motion. Run after each shot; keep only GOOD bursts.
"""Check 70 inner corners sit inside the DLP patch. Writes a white overlay JPEG."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
SIDECAR_FPP = ROOT.parents[2] / "sidecars" / "fpp"
DEFAULT_BOARD = ROOT.parent / "bfs_cal" / "board.yaml"
DEFAULT_INPUT = ROOT / "checkerboard"

if str(SIDECAR_FPP) not in sys.path:
    sys.path.insert(0, str(SIDECAR_FPP))

from fpp_depth.board_detect import gray_u8, load_board  # noqa: E402
from fpp_depth.capture import list_burst_jobs, load_burst  # noqa: E402
from fpp_depth.decode import decode_burst  # noqa: E402
from fpp_depth.geometry import observe_burst  # noqa: E402
from fpp_depth.undistort import undistort_burst  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="checkerboard/ or one burst / parent of multiview_*")
    p.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    p.add_argument("--out", type=Path, default=None, help="Overlay folder (default: <input>/board_check)")
    p.add_argument("--min-modulation", type=float, default=0.15)
    return p.parse_args()


def _overlay(white: np.ndarray, corners: np.ndarray, in_patch: np.ndarray, mask: np.ndarray) -> np.ndarray:
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


def main() -> int:
    args = parse_args()
    board = load_board(args.board)
    jobs = list_burst_jobs(args.input)
    if not jobs:
        raise FileNotFoundError(f"No FPP burst in {args.input}")
    dest = Path(args.out) if args.out is not None else Path(args.input) / "board_check"
    dest.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    n_good = 0
    for burst_dir, start in jobs:
        rec: dict = {"dir": str(burst_dir), "start": start.name, "ok": False}
        try:
            burst = load_burst(burst_dir, start=start)
            undistort_burst(burst)
            decoded = decode_burst(burst, min_modulation=args.min_modulation)
            view, reason = observe_burst(burst, decoded, board, require_all_in_patch=False)
            rec["decode_px"] = int(decoded.mask.sum())
            if view is None:
                rec["reason"] = reason
                print(f"BAD  {start.name}  {reason}", flush=True)
                rows.append(rec)
                continue
            rec.update(
                {
                    "n_corners": view["n_corners"],
                    "n_in_patch": view["n_in_patch"],
                    "need": int(board["n_corners"]),
                    "reason": reason,
                }
            )
            need = int(board["n_corners"])
            good = (
                view["n_corners"] == need
                and view["n_in_patch"] == need
                and reason == "ok"
            )
            rec["ok"] = good
            label = "GOOD" if good else "BAD"
            if not good:
                rec["reason"] = f"corners_in_dark={need - view['n_in_patch']}"
            overlay = _overlay(view["white"], view["camera_xy"], view["in_patch"], decoded.mask)
            name = f"{burst_dir.name}_{start.stem}_{label.lower()}.jpg"
            cv2.imwrite(str(dest / name), overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            rec["overlay"] = name
            if good:
                n_good += 1
            print(
                f"{label}  {burst_dir.name}/{start.name}  "
                f"corners={view['n_corners']}/{need}  in_patch={view['n_in_patch']}  "
                f"decode_px={rec['decode_px']}",
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001
            rec["reason"] = str(exc)
            print(f"BAD  {start.name}  {exc}", flush=True)
        rows.append(rec)
    report = {
        "ok": n_good > 0,
        "n": len(rows),
        "n_good": n_good,
        "n_bad": len(rows) - n_good,
        "need_for_fit": 13,
        "need_holdout": 3,
        "bursts": rows,
    }
    (dest / "board_check.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"n_good": n_good, "n_bad": report["n_bad"], "out": str(dest)}, indent=2))
    return 0 if n_good else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise SystemExit(1)
