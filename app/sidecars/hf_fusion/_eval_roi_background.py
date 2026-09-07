# Evaluate GSAM ROI masks for background contamination (offline QA helper).
# Scores masks; does not rewrite them. Used to choose better stage-2 prompts.
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def _mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def score_roi(rgb: np.ndarray, mask: np.ndarray) -> dict:
    """Higher bg_score => more likely background included."""
    h, w = mask.shape
    n = int(mask.sum())
    if n == 0:
        return {"n": 0, "bg_score": 1.0, "flags": ["empty"]}

    bb = _mask_bbox(mask)
    assert bb is not None
    x0, y0, x1, y1 = bb
    bw, bh = x1 - x0, y1 - y0
    box_area = max(1, bw * bh)
    fill = n / float(box_area)
    area_frac = n / float(h * w)

    # Border contact of the mask within image (touches crop edge => often spilled).
    edge = np.zeros_like(mask, dtype=bool)
    edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
    edge_frac = float((mask & edge).sum()) / float(n)

    # Luminance contrast: grape vs nearby unmasked pixels in expanded bbox.
    lum = rgb.astype(np.float32).mean(axis=2)
    pad = 4
    ex0, ey0 = max(0, x0 - pad), max(0, y0 - pad)
    ex1, ey1 = min(w, x1 + pad), min(h, y1 + pad)
    local = np.zeros_like(mask, dtype=bool)
    local[ey0:ey1, ex0:ex1] = True
    outside = local & ~mask
    in_mean = float(lum[mask].mean())
    out_mean = float(lum[outside].mean()) if outside.any() else in_mean
    contrast = abs(in_mean - out_mean) / 255.0

    # Ring of mask near bbox border (well rim) vs core.
    core = np.zeros_like(mask, dtype=bool)
    inset = max(2, int(0.15 * min(bw, bh)))
    core[y0 + inset : max(y0 + inset + 1, y1 - inset), x0 + inset : max(x0 + inset + 1, x1 - inset)] = True
    ring = mask & ~core
    ring_frac = float(ring.sum()) / float(n)

    flags = []
    bg_score = 0.0
    # Large fill of bounding box often means whole well / background included.
    if fill > 0.78:
        bg_score += 0.45
        flags.append("high_bbox_fill")
    elif fill > 0.65:
        bg_score += 0.25
        flags.append("med_bbox_fill")
    if edge_frac > 0.02:
        bg_score += 0.25
        flags.append("touches_image_edge")
    if ring_frac > 0.45 and fill > 0.55:
        bg_score += 0.20
        flags.append("thick_rim")
    if contrast < 0.04 and fill > 0.5:
        bg_score += 0.15
        flags.append("low_contrast")
    # Very large vs image => likely hole/well not berry.
    if area_frac > 0.045:
        bg_score += 0.15
        flags.append("large_area")

    return {
        "n": n,
        "fill": round(fill, 3),
        "edge_frac": round(edge_frac, 4),
        "ring_frac": round(ring_frac, 3),
        "contrast": round(contrast, 3),
        "area_frac": round(area_frac, 4),
        "bg_score": round(min(1.0, bg_score), 3),
        "flags": flags,
    }


def evaluate_stream(session: Path, mode: str, camera: str, stem: str) -> dict:
    pre = session / mode / camera / "preprocessed"
    seg = pre / "segmentation"
    rgb_path = pre / f"{stem}_rgb.png"
    if not rgb_path.is_file():
        matches = sorted(pre.glob("*_rgb.png"))
        if not matches:
            raise FileNotFoundError(pre)
        rgb_path = matches[0]
    rgb = np.array(Image.open(rgb_path).convert("RGB"))
    man = json.loads((seg / "segmentation_results.json").read_text(encoding="utf-8"))
    rows = []
    for det in man.get("detections") or []:
        roi = int(det["roi"])
        mask = np.load(seg / "masks" / f"mask_{roi:03d}.npy") > 0
        s = score_roi(rgb, mask)
        s["roi"] = roi
        s["score"] = float(det.get("score", 0.0))
        s["label"] = det.get("label", "")
        rows.append(s)
    bad = [r for r in rows if r["bg_score"] >= 0.35]
    return {
        "stream": f"{mode}/{camera}",
        "count": len(rows),
        "bad_count": len(bad),
        "mean_bg": round(float(np.mean([r["bg_score"] for r in rows])) if rows else 0.0, 3),
        "bad_rois": [r["roi"] for r in bad],
        "worst": sorted(rows, key=lambda r: r["bg_score"], reverse=True)[:8],
        "rows": rows,
    }


def main() -> int:
    session = Path(sys.argv[1] if len(sys.argv) > 1 else r"E:\2026_Grape_Data_Collection\2026_Geneva_Concord\Unripe_T1")
    stem = sys.argv[2] if len(sys.argv) > 2 else session.name
    streams = [
        ("reflectance", "fx10e"),
        ("reflectance", "swir3"),
        ("transmittance", "fx10e"),
        ("transmittance", "swir3"),
    ]
    reports = []
    for mode, camera in streams:
        try:
            reports.append(evaluate_stream(session, mode, camera, stem))
        except Exception as exc:
            reports.append({"stream": f"{mode}/{camera}", "error": str(exc)})
    print(json.dumps({"session": str(session), "reports": reports}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
