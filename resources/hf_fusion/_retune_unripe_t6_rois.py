# Unripe_T6 ROI retune only (offline). Re-runs GSAM stage-2 on listed grapes;
# does not move other masks. No hardware I/O.
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from _eval_roi_background import score_roi
from src.gsam_client import DEFAULT_GSAM_URL, post_json, require_gsam_ready, win_to_wsl
from src.gsam_overlay_sheet import write_session_qa_sheets
from src.gsam_plan_segment import STAGE2_QUERY_BOX_FLOOR, _pick_stage2_mask
from src.gsam_segment_io import rewrite_segmentation_from_masks
from src.utils.envi import parse_envi_hdr, read_bil_cube

SESSION = Path(r"E:\2026_Grape_Data_Collection\2026_Geneva_Concord\Unripe_T6")
STEM = "Unripe_T6"
GSAM_URL = DEFAULT_GSAM_URL

JOBS = [
    ("reflectance", "fx10e", [17], ["grape", "green grape", "green ball", "unripe grape", "green berry", "ball"]),
    ("reflectance", "swir3", [9, 17, 21], ["grape", "green ball", "ball", "green grape", "sphere"]),
    ("transmittance", "fx10e", [17], ["grape", "white ball", "ball", "green grape", "dark grape"]),
    ("transmittance", "swir3", [9, 21], ["grape", "pink ball", "ball", "white ball", "sphere"]),
]


def _load_stream(mode: str, camera: str):
    pre = SESSION / mode / camera / "preprocessed"
    seg = pre / "segmentation"
    rgb = np.array(Image.open(pre / f"{STEM}_rgb.png").convert("RGB"))
    ffc = parse_envi_hdr(pre / f"{STEM}_ffc.hdr")
    cube = read_bil_cube(ffc)
    wl = np.array(ffc.wavelengths_nm)
    man = json.loads((seg / "segmentation_results.json").read_text(encoding="utf-8"))
    dets = sorted(man["detections"], key=lambda det: int(det["roi"]))
    masks = [np.load(seg / "masks" / f"mask_{int(det['roi']):03d}.npy") > 0 for det in dets]
    scores = [float(det.get("score", 0.0)) for det in dets]
    labels = [str(det.get("label") or "") for det in dets]
    return pre, seg, rgb, cube, wl, man, masks, scores, labels


def _neighbor_median_n(masks: list[np.ndarray], skip: set[int]) -> float:
    ns = [int(mask.sum()) for i, mask in enumerate(masks, start=1) if i not in skip]
    return float(np.median(ns)) if ns else 1.0


def _crop_around(mask: np.ndarray, rgb: np.ndarray, pad: int) -> tuple[int, int, int, int]:
    ys, xs = np.nonzero(mask)
    h, w = mask.shape
    if ys.size == 0:
        return 0, 0, w, h
    x0 = max(0, int(xs.min()) - pad)
    y0 = max(0, int(ys.min()) - pad)
    x1 = min(w, int(xs.max()) + 1 + pad)
    y1 = min(h, int(ys.max()) + 1 + pad)
    min_side = 80 if w > 500 else 40
    if (x1 - x0) < min_side:
        cx = (x0 + x1) // 2
        x0 = max(0, cx - min_side // 2)
        x1 = min(w, x0 + min_side)
    if (y1 - y0) < min_side:
        cy = (y0 + y1) // 2
        y0 = max(0, cy - min_side // 2)
        y1 = min(h, y0 + min_side)
    return x0, y0, x1, y1


def _try_prompts(crop: np.ndarray, crop_dir: Path, prompts: list[str]) -> list[dict]:
    crop_dir.mkdir(parents=True, exist_ok=True)
    crop_png = crop_dir / "crop.png"
    Image.fromarray(crop, mode="RGB").save(crop_png)
    found: list[dict] = []
    for prompt in prompts:
        out = crop_dir / prompt.replace(" ", "_")
        if out.exists():
            shutil.rmtree(out)
        body = {
            "input_rgb": win_to_wsl(crop_png),
            "out_dir": win_to_wsl(out),
            "image_name": crop_png.name,
            "prompt": prompt,
            "max_dets": 8,
            "box_threshold": STAGE2_QUERY_BOX_FLOOR,
            "max_box_area_frac": 0.85,
        }
        result = post_json(GSAM_URL, "/segment", body)
        mask, note = _pick_stage2_mask(out, crop.shape[:2], min_score=0.12)
        if mask is None or not result.get("ok"):
            print(f"    prompt={prompt!r} miss ({note})")
            continue
        n = int(mask.sum())
        qa = score_roi(crop, mask)
        found.append({"prompt": prompt, "mask": mask, "n": n, "note": note, "qa": qa})
        print(f"    prompt={prompt!r} n={n} fill={qa['fill']} bg={qa['bg_score']} {note}")
    return found


def _pick_best(cands: list[dict], median_n: float, current_n: int) -> dict | None:
    if not cands:
        return None
    lo, hi = 0.55 * median_n, 1.20 * median_n
    ranked = []
    for item in cands:
        n = item["n"]
        qa = item["qa"]
        size_pen = abs(n - median_n) / max(1.0, median_n)
        huge = 0.6 if n > 1.35 * median_n else 0.0
        tiny = 0.5 if n < 0.40 * median_n else 0.0
        in_band = 0.0 if lo <= n <= hi else 0.35
        score = -qa["bg_score"] - 0.8 * size_pen - huge - tiny - in_band + 0.15 * qa["contrast"]
        if n < current_n * 0.92 and lo <= n <= hi:
            score += 0.12
        ranked.append((score, item))
    ranked.sort(key=lambda pair: pair[0], reverse=True)
    best_score, best = ranked[0]
    if best["n"] > 1.30 * median_n and best["n"] >= current_n * 0.95:
        return None
    _ = best_score
    return best


def retune_stream(mode: str, camera: str, rois: list[int], prompts: list[str]) -> None:
    pre, seg, rgb, cube, wl, man, masks, scores, labels = _load_stream(mode, camera)
    median_n = _neighbor_median_n(masks, set(rois))
    print(f"== {mode}/{camera} median_other={median_n:.0f} retune={rois}")
    tmp = pre / "_retune_tmp"
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir()
    changed = False
    pad = 36 if camera == "fx10e" else 14
    if mode == "transmittance" and camera == "fx10e":
        pad = 48
    for roi in rois:
        idx = roi - 1
        current = masks[idx]
        current_n = int(current.sum())
        x0, y0, x1, y1 = _crop_around(current, rgb, pad)
        crop = rgb[y0:y1, x0:x1]
        print(f"  ROI {roi:02d} current_n={current_n} crop={x1-x0}x{y1-y0}")
        cands = _try_prompts(crop, tmp / f"roi_{roi:03d}", prompts)
        best = _pick_best(cands, median_n, current_n)
        if best is None:
            print(f"  ROI {roi:02d}: keep current")
            continue
        full = np.zeros(current.shape, dtype=bool)
        full[y0:y1, x0:x1] = best["mask"] > 0
        new_n = int(full.sum())
        print(f"  ROI {roi:02d}: {current_n} -> {new_n} via {best['prompt']!r} ({best['note']})")
        masks[idx] = full
        scores[idx] = float(best["qa"].get("contrast", 0.0))
        labels[idx] = best["prompt"]
        changed = True
    if not changed:
        print("  no replacements")
        return
    prompt = str(man.get("prompt") or "") + " + retune " + ",".join(str(r) for r in rois)
    rewrite_segmentation_from_masks(
        seg,
        rgb,
        masks,
        scores,
        labels,
        f"{STEM}_rgb.png",
        prompt,
        cube,
        wl,
    )


def main() -> int:
    print("GSAM", require_gsam_ready(GSAM_URL))
    for mode, camera, rois, prompts in JOBS:
        retune_stream(mode, camera, rois, prompts)
    sheets = write_session_qa_sheets(SESSION)
    for path in sheets.values():
        print(f"  wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
