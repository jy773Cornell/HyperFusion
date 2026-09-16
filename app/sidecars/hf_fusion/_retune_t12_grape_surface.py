# Veraison_T12 overlay retune (offline). Re-runs GSAM on masks that do not
# form a filled grape disk. Backend only; no hardware I/O.
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.gsam_client import DEFAULT_GSAM_URL, post_json, require_gsam_ready, win_to_wsl
from src.gsam_overlay_sheet import write_session_qa_sheets
from src.gsam_plan_segment import STAGE2_QUERY_BOX_FLOOR
from src.gsam_segment_io import rewrite_segmentation_from_masks
from src.utils.envi import parse_envi_hdr, read_bil_cube

SESSION = Path(r"D:\Data\2026_Grape_Data_Collection\2026_Geneva_Concord\Veraison_T12")
STEM = "Veraison_T12"
GSAM_URL = DEFAULT_GSAM_URL

# Confirmed incomplete / fragmented from overlay + close-ups.
JOBS = [
    (
        "reflectance",
        "fx10e",
        [5, 7, 18, 21],
        72,
        ["purple grape", "purple marble", "purple berry", "grape", "berry", "marble", "purple ball", "ball"],
    ),
    (
        "reflectance",
        "swir3",
        [13, 20],
        52,
        ["green marble", "marble", "grape", "berry", "green grape", "green ball", "ball", "pea"],
    ),
    (
        "transmittance",
        "fx10e",
        [15],
        70,
        ["white marble", "marble", "grape", "berry", "white grape", "balloon", "white ball"],
    ),
]


def _largest_cc(mask: np.ndarray) -> np.ndarray:
    u8 = (mask > 0).astype(np.uint8)
    n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(u8, connectivity=8)
    if n_lab <= 1:
        return u8.astype(bool)
    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = 1 + int(np.argmax(areas))
    return labels == keep


def _fill_holes(mask: np.ndarray) -> np.ndarray:
    u8 = (mask > 0).astype(np.uint8)
    pad = np.pad(u8, 1, mode="constant", constant_values=0)
    h, w = pad.shape
    flood = pad.copy()
    ff = np.zeros((h + 2, w + 2), np.uint8)
    cv2.floodFill(flood, ff, (0, 0), 1)
    holes = flood == 0
    filled = (pad > 0) | holes
    return filled[1:-1, 1:-1].astype(bool)


def _solidify(mask: np.ndarray) -> np.ndarray:
    return _fill_holes(_largest_cc(mask))


def _mask_stats(mask: np.ndarray) -> dict:
    binary = mask > 0
    n = int(binary.sum())
    if n == 0:
        return {"n": 0, "fill": 0.0, "aspect": 0.0, "bw": 0, "bh": 0, "cc": 0.0}
    ys, xs = np.nonzero(binary)
    bw = int(xs.max() - xs.min() + 1)
    bh = int(ys.max() - ys.min() + 1)
    fill = n / float(bw * bh)
    aspect = max(bw, bh) / float(min(bw, bh))
    solid = _solidify(binary)
    cc = n / float(max(1, int(solid.sum())))
    return {"n": n, "fill": fill, "aspect": aspect, "bw": bw, "bh": bh, "cc": cc}


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
    return pre, seg, rgb, cube, wl, masks, scores, labels


def _neighbor_median_n(masks: list[np.ndarray], skip: set[int]) -> float:
    ns = [int(mask.sum()) for i, mask in enumerate(masks, start=1) if i not in skip]
    return float(np.median(ns)) if ns else 1.0


def _well_crop(mask: np.ndarray, rgb: np.ndarray, half: int) -> tuple[int, int, int, int]:
    h, w = rgb.shape[:2]
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return 0, 0, w, h
    cx = int((int(xs.min()) + int(xs.max())) // 2)
    cy = int((int(ys.min()) + int(ys.max())) // 2)
    x0 = max(0, cx - half)
    y0 = max(0, cy - half)
    x1 = min(w, cx + half)
    y1 = min(h, cy + half)
    return x0, y0, x1, y1


def _collect_dets(crop: np.ndarray, crop_dir: Path, prompts: list[str]) -> list[dict]:
    if crop_dir.exists():
        shutil.rmtree(crop_dir)
    crop_dir.mkdir(parents=True)
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
            "max_dets": 10,
            "box_threshold": STAGE2_QUERY_BOX_FLOOR,
            "max_box_area_frac": 0.90,
        }
        result = post_json(GSAM_URL, "/segment", body)
        if not result.get("ok"):
            print(f"    prompt {prompt!r} failed: {result}")
            continue
        man_path = out / "segmentation_results.json"
        if not man_path.is_file():
            continue
        man = json.loads(man_path.read_text(encoding="utf-8"))
        for det in man.get("detections") or []:
            roi = int(det["roi"])
            path = out / "masks" / f"mask_{roi:03d}.npy"
            if not path.is_file():
                continue
            raw = np.load(path) > 0
            if not raw.any():
                continue
            solid = _solidify(raw)
            st = _mask_stats(solid)
            found.append(
                {
                    "prompt": prompt,
                    "score": float(det.get("score", 0.0)),
                    "mask": solid,
                    **st,
                }
            )
    return found


def _pick(cands: list[dict], med: float, crop_area: int) -> dict | None:
    # Full grape disks are often a bit larger than the tray median. Reject
    # fragments and whole-crop well fills; then take the highest GSAM score.
    disks = [
        c
        for c in cands
        if c["fill"] >= 0.72
        and c["aspect"] <= 1.20
        and c["n"] >= 0.70 * med
        and c["n"] < 0.65 * crop_area
    ]
    if not disks:
        return None
    disks.sort(key=lambda c: (c["score"], c["n"]), reverse=True)
    return disks[0]


def _print_stream_stats(mode: str, camera: str, masks: list[np.ndarray], labels: list[str]) -> None:
    print(f"--- {mode}/{camera} ---")
    for i, mask in enumerate(masks, start=1):
        st = _mask_stats(mask)
        flag = ""
        if st["fill"] < 0.70 or st["aspect"] > 1.25 or st["cc"] < 0.90:
            flag = "  << broken surface"
        print(
            f"  {i:02d} n={st['n']:5d} fill={st['fill']:.2f} "
            f"asp={st['aspect']:.2f} {st['bw']}x{st['bh']} {labels[i-1]!r}{flag}"
        )


def retune_job(mode: str, camera: str, rois: list[int], half: int, prompts: list[str]) -> bool:
    pre, seg, rgb, cube, wl, masks, scores, labels = _load_stream(mode, camera)
    _print_stream_stats(mode, camera, masks, labels)
    skip = set(rois)
    med = _neighbor_median_n(masks, skip)
    print(f"  neighbor median n={med:.0f}  retune={rois}")
    changed = False
    tmp = pre / "_surface_retune"
    tmp.mkdir(exist_ok=True)
    for roi in rois:
        idx = roi - 1
        crop_halfs = [half, half + 16]
        chosen = None
        used_half = half
        for try_half in crop_halfs:
            x0, y0, x1, y1 = _well_crop(masks[idx], rgb, try_half)
            crop = rgb[y0:y1, x0:x1]
            print(f"  ROI {roi:02d} crop {x1-x0}x{y1-y0} half={try_half}")
            cands = _collect_dets(crop, tmp / f"{camera}_{roi:02d}_{try_half}", prompts)
            for c in sorted(cands, key=lambda d: d["score"], reverse=True)[:6]:
                print(
                    f"    {c['prompt']!r} score={c['score']:.3f} n={c['n']} "
                    f"fill={c['fill']:.2f} asp={c['aspect']:.2f}"
                )
            pick = _pick(cands, med, crop.shape[0] * crop.shape[1])
            if pick is not None:
                full = np.zeros(masks[idx].shape, dtype=bool)
                full[y0:y1, x0:x1] = pick["mask"]
                chosen = (full, pick)
                used_half = try_half
                break
        if chosen is None:
            print(f"  ROI {roi:02d}: no disk-like GSAM hit, keep current n={int(masks[idx].sum())}")
            continue
        full, pick = chosen
        old_n = int(masks[idx].sum())
        masks[idx] = full
        scores[idx] = pick["score"]
        labels[idx] = pick["prompt"]
        changed = True
        print(
            f"  ROI {roi:02d}: {old_n} -> {int(full.sum())} "
            f"{pick['prompt']!r} score={pick['score']:.3f} fill={pick['fill']:.2f} half={used_half}"
        )
    if changed:
        rewrite_segmentation_from_masks(
            seg,
            rgb,
            [m.astype(np.uint8) for m in masks],
            scores,
            labels,
            f"{STEM}_rgb.png",
            "grape-surface-retune",
            cube,
            wl,
        )
        print(f"  wrote {mode}/{camera}")
    return changed


def main() -> int:
    print("GSAM", require_gsam_ready(GSAM_URL))
    any_changed = False
    for mode, camera, rois, half, prompts in JOBS:
        if retune_job(mode, camera, rois, half, prompts):
            any_changed = True
    if any_changed:
        try:
            sheets = write_session_qa_sheets(SESSION)
            print("QA", {k: str(v) for k, v in sheets.items()})
        except OSError as exc:
            print(f"QA sheets skipped: {exc}")
    else:
        print("no masks changed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
