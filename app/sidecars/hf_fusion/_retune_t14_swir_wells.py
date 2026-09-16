# Veraison_T14 SWIR mask reuse + ROI 25 GSAM retune (offline). Backend only; no hardware I/O.
from __future__ import annotations

from pathlib import Path

import numpy as np

import _retune_t12_grape_surface as r
from src.gsam_plan_segment import _resize_mask

r.SESSION = Path(r"D:\Data_JY\2026_Grape_Data_Collection\2026_Geneva_Concord\Veraison_T14")
r.STEM = "Veraison_T14"
CROP_HALFS = [24, 28, 32, 36, 40, 44, 52]
REFL_PROMPTS = ["green marble", "marble", "grape", "berry", "green grape", "green ball", "ball", "pea"]
TX_PROMPTS = [
    "grape",
    "berry",
    "marble",
    "white marble",
    "ball",
    "pea",
    "circle",
    "pink grape",
]


def _pick(cands: list[dict], med: float, crop_area: int) -> dict | None:
    disks = [
        c
        for c in cands
        if c["fill"] >= 0.72
        and c["aspect"] <= 1.22
        and c["n"] >= 0.70 * med
        and c["n"] <= 1.18 * med
        and c["n"] < 0.65 * crop_area
        and bool(c["mask"][c["mask"].shape[0] // 2, c["mask"].shape[1] // 2])
    ]
    if not disks:
        return None
    disks.sort(key=lambda c: (c["score"], c["n"]), reverse=True)
    return disks[0]


def _copy_rois(src_mode: str, dst_mode: str, rois: list[int]) -> None:
    _, _, src_rgb, _, _, src_masks, src_scores, src_labels = r._load_stream(src_mode, "swir3")
    pre, seg, dst_rgb, cube, wl, dst_masks, dst_scores, dst_labels = r._load_stream(dst_mode, "swir3")
    height, width = dst_rgb.shape[:2]
    for roi in rois:
        idx = roi - 1
        copied = _resize_mask(src_masks[idx].astype(np.uint8), height, width) > 0
        old_n = int(dst_masks[idx].sum())
        dst_masks[idx] = copied
        dst_scores[idx] = src_scores[idx]
        dst_labels[idx] = src_labels[idx]
        print(
            f"  copy {src_mode}/swir3 ROI {roi:02d} -> {dst_mode}/swir3 "
            f"{old_n} -> {int(copied.sum())} (src {src_rgb.shape[1]}x{src_rgb.shape[0]} "
            f"dst {width}x{height})"
        )
    r.rewrite_segmentation_from_masks(
        seg,
        dst_rgb,
        [m.astype(np.uint8) for m in dst_masks],
        dst_scores,
        dst_labels,
        f"{r.STEM}_rgb.png",
        f"reused from {src_mode}/swir3 rois {rois}",
        cube,
        wl,
    )
    print(f"  wrote {dst_mode}/swir3")


def retune_rois(mode: str, rois: list[int], prompts: list[str], crop_mask: np.ndarray | None) -> bool:
    pre, seg, rgb, cube, wl, masks, scores, labels = r._load_stream(mode, "swir3")
    r._print_stream_stats(mode, "swir3", masks, labels)
    skip = set(rois)
    med = r._neighbor_median_n(masks, skip)
    print(f"  neighbor median n={med:.0f}  retune={rois}")
    changed = False
    tmp = pre / "_surface_retune"
    tmp.mkdir(exist_ok=True)
    for roi in rois:
        idx = roi - 1
        seed = crop_mask if crop_mask is not None else masks[idx]
        chosen = None
        used_half = CROP_HALFS[0]
        for try_half in CROP_HALFS:
            x0, y0, x1, y1 = r._well_crop(seed, rgb, try_half)
            crop = rgb[y0:y1, x0:x1]
            print(f"  {mode} ROI {roi:02d} crop {x1-x0}x{y1-y0} half={try_half}")
            cands = r._collect_dets(crop, tmp / f"{mode}_swir3_{roi:02d}_{try_half}", prompts)
            shown = 0
            for c in sorted(cands, key=lambda d: d["score"], reverse=True):
                cy, cx = c["mask"].shape[0] // 2, c["mask"].shape[1] // 2
                if not bool(c["mask"][cy, cx]):
                    continue
                print(
                    f"    ctr {c['prompt']!r} score={c['score']:.3f} n={c['n']} "
                    f"fill={c['fill']:.2f} asp={c['aspect']:.2f}"
                )
                shown += 1
                if shown >= 8:
                    break
            pick = _pick(cands, med, crop.shape[0] * crop.shape[1])
            if pick is not None:
                full = np.zeros(masks[idx].shape, dtype=bool)
                full[y0:y1, x0:x1] = pick["mask"]
                chosen = (full, pick)
                used_half = try_half
                break
        if chosen is None:
            print(f"  {mode} ROI {roi:02d}: no centered disk, keep n={int(masks[idx].sum())}")
            continue
        full, pick = chosen
        old_n = int(masks[idx].sum())
        masks[idx] = full
        scores[idx] = pick["score"]
        labels[idx] = pick["prompt"]
        changed = True
        print(
            f"  {mode} ROI {roi:02d}: {old_n} -> {int(full.sum())} "
            f"{pick['prompt']!r} score={pick['score']:.3f} fill={pick['fill']:.2f} half={used_half}"
        )
    if changed:
        r.rewrite_segmentation_from_masks(
            seg,
            rgb,
            [m.astype(np.uint8) for m in masks],
            scores,
            labels,
            f"{r.STEM}_rgb.png",
            "grape-surface-retune",
            cube,
            wl,
        )
        print(f"  wrote {mode}/swir3")
    return changed


def main() -> int:
    print("GSAM", r.require_gsam_ready(r.GSAM_URL))
    print("--- copy TX SWIR 5,15 -> reflectance ---")
    _copy_rois("transmittance", "reflectance", [5, 15])
    print("--- copy refl SWIR 19 -> transmittance ---")
    _copy_rois("reflectance", "transmittance", [19])

    _, _, _, _, _, refl_masks, _, _ = r._load_stream("reflectance", "swir3")
    seed25 = refl_masks[24]
    changed = False
    print("--- GSAM retune reflectance SWIR 25 ---")
    changed = retune_rois("reflectance", [25], REFL_PROMPTS, seed25) or changed
    print("--- GSAM retune transmittance SWIR 25 ---")
    changed = retune_rois("transmittance", [25], TX_PROMPTS, seed25) or changed
    try:
        sheets = r.write_session_qa_sheets(r.SESSION)
        print("QA", {k: str(v) for k, v in sheets.items()})
    except OSError as exc:
        print(f"QA sheets skipped: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
