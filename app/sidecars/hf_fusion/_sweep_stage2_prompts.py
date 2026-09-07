# Sweep stage-2 prompts on one session stream; keep FFC; pick best by QA score.
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from _eval_roi_background import evaluate_stream
from _reprocess_unripe_t1 import resegment_stream
from src.gsam_client import DEFAULT_GSAM_URL, require_gsam_ready

SESSION = Path(r"E:\2026_Grape_Data_Collection\2026_Geneva_Concord\Unripe_T2")
PLAN_PATH = Path(r"D:\Pototypy\HyperFusion\app\preset\gsam_plans\unripe_5by5_grape_tray.json")
STEM = "Unripe_T2"

# TX streams reuse reflectance masks — sweep reflectance only unless reuse is off.
CANDIDATES: dict[str, list[str]] = {
    "reflectance/fx10e": [
        "green grape",
        "grape",
        "green ball",
        "unripe grape",
        "green berry",
        "round green fruit",
        "green fruit",
    ],
    "reflectance/swir3": [
        "green ball",
        "green sphere",
        "ball",
        "grape",
        "green grape",
        "sphere",
        "round fruit",
    ],
}


def quality(report: dict) -> float:
    """Higher is better: fewer background ROIs, less incomplete (tiny) masks."""
    rows = report.get("rows") or []
    if not rows:
        return -1e9
    mean_bg = float(report["mean_bg"])
    bad = int(report["bad_count"])
    tiny = sum(1 for r in rows if r["fill"] < 0.35 or r["n"] < 800)
    huge = sum(1 for r in rows if r["fill"] > 0.88)
    # Prefer solid berry fill ~0.45-0.75
    mean_fill = sum(r["fill"] for r in rows) / len(rows)
    fill_pen = abs(mean_fill - 0.62)
    return -3.0 * mean_bg - 0.15 * bad - 0.35 * tiny - 0.25 * huge - 0.8 * fill_pen


def backup_seg(seg: Path, dest: Path) -> None:
    if dest.exists():
        shutil.rmtree(dest)
    if seg.exists():
        shutil.copytree(seg, dest)


def restore_seg(src: Path, seg: Path) -> None:
    if seg.exists():
        shutil.rmtree(seg)
    if src.exists():
        shutil.copytree(src, seg)


def main() -> int:
    plan = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
    print("GSAM", require_gsam_ready(DEFAULT_GSAM_URL))
    results = {}
    for stream, prompts in CANDIDATES.items():
        mode, camera = stream.split("/", 1)
        seg = SESSION / mode / camera / "preprocessed" / "segmentation"
        bak = SESSION / mode / camera / "preprocessed" / "_seg_backup_sweep"
        backup_seg(seg, bak)
        baseline = evaluate_stream(SESSION, mode, camera, STEM)
        best = {
            "prompt": plan["streams"][stream]["two_stage"].get("stage2_prompt"),
            "quality": quality(baseline),
            "report": {
                "mean_bg": baseline["mean_bg"],
                "bad_count": baseline["bad_count"],
                "bad_rois": baseline["bad_rois"],
            },
            "kept_baseline": True,
        }
        print(f"\n== {stream} baseline q={best['quality']:.3f} {best['report']}")
        for prompt in prompts:
            print(f"  try stage2='{prompt}'")
            resegment_stream(
                mode,
                camera,
                plan,
                prompt_override=prompt,
                force_two_stage=True,
                stage2_prompt=prompt,
            )
            rep = evaluate_stream(SESSION, mode, camera, STEM)
            q = quality(rep)
            summary = {
                "mean_bg": rep["mean_bg"],
                "bad_count": rep["bad_count"],
                "bad_rois": rep["bad_rois"],
                "tiny": sum(1 for r in rep["rows"] if r["fill"] < 0.35),
            }
            print(f"    q={q:.3f} {summary}")
            if q > best["quality"]:
                best = {
                    "prompt": prompt,
                    "quality": q,
                    "report": summary,
                    "kept_baseline": False,
                }
                # snapshot best seg
                best_dir = SESSION / mode / camera / "preprocessed" / "_seg_best_sweep"
                backup_seg(seg, best_dir)
        # restore best
        best_dir = SESSION / mode / camera / "preprocessed" / "_seg_best_sweep"
        if best["kept_baseline"]:
            restore_seg(bak, seg)
        else:
            restore_seg(best_dir, seg)
        # cleanup
        for p in (bak, best_dir):
            if p.exists():
                shutil.rmtree(p, ignore_errors=True)
        results[stream] = best
        print(f"  BEST {stream}: {best['prompt']} q={best['quality']:.3f} {best['report']}")

    out = _ROOT / f"_{STEM.lower()}_prompt_sweep.json"
    out.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print("wrote", out)
    return 0


if __name__ == "__main__":
    # Allow importing helpers that use global SESSION in reprocess script.
    import _reprocess_unripe_t1 as rp

    rp.SESSION = SESSION
    rp.PLAN_PATH = PLAN_PATH
    rp.STEM = STEM
    raise SystemExit(main())
