# One-off Unripe_T1 re-segment. Does not modify the GSAM2 sidecar.
# GroundingDINO boxes + raw SAM2 box fill, RTL ROI order via run_segment.
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from _sam2_bootstrap import ensure_sam2_package

ensure_sam2_package()

from gsam2_server import run_segment  # noqa: E402

SESSION = Path("/mnt/e/2026_Grape_Data_Collection/2026_Geneva_Concord/Unripe_T1")
STREAMS = [
    ("reflectance", "fx10e", "green grape", 0.30),
    ("reflectance", "swir3", "clear round ball", 0.08),
    ("transmittance", "fx10e", "dark round ball", 0.08),
    ("transmittance", "swir3", "dark round ball", 0.08),
]


def install_dino_threshold_patch(segmenter, state: dict) -> None:
    proc = segmenter._hf_processor
    if getattr(proc, "_hf_unripe_thr_patched", False):
        return
    orig = proc.post_process_grounded_object_detection

    def _post(*args, **kwargs):
        kwargs["threshold"] = float(state["threshold"])
        return orig(*args, **kwargs)

    proc.post_process_grounded_object_detection = _post
    proc._hf_unripe_thr_patched = True


def main() -> int:
    args = SimpleNamespace(
        hf_model_id="IDEA-Research/grounding-dino-base",
        detector_device="cuda",
        sam2_config="configs/sam2.1/sam2.1_hiera_l.yaml",
        sam2_checkpoint=str(ROOT / "checkpoints/sam2.1_hiera_large.pt"),
        sam2_device="cuda",
        box_threshold=0.30,
        max_dets_default=25,
        multimask_output=False,
    )
    from gsam2_server import get_segmenter

    segmenter = get_segmenter(args)
    dino_thr = {"threshold": 0.30}
    install_dino_threshold_patch(segmenter, dino_thr)

    for mode, cam, prompt, thr in STREAMS:
        pre = SESSION / mode / cam / "preprocessed"
        out = pre / "segmentation"
        if out.exists():
            shutil.rmtree(out)
        dino_thr["threshold"] = thr
        result = run_segment(
            {
                "input_rgb": str(pre / "Unripe_T1_rgb.png"),
                "out_dir": str(out),
                "prompt": prompt,
                "max_dets": 25,
                "box_threshold": thr,
                "image_name": "Unripe_T1_rgb.png",
            },
            args,
        )
        n = int(result.get("detection_count", 0))
        scores = [float(d["score"]) for d in result.get("detections", [])]
        mean_sc = (sum(scores) / len(scores)) if scores else 0.0
        print(
            f"{mode}/{cam}: prompt={prompt!r} thr={thr:.2f} n={n} mean_score={mean_sc:.3f}",
            flush=True,
        )
        manifest = pre / "processing_manifest.json"
        if manifest.is_file():
            data = json.loads(manifest.read_text(encoding="utf-8"))
            data["gsamPrompt"] = prompt
            data["gsamSampleCount"] = n
            manifest.write_text(json.dumps(data, indent=4), encoding="utf-8")
    print("RAW_SAM2_DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
