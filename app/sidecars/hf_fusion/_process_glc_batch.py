# Offline GSAM + reflectance fuse for E:\sk\GLC_40_b, GLC_40_w, GLC_30_w, GLC_30_b.
# FX and SWIR are segmented independently (no mask copy). Uses GSAM on :8767.
print("boot", flush=True)
from pathlib import Path
import shutil
import subprocess
import sys

from src.gsam_client import require_gsam_ready
from src.gsam_plan_segment import segment_stream_from_plan

ROOT = Path("/mnt/e/sk")
CFG = Path("/mnt/d/Pototypy/HyperFusion/app/preset/hyperfusion.cfg")
FUSION = Path("/mnt/d/Pototypy/HyperFusion/app/sidecars/hf_fusion/fusion_cli.py")
PY = sys.executable
GSAM = "http://127.0.0.1:8767"
SESSIONS = ["GLC_40_b", "GLC_40_w", "GLC_30_w", "GLC_30_b"]

FX_TRIES = [
    ("cheese", 0.25, 0.32, 0.40),
    ("cheese", 0.20, 0.35, 0.40),
    ("white rectangle", 0.20, 0.35, 0.40),
]
SWIR_TRIES = [
    ("green rectangle", 0.08, 0.28, 0.35),
    ("cyan square", 0.08, 0.28, 0.35),
    ("green marble", 0.08, 0.28, 0.35),
    ("cheese", 0.12, 0.28, 0.35),
]


def try_stream(session: Path, stem: str, camera: str, tries: list[tuple]) -> None:
    last_err = None
    stream_key = f"reflectance/{camera}"
    for prompt, box_thr, area, nms in tries:
        plan = {
            "name": "cheese 3 slices",
            "version": 1,
            "sample_count": 3,
            "streams": {
                stream_key: {
                    "prompt": prompt,
                    "box_threshold": box_thr,
                    "max_box_area_frac": area,
                    "nms_iou": nms,
                }
            },
        }
        print(f"  try {camera} prompt={prompt!r} box={box_thr} area={area} nms={nms}", flush=True)
        try:
            segment_stream_from_plan(
                session,
                "reflectance",
                camera,
                plan,
                stem=stem,
                gsam_url=GSAM,
                prompt_override=prompt,
                max_dets=20,
                nms_iou=nms,
            )
            last_err = None
            break
        except RuntimeError as exc:
            last_err = exc
            print(f"  failed: {exc}", flush=True)
    if last_err is not None:
        raise last_err


def process_session(stem: str) -> None:
    session = ROOT / stem
    print(f"==== {stem} ====", flush=True)
    try_stream(session, stem, "fx10e", FX_TRIES)
    try_stream(session, stem, "swir3", SWIR_TRIES)
    fusion_dir = session / "reflectance" / "fusion"
    if fusion_dir.is_dir():
        shutil.rmtree(fusion_dir)
        print("  cleared old fusion", flush=True)
    subprocess.run(
        [PY, "-u", str(FUSION), "--session", str(session), "--mode", "reflectance", "--cfg", str(CFG)],
        check=True,
    )
    print(f"DONE {stem}", flush=True)


def main() -> int:
    print("GSAM", require_gsam_ready(GSAM), flush=True)
    for stem in SESSIONS:
        process_session(stem)
    print("DONE all GLC batch", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
