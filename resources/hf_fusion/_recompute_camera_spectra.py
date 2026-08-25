# Recompute per-camera ROI mean spectra from FFC cubes + current masks.
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from src.utils.envi import parse_envi_hdr, read_bil_cube
from src.utils.roi_spectra import (
    RoiSpectraRow,
    save_all_roi_spectra_plot_png,
    spectrum_y_axis_label,
    write_roi_spectra_csv,
)

SESSION = Path(r"E:\2026_Grape_Data_Collection\2026_Geneva_Concord\Unripe_T1")
STREAMS = [
    ("reflectance", "fx10e"),
    ("reflectance", "swir3"),
    ("transmittance", "fx10e"),
    ("transmittance", "swir3"),
]


def extract_stream(mode: str, camera: str) -> None:
    pre = SESSION / mode / camera / "preprocessed"
    seg = pre / "segmentation"
    meta = parse_envi_hdr(pre / "Unripe_T1_ffc.hdr")
    cube = read_bil_cube(meta)
    data = json.loads((seg / "segmentation_results.json").read_text(encoding="utf-8"))
    rows: list[RoiSpectraRow] = []
    for det in data["detections"]:
        roi = int(det["roi"])
        mask = np.load(seg / "masks" / f"mask_{roi:03d}.npy") > 0
        if mask.shape[:2] != cube.shape[:2]:
            raise ValueError(f"{mode}/{camera} mask {mask.shape} != cube {cube.shape}")
        region = cube[mask]
        rows.append(
            RoiSpectraRow(
                image="Unripe_T1_rgb.png",
                label=str(det.get("label", "")),
                roi=roi,
                pixel_num=int(region.shape[0]),
                wavelengths_nm=list(meta.wavelengths_nm),
                mean=region.mean(axis=0),
                std=region.std(axis=0, ddof=0),
            )
        )
    write_roi_spectra_csv(seg / "roi_spectra.csv", rows)
    save_all_roi_spectra_plot_png(
        seg / "roi_spectra_plot.png",
        rows,
        title=f"Unripe_T1 {mode}/{camera}",
        y_axis_label=spectrum_y_axis_label(mode),
    )
    print(f"{mode}/{camera}: {len(rows)} ROIs", flush=True)


def main() -> None:
    for mode, camera in STREAMS:
        extract_stream(mode, camera)


if __name__ == "__main__":
    main()
