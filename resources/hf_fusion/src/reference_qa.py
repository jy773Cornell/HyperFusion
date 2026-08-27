# Copy DARKREF/WHITEREF into preprocessed and write intensity QA plots (backend/offline).
# Per-stream plots stay in preprocessed/; the session collage is {session}/preview/.
from __future__ import annotations

import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from src.gsam_overlay_sheet import PREFERRED_STREAMS, write_labeled_image_sheet

# Avoid circular import at module load; overlay_sheet may import this lazily.
from src.utils.envi import parse_envi_hdr, read_bil_cube

REF_SHEET_NAME = "reference_intensity.png"


def _copy_pair(src_hdr: Path, dest_dir: Path) -> tuple[Path, Path]:
    dest_hdr = dest_dir / src_hdr.name
    dest_raw = dest_dir / src_hdr.with_suffix(".raw").name
    shutil.copy2(src_hdr, dest_hdr)
    shutil.copy2(src_hdr.with_suffix(".raw"), dest_raw)
    return dest_hdr, dest_raw


def copy_capture_refs_to_preprocessed(session: Path, mode: str, camera: str) -> dict[str, Path]:
    capture = session / mode / camera / "capture"
    pre = session / mode / camera / "preprocessed"
    if not capture.is_dir() or not pre.is_dir():
        raise FileNotFoundError(f"missing capture or preprocessed: {mode}/{camera}")
    written: dict[str, Path] = {}
    dark = next(capture.glob("DARKREF_*.hdr"))
    white = next(capture.glob("WHITEREF_*.hdr"))
    written["dark_hdr"], written["dark_raw"] = _copy_pair(dark, pre)
    written["white_hdr"], written["white_raw"] = _copy_pair(white, pre)
    return written


def _band_mean_std(cube: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pixels = cube.reshape(-1, cube.shape[-1]).astype(np.float64, copy=False)
    mean = pixels.mean(axis=0)
    std = pixels.std(axis=0)
    return mean, std


def _axis_max(camera: str, mean: np.ndarray, std: np.ndarray) -> float:
    peak = float(np.nanmax(mean + std)) if mean.size else 1.0
    if camera == "swir3":
        return max(65535.0, peak)
    return 4096.0 if peak <= 4096.0 else max(65535.0, peak)


def save_ref_plot_png(
    wavelengths_nm: np.ndarray,
    mean_dn: np.ndarray,
    std_dn: np.ndarray,
    title: str,
    output_path: Path,
    y_max: float,
) -> Path:
    fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=100)
    wl = wavelengths_nm
    ax.fill_between(wl, mean_dn - std_dn, mean_dn + std_dn, color=(0.47, 0.67, 0.90, 0.35), linewidth=0)
    ax.plot(wl, mean_dn, color=(0.12, 0.35, 0.71), linewidth=2.0)
    ax.set_xlim(float(wl[0]), float(wl[-1]))
    ax.set_ylim(0.0, max(1.0, y_max))
    ax.set_title(title, fontweight="bold")
    ax.set_xlabel("Wavelength (nm)")
    ylabel = "Intensity (DN, 0–65535, 16-bit)" if y_max > 4096.0 else "Intensity (DN, 0–4096)"
    ax.set_ylabel(ylabel)
    ax.grid(True, color="#dcdcdc")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=100)
    plt.close(fig)
    return output_path


def write_stream_reference_plots(session: Path, mode: str, camera: str, stem: str) -> dict[str, Path]:
    pre = session / mode / camera / "preprocessed"
    dark_hdr = next(pre.glob("DARKREF_*.hdr"))
    white_hdr = next(pre.glob("WHITEREF_*.hdr"))
    dark_meta = parse_envi_hdr(dark_hdr)
    white_meta = parse_envi_hdr(white_hdr)
    dark = read_bil_cube(dark_meta)
    white = read_bil_cube(white_meta)
    wl_d = np.array(dark_meta.wavelengths_nm, dtype=np.float64)
    wl_w = np.array(white_meta.wavelengths_nm, dtype=np.float64)
    d_mean, d_std = _band_mean_std(dark)
    w_mean, w_std = _band_mean_std(white)
    dark_plot = pre / f"DARKREF_{stem}_ref_plot.png"
    white_plot = pre / f"WHITEREF_{stem}_ref_plot.png"
    save_ref_plot_png(
        wl_d,
        d_mean,
        d_std,
        "Dark Reference Mean ±1σ",
        dark_plot,
        _axis_max(camera, d_mean, d_std),
    )
    save_ref_plot_png(
        wl_w,
        w_mean,
        w_std,
        "White Reference Mean ±1σ",
        white_plot,
        _axis_max(camera, w_mean, w_std),
    )
    return {"dark_plot": dark_plot, "white_plot": white_plot}


def collect_reference_plots(session: Path, stem: str) -> list[tuple[str, Path]]:
    panels: list[tuple[str, Path]] = []
    for mode, camera in PREFERRED_STREAMS:
        pre = session / mode / camera / "preprocessed"
        dark = pre / f"DARKREF_{stem}_ref_plot.png"
        white = pre / f"WHITEREF_{stem}_ref_plot.png"
        if not dark.is_file():
            matches = sorted(pre.glob("DARKREF_*_ref_plot.png"))
            dark = matches[0] if matches else dark
        if not white.is_file():
            matches = sorted(pre.glob("WHITEREF_*_ref_plot.png"))
            white = matches[0] if matches else white
        if dark.is_file():
            panels.append((f"{mode}/{camera}  black", dark))
        if white.is_file():
            panels.append((f"{mode}/{camera}  white", white))
    return panels


def write_session_reference_intensity(session: Path, stem: str | None = None) -> Path | None:
    stem = stem or session.name
    return write_labeled_image_sheet(
        session,
        collect_reference_plots(session, stem),
        out_name=REF_SHEET_NAME,
        title="black / white reference intensity",
        cols=2,
    )


from src.gsam_overlay_sheet import write_collection_image_sheet


def write_collection_reference_sheet(collection: Path, session_sheets: list[Path]) -> Path | None:
    return write_collection_image_sheet(
        collection,
        session_sheets,
        out_name=REF_SHEET_NAME,
        title="reference intensity QA",
    )


def process_session_references(session: Path, stem: str | None = None) -> dict[str, Path]:
    stem = stem or session.name
    written: dict[str, Path] = {}
    for mode, camera in PREFERRED_STREAMS:
        pre = session / mode / camera / "preprocessed"
        capture = session / mode / camera / "capture"
        if not pre.is_dir() or not capture.is_dir():
            continue
        copy_capture_refs_to_preprocessed(session, mode, camera)
        plots = write_stream_reference_plots(session, mode, camera, stem)
        written[f"{mode}/{camera}/dark"] = plots["dark_plot"]
        written[f"{mode}/{camera}/white"] = plots["white_plot"]
    sheet = write_session_reference_intensity(session, stem)
    if sheet is not None:
        written["session_sheet"] = sheet
    return written
