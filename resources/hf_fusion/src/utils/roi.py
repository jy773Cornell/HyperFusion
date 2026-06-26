# Per-ROI folder naming and artifact filenames for dual-camera fusion alignment.
from __future__ import annotations


def roi_prefix(fx_roi: int) -> str:
    """Shared prefix for all artifacts in an ROI folder, e.g. roi_003."""
    return f"roi_{fx_roi:03d}"


def roi_folder_name(fx_roi: int) -> str:
    """Folder name for a matched FX10e/SWIR3 object tile, e.g. roi_003_fx10e_swir3."""
    return f"{roi_prefix(fx_roi)}_fx10e_swir3"


def fused_cube_basename(fx_roi: int) -> str:
    return f"{roi_folder_name(fx_roi)}.raw"


def roi_rgb_overlay_name(fx_roi: int) -> str:
    return f"{roi_prefix(fx_roi)}_rgb_overlay.png"


def roi_mask_overlay_outline_name(fx_roi: int) -> str:
    return f"{roi_prefix(fx_roi)}_maskoverlay_outline.png"


def roi_mask_npy_name(fx_roi: int) -> str:
    return f"{roi_prefix(fx_roi)}_mask.npy"


def roi_mask_png_name(fx_roi: int) -> str:
    return f"{roi_prefix(fx_roi)}_mask.png"


def roi_spectra_csv_name() -> str:
    return "roi_spectra.csv"


def roi_spectra_plot_name() -> str:
    return "roi_spectra_plot.png"


def combined_roi_spectra_csv_name(mode: str) -> str:
    return f"roi_spectra_fx10e_swir_{mode}.csv"


def combined_roi_spectra_plot_name(mode: str) -> str:
    return f"roi_spectra_fx10e_swir_{mode}.png"
