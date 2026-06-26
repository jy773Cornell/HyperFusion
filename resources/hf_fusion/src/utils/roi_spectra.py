# Fused ROI mean reflectance CSV + mean±std spectrum plots (backend/offline).
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.utils.dataset import load_segmentation, resolve_rgb_path
from src.utils.roi import (
    combined_roi_spectra_csv_name,
    combined_roi_spectra_plot_name,
    roi_spectra_csv_name,
    roi_spectra_plot_name,
)


@dataclass(frozen=True)
class RoiSpectraRow:
    image: str
    label: str
    roi: int
    pixel_num: int
    wavelengths_nm: list[float]
    mean: np.ndarray
    std: np.ndarray


def fx_roi_label(session: Path, mode: str, roi: int) -> str:
    seg = load_segmentation(session, mode, "fx10e")
    for det in seg.get("detections", []):
        if int(det["roi"]) == roi:
            return str(det.get("label", ""))
    return ""


def fusion_spectra_image_name(session: Path, mode: str) -> str:
    return resolve_rgb_path(session, mode, "fx10e").name


def masked_mean_std_spectrum(cube: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    """Mean and population std per band inside a binary mask (cube H×W×B)."""
    region = mask > 0
    pixel_count = int(np.count_nonzero(region))
    if pixel_count == 0:
        raise ValueError("ROI mask is empty — cannot compute spectrum")

    values = cube[region].astype(np.float64, copy=False)
    mean = values.mean(axis=0)
    std = values.std(axis=0, ddof=0)
    return mean, std, pixel_count


def write_roi_spectra_csv(path: Path, rows: list[RoiSpectraRow]) -> None:
    if not rows:
        raise ValueError("No ROI spectrum rows to write")

    wavelengths = rows[0].wavelengths_nm
    for row in rows[1:]:
        if len(row.wavelengths_nm) != len(wavelengths):
            raise ValueError("All ROI spectrum rows must share the same wavelength grid")

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        header = ["image", "label", "roi#", "pixel_num", *wavelengths]
        writer.writerow(header)
        for row in rows:
            writer.writerow(
                [
                    row.image,
                    row.label,
                    row.roi,
                    row.pixel_num,
                    *[float(value) for value in row.mean],
                ]
            )


def _plot_mean_std_series(
    ax,
    wl: np.ndarray,
    mean: np.ndarray,
    std: np.ndarray,
    *,
    color: str,
    label: str,
    zorder_fill: int,
    zorder_line: int,
) -> None:
    mean_arr = np.asarray(mean, dtype=np.float64)
    std_arr = np.asarray(std, dtype=np.float64)
    ax.fill_between(
        wl,
        mean_arr - std_arr,
        mean_arr + std_arr,
        color=color,
        alpha=0.25,
        linewidth=0,
        zorder=zorder_fill,
    )
    ax.plot(wl, mean_arr, color=color, linewidth=1.8, label=label, zorder=zorder_line)


def _reflectance_ylim(rows: list[RoiSpectraRow]) -> tuple[float, float]:
    upper = 1.0
    for row in rows:
        mean_arr = np.asarray(row.mean, dtype=np.float64)
        std_arr = np.asarray(row.std, dtype=np.float64)
        upper = max(upper, float(np.max(mean_arr + std_arr)))
    return 0.0, min(1.05, upper * 1.02)


def save_roi_spectra_plot_png(
    path: Path,
    *,
    wavelengths_nm: list[float],
    mean: np.ndarray,
    std: np.ndarray,
    title: str,
    series_label: str,
    y_axis_label: str = "Reflectance",
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    wl = np.asarray(wavelengths_nm, dtype=np.float64)

    fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=100)
    _plot_mean_std_series(
        ax,
        wl,
        mean,
        std,
        color="#1f77b4",
        label=series_label,
        zorder_fill=1,
        zorder_line=2,
    )
    ax.set_title(title)
    ax.set_xlabel("Wavelength (nm)")
    ax.set_ylabel(y_axis_label)
    ax.set_xlim(float(wl.min()), float(wl.max()))
    ax.set_ylim(0.0, 1.0)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


_SERIES_COLORS = (
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
)


def save_all_roi_spectra_plot_png(
    path: Path,
    rows: list[RoiSpectraRow],
    *,
    title: str,
    y_axis_label: str = "Reflectance",
) -> None:
    """Multi-ROI mean ± 1σ plot (same style as GSAM roi_spectra_plot.png)."""
    if not rows:
        raise ValueError("No ROI spectrum rows to plot")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    wavelengths = rows[0].wavelengths_nm
    for row in rows[1:]:
        if row.wavelengths_nm != wavelengths:
            raise ValueError("All ROI spectrum rows must share the same wavelength grid")

    wl = np.asarray(wavelengths, dtype=np.float64)
    fig, ax = plt.subplots(figsize=(9.6, 5.4), dpi=100)

    for index, row in enumerate(rows):
        color = _SERIES_COLORS[index % len(_SERIES_COLORS)]
        _plot_mean_std_series(
            ax,
            wl,
            row.mean,
            row.std,
            color=color,
            label=f"ROI {row.roi}: {row.label}" if row.label else f"ROI {row.roi}",
            zorder_fill=index + 1,
            zorder_line=len(rows) + index + 1,
        )

    y_min, y_max = _reflectance_ylim(rows)
    ax.set_title(f"{title} (mean ± 1σ)")
    ax.set_xlabel("Wavelength (nm)")
    ax.set_ylabel(y_axis_label)
    ax.set_xlim(float(wl.min()), float(wl.max()))
    ax.set_ylim(y_min, y_max)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def export_fusion_roi_spectra(
    *,
    session: Path,
    mode: str,
    roi_dir: Path,
    fx_roi: int,
    fused_cube: np.ndarray,
    wavelengths_nm: list[float],
    mask: np.ndarray,
) -> RoiSpectraRow:
    """Write per-ROI roi_spectra.csv + roi_spectra_plot.png; return row for session combine."""
    mean, std, pixel_count = masked_mean_std_spectrum(fused_cube, mask)
    row = RoiSpectraRow(
        image=fusion_spectra_image_name(session, mode),
        label=fx_roi_label(session, mode, fx_roi),
        roi=fx_roi,
        pixel_num=pixel_count,
        wavelengths_nm=list(wavelengths_nm),
        mean=mean,
        std=std,
    )

    csv_path = roi_dir / roi_spectra_csv_name()
    plot_path = roi_dir / roi_spectra_plot_name()
    write_roi_spectra_csv(csv_path, [row])
    save_roi_spectra_plot_png(
        plot_path,
        wavelengths_nm=row.wavelengths_nm,
        mean=row.mean,
        std=row.std,
        title=f"ROI {fx_roi} fused FX10e+SWIR3 spectrum",
        series_label=f"ROI {fx_roi}: {row.label}" if row.label else f"ROI {fx_roi}",
    )
    return row


def write_combined_roi_spectra_artifacts(
    fusion_out: Path, mode: str, rows: list[RoiSpectraRow]
) -> tuple[Path, Path]:
    """Write fusion/roi_spectra_fx10e_swir_{mode}.csv and matching multi-ROI plot."""
    if not rows:
        raise ValueError("No ROI spectrum rows to combine")

    csv_path = fusion_out / combined_roi_spectra_csv_name(mode)
    plot_path = fusion_out / combined_roi_spectra_plot_name(mode)
    write_roi_spectra_csv(csv_path, rows)
    save_all_roi_spectra_plot_png(
        plot_path,
        rows,
        title=f"Fused FX10e+SWIR3 ROI spectra ({mode})",
    )
    return csv_path, plot_path
