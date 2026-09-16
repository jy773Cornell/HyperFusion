# Offline: Unripe vs Veraison fused mean±std (R and T). No hardware I/O.
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

ROOT = Path(r"D:\Data_JY\2026_Grape_Data_Collection\2026_Geneva_Concord")
OUT_PNG = ROOT / "unripe_veraison_rt_spectra.png"
OUT_SVG = ROOT / "unripe_veraison_rt_spectra.svg"

SKIP_COLS = {"image", "label", "roi#", "pixel_num", "tray", "Brix", "pH"}

SERIES = [
    ("Unripe", "reflectance", "Unripe-R", "#1f77b4", "-"),
    ("Unripe", "transmittance", "Unripe-T", "#d62728", "-"),
    ("Veraison", "reflectance", "Veraison-R", "#2ca02c", "-"),
    ("Veraison", "transmittance", "Veraison-T", "#ff7f0e", "-"),
]


def load_group(stage: str, mode: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    rows: list[np.ndarray] = []
    wavelengths: np.ndarray | None = None
    n_files = 0
    for csv_path in sorted(ROOT.glob(f"{stage}_T*/{mode}/fusion/roi_spectra_fx10e_swir_{mode}.csv")):
        n_files += 1
        with csv_path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            wl_cols = [c for c in (reader.fieldnames or []) if c not in SKIP_COLS]
            if wavelengths is None:
                wavelengths = np.asarray([float(c) for c in wl_cols], dtype=np.float64)
            for rec in reader:
                rows.append(np.asarray([float(rec[c]) for c in wl_cols], dtype=np.float64))
    if not rows or wavelengths is None:
        raise RuntimeError(f"No spectra for {stage} {mode}")
    stack = np.vstack(rows)
    mean = np.nanmean(stack, axis=0)
    std = np.nanstd(stack, axis=0, ddof=1)
    print(f"{stage} {mode}: trays={n_files} rois={stack.shape[0]} peak={float(mean.max()):.3f}")
    return wavelengths, mean, std, stack.shape[0]


def main() -> int:
    fig, ax = plt.subplots(figsize=(8.2, 5.6), dpi=160, facecolor="none")
    ax.set_facecolor("white")
    for stage, mode, label, color, ls in SERIES:
        wl, mean, std, _n = load_group(stage, mode)
        ax.fill_between(wl, mean - std, mean + std, color=color, alpha=0.22, linewidth=0, zorder=1)
        ax.plot(wl, mean, color=color, linewidth=2.2, linestyle=ls, label=label, zorder=2)

    ink = "white"
    label_fs = 18
    tick_fs = 15
    ax.set_xlabel("Wavelength (nm)", fontweight="bold", fontsize=label_fs, color=ink)
    ax.set_ylabel("Intensity", fontweight="bold", fontsize=label_fs, color=ink)
    ax.set_xlim(400, 2600)
    ax.set_ylim(0.0, 0.6)
    ax.xaxis.set_major_locator(MultipleLocator(400))
    ax.yaxis.set_major_locator(MultipleLocator(0.1))
    ax.tick_params(colors=ink, labelsize=tick_fs, width=1.4, length=6)
    for tick in ax.get_xticklabels() + ax.get_yticklabels():
        tick.set_fontweight("bold")
        tick.set_color(ink)
    for spine in ax.spines.values():
        spine.set_color(ink)
        spine.set_linewidth(1.4)
    ax.grid(True, linestyle=":", linewidth=0.9, color="#888888", alpha=0.85)
    leg = ax.legend(
        loc="upper right",
        frameon=True,
        fancybox=False,
        facecolor="#222222",
        edgecolor=ink,
        labelcolor=ink,
        fontsize=13,
        framealpha=0.95,
        ncol=1,
    )
    for text in leg.get_texts():
        text.set_fontweight("bold")
        text.set_color(ink)

    fig.tight_layout()
    fig.patch.set_alpha(0.0)
    ax.patch.set_facecolor("white")
    ax.patch.set_alpha(1.0)
    fig.savefig(OUT_PNG, bbox_inches="tight", facecolor=(0, 0, 0, 0), transparent=False)
    fig.savefig(OUT_SVG, bbox_inches="tight", facecolor=(0, 0, 0, 0), transparent=False)
    plt.close(fig)
    print(f"wrote {OUT_PNG}")
    print(f"wrote {OUT_SVG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
