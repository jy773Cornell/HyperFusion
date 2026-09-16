# Offline: plot fused FX10e+SWIR reflectance + transmittance mean±std as SVG/PNG.
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

from src.utils.envi import parse_envi_hdr, read_bil_cube
from src.utils.roi_spectra import masked_mean_std_spectrum


def main() -> None:
    session = Path(r"D:\Data_JY\2026_Grape_Data_Collection\Demo\concord_cluster2")
    out_svg = session / "preview" / "fused_reflectance_transmittance.svg"
    out_png = session / "preview" / "fused_reflectance_transmittance.png"

    series: list[tuple[np.ndarray, np.ndarray, np.ndarray, str, str, int]] = []
    for mode, color, label in (
        ("reflectance", "#1f77b4", "Reflectance"),
        ("transmittance", "#d62728", "Transmittance"),
    ):
        roi_dir = session / mode / "fusion" / "roi_001_fx10e_swir3"
        hdr = next(roi_dir.glob("*.hdr"))
        mask_path = roi_dir / "roi_001_mask.npy"
        meta = parse_envi_hdr(hdr)
        cube = read_bil_cube(meta)
        mask = np.load(mask_path)
        if mask.ndim == 3:
            mask = mask[..., 0] if mask.shape[-1] in (1, 3, 4) else mask[0]
        mean, std, n = masked_mean_std_spectrum(cube, mask)
        wl = np.asarray(meta.wavelengths_nm, dtype=np.float64)
        series.append((wl, mean, std, color, label, n))
        print(f"{label}: bands={len(wl)} pixels={n} mean_peak={float(mean.max()):.3f}")

    # Square canvas, transparent figure/axes background.
    fig, ax = plt.subplots(figsize=(8.0, 8.0), dpi=160, facecolor="none")
    ax.set_facecolor("none")
    for wl, mean, std, color, label, _n in series:
        ax.fill_between(
            wl, mean - std, mean + std, color=color, alpha=0.28, linewidth=0, zorder=1
        )
        ax.plot(wl, mean, color=color, linewidth=2.4, label=label, zorder=2)

    label_fs = 18
    tick_fs = 15
    ink = "white"
    ax.set_xlabel("Wavelength (nm)", fontweight="bold", fontsize=label_fs, color=ink)
    ax.set_ylabel("Intensity", fontweight="bold", fontsize=label_fs, color=ink)
    ax.set_xlim(400, 2600)
    ax.set_ylim(0.0, 0.6)
    ax.xaxis.set_major_locator(MultipleLocator(400))
    ax.yaxis.set_major_locator(MultipleLocator(0.1))
    ax.tick_params(colors=ink, labelsize=tick_fs, width=1.4, length=6)
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontweight("bold")
        label.set_color(ink)
    for spine in ax.spines.values():
        spine.set_color(ink)
        spine.set_linewidth(1.4)
    ax.grid(True, linestyle=":", linewidth=0.9, color="#dddddd", alpha=0.55)
    leg = ax.legend(
        loc="upper right",
        frameon=True,
        fancybox=False,
        facecolor="#222222",
        edgecolor=ink,
        labelcolor=ink,
        fontsize=14,
        framealpha=0.92,
    )
    for text in leg.get_texts():
        text.set_fontweight("bold")
        text.set_color(ink)

    fig.tight_layout()
    out_svg.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(
        out_svg,
        format="svg",
        bbox_inches="tight",
        facecolor="none",
        transparent=True,
    )
    fig.savefig(
        out_png,
        format="png",
        bbox_inches="tight",
        facecolor="none",
        transparent=True,
    )
    plt.close(fig)
    print(f"wrote {out_svg}")
    print(f"wrote {out_png}")


if __name__ == "__main__":
    main()
