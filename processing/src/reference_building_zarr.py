"""Reference building utilities for white and dark references.

Author: Jinhong Yu
Institution: Cornell University, CALS
Lab: Postharvest Technology Lab & CAIR Lab
Email: jy773@cornell.edu
"""

import numpy as np
import zarr
import dask.array as da
from skimage import color, filters, morphology, measure
from typing import Optional, Literal, Dict, Any, Tuple

from .hsi2color_zarr import hsi_to_color_zarr
from .envi_zarr_conversion import _get_rcb_permutation, _fix_bands_axis


def _extract_wavelengths(attrs: dict):
    """Read wavelengths from attrs with tolerant key matching."""
    for key in ("wavelength", "Wavelength"):
        if key in attrs and attrs[key] is not None:
            return attrs[key]
    return None


def _write_rowref_attrs(out_rowref_zarr: str, *, interleave: str, wavelengths) -> None:
    """Attach interleave + wavelength metadata to a row-reference Zarr."""
    if wavelengths is None:
        raise ValueError(
            f"Cannot write row-reference without wavelengths: {out_rowref_zarr}"
        )
    out = zarr.open(out_rowref_zarr, mode="r+")
    arr = out if isinstance(out, zarr.Array) else out[out.array_keys()[0]]
    arr.attrs["interleave"] = interleave
    w = np.asarray(wavelengths, dtype=float).ravel()
    if w.size == 0:
        raise ValueError(f"Empty wavelength list for row-reference: {out_rowref_zarr}")
    arr.attrs["wavelength"] = [float(x) for x in w]


def build_white_reference_zarr(
    white_zarr_path: str,
    illuminants_path: str,
    *,
    interleave: Optional[Literal["bip", "bil", "bsq"]] = None,
    normalize: Literal["none", "global"] = "global",
    otsu_sigma: float = 1.0,  # light blur before Otsu (set 0 to skip)
    min_area_frac: float = 0.005,  # min puck area fraction to keep
    morph_close: int = 5,  # closing radius (pixels)
    morph_open: int = 3,  # opening radius (pixels)
    fill_edge_window: int = 200,  # mean of N valid edge columns
    out_rowref_zarr: Optional[str] = None,  # write one-row reference here if set
    overwrite: bool = True,
) -> Dict[str, Any]:
    """Build white reference with automatic puck detection using Otsu segmentation."""
    # Open white reference data
    root = zarr.open(white_zarr_path, mode="r")
    arr = root if isinstance(root, zarr.Array) else root[list(root.array_keys())[0]]
    if arr.ndim != 3:
        raise ValueError(f"Expected 3D array; got {arr.shape}")
    attrs = arr.attrs.asdict() if hasattr(arr.attrs, "asdict") else dict(arr.attrs)

    iv = interleave or attrs.get("interleave", None)
    if iv is None:
        raise ValueError(
            "Interleave missing; pass interleave= or set arr.attrs['interleave']."
        )
    iv = str(iv).strip().lower()

    wavelengths = _extract_wavelengths(attrs)
    if wavelengths is None:
        raise ValueError(
            "Missing wavelengths in white zarr attrs['wavelength'] (nm). "
            "Ensure convert_envi_to_zarr copied metadata from the ENVI hdr."
        )
    w = np.asarray(wavelengths, dtype=np.float64).ravel()

    rgb = hsi_to_color_zarr(
        zarr_path=white_zarr_path,
        illuminants_path=illuminants_path,
        illuminant_d=65,
        normalize=normalize,
        compute=True,
    )

    rows, cols = rgb.shape[:2]

    # Segment white puck using Otsu thresholding
    gray = color.rgb2gray(rgb)
    if otsu_sigma and otsu_sigma > 0:
        from skimage.filters import gaussian

        gimg = gaussian(gray, sigma=otsu_sigma)
    else:
        gimg = gray

    thr = filters.threshold_otsu(gimg)
    mask = gimg > thr

    # Clean up mask
    area_thresh = int(min_area_frac * rows * cols)
    mask = morphology.remove_small_objects(mask, min_size=area_thresh)
    if morph_close > 0:
        mask = morphology.binary_closing(mask, morphology.disk(morph_close))
    if morph_open > 0:
        mask = morphology.binary_opening(mask, morphology.disk(morph_open))

    # Keep largest connected component
    lbl = measure.label(mask)
    if lbl.max() > 0:
        areas = np.bincount(lbl.ravel())
        areas[0] = 0
        keep = areas.argmax()
        mask = lbl == keep
    mask = morphology.remove_small_holes(mask, area_threshold=area_thresh)

    # Compute row-mean spectra within mask
    dA = da.from_zarr(arr)
    perm0 = _get_rcb_permutation(iv)
    cube = da.transpose(dA, perm0)
    perm_fix = _fix_bands_axis(arr.shape, w.size, perm0)
    if perm_fix != (0, 1, 2):
        cube = da.transpose(cube, perm_fix)

    mask_da = da.from_array(mask, chunks=cube.chunks[:2])  # (R,C)
    masked = da.where(mask_da[..., None], cube, 0.0)  # zero outside puck
    sum_rc = masked.sum(axis=0)  # (C,B): sum across rows
    cnt_c = mask_da.sum(axis=0).astype(np.float32)  # (C,): count per column

    mean_cb = sum_rc / cnt_c[:, None]  # (C,B), NaNs where cnt=0
    cnt_np = cnt_c.compute()
    mean_cb_np = mean_cb.compute().astype(np.float32)  # (cols, bands)
    valid_cols = cnt_np > 0
    if not valid_cols.any():
        raise RuntimeError("Mask found no valid white-reference pixels.")

    # Fill missing columns from edge means
    idx = np.flatnonzero(valid_cols)
    left_end = idx[:fill_edge_window] if idx.size >= fill_edge_window else idx
    right_end = idx[-fill_edge_window:] if idx.size >= fill_edge_window else idx
    left_mean = mean_cb_np[left_end].mean(axis=0, dtype=np.float32)
    right_mean = mean_cb_np[right_end].mean(axis=0, dtype=np.float32)

    first, last = idx[0], idx[-1]
    if first > 0:
        mean_cb_np[:first, :] = left_mean[None, :]
    if last < cols - 1:
        mean_cb_np[last + 1 :, :] = right_mean[None, :]

    prev = mean_cb_np[first].copy()
    for c in range(first, last + 1):
        if not valid_cols[c]:
            mean_cb_np[c] = prev
        else:
            prev = mean_cb_np[c]

    # Pack 1-row reference in original interleave
    if iv == "bil":
        row_ref = mean_cb_np.T[None, ...]
    elif iv == "bip":
        row_ref = mean_cb_np[None, ...]
    else:  # "bsq"
        row_ref = mean_cb_np.T[:, None, :]
    row_ref = row_ref.astype(np.float32, copy=False)

    # Write reference to Zarr if requested
    if out_rowref_zarr:
        da.from_array(row_ref, chunks=tuple(max(1, s) for s in row_ref.shape)).to_zarr(
            out_rowref_zarr, overwrite=overwrite
        )
        _write_rowref_attrs(out_rowref_zarr, interleave=iv, wavelengths=w)

    return {
        "row_ref": row_ref,
        "mask": mask,
        "rgb": rgb,
        "out_rowref_zarr": out_rowref_zarr,
    }


def build_black_reference_zarr(
    black_zarr_path: str,
    out_rowref_zarr: str,
    *,
    interleave: Optional[Literal["bip", "bil", "bsq"]] = None,
    reducer: Literal["mean", "median"] = "mean",
    dtype_out: np.dtype = np.float32,
    overwrite: bool = True,
) -> Dict[str, Any]:
    """Build 1-row black reference for flat field correction by reducing across rows."""
    # Open black reference data
    root = zarr.open(black_zarr_path, mode="r")
    arr = root if isinstance(root, zarr.Array) else root[list(root.array_keys())[0]]
    if arr.ndim != 3:
        raise ValueError(f"Expected 3D array; got {arr.shape}")

    attrs = arr.attrs.asdict() if hasattr(arr.attrs, "asdict") else dict(arr.attrs)
    iv = interleave or attrs.get("interleave")
    if iv is None:
        raise ValueError(
            "Interleave not found; pass interleave= or set arr.attrs['interleave']."
        )
    iv = str(iv).strip().lower()

    wavelengths = _extract_wavelengths(attrs)
    if wavelengths is None:
        raise ValueError(
            "Missing wavelengths in black zarr attrs['wavelength'] (nm). "
            "Ensure convert_envi_to_zarr copied metadata from the ENVI hdr."
        )

    # Reorder to (rows, cols, bands)
    dA = da.from_zarr(arr)
    perm0 = _get_rcb_permutation(iv)
    cube = da.transpose(dA, perm0)

    if wavelengths is not None:
        w = np.asarray(wavelengths, dtype=float).ravel()
        perm_fix = _fix_bands_axis(arr.shape, w.size, perm0)
    else:
        perm_fix = (0, 1, 2)

    if perm_fix != (0, 1, 2):
        cube = da.transpose(cube, perm_fix)

    # Reduce across rows
    if reducer == "mean":
        cb = cube.astype(dtype_out, copy=False).mean(axis=0)
    else:  # "median"
        cb = da.median(cube.astype(dtype_out, copy=False), axis=0)

    # Expand to 1-row slab in original interleave
    if iv == "bil":
        row_ref_da = cb.T[None, ...]
    elif iv == "bip":
        row_ref_da = cb[None, ...]
    else:  # "bsq"
        row_ref_da = cb.T[:, None, :]

    row_ref_da = row_ref_da.astype(dtype_out)

    # Write to Zarr
    row_ref_da.to_zarr(out_rowref_zarr, overwrite=overwrite)

    _write_rowref_attrs(out_rowref_zarr, interleave=iv, wavelengths=wavelengths)

    out_shape = row_ref_da.shape
    out_chunks = getattr(row_ref_da, "chunks", None)
