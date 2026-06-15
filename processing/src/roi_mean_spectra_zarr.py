"""ROI statistics computation for hyperspectral Zarr arrays.

Author: Jinhong Yu
Institution: Cornell University, CALS
Lab: Postharvest Technology Lab & CAIR Lab
Email: jy773@cornell.edu
"""

import numpy as np
import zarr
import dask.array as da
from typing import Optional, Literal, Tuple
from .envi_zarr_conversion import _get_rcb_permutation


def _extract_wavelengths(attrs: dict):
    """Read wavelengths from attrs with tolerant key matching."""
    for key in ("wavelength", "Wavelength"):
        if key in attrs and attrs[key] is not None:
            return attrs[key]
    return None


def roi_mean_spectra_zarr(
    mask: np.ndarray,
    zarr_path: str,
    *,
    interleave: Optional[Literal["bil", "bip", "bsq"]] = None,
    rechunk_2d: Optional[Tuple[int, int]] = None,
    dtype_work=np.float32,
):
    """Compute mean and std spectra for masked ROI using efficient bbox cropping for performance.

    Returns:
        wavelengths: (B,) float64
        mean: (B,) float32
        std: (B,) float32
    """
    # Open Zarr data
    root = zarr.open(zarr_path, mode="r")
    arr = root if isinstance(root, zarr.Array) else root[list(root.array_keys())[0]]
    if arr.ndim != 3:
        raise ValueError(f"Expected 3D; got {arr.shape}")

    attrs = arr.attrs.asdict() if hasattr(arr.attrs, "asdict") else dict(arr.attrs)
    iv = interleave or attrs.get("interleave")
    if iv is None:
        raise ValueError(
            "Missing interleave (pass interleave= or set attrs['interleave'])."
        )
    iv = str(iv).strip().lower()

    wl = _extract_wavelengths(attrs)
    if wl is None:
        raise ValueError(
            "Missing wavelengths in Zarr attrs; cannot plot intensity vs real band nm."
        )
    wavelengths = np.asarray(wl, dtype=float).ravel()

    # Convert to (rows, cols, bands)
    dA = da.from_zarr(arr)
    perm = _get_rcb_permutation(iv)
    cube = da.transpose(dA, perm)
    R, C, B = cube.shape

    wavelengths = wavelengths[:B]

    # Validate mask and get bounding box
    if mask.shape != (R, C):
        raise ValueError(f"Mask shape {mask.shape} != (rows, cols)=({R},{C})")

    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        raise ValueError("Mask is empty.")
    y0, y1 = ys.min(), ys.max() + 1
    x0, x1 = xs.min(), xs.max() + 1

    mask_crop = mask[y0:y1, x0:x1]
    cube_crop = cube[y0:y1, x0:x1, :]

    # Optional rechunking for better performance
    if rechunk_2d is not None:
        ry, rx = rechunk_2d
        bchunk = cube_crop.chunks[2][0] if hasattr(cube_crop, "chunks") else B
        cube_crop = cube_crop.rechunk((ry, rx, bchunk))

    # Compute masked statistics
    m_da = da.from_array(mask_crop, chunks=cube_crop.chunks[:2])
    m3 = m_da[..., None]
    y = cube_crop.astype(dtype_work) * m3.astype(dtype_work)
    sum_b = y.sum(axis=(0, 1))
    cnt = m_da.sum().astype(dtype_work)
    cnt_b = da.full_like(sum_b, fill_value=cnt, dtype=dtype_work)

    mean = (sum_b / da.maximum(cnt_b, 1)).astype(np.float32)

    # Compute standard deviation
    sumsq_b = (y * cube_crop.astype(dtype_work)).sum(axis=(0, 1))
    ex2 = sumsq_b / da.maximum(cnt_b, 1)
    var = da.clip(ex2 - mean.astype(dtype_work) ** 2, 0.0, np.inf)
    std = da.sqrt(var).astype(np.float32)

    mean_np = mean.compute()
    std_np = std.compute()
    return wavelengths, mean_np, std_np
