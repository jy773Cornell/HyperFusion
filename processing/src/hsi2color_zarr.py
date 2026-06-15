"""Hyperspectral to color conversion utilities.

Author: Jinhong Yu
Institution: Cornell University, CALS
Lab: Postharvest Technology Lab & CAIR Lab
Email: jy773@cornell.edu
"""

from __future__ import annotations
import numpy as np
import zarr
import dask.array as da
import scipy.io as spio
from scipy.interpolate import PchipInterpolator
from typing import Optional, Literal, Union, Tuple
from .envi_zarr_conversion import _get_rcb_permutation, _fix_bands_axis


def _extract_wavelengths(attrs: dict):
    """Read wavelengths from attrs with tolerant key matching."""
    for key in ("wavelength", "Wavelength"):
        if key in attrs and attrs[key] is not None:
            return attrs[key]
    return None


def _trapz_dask(y: da.Array, x: np.ndarray, axis: int = -1) -> da.Array:
    """Dask-friendly trapezoidal integration with non-uniform x for spectral integration."""
    dx = np.diff(x).astype(np.float32)  # (B-1,)
    sl0 = [slice(None)] * y.ndim
    sl1 = [slice(None)] * y.ndim
    # slices along the integration axis
    if axis < 0:
        axis = y.ndim + axis
    sl0[axis] = slice(0, -1)
    sl1[axis] = slice(1, None)
    y0 = y[tuple(sl0)]
    y1 = y[tuple(sl1)]
    return da.tensordot((y0 + y1) * 0.5, dx, axes=([axis], [0]))


def _load_cmf_and_illuminant(
    illuminants_path: str, illuminant_d: int, w_target: np.ndarray
):
    """Load color matching functions and D-illuminant and interpolate to target wavelengths.

    Returns:
        (Iw, xx, yy, zz): each shape (B,)
    """
    mat = spio.loadmat(illuminants_path)
    w_xyz = mat["wxyz"][:, 0]
    xbar = mat["wxyz"][:, 1]
    ybar = mat["wxyz"][:, 2]
    zbar = mat["wxyz"][:, 3]
    Dset = mat["D"]
    illum_idx = {50: 2, 55: 3, 65: 1, 75: 4}[int(illuminant_d)]
    wI = Dset[:, 0]
    I = Dset[:, illum_idx]

    Iw = PchipInterpolator(wI, I, extrapolate=True)(w_target)
    xx = PchipInterpolator(w_xyz, xbar, extrapolate=True)(w_target)
    yy = PchipInterpolator(w_xyz, ybar, extrapolate=True)(w_target)
    zz = PchipInterpolator(w_xyz, zbar, extrapolate=True)(w_target)
    return (
        Iw.astype(np.float32),
        xx.astype(np.float32),
        yy.astype(np.float32),
        zz.astype(np.float32),
    )


def _xyz_to_lab_dask(
    X: da.Array, Y: da.Array, Z: da.Array, Xn: float, Yn: float, Zn: float
) -> da.Array:
    """Dask-friendly XYZ→Lab (CIE 1976) with given reference white (Xn,Yn,Zn)."""
    X = X.astype(np.float32)
    Y = Y.astype(np.float32)
    Z = Z.astype(np.float32)
    Xn = np.float32(Xn)
    Yn = np.float32(Yn)
    Zn = np.float32(Zn)

    # f(t) piecewise
    delta = np.float32(6.0 / 29.0)
    delta3 = delta**3
    inv_3delta2 = np.float32(1.0 / (3.0 * delta**2))
    offset = np.float32(4.0 / 29.0)

    def _f(t: da.Array) -> da.Array:
        c = t > delta3
        return da.where(c, da.power(t, np.float32(1.0 / 3.0)), t * inv_3delta2 + offset)

    xr = X / Xn
    yr = Y / Yn
    zr = Z / Zn

    fx = _f(xr)
    fy = _f(yr)
    fz = _f(zr)

    L = np.float32(116.0) * fy - np.float32(16.0)
    a = np.float32(500.0) * (fx - fy)
    b = np.float32(200.0) * (fy - fz)
    return da.stack([L, a, b], axis=-1).astype(np.float32)


def hsi_to_color_zarr(
    zarr_path: str,
    illuminants_path: str,
    *,
    illuminant_d: Literal[50, 55, 65, 75] = 65,
    assume_interleave: Optional[Literal["bip", "bil", "bsq"]] = None,
    truncate_nm: float = 780.0,
    sort_wavelengths: bool = True,
    normalize: Literal["none", "global"] = "none",
    exposure: float = 1.0,
    output: Literal["srgb", "lab", "xyz"] = "srgb",
    compute: bool = True,
) -> Union[np.ndarray, da.Array]:
    """
    Unified HSI→color: choose output in {'srgb','lab','xyz'}.
    """

    # Open Zarr data
    root = zarr.open(zarr_path, mode="r")
    arr = root if isinstance(root, zarr.Array) else root[list(root.array_keys())[0]]
    if arr.ndim != 3:
        raise ValueError(f"Expected 3D array; got {arr.shape}")
    attrs = arr.attrs.asdict() if hasattr(arr.attrs, "asdict") else dict(arr.attrs)
    interleave = assume_interleave or attrs.get("interleave")
    if interleave is None:
        raise ValueError(
            "Missing interleave; set arr.attrs['interleave'] or pass assume_interleave."
        )
    interleave = str(interleave).strip().lower()
    perm = _get_rcb_permutation(interleave)

    wavelengths = _extract_wavelengths(attrs)
    if wavelengths is None:
        raise ValueError("Missing wavelengths in arr.attrs['wavelength'] (nm).")
    w = np.asarray(wavelengths, dtype=np.float64).ravel()
    if w.size == 0:
        raise ValueError("Empty wavelengths array in attrs['wavelength'].")

    # Sort wavelengths if needed
    order = None
    if sort_wavelengths and not np.all(np.diff(w) > 0):
        order = np.argsort(w)
        w = w[order]

    # Convert to (rows, cols, bands)
    dA = da.from_zarr(arr)
    cube = da.transpose(dA, perm)

    # Ensure bands last equals len(w)
    final_perm = _fix_bands_axis(arr.shape, w.size, perm)
    if final_perm != (0, 1, 2):
        cube = da.transpose(cube, final_perm)
    if order is not None:
        cube = cube[..., order]

    # Truncate to visible range
    cut = np.searchsorted(w, truncate_nm, side="right")
    w = w[:cut]
    cube = cube[..., :cut]

    # Normalize if requested
    cube = cube.astype(np.float32)
    if normalize == "global":
        eps = np.float32(1e-12)
        maxv = da.maximum(cube.max(), eps).astype(np.float32)
        cube = cube / maxv

    # Load color matching functions and illuminant
    Iw, xx, yy, zz = _load_cmf_and_illuminant(illuminants_path, illuminant_d, w)

    # Spectral weights
    wy = (Iw * yy).astype(np.float32)
    wx = (Iw * xx).astype(np.float32)
    wz = (Iw * zz).astype(np.float32)

    # Normalization constant
    k = np.float32(1.0 / float(np.trapezoid(wy, w)))

    # Integrate along bands
    X = k * _trapz_dask(cube * wx, w, axis=-1)
    Y = k * _trapz_dask(cube * wy, w, axis=-1)
    Z = k * _trapz_dask(cube * wz, w, axis=-1)

    # Select output
    if output == "xyz":
        XYZ = da.stack([X, Y, Z], axis=-1).astype(np.float32)
        return XYZ.compute() if compute else XYZ

    if output == "lab":
        Xn = k * np.float32(np.trapezoid(wx, w))
        Yn = k * np.float32(np.trapezoid(wy, w))
        Zn = k * np.float32(np.trapezoid(wz, w))
        Lab = _xyz_to_lab_dask(X, Y, Z, float(Xn), float(Yn), float(Zn))
        return Lab.compute() if compute else Lab

    # output == "srgb"
    M = np.array(
        [
            [3.2404542, -1.5371385, -0.4985314],
            [-0.9692660, 1.8760108, 0.0415560],
            [0.0556434, -0.2040259, 1.0572252],
        ],
        dtype=np.float32,
    )
    XYZ = da.stack([X, Y, Z], axis=-1).astype(np.float32)
    sRGB_lin = da.tensordot(XYZ, M.T, axes=([2], [0])) * np.float32(exposure)
    thresh = np.float32(0.0031308)  # Apply sRGB gamma correction
    sRGB = da.where(
        sRGB_lin > thresh,
        np.float32(1.055) * da.power(sRGB_lin, np.float32(1.0 / 2.4))
        - np.float32(0.055),
        np.float32(12.92) * sRGB_lin,
    ).astype(np.float32)
    sRGB = da.clip(sRGB, 0.0, 1.0)
    return sRGB.compute() if compute else sRGB
