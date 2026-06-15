"""Flat field correction for hyperspectral Zarr arrays.

Author: Jinhong Yu
Institution: Cornell University, CALS
Lab: Postharvest Technology Lab & CAIR Lab
Email: jy773@cornell.edu
"""

import json
import numpy as np
import dask.array as da
import zarr
from typing import Optional, Iterable, Tuple, Literal
from .envi_zarr_conversion import _get_interleave_info

# #region agent log
import os as _os
_DEBUG_LOG = "/home/jy773/workspace/Hyper-Studio/.cursor/debug-9b438d.log"
def _dlog(msg: str, data: dict, hypothesis_id: str):
    try:
        _os.makedirs(_os.path.dirname(_DEBUG_LOG), exist_ok=True)
        with open(_DEBUG_LOG, "a") as f:
            f.write(json.dumps({"sessionId": "9b438d", "location": "flat_field_correction_zarr.py", "message": msg, "data": data, "hypothesisId": hypothesis_id, "timestamp": __import__("time").time() * 1000}) + "\n")
    except Exception:
        pass
# #endregion


def flat_field_correction_zarr(
    raw_zarr_path: str,
    white_zarr_path: str,
    dark_zarr_path: str,
    out_zarr_path: str,
    *,
    interleave: Optional[Literal["bip", "bil", "bsq"]] = None,
    eps: float = 1e-6,
    clamp_min: Optional[float] = 0.0,
    clamp_max: Optional[float] = 1.0,
    dtype_out=np.float32,
    chunks: Optional[Tuple[int, int, int]] = None,
    overwrite: bool = True,
) -> str:
    """Apply flat field correction using white and dark references to remove illumination artifacts."""
    raw = da.from_zarr(raw_zarr_path)
    white = da.from_zarr(white_zarr_path)
    dark = da.from_zarr(dark_zarr_path)

    if raw.ndim != 3 or white.ndim != 3 or dark.ndim != 3:
        raise ValueError("All inputs must be 3D arrays.")

    # Get interleave info
    za = zarr.open(raw_zarr_path, mode="r")
    if not isinstance(za, zarr.Array):
        keys = [k for k in za.array_keys()]
        if not keys:
            raise ValueError(f"No arrays found in {raw_zarr_path}")
        za = za[keys[0]]
    interleave, row_axis = _get_interleave_info(za, interleave)

    # Compute row-mean references
    white_mean = white.mean(axis=row_axis, dtype=dtype_out)
    dark_mean = dark.mean(axis=row_axis, dtype=dtype_out)

    white_row = da.expand_dims(white_mean.astype(dtype_out, copy=False), axis=row_axis)
    dark_row = da.expand_dims(dark_mean.astype(dtype_out, copy=False), axis=row_axis)

    raw_f32 = raw.astype(dtype_out, copy=False)
    eps_scalar = dtype_out(eps)
    diff = white_row - dark_row
    diff_safe = da.where(da.isnan(diff), eps_scalar, diff)
    denom = da.maximum(diff_safe, eps_scalar)
    # #region agent log
    try:
        wm = da.atleast_1d(white_mean).ravel()[0]
        dm = da.atleast_1d(dark_mean).ravel()[0]
        dn = da.atleast_1d(denom).ravel()[0]
        wm, dm, dn = da.compute(wm, dm, dn)
        _dlog("pre_divide", {"white_mean_sample": float(wm), "dark_mean_sample": float(dm), "denom_sample": float(dn), "has_nan_denom": bool(np.isnan(dn)), "eps": eps}, "C")
    except Exception as e:
        _dlog("pre_divide_error", {"error": str(e)}, "C")
    # #endregion
    refl = (raw_f32 - dark_row) / denom

    if clamp_min is not None or clamp_max is not None:
        lo = (
            dtype_out(clamp_min)
            if clamp_min is not None
            else np.array(-np.inf, dtype=dtype_out)
        )
        hi = (
            dtype_out(clamp_max)
            if clamp_max is not None
            else np.array(np.inf, dtype=dtype_out)
        )
        refl = da.clip(refl, lo, hi)

    if chunks is not None:
        if len(chunks) != 3:
            raise ValueError("chunks must be length-3 in the stored axis order.")
        refl = refl.rechunk(chunks)

    # #region agent log
    _dlog("before_to_zarr", {"zarr_lib": getattr(zarr, "__version__", "?")}, "A")
    # #endregion
    refl.to_zarr(out_zarr_path, overwrite=overwrite)

    # Preserve metadata
    out_za = zarr.open(out_zarr_path, mode="r+")
    if isinstance(out_za, zarr.Array):
        out_za.attrs["interleave"] = interleave
        in_attrs = za.attrs.asdict() if hasattr(za.attrs, "asdict") else dict(za.attrs)
        if "wavelength" in in_attrs:
            out_za.attrs["wavelength"] = in_attrs["wavelength"]
    else:
        keys = [k for k in out_za.array_keys()]
        if keys:
            arr = out_za[keys[0]]
            arr.attrs["interleave"] = interleave
            in_attrs = (
                za.attrs.asdict() if hasattr(za.attrs, "asdict") else dict(za.attrs)
            )
            if "wavelength" in in_attrs:
                arr.attrs["wavelength"] = in_attrs["wavelength"]

    return out_zarr_path
