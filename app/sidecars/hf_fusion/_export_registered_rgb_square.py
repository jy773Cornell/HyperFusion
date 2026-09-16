# Square crop from native FX/SWIR RGB; light-gray outside the ROI mask.
from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

LIGHT_GRAY = (220, 220, 220)


def mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
    ys, xs = np.nonzero(mask > 0)
    if len(xs) == 0:
        raise ValueError("empty mask")
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def expand_bbox_to_square(
    x0: int, y0: int, x1: int, y1: int, width: int, height: int, *, margin_px: int = 12
) -> tuple[int, int, int, int]:
    x0 = max(0, x0 - margin_px)
    y0 = max(0, y0 - margin_px)
    x1 = min(width, x1 + margin_px)
    y1 = min(height, y1 + margin_px)
    w, h = x1 - x0, y1 - y0
    side = min(max(w, h), width, height)
    cx = 0.5 * (x0 + x1)
    cy = 0.5 * (y0 + y1)
    sx0 = int(round(cx - 0.5 * side))
    sy0 = int(round(cy - 0.5 * side))
    sx1 = sx0 + side
    sy1 = sy0 + side
    if sx0 < 0:
        sx1 -= sx0
        sx0 = 0
    if sy0 < 0:
        sy1 -= sy0
        sy0 = 0
    if sx1 > width:
        shift = sx1 - width
        sx0 -= shift
        sx1 = width
    if sy1 > height:
        shift = sy1 - height
        sy0 -= shift
        sy1 = height
    sx0 = max(0, sx0)
    sy0 = max(0, sy0)
    side_f = min(sx1 - sx0, sy1 - sy0, width - sx0, height - sy0)
    return sx0, sy0, sx0 + side_f, sy0 + side_f


def export_camera_square(
    session: Path, mode: str, camera: str, fx_roi: int = 1
) -> Path:
    rgb_dir = session / mode / camera / "preprocessed"
    rgb_path = next(rgb_dir.glob("*_rgb.png"))
    mask_path = rgb_dir / "segmentation" / "masks" / f"mask_{fx_roi:03d}.npy"
    rgb = np.asarray(Image.open(rgb_path).convert("RGB")).copy()
    mask = np.load(mask_path)
    if mask.ndim == 3:
        mask = mask[..., 0]
    if mask.shape[:2] != rgb.shape[:2]:
        mask_img = Image.fromarray((mask > 0).astype(np.uint8) * 255, mode="L")
        mask = np.asarray(
            mask_img.resize((rgb.shape[1], rgb.shape[0]), Image.NEAREST)
        )
    mask_bool = mask > 0

    x0, y0, x1, y1 = mask_bbox(mask_bool)
    sx0, sy0, sx1, sy1 = expand_bbox_to_square(
        x0, y0, x1, y1, rgb.shape[1], rgb.shape[0], margin_px=12
    )
    rgb_crop = rgb[sy0:sy1, sx0:sx1]
    mask_crop = mask_bool[sy0:sy1, sx0:sx1]
    assert rgb_crop.shape[0] == rgb_crop.shape[1], rgb_crop.shape

    # Keep ROI pixels; fill everything else with light gray.
    out_rgb = np.empty_like(rgb_crop)
    out_rgb[:] = np.asarray(LIGHT_GRAY, dtype=np.uint8)
    out_rgb[mask_crop] = rgb_crop[mask_crop]

    out_dir = session / mode / "fusion" / f"roi_{fx_roi:03d}_fx10e_swir3"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"roi_{fx_roi:03d}_{camera}_registered_rgb.png"
    Image.fromarray(out_rgb).save(out)
    print(
        f"{mode}/{camera}: square {out_rgb.shape[0]}  "
        f"roi_pixels={int(mask_crop.sum())}  bg=light_gray{LIGHT_GRAY}"
    )
    print(f"  {out}")
    return out


def main() -> None:
    session = Path(r"D:\Data_JY\2026_Grape_Data_Collection\Demo\concord_cluster2")
    for mode in ("reflectance", "transmittance"):
        for camera in ("fx10e", "swir3"):
            export_camera_square(session, mode, camera, fx_roi=1)


if __name__ == "__main__":
    main()
