# Generate 24-bit RGB sine PNGs (calibration / HDMI FPP). No hardware I/O.
# Writes 1280×720 u (vertical bars) and v (horizontal bars) under ../patterns/psp.
"""Write black, white, and 1/8/80-period 4-shift sine frames (u and v)."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

WIDTH = 1280
HEIGHT = 720
FREQUENCIES = (1, 8, 80)
SHIFTS_DEG = (0, 90, 180, 270)


def sine_line(length: int, freq: int, shift_deg: int) -> np.ndarray:
    x = np.arange(length, dtype=np.float64)
    theta = 2.0 * np.pi * float(freq) * (x / float(length)) - np.deg2rad(float(shift_deg))
    line = 0.5 + 0.5 * np.cos(theta)
    return np.clip(np.round(line * 255.0), 0, 255).astype(np.uint8)


def write_u(path: Path, column: np.ndarray) -> None:
    gray = np.tile(column.reshape(1, -1), (HEIGHT, 1))
    bgr = np.stack((gray, gray, gray), axis=-1)
    cv2.imwrite(str(path), bgr)


def write_v(path: Path, row: np.ndarray) -> None:
    gray = np.tile(row.reshape(-1, 1), (1, WIDTH))
    bgr = np.stack((gray, gray, gray), axis=-1)
    cv2.imwrite(str(path), bgr)


def main() -> int:
    dest = Path(__file__).resolve().parents[1] / "patterns" / "psp"
    dest.mkdir(parents=True, exist_ok=True)
    write_u(dest / "black.png", np.zeros(WIDTH, dtype=np.uint8))
    write_u(dest / "white.png", np.full(WIDTH, 255, dtype=np.uint8))
    for freq in FREQUENCIES:
        for shift in SHIFTS_DEG:
            name = f"sine_{freq}_{shift}.png"
            write_u(dest / name, sine_line(WIDTH, freq, shift))
            print(name)
            vname = f"sine_v_{freq}_{shift}.png"
            write_v(dest / vname, sine_line(HEIGHT, freq, shift))
            print(vname)
    print(f"wrote {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
