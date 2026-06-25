# ENVI BIL float cube I/O for fusion HSI processing (backend/offline pipeline).
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


_ENVI_DTYPES: dict[int, type[np.generic]] = {
    1: np.uint8,
    2: np.int16,
    3: np.int32,
    4: np.float32,
    5: np.float64,
    12: np.uint16,
}


@dataclass
class EnviMetadata:
    hdr_path: Path
    raw_path: Path
    samples: int
    lines: int
    bands: int
    data_type: int
    interleave: str
    wavelengths_nm: list[float] = field(default_factory=list)
    extra_lines: list[str] = field(default_factory=list)


def resolve_ffc_hdr(session_dir: Path, mode: str, camera: str) -> Path:
    preprocessed = session_dir / mode / camera / "preprocessed"
    matches = sorted(preprocessed.glob("*_ffc.hdr"))
    if not matches:
        raise FileNotFoundError(f"No *_ffc.hdr under {preprocessed}")
    return matches[0]


def _parse_numeric_list(text: str) -> list[float]:
    return [float(part.strip()) for part in text.replace("{", " ").replace("}", " ").split(",") if part.strip()]


def parse_envi_hdr(hdr_path: Path) -> EnviMetadata:
    text = hdr_path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()

    values: dict[str, str] = {}
    extra_lines: list[str] = []
    wavelengths: list[float] = []
    idx = 0
    while idx < len(lines):
        line = lines[idx]
        lower = line.strip().lower()
        if lower.startswith("wavelength"):
            block = line.split("=", 1)[1].strip()
            idx += 1
            while idx < len(lines) and "}" not in block:
                block += " " + lines[idx].strip()
                idx += 1
            wavelengths = _parse_numeric_list(block)
            continue
        if "=" in line and not lower.startswith("wavelength"):
            key, value = line.split("=", 1)
            values[key.strip().lower()] = value.strip()
        else:
            extra_lines.append(line)
        idx += 1

    samples = int(values.get("samples", "0"))
    lines_count = int(values.get("lines", "0"))
    bands = int(values.get("bands", "0"))
    data_type = int(values.get("data type", "4"))
    interleave = values.get("interleave", "bil").lower()

    raw_path = hdr_path.with_suffix(".raw")
    if "data file" in values:
        candidate = Path(values["data file"].strip().strip("{}"))
        if not candidate.is_absolute():
            candidate = hdr_path.parent / candidate
        raw_path = candidate

    if not raw_path.is_file():
        raise FileNotFoundError(f"ENVI raw file not found for {hdr_path}: {raw_path}")

    return EnviMetadata(
        hdr_path=hdr_path,
        raw_path=raw_path,
        samples=samples,
        lines=lines_count,
        bands=bands,
        data_type=data_type,
        interleave=interleave,
        wavelengths_nm=wavelengths,
        extra_lines=extra_lines,
    )


def _dtype_for_metadata(metadata: EnviMetadata) -> type[np.generic]:
    if metadata.data_type not in _ENVI_DTYPES:
        raise ValueError(f"Unsupported ENVI data type {metadata.data_type} in {metadata.hdr_path}")
    return _ENVI_DTYPES[metadata.data_type]


def open_bil_memmap(metadata: EnviMetadata) -> np.memmap:
    dtype = _dtype_for_metadata(metadata)
    if metadata.interleave != "bil":
        raise ValueError(f"Only BIL interleave supported, got {metadata.interleave}")
    element_count = metadata.lines * metadata.bands * metadata.samples
    return np.memmap(metadata.raw_path, dtype=dtype, mode="r", shape=(element_count,))


def read_bil_cube(metadata: EnviMetadata) -> np.ndarray:
    """Return cube as (lines, samples, bands) float32."""
    data = open_bil_memmap(metadata).reshape(metadata.lines, metadata.bands, metadata.samples)
    cube = np.asarray(data.transpose(0, 2, 1))
    if cube.dtype != np.float32:
        cube = cube.astype(np.float32)
    return cube


def write_envi_hdr(
    hdr_path: Path,
    metadata: EnviMetadata,
    *,
    samples: int,
    lines: int,
    bands: int,
    raw_basename: str,
    description: str,
) -> None:
    lines_out: list[str] = ["ENVI", f"description = {{{description}}}", "file type = ENVI", ""]
    if metadata.extra_lines:
        for line in metadata.extra_lines:
            if line.strip() and not line.strip().lower().startswith("description"):
                lines_out.append(line)
    lines_out.extend(
        [
            f"samples = {samples}",
            f"bands = {bands}",
            f"lines = {lines}",
            "",
            "interleave = bil",
            f"data type = {metadata.data_type}",
            "header offset = 0",
            "byte order = 0",
            "",
        ]
    )
    if metadata.wavelengths_nm:
        lines_out.append("Wavelength = {")
        for index, value in enumerate(metadata.wavelengths_nm):
            suffix = "," if index + 1 < len(metadata.wavelengths_nm) else ""
            lines_out.append(f"{value:.6f}{suffix}")
        lines_out.append("}")
    lines_out.append("")
    lines_out.append(f"data file = {raw_basename}")
    hdr_path.write_text("\n".join(lines_out) + "\n", encoding="utf-8")


def write_bil_cube(raw_path: Path, cube: np.ndarray) -> None:
    """Write (lines, samples, bands) float32 cube as BIL."""
    if cube.ndim != 3:
        raise ValueError(f"Expected HxWxB cube, got shape {cube.shape}")
    bil = np.transpose(cube.astype(np.float32, copy=False), (0, 2, 1))
    bil.tofile(raw_path)


def crop_bil_memmap_to_file(
    metadata: EnviMetadata,
    raw_out: Path,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
) -> tuple[int, int, int]:
    """Crop samples/lines from source BIL memmap without loading the full cube."""
    src = open_bil_memmap(metadata).reshape(metadata.lines, metadata.bands, metadata.samples)
    out_lines = y1 - y0
    out_samples = x1 - x0
    raw_out.parent.mkdir(parents=True, exist_ok=True)
    with raw_out.open("wb") as handle:
        for line_idx in range(y0, y1):
            line_bil = np.asarray(src[line_idx, :, x0:x1], dtype=np.float32)
            handle.write(line_bil.tobytes())
    return out_samples, out_lines, metadata.bands
