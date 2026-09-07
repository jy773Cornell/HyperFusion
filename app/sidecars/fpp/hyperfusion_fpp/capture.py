"""Load one HyperFusion FPP pin burst (26 HDMI PSP TIFF + JSON, or legacy 14 u-only)."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import tifffile

STEP_SPECS: tuple[tuple[int, str, str], ...] = (
    (0, "PSP black", "Black"),
    (1, "PSP white", "White"),
    (2, "PSP sine 1 0", "1-period sine 0"),
    (3, "PSP sine 1 90", "1-period sine 90"),
    (4, "PSP sine 1 180", "1-period sine 180"),
    (5, "PSP sine 1 270", "1-period sine 270"),
    (6, "PSP sine 8 0", "8-period sine 0"),
    (7, "PSP sine 8 90", "8-period sine 90"),
    (8, "PSP sine 8 180", "8-period sine 180"),
    (9, "PSP sine 8 270", "8-period sine 270"),
    (10, "PSP sine 80 0", "80-period sine 0"),
    (11, "PSP sine 80 90", "80-period sine 90"),
    (12, "PSP sine 80 180", "80-period sine 180"),
    (13, "PSP sine 80 270", "80-period sine 270"),
    (14, "PSP sine v 1 0", "1-period sine v 0"),
    (15, "PSP sine v 1 90", "1-period sine v 90"),
    (16, "PSP sine v 1 180", "1-period sine v 180"),
    (17, "PSP sine v 1 270", "1-period sine v 270"),
    (18, "PSP sine v 8 0", "8-period sine v 0"),
    (19, "PSP sine v 8 90", "8-period sine v 90"),
    (20, "PSP sine v 8 180", "8-period sine v 180"),
    (21, "PSP sine v 8 270", "8-period sine v 270"),
    (22, "PSP sine v 80 0", "80-period sine v 0"),
    (23, "PSP sine v 80 90", "80-period sine v 90"),
    (24, "PSP sine v 80 180", "80-period sine v 180"),
    (25, "PSP sine v 80 270", "80-period sine v 270"),
)
STEP_COUNT = len(STEP_SPECS)
U_ONLY_STEP_COUNT = 14
PROJECTOR_WIDTH_PX = 1280
PROJECTOR_HEIGHT_PX = 720

_LEGACY_MARKERS = ("splash", "fpp lines", "horizontal ramp")


def _to_gray_f32(image: np.ndarray) -> np.ndarray:
    arr = np.asarray(image)
    if arr.ndim == 3:
        arr = arr[..., :3].astype(np.float32)
        gray = 0.299 * arr[..., 0] + 0.587 * arr[..., 1] + 0.114 * arr[..., 2]
    else:
        gray = arr.astype(np.float32)
    if gray.max() > 1.5:
        gray = gray / 255.0
    return np.clip(gray, 0.0, 1.0)


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _stem_sort_key(path: Path) -> tuple[int, str]:
    match = re.search(r"(\d+)$", path.stem)
    if match:
        return int(match.group(1)), path.stem
    return 10**9, path.stem


def _reject_legacy(pattern: str, label: str) -> None:
    text = f"{pattern} {label}".lower()
    if any(marker in text for marker in _LEGACY_MARKERS):
        raise ValueError(
            "USB TPG/hybrid FPP bursts are removed. Recapture with HDMI PSP "
            "(PSP sine 1/8/80 u and v)."
        )


@dataclass
class FppFrame:
    path: Path
    step_index: int
    pattern: str
    label: str
    meta: dict[str, Any] = field(default_factory=dict)
    image: np.ndarray = field(repr=False, default_factory=lambda: np.zeros((0, 0), np.float32))


@dataclass
class FppBurst:
    directory: Path
    frames: list[FppFrame]
    by_index: dict[int, FppFrame] = field(default_factory=dict)

    def image(self, step_index: int) -> np.ndarray:
        return self.by_index[step_index].image

    def first_meta(self) -> dict[str, Any]:
        for frame in self.frames:
            if frame.meta:
                return frame.meta
        return {}


def _match_step(
    meta: dict[str, Any],
    fallback_index: int,
    specs: tuple[tuple[int, str, str], ...] | None = None,
) -> tuple[int, str, str]:
    specs = specs or STEP_SPECS
    pattern = str(meta.get("fpp_pattern", "")).strip()
    label = str(meta.get("fpp_step_label", "")).strip()
    _reject_legacy(pattern, label)
    if "fpp_step_index" in meta:
        try:
            index = int(meta["fpp_step_index"])
            if 0 <= index < len(specs):
                spec = specs[index]
                return index, pattern or spec[1], label or spec[2]
        except (TypeError, ValueError):
            pass
    for index, spec_pattern, spec_label in specs:
        if pattern and pattern.lower() == spec_pattern.lower():
            return index, spec_pattern, label or spec_label
        if label and label.lower() == spec_label.lower():
            return index, pattern or spec_pattern, spec_label
    spec = specs[min(fallback_index, len(specs) - 1)]
    return fallback_index, pattern or spec[1], label or spec[2]


def _burst_stride(tiffs: list[Path], offset: int) -> int:
    remaining = len(tiffs) - offset
    limit = min(STEP_COUNT, remaining)
    for path in tiffs[offset : offset + limit]:
        json_path = path.with_suffix(".json")
        if not json_path.is_file():
            continue
        meta = _read_json(json_path)
        pattern = str(meta.get("fpp_pattern", "")).lower()
        label = str(meta.get("fpp_step_label", "")).lower()
        try:
            index = int(meta.get("fpp_step_index", -1))
        except (TypeError, ValueError):
            index = -1
        if "sine v" in pattern or "sine v" in label or index >= U_ONLY_STEP_COUNT:
            return STEP_COUNT
    if remaining >= STEP_COUNT:
        return STEP_COUNT
    if remaining >= U_ONLY_STEP_COUNT:
        return U_ONLY_STEP_COUNT
    return STEP_COUNT


def list_burst_starts(directory: Path) -> list[Path]:
    """TIFF stems that begin a 26-frame (u+v) or legacy 14-frame (u) HDMI PSP burst."""
    tiffs = sorted(directory.glob("*.tif"), key=_stem_sort_key)
    tiffs += sorted(p for p in directory.glob("*.tiff") if p not in tiffs)
    if not tiffs:
        return []
    starts: list[Path] = []
    i = 0
    while i < len(tiffs):
        stride = _burst_stride(tiffs, i)
        json_path = tiffs[i].with_suffix(".json")
        if json_path.is_file():
            meta = _read_json(json_path)
            index, _, _ = _match_step(meta, 0, STEP_SPECS[:stride])
            if index == 0:
                starts.append(tiffs[i])
                i += stride
                continue
        if i + stride <= len(tiffs) and (i % stride) == 0:
            starts.append(tiffs[i])
            i += stride
            continue
        i += 1
    if not starts and len(tiffs) >= U_ONLY_STEP_COUNT:
        starts.append(tiffs[0])
    return starts


def load_burst(directory: Path, start: Path | None = None) -> FppBurst:
    directory = directory.resolve()
    tiffs = sorted(directory.glob("*.tif"), key=_stem_sort_key)
    tiffs += sorted(p for p in directory.glob("*.tiff") if p not in set(tiffs))
    if start is None:
        starts = list_burst_starts(directory)
        if not starts:
            raise FileNotFoundError(f"{directory} has no FPP burst start.")
        start = starts[0]
    start_name = Path(start).name
    offset = next((i for i, p in enumerate(tiffs) if p.name == start_name), None)
    if offset is None:
        raise FileNotFoundError(f"Burst start {start_name} is not in {directory}")
    stride = _burst_stride(tiffs, offset)
    specs = STEP_SPECS[:stride]
    if len(tiffs) < offset + stride:
        raise FileNotFoundError(
            f"{directory} needs {stride} HDMI PSP TIFFs from {start.name}. "
            f"Found {len(tiffs) - offset}."
        )
    chunk = tiffs[offset : offset + stride]

    frames: list[FppFrame] = []
    by_index: dict[int, FppFrame] = {}
    for fallback, path in enumerate(chunk):
        meta: dict[str, Any] = {}
        json_path = path.with_suffix(".json")
        if json_path.is_file():
            meta = _read_json(json_path)
        index, pattern, label = _match_step(meta, fallback, specs)
        image = _to_gray_f32(tifffile.imread(path))
        frame = FppFrame(
            path=path,
            step_index=index,
            pattern=pattern,
            label=label,
            meta=meta,
            image=image,
        )
        frames.append(frame)
        by_index[index] = frame

    missing = [spec[2] for spec in specs if spec[0] not in by_index]
    if missing:
        raise ValueError(f"FPP burst missing steps: {', '.join(missing)}")
    return FppBurst(directory=directory, frames=frames, by_index=by_index)


def camera_intrinsics(meta: dict[str, Any]) -> tuple[np.ndarray, np.ndarray] | None:
    inner = meta.get("intrinsics") if isinstance(meta.get("intrinsics"), dict) else meta
    try:
        fx = float(inner["fx"])
        fy = float(inner["fy"])
        cx = float(inner["cx"])
        cy = float(inner["cy"])
    except (KeyError, TypeError, ValueError):
        return None
    if fx <= 0.0 or fy <= 0.0:
        return None
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.asarray(inner.get("distortion") or [], dtype=np.float64)
    return k, dist


def camera_extrinsics_rt(meta: dict[str, Any]) -> tuple[np.ndarray, np.ndarray] | None:
    """OpenCV camera R,t in base_link (room / projector frame).

    Apex JSON may store t in the MVS sample-static frame. Undo
    stage_output.output_translation_m so rays meet the real tray / DLP.
    """
    ext = meta.get("extrinsics")
    if not isinstance(ext, dict):
        return None
    try:
        r = np.asarray(ext["R"], dtype=np.float64)
        t = np.asarray(ext["t"], dtype=np.float64).reshape(3)
    except (KeyError, TypeError, ValueError):
        return None
    if r.shape != (3, 3):
        return None
    stage = meta.get("stage_output")
    if isinstance(stage, dict):
        shift = stage.get("output_translation_m")
        if isinstance(shift, dict):
            t = t - np.array(
                [
                    float(shift.get("x_m") or 0.0),
                    float(shift.get("y_m") or 0.0),
                    float(shift.get("z_m") or 0.0),
                ],
                dtype=np.float64,
            )
    return r, t
