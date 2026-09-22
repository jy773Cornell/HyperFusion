"""Load one HyperFusion FPP pin burst.

New captures are a 26-frame HDMI PSP burst (black + white + 12u + 12v).
Older captures that start with a full-white RGB visual TIFF are still loaded;
color for new scans comes from the plan RGB ring, not that visual frame.
"""

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
CAPTURE_STEP_COUNT = STEP_COUNT + 1
U_ONLY_STEP_COUNT = 14
VISUAL_COLOR_PATTERN = "fpp visual color"
PROJECTOR_WIDTH_PX = 1280
PROJECTOR_HEIGHT_PX = 720

_LEGACY_MARKERS = ("splash", "fpp lines", "horizontal ramp")
_CHANNEL_ALIASES = {
    "r": "r",
    "red": "r",
    "g": "g",
    "green": "g",
    "b": "b",
    "blue": "b",
    "luma": "luma",
    "gray": "luma",
    "grey": "luma",
    "rgb": "luma",
    "auto": "auto",
}


def _normalize_channel(name: str | None) -> str:
    if name is None:
        return "auto"
    return _CHANNEL_ALIASES.get(str(name).strip().lower(), "auto")


def decode_channel_from_led_ma(red_ma: float, green_ma: float, blue_ma: float) -> str:
    """Pick r/g/b when one LED dominates; otherwise luma."""
    r, g, b = float(red_ma), float(green_ma), float(blue_ma)
    mx = max(r, g, b)
    if mx < 50.0:
        return "luma"
    second = sorted((r, g, b), reverse=True)[1]
    if second * 2.0 > mx:
        return "luma"
    if r >= g and r >= b:
        return "r"
    if g >= r and g >= b:
        return "g"
    return "b"


def decode_channel_from_meta(meta: dict[str, Any] | None) -> str | None:
    """Return channel from pose JSON, or None if unknown."""
    if not isinstance(meta, dict):
        return None
    direct = _normalize_channel(meta.get("decode_channel"))
    if direct != "auto":
        return direct
    led = meta.get("dlp_led")
    if isinstance(led, dict):
        led_ch = _normalize_channel(led.get("decode_channel"))
        if led_ch != "auto":
            return led_ch
        try:
            return decode_channel_from_led_ma(
                float(led.get("red_ma", 0) or 0),
                float(led.get("green_ma", 0) or 0),
                float(led.get("blue_ma", 0) or 0),
            )
        except (TypeError, ValueError):
            return None
    return None


def decode_channel_from_rgb_image(image: np.ndarray) -> str:
    """Heuristic when JSON has no LED: dominant mean channel on a color still."""
    arr = np.asarray(image)
    if arr.ndim != 3 or arr.shape[2] < 3:
        return "luma"
    sample = arr[::4, ::4] if arr.shape[0] * arr.shape[1] > 512 * 512 else arr
    means = sample[..., :3].reshape(-1, 3).astype(np.float64).mean(axis=0)
    if means.max() <= 1.5:
        means = means * 255.0
    mx = float(means.max())
    if mx < 8.0:
        return "luma"
    second = float(sorted(means, reverse=True)[1])
    if second * 1.8 > mx:
        return "luma"
    return ("r", "g", "b")[int(np.argmax(means))]


def _to_gray_f32(image: np.ndarray, channel: str = "luma") -> np.ndarray:
    arr = np.asarray(image)
    ch = _normalize_channel(channel)
    if ch == "auto":
        ch = "luma"
    if arr.ndim == 3 and arr.shape[2] >= 3:
        rgb = arr[..., :3].astype(np.float32)
        if ch == "r":
            gray = rgb[..., 0]
        elif ch == "g":
            gray = rgb[..., 1]
        elif ch == "b":
            gray = rgb[..., 2]
        else:
            gray = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
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
    decode_channel: str = "luma"
    visual_color_path: Path | None = None

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


def _is_visual_color_meta(meta: dict[str, Any]) -> bool:
    pattern = str(meta.get("fpp_pattern", "")).strip().lower()
    label = str(meta.get("fpp_step_label", "")).strip().lower()
    return pattern == VISUAL_COLOR_PATTERN or "visual color" in label


def _has_fpp_fields(meta: dict[str, Any]) -> bool:
    """True when pose JSON is an FPP still (not a plain RGB ring capture)."""
    if _is_visual_color_meta(meta):
        return True
    if "fpp_step_index" in meta:
        return True
    pattern = str(meta.get("fpp_pattern", "")).strip()
    label = str(meta.get("fpp_step_label", "")).strip()
    return bool(pattern or label)


def _burst_stride(tiffs: list[Path], offset: int) -> int:
    remaining = len(tiffs) - offset
    first_json = tiffs[offset].with_suffix(".json")
    if first_json.is_file() and _is_visual_color_meta(_read_json(first_json)):
        return CAPTURE_STEP_COUNT
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
    """TIFF stems beginning a visual+26, existing 26, or legacy 14 burst.

    Plain RGB ring stills (no ``fpp_*`` JSON fields) are skipped so they are not
    mistaken for FPP black frames when their indices align to the burst stride.
    """
    tiffs = sorted(directory.glob("*.tif"), key=_stem_sort_key)
    tiffs += sorted(p for p in directory.glob("*.tiff") if p not in tiffs)
    if not tiffs:
        return []
    starts: list[Path] = []
    i = 0
    while i < len(tiffs):
        json_path = tiffs[i].with_suffix(".json")
        if not json_path.is_file():
            i += 1
            continue
        meta = _read_json(json_path)
        if not _has_fpp_fields(meta):
            i += 1
            continue
        stride = _burst_stride(tiffs, i)
        if i + stride > len(tiffs):
            # Incomplete trailing burst (e.g. scan stopped mid-pin) — skip.
            break
        if _is_visual_color_meta(meta):
            starts.append(tiffs[i])
            i += stride
            continue
        index, _, _ = _match_step(meta, -1, STEP_SPECS[:stride])
        if index == 0:
            starts.append(tiffs[i])
            i += stride
            continue
        i += 1
    return starts


def list_burst_jobs(root: Path) -> list[tuple[Path, Path]]:
    """(burst_dir, start_tiff) in *root* and one-level child folders (Execute's multiview_*)."""
    root = Path(root)
    jobs: list[tuple[Path, Path]] = [(root, start) for start in list_burst_starts(root)]
    if not root.is_dir():
        return jobs
    seen = {(str(d.resolve()), s.name) for d, s in jobs}
    for child in sorted(p for p in root.iterdir() if p.is_dir()):
        for start in list_burst_starts(child):
            key = (str(child.resolve()), start.name)
            if key in seen:
                continue
            seen.add(key)
            jobs.append((child, start))
    return jobs


def resolve_decode_channel(
    *,
    meta: dict[str, Any] | None = None,
    channel: str | None = "auto",
    probe_image: np.ndarray | None = None,
) -> str:
    """CLI / JSON / image priority for the gray channel used by PSP decode."""
    override = _normalize_channel(channel)
    if override != "auto":
        return override
    from_meta = decode_channel_from_meta(meta)
    if from_meta is not None:
        return from_meta
    if probe_image is not None:
        return decode_channel_from_rgb_image(probe_image)
    return "luma"


def load_burst(
    directory: Path,
    start: Path | None = None,
    *,
    channel: str | None = "auto",
) -> FppBurst:
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
    capture_stride = _burst_stride(tiffs, offset)
    if len(tiffs) < offset + capture_stride:
        raise FileNotFoundError(
            f"{directory} needs {capture_stride} FPP TIFFs from {start.name}. "
            f"Found {len(tiffs) - offset}."
        )
    capture_chunk = tiffs[offset : offset + capture_stride]
    first_capture_meta: dict[str, Any] = {}
    first_capture_json = capture_chunk[0].with_suffix(".json")
    if first_capture_json.is_file():
        first_capture_meta = _read_json(first_capture_json)
    has_visual_color = _is_visual_color_meta(first_capture_meta)
    visual_color_path = capture_chunk[0] if has_visual_color else None
    chunk = capture_chunk[1:] if has_visual_color else capture_chunk
    decode_stride = len(chunk)
    specs = STEP_SPECS[:decode_stride]

    first_meta: dict[str, Any] = {}
    first_json = chunk[0].with_suffix(".json")
    if first_json.is_file():
        first_meta = _read_json(first_json)

    probe = tifffile.imread(chunk[min(1, len(chunk) - 1)])
    decode_ch = resolve_decode_channel(meta=first_meta, channel=channel, probe_image=probe)

    frames: list[FppFrame] = []
    by_index: dict[int, FppFrame] = {}
    for fallback, path in enumerate(chunk):
        meta: dict[str, Any] = {}
        json_path = path.with_suffix(".json")
        if json_path.is_file():
            meta = _read_json(json_path)
        index, pattern, label = _match_step(meta, fallback, specs)
        image = _to_gray_f32(tifffile.imread(path), decode_ch)
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
    return FppBurst(
        directory=directory,
        frames=frames,
        by_index=by_index,
        decode_channel=decode_ch,
        visual_color_path=visual_color_path,
    )


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
