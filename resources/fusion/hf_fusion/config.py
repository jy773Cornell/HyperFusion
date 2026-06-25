# Load spatial calibration from hyperfusion.cfg for fusion alignment.
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SpatialCalibration:
    fx10e_mm_per_pixel: float
    swir3_mm_per_pixel: float

    @property
    def upsample_scale(self) -> float:
        return self.swir3_mm_per_pixel / self.fx10e_mm_per_pixel


def find_hyperfusion_cfg(start: Path | None = None) -> Path:
    candidates: list[Path] = []
    if start is not None:
        for parent in [start, *start.parents]:
            candidates.append(parent / "app" / "hyperfusion.cfg")
            candidates.append(parent / "hyperfusion.cfg")
    repo_root = Path(__file__).resolve().parents[3]
    candidates.extend(
        [
            repo_root / "app" / "hyperfusion.cfg",
            repo_root / "hyperfusion.cfg",
        ]
    )
    seen: set[Path] = set()
    for path in candidates:
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.is_file():
            return resolved
    raise FileNotFoundError("hyperfusion.cfg not found")


def load_spatial_calibration(cfg_path: Path | None = None) -> SpatialCalibration:
    path = cfg_path or find_hyperfusion_cfg()
    text = path.read_text(encoding="utf-8")
    values: dict[str, float] = {}
    for key in ("fx10e_spatial_mm_per_pixel", "swir3_spatial_mm_per_pixel"):
        match = re.search(rf"^{re.escape(key)}\s*=\s*([0-9.+-eE]+)", text, re.MULTILINE)
        if not match:
            raise ValueError(f"Missing {key} in {path}")
        values[key] = float(match.group(1))
    return SpatialCalibration(
        fx10e_mm_per_pixel=values["fx10e_spatial_mm_per_pixel"],
        swir3_mm_per_pixel=values["swir3_spatial_mm_per_pixel"],
    )
