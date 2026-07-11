"""Robot base mount transform (world -> base_link) for URDF / ROS launch."""
from __future__ import annotations

import math
import os
from dataclasses import dataclass


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return default
    return float(raw)


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return default
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class MountConfig:
    ceiling_mount: bool = True
    height_m: float = 0.65
    roll_deg: float = 180.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    offset_x_m: float = 0.0
    offset_y_m: float = 0.0

    @classmethod
    def from_env(cls) -> MountConfig:
        return cls(
            ceiling_mount=_env_bool("HYPERFUSION_CEILING_MOUNT", True),
            height_m=_env_float("HYPERFUSION_CEILING_MOUNT_HEIGHT_M", 0.65),
            roll_deg=_env_float("HYPERFUSION_MOUNT_ROLL_DEG", 180.0),
            pitch_deg=_env_float("HYPERFUSION_MOUNT_PITCH_DEG", 0.0),
            yaw_deg=_env_float("HYPERFUSION_MOUNT_YAW_DEG", 0.0),
            offset_x_m=_env_float("HYPERFUSION_MOUNT_OFFSET_X_M", 0.0),
            offset_y_m=_env_float("HYPERFUSION_MOUNT_OFFSET_Y_M", 0.0),
        )

    @classmethod
    def from_cli(
        cls,
        *,
        ceiling_mount_height_m: float,
        roll_deg: float = 180.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        offset_x_mm: float = 0.0,
        offset_y_mm: float = 0.0,
        ceiling_mount: bool = True,
    ) -> MountConfig:
        return cls(
            ceiling_mount=ceiling_mount,
            height_m=ceiling_mount_height_m,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            offset_x_m=offset_x_mm / 1000.0,
            offset_y_m=offset_y_mm / 1000.0,
        )

    def apply_to_environ(self) -> None:
        os.environ["HYPERFUSION_CEILING_MOUNT"] = "true" if self.ceiling_mount else "false"
        os.environ["HYPERFUSION_CEILING_MOUNT_HEIGHT_M"] = f"{self.height_m:.6f}"
        os.environ["HYPERFUSION_MOUNT_ROLL_DEG"] = f"{self.roll_deg:.6f}"
        os.environ["HYPERFUSION_MOUNT_PITCH_DEG"] = f"{self.pitch_deg:.6f}"
        os.environ["HYPERFUSION_MOUNT_YAW_DEG"] = f"{self.yaw_deg:.6f}"
        os.environ["HYPERFUSION_MOUNT_OFFSET_X_M"] = f"{self.offset_x_m:.6f}"
        os.environ["HYPERFUSION_MOUNT_OFFSET_Y_M"] = f"{self.offset_y_m:.6f}"

    @property
    def roll_rad(self) -> float:
        return math.radians(self.roll_deg)

    @property
    def pitch_rad(self) -> float:
        return math.radians(self.pitch_deg)

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)

    def bash_exports(self) -> str:
        return (
            f"export HYPERFUSION_CEILING_MOUNT='{'true' if self.ceiling_mount else 'false'}' && "
            f"export HYPERFUSION_CEILING_MOUNT_HEIGHT_M='{self.height_m:.6f}' && "
            f"export HYPERFUSION_MOUNT_ROLL_DEG='{self.roll_deg:.6f}' && "
            f"export HYPERFUSION_MOUNT_PITCH_DEG='{self.pitch_deg:.6f}' && "
            f"export HYPERFUSION_MOUNT_YAW_DEG='{self.yaw_deg:.6f}' && "
            f"export HYPERFUSION_MOUNT_OFFSET_X_M='{self.offset_x_m:.6f}' && "
            f"export HYPERFUSION_MOUNT_OFFSET_Y_M='{self.offset_y_m:.6f}' && "
        )
