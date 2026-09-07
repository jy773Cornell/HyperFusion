"""Sidecar YAML config loading."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml

from hyperfusion_ur3e import PKG_ROOT

DEFAULT_CONFIG = PKG_ROOT / "config" / "ur3e_sidecar.yaml"


def load_yaml_config(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else {}
