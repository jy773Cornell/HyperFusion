"""Register the vendored SAM2 package when the repo folder is named `gsam2`.

The Meta SAM2 Python tree uses `import sam2.*` everywhere. HyperFusion keeps that
tree under `resources/gsam2/` (not `resources/sam2/`). Call `ensure_sam2_package()`
before any `sam2` imports.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_GSAM2_ROOT = Path(__file__).resolve().parent


def ensure_sam2_package() -> Path:
    """Expose this directory as the top-level `sam2` package. Returns gsam2 root."""
    root = str(_GSAM2_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)

    if "sam2" not in sys.modules:
        init_py = _GSAM2_ROOT / "__init__.py"
        spec = importlib.util.spec_from_file_location(
            "sam2",
            init_py,
            submodule_search_locations=[root],
        )
        if spec is None or spec.loader is None:
            raise ImportError(f"Could not load sam2 package from {init_py}")

        module = importlib.util.module_from_spec(spec)
        sys.modules["sam2"] = module
        spec.loader.exec_module(module)

    return _GSAM2_ROOT
