# CLI entry for FX10e + SWIR3 fusion (backend/offline). Used by the HyperFusion app via QProcess.
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.hf_fusion_pipeline import FusionPipelineParams, run_fusion_pipeline


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HyperFusion FX10e + SWIR3 spatial registration and spectral fusion")
    parser.add_argument("--session", type=Path, required=True, help="Capture session root directory")
    parser.add_argument("--mode", default="reflectance", help="Illumination mode folder (default: reflectance)")
    parser.add_argument("--cfg", type=Path, default=None, help="hyperfusion.cfg path (optional)")
    parser.add_argument("--margin-mm", type=float, default=5.0, help="Crop margin around chip masks (mm)")
    parser.add_argument(
        "--no-hsi",
        action="store_true",
        help="Coarse alignment + mask QA only; skip phase correction and spectral fusion",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    session = args.session.expanduser().resolve()
    if not session.is_dir():
        print(f"Session directory not found: {session}", file=sys.stderr)
        return 1

    process_hsi = not args.no_hsi
    if process_hsi:
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            print(
                "matplotlib is required for fused ROI spectrum plots. "
                "Run: cd resources\\hf_fusion ; .\\.venv\\Scripts\\python.exe -m pip install -r requirements.txt",
                file=sys.stderr,
            )
            return 1

    params = FusionPipelineParams(
        session=session,
        mode=args.mode,
        cfg=args.cfg,
        margin_mm=args.margin_mm,
        process_hsi=process_hsi,
    )

    try:
        result = run_fusion_pipeline(params)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    payload = {
        "ok": True,
        "session": str(result.session),
        "mode": result.mode,
        "alignment_json": str(result.alignment_json),
        "roi_count": len(result.pairs),
        "pipeline_complete": result.pipeline_complete,
    }
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
