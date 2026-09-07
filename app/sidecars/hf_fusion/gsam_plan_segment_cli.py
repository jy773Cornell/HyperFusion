# Plan-driven GSAM segmentation CLI (backend/offline). Used by the app via QProcess
# and for testing any capture session folder against app/preset/gsam_plans/*.json.
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.gsam_client import DEFAULT_GSAM_URL, require_gsam_ready
from src.gsam_overlay_sheet import preview_sheet_paths, session_preview_dir, write_session_qa_sheets
from src.gsam_plan_segment import (
    load_gsam_plan,
    segment_stream_from_plan,
    stream_needs_gsam,
    streams_with_rgb,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run GSAM from a plan file. Streams with two_stage.enabled use hole boxes then "
            "an object prompt inside each DINO crop. Streams with reuse_reflectance_masks "
            "(or reuse_masks_from) copy masks from another stream instead of calling GSAM."
        )
    )
    parser.add_argument("--session", type=Path, required=True, help="Capture session root")
    parser.add_argument("--plan", type=Path, default=None, help="GSAM plan JSON")
    parser.add_argument(
        "--stream",
        default=None,
        help="mode/camera (e.g. reflectance/swir3). Omit to process every plan stream that has RGB.",
    )
    parser.add_argument("--stem", default=None, help="Dataset stem (default: session folder name)")
    parser.add_argument("--gsam-url", default=DEFAULT_GSAM_URL)
    parser.add_argument(
        "--write-sheet",
        action="store_true",
        help="Only write {session}/preview QA sheets from existing preprocessed outputs",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    session = args.session.expanduser().resolve()
    if not session.is_dir():
        print(f"Session directory not found: {session}", file=sys.stderr)
        return 1

    if args.write_sheet:
        try:
            session_preview_dir(session)
            sheets = write_session_qa_sheets(session)
        except Exception as exc:
            print(str(exc), file=sys.stderr)
            return 1
        payload = {
            "ok": True,
            "session": str(session),
            "sheet_count": len(sheets),
            **preview_sheet_paths(session),
        }
        print(json.dumps(payload))
        return 0

    plan_path = args.plan.expanduser().resolve() if args.plan is not None else None
    if plan_path is None or not plan_path.is_file():
        print("GSAM plan not found (pass --plan, or --write-sheet)", file=sys.stderr)
        return 1

    stem = args.stem or session.name
    plan = load_gsam_plan(plan_path)

    if args.stream:
        if "/" not in args.stream:
            print(" --stream must be mode/camera", file=sys.stderr)
            return 1
        jobs = [tuple(args.stream.split("/", 1))]
    else:
        jobs = streams_with_rgb(session, plan, stem)
        if not jobs:
            print(f"No preprocessed RGB found under {session} for plan streams", file=sys.stderr)
            return 1

    needs_gsam = False
    for mode, camera in jobs:
        stream_key = f"{mode}/{camera}"
        stream_plan = plan["streams"].get(stream_key) or plan["streams"].get(stream_key.lower())
        if stream_plan is None:
            print(f"Plan has no stream {stream_key}", file=sys.stderr)
            return 1
        if stream_needs_gsam(stream_plan, mode, camera):
            needs_gsam = True
            break
    if needs_gsam:
        health = require_gsam_ready(args.gsam_url)
        print("GSAM health", health)
    else:
        print("GSAM not required (all selected streams reuse masks)")

    results = []
    try:
        for mode, camera in jobs:
            print(f"== {mode}/{camera} ==")
            result = segment_stream_from_plan(
                session,
                mode,
                camera,
                plan,
                stem=stem,
                gsam_url=args.gsam_url,
            )
            results.append(result)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    payload = {
        "ok": True,
        "session": str(session),
        "plan": str(plan_path),
        "stem": stem,
        "detection_count": results[-1].detection_count if results else 0,
        "two_stage": results[-1].two_stage if len(results) == 1 else any(item.two_stage for item in results),
        "prompt": results[-1].prompt if results else "",
        "reused_from": results[-1].reused_from if results else "",
        "segmentation_dir": str(results[-1].segmentation_dir) if results else "",
        **preview_sheet_paths(session),
        "streams": [
            {
                "stream": item.stream,
                "detection_count": item.detection_count,
                "two_stage": item.two_stage,
                "prompt": item.prompt,
                "reused_from": item.reused_from,
                "segmentation_dir": str(item.segmentation_dir),
            }
            for item in results
        ],
    }
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
