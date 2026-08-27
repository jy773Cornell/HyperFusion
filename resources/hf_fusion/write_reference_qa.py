# Copy white/black refs into preprocessed and write {session}/preview QA plots.
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from src.gsam_overlay_sheet import write_session_qa_sheets, write_session_rgb_all
from src.reference_qa import process_session_references

DEFAULT_COLLECTION = Path(r"E:\2026_Grape_Data_Collection\2026_Geneva_Concord")
PROCESSED_HINT = ("Unripe_T1", "Unripe_T2", "Unripe_T3", "Unripe_T4", "Unripe_T5", "Veraison_T1")


def _session_is_processed(session: Path) -> bool:
    return any(session.glob("*/**/preprocessed/*_ffc.hdr"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--collection", type=Path, default=DEFAULT_COLLECTION)
    parser.add_argument("--session", type=Path, default=None)
    parser.add_argument("--refs-only", action="store_true")
    parser.add_argument("--rgb-only", action="store_true")
    args = parser.parse_args()

    if args.session is not None:
        sessions = [args.session]
    else:
        named = [args.collection / name for name in PROCESSED_HINT]
        sessions = [p for p in named if p.is_dir() and _session_is_processed(p)]
        extra = [
            p
            for p in sorted(args.collection.iterdir())
            if p.is_dir() and p not in sessions and _session_is_processed(p)
        ]
        sessions.extend(extra)

    for session in sessions:
        print(f"== {session.name} ==")
        if args.refs_only:
            written = process_session_references(session)
            sheet = written.get("session_sheet")
            if sheet is not None:
                print(f"  wrote {sheet}")
            continue
        if args.rgb_only:
            rgb = write_session_rgb_all(session)
            if rgb is not None:
                print(f"  wrote {rgb}")
            continue
        sheets = write_session_qa_sheets(session)
        if not sheets:
            print("  (no preview sheets)")
            continue
        for path in sheets.values():
            print(f"  wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
