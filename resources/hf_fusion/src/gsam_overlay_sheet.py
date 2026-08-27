# Session QA collages written to {session}/preview/ (backend/offline).
# FFC-only: rgb_all + reference_intensity. GSAM also adds mask_overlaps + roi_spectra_plots.
# Per-stream ROI spectra files stay in preprocessed/segmentation/.
from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PREFERRED_STREAMS = (
    ("reflectance", "fx10e"),
    ("reflectance", "swir3"),
    ("transmittance", "fx10e"),
    ("transmittance", "swir3"),
)
PREVIEW_DIR_NAME = "preview"
MASK_SHEET_NAME = "mask_overlaps.png"
SPECTRA_SHEET_NAME = "roi_spectra_plots.png"
REF_SHEET_NAME = "reference_intensity.png"
RGB_SHEET_NAME = "rgb_all.png"
QA_SHEET_NAMES = (
    MASK_SHEET_NAME,
    SPECTRA_SHEET_NAME,
    REF_SHEET_NAME,
    RGB_SHEET_NAME,
)


def session_preview_dir(session: Path) -> Path:
    dest = session / PREVIEW_DIR_NAME
    dest.mkdir(parents=True, exist_ok=True)
    return dest


def migrate_legacy_session_qa_pngs(session: Path) -> None:
    """Move leftover session-root QA PNGs into preview/ (do not leave duplicates)."""
    dest = session_preview_dir(session)
    for name in QA_SHEET_NAMES:
        src = session / name
        if not src.is_file():
            continue
        target = dest / name
        if target.exists():
            src.unlink()
        else:
            src.replace(target)


def preview_sheet_paths(session: Path) -> dict[str, str]:
    preview = session / PREVIEW_DIR_NAME
    mapping = {
        "mask_overlaps": MASK_SHEET_NAME,
        "roi_spectra_plots": SPECTRA_SHEET_NAME,
        "reference_intensity": REF_SHEET_NAME,
        "rgb_all": RGB_SHEET_NAME,
    }
    return {
        key: str(preview / name) if (preview / name).is_file() else ""
        for key, name in mapping.items()
    }


def _stream_label(session: Path, path: Path) -> str:
    try:
        relative = path.relative_to(session)
        parts = relative.parts
        if len(parts) >= 2:
            return f"{parts[0]} / {parts[1]}"
    except ValueError:
        pass
    return path.parent.parent.name


def _collect_stream_artifacts(session: Path, relative_under_stream: str) -> list[tuple[str, Path]]:
    """Collect one PNG per mode/camera under preprocessed/, preferred order first."""
    found: dict[str, Path] = {}
    pattern = f"**/preprocessed/{relative_under_stream}"
    for path in session.glob(pattern):
        if path.is_file():
            found[_stream_label(session, path)] = path
    ordered: list[tuple[str, Path]] = []
    used: set[str] = set()
    for mode, camera in PREFERRED_STREAMS:
        label = f"{mode} / {camera}"
        path = found.get(label)
        if path is None:
            path = session / mode / camera / "preprocessed" / Path(relative_under_stream)
        if path.is_file():
            ordered.append((label, path))
            used.add(label)
    for label, path in sorted(found.items()):
        if label not in used:
            ordered.append((label, path))
    return ordered


def collect_final_overlays(session: Path) -> list[tuple[str, Path]]:
    return _collect_stream_artifacts(session, "segmentation/overlay.png")


def collect_roi_spectra_plots(session: Path) -> list[tuple[str, Path]]:
    return _collect_stream_artifacts(session, "segmentation/roi_spectra_plot.png")


def collect_rgb_previews(session: Path) -> list[tuple[str, Path]]:
    panels: list[tuple[str, Path]] = []
    for mode, camera in PREFERRED_STREAMS:
        pre = session / mode / camera / "preprocessed"
        matches = sorted(pre.glob("*_rgb.png"))
        if matches:
            panels.append((f"{mode} / {camera}", matches[0]))
    return panels


def write_labeled_image_sheet(
    session: Path,
    panels: list[tuple[str, Path]],
    *,
    out_name: str,
    title: str,
    cols: int | None = None,
) -> Path | None:
    if not panels:
        return None

    images = [(label, Image.open(path).convert("RGB")) for label, path in panels]
    cell_w = max(im.width for _, im in images)
    cell_h = max(im.height for _, im in images)
    label_h = 36
    pad = 12
    header_h = 48
    count = len(images)
    if cols is None:
        cols = 1 if count == 1 else 2
    rows = (count + cols - 1) // cols
    canvas_w = pad + cols * (cell_w + pad)
    canvas_h = header_h + pad + rows * (label_h + cell_h + pad)
    canvas = Image.new("RGB", (canvas_w, canvas_h), (18, 18, 18))
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("arial.ttf", 22)
        font_h = ImageFont.truetype("arial.ttf", 28)
    except OSError:
        font = ImageFont.load_default()
        font_h = font
    draw.text((pad, 10), f"{session.name}  {title}", fill=(255, 255, 255), font=font_h)
    for i, (label, im) in enumerate(images):
        r, c = divmod(i, cols)
        x0 = pad + c * (cell_w + pad)
        y0 = header_h + pad + r * (label_h + cell_h + pad)
        scale = min(cell_w / im.width, cell_h / im.height)
        nw, nh = max(1, int(im.width * scale)), max(1, int(im.height * scale))
        fitted = im.resize((nw, nh), Image.Resampling.LANCZOS)
        ox = x0 + (cell_w - nw) // 2
        oy = y0 + label_h + (cell_h - nh) // 2
        draw.text((x0, y0 + 6), label, fill=(230, 230, 230), font=font)
        canvas.paste(fitted, (ox, oy))
    out = session_preview_dir(session) / out_name
    canvas.save(out, optimize=True)
    return out


def write_session_mask_overlaps(session: Path) -> Path | None:
    return write_labeled_image_sheet(
        session,
        collect_final_overlays(session),
        out_name=MASK_SHEET_NAME,
        title="GSAM final overlays",
    )


def write_session_roi_spectra_plots(session: Path) -> Path | None:
    return write_labeled_image_sheet(
        session,
        collect_roi_spectra_plots(session),
        out_name=SPECTRA_SHEET_NAME,
        title="GSAM ROI spectra",
    )


def write_session_rgb_all(session: Path) -> Path | None:
    return write_labeled_image_sheet(
        session,
        collect_rgb_previews(session),
        out_name=RGB_SHEET_NAME,
        title="RGB previews",
        cols=2,
    )


def write_collection_image_sheet(
    collection: Path,
    session_sheets: list[Path],
    *,
    out_name: str,
    title: str,
    max_width: int = 1600,
) -> Path | None:
    if not session_sheets:
        return None
    images = [(path.parent.name, Image.open(path).convert("RGB")) for path in session_sheets]
    scaled: list[tuple[str, Image.Image]] = []
    for label, im in images:
        if im.width > max_width:
            nh = max(1, int(im.height * max_width / im.width))
            im = im.resize((max_width, nh), Image.Resampling.LANCZOS)
        scaled.append((label, im))
    images = scaled
    cell_w = max(im.width for _, im in images)
    cell_h = max(im.height for _, im in images)
    label_h = 32
    pad = 10
    header_h = 44
    canvas_w = pad + cell_w + pad
    canvas_h = header_h + pad + len(images) * (label_h + cell_h + pad)
    canvas = Image.new("RGB", (canvas_w, canvas_h), (18, 18, 18))
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("arial.ttf", 20)
        font_h = ImageFont.truetype("arial.ttf", 26)
    except OSError:
        font = ImageFont.load_default()
        font_h = font
    draw.text((pad, 8), f"{collection.name}  {title}", fill=(255, 255, 255), font=font_h)
    for i, (label, im) in enumerate(images):
        y0 = header_h + pad + i * (label_h + cell_h + pad)
        scale = min(cell_w / im.width, cell_h / im.height)
        nw, nh = max(1, int(im.width * scale)), max(1, int(im.height * scale))
        fitted = im.resize((nw, nh), Image.Resampling.LANCZOS)
        draw.text((pad, y0 + 4), label, fill=(230, 230, 230), font=font)
        canvas.paste(fitted, (pad, y0 + label_h))
    out = collection / out_name
    canvas.save(out, optimize=True)
    return out


def write_session_qa_sheets(session: Path) -> dict[str, Path]:
    """Write session preview sheets under {session}/preview/.

    Always writes rgb_all and reference_intensity when those sources exist.
    mask_overlaps and roi_spectra_plots are written only when GSAM outputs exist.
    Per-stream ROI spectra files stay in preprocessed/segmentation/.
    """
    migrate_legacy_session_qa_pngs(session)
    written: dict[str, Path] = {}
    rgb = write_session_rgb_all(session)
    if rgb is not None:
        written["rgb_all"] = rgb
    try:
        from src.reference_qa import write_session_reference_intensity

        ref = write_session_reference_intensity(session)
        if ref is not None:
            written["reference_intensity"] = ref
    except Exception:
        pass
    mask = write_session_mask_overlaps(session)
    if mask is not None:
        written["mask_overlaps"] = mask
    spectra = write_session_roi_spectra_plots(session)
    if spectra is not None:
        written["roi_spectra_plots"] = spectra
    return written
