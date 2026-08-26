# Session QA collages of GSAM overlay.png and roi_spectra_plot.png (backend/offline).
from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PREFERRED_STREAMS = (
    ("reflectance", "fx10e"),
    ("reflectance", "swir3"),
    ("transmittance", "fx10e"),
    ("transmittance", "swir3"),
)
MASK_SHEET_NAME = "mask_overlaps.png"
SPECTRA_SHEET_NAME = "roi_spectra_plots.png"


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


def write_labeled_image_sheet(
    session: Path,
    panels: list[tuple[str, Path]],
    *,
    out_name: str,
    title: str,
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
    out = session / out_name
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


def write_session_qa_sheets(session: Path) -> dict[str, Path]:
    """Write mask_overlaps.png and roi_spectra_plots.png when source panels exist."""
    written: dict[str, Path] = {}
    mask = write_session_mask_overlaps(session)
    if mask is not None:
        written["mask_overlaps"] = mask
    spectra = write_session_roi_spectra_plots(session)
    if spectra is not None:
        written["roi_spectra_plots"] = spectra
    return written
