# Session reprocess: SWIR ref-BPR + FFC + GSAM plan (+ optional TX ROI20 manual mask).
from __future__ import annotations

import argparse
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from src.gsam_client import DEFAULT_GSAM_URL, require_gsam_ready, win_to_wsl
from src.gsam_plan_segment import segment_stream_from_plan
from src.gsam_segment_io import write_roi_csv
from src.utils.envi import parse_envi_hdr, read_bil_cube, write_bil_cube, write_envi_hdr

SESSION = Path(r"E:\2026_Grape_Data_Collection\2026_Geneva_Concord\Unripe_T1")
PLAN_PATH = Path(r"D:\Pototypy\HyperFusion\app\preset\gsam_plans\unripe_5by5_grape_tray.json")
STEM = "Unripe_T1"
CFG_PATH = Path(r"D:\Pototypy\HyperFusion\app\preset\hyperfusion.cfg")
ILLUMINANT_JSON = Path(
    r"D:\Pototypy\HyperFusion\app\src\backend\camera\processing\reference\D_illuminants.json"
)
BACKUP = SESSION / "_roi20_manual_backup"
XML_FX = Path(r"C:\Users\jy773\Downloads\fx10e_#20.xml")
XML_SW = Path(r"C:\Users\jy773\Downloads\swir_#20.xml")
GSAM_URL = DEFAULT_GSAM_URL
STREAMS = [
    ("reflectance", "fx10e"),
    ("reflectance", "swir3"),
    ("transmittance", "fx10e"),
    ("transmittance", "swir3"),
]


def neighbor_med(arr: np.ndarray, radius: int = 2) -> np.ndarray:
    # arr: (bands, samples). No wrap — matches SwirRefBprCorrector.
    samples = arr.shape[1]
    stacked = []
    for delta in range(-radius, radius + 1):
        if delta == 0:
            continue
        shifted = np.full(arr.shape, np.nan, dtype=np.float64)
        if delta < 0:
            shifted[:, -delta:] = arr[:, : samples + delta]
        else:
            shifted[:, : samples - delta] = arr[:, delta:]
        stacked.append(shifted)
    return np.nanmedian(np.stack(stacked, axis=0), axis=0)


def detect_bad_mask(white_bs: np.ndarray, dark_bs: np.ndarray) -> np.ndarray:
    wmed = neighbor_med(white_bs, 2)
    dmed = neighbor_med(dark_bs, 2)
    valid = np.isfinite(wmed) & np.isfinite(dmed)
    w_ratio = np.ones_like(white_bs, dtype=np.float64)
    w_ratio[valid] = (white_bs[valid] + 1.0) / (wmed[valid] + 1.0)
    d_diff = np.zeros_like(dark_bs, dtype=np.float64)
    d_diff[valid] = np.abs(dark_bs[valid] - dmed[valid])
    dark_thresh = max(40.0, 4.0 * float(np.median(d_diff[valid])) if valid.any() else 40.0)
    mask = np.zeros(white_bs.shape, dtype=bool)
    mask[valid] = (w_ratio[valid] > 1.12) | (w_ratio[valid] < 0.88) | (d_diff[valid] > dark_thresh)
    mask[:, mask.mean(axis=0) >= 0.25] = True
    return mask


def interpolate_spatial(plane: np.ndarray, bad_samples: np.ndarray) -> np.ndarray:
    good_idx = np.flatnonzero(~bad_samples)
    bad_idx = np.flatnonzero(bad_samples)
    if good_idx.size == 0 or bad_idx.size == 0:
        return plane
    pos = np.searchsorted(good_idx, bad_idx)
    left_i = np.clip(pos - 1, 0, good_idx.size - 1)
    right_i = np.clip(pos, 0, good_idx.size - 1)
    left = good_idx[left_i]
    right = good_idx[right_i]
    span = (right - left).astype(np.float32)
    span[span == 0] = 1.0
    w_right = (bad_idx.astype(np.float32) - left.astype(np.float32)) / span
    w_left = 1.0 - w_right
    out = plane.copy()
    out[..., bad_idx] = plane[..., left] * w_left.astype(plane.dtype) + plane[..., right] * w_right.astype(
        plane.dtype
    )
    return out


def apply_mask_to_cube(cube_lsb: np.ndarray, mask_bs: np.ndarray) -> np.ndarray:
    out = cube_lsb.copy()
    for band in range(mask_bs.shape[0]):
        out[:, :, band] = interpolate_spatial(out[:, :, band], mask_bs[band])
    return out


def ffc_cube(sample: np.ndarray, dark_row: np.ndarray, white_row: np.ndarray) -> np.ndarray:
    denom = np.maximum(white_row - dark_row, 1e-6)
    return np.clip((sample - dark_row[np.newaxis, ...]) / denom[np.newaxis, ...], 0.0, 1.0).astype(
        np.float32
    )


def stretch_channel(plane: np.ndarray) -> np.ndarray:
    lo = float(np.min(plane))
    hi = float(np.max(plane))
    scale = 255.0 / (hi - lo) if hi > lo else 1.0
    return np.clip((plane - lo) * scale, 0, 255).astype(np.uint8)


def false_color(cube: np.ndarray, wavelengths: np.ndarray, ranges: list[tuple[float, float]]) -> np.ndarray:
    channels = []
    for lo, hi in ranges:
        idx = np.where((wavelengths >= lo) & (wavelengths <= hi))[0]
        if idx.size == 0:
            raise RuntimeError(f"SWIR false-color range {lo}-{hi} nm matched no bands")
        plane = cube[:, :, idx].mean(axis=2)
        channels.append(stretch_channel(plane))
    return np.stack(channels, axis=2)


def fx_srgb(cube: np.ndarray, wavelengths: np.ndarray) -> np.ndarray:
    tables = json.loads(ILLUMINANT_JSON.read_text(encoding="utf-8"))
    cmf_wl = np.asarray(tables["wxyz_wavelength_nm"], dtype=np.float64)
    ill_wl = np.asarray(tables["D_wavelength_nm"], dtype=np.float64)
    xbar_src = np.asarray(tables["xbar"], dtype=np.float64)
    ybar_src = np.asarray(tables["ybar"], dtype=np.float64)
    zbar_src = np.asarray(tables["zbar"], dtype=np.float64)
    ill_src = np.asarray(tables["D65"], dtype=np.float64)

    wl = np.asarray(wavelengths, dtype=np.float64)
    keep = wl <= 780.0
    wl_t = wl[keep]
    cube_t = cube[:, :, keep]
    if wl_t.size < 2:
        raise RuntimeError("No FX10e bands remain after 780 nm truncate")

    weights = np.zeros_like(wl_t)
    dx = np.diff(wl_t)
    weights[:-1] += 0.5 * dx
    weights[1:] += 0.5 * dx
    xbar = np.interp(wl_t, cmf_wl, xbar_src)
    ybar = np.interp(wl_t, cmf_wl, ybar_src)
    zbar = np.interp(wl_t, cmf_wl, zbar_src)
    illum = np.interp(wl_t, ill_wl, ill_src)
    y_integral = float(np.sum(ybar * illum * weights))
    if y_integral <= 0.0:
        raise RuntimeError("Illuminant Y integral is zero")
    scale = (illum * weights) / y_integral
    x = np.tensordot(cube_t, xbar * scale, axes=([2], [0]))
    y = np.tensordot(cube_t, ybar * scale, axes=([2], [0]))
    z = np.tensordot(cube_t, zbar * scale, axes=([2], [0]))
    r_lin = np.clip(3.2404542 * x - 1.5371385 * y - 0.4985314 * z, 0.0, None)
    g_lin = np.clip(-0.9692660 * x + 1.8760108 * y + 0.0415560 * z, 0.0, None)
    b_lin = np.clip(0.0556434 * x - 0.2040259 * y + 1.0572252 * z, 0.0, None)

    def srgb_gamma(linear: np.ndarray) -> np.ndarray:
        return np.where(linear <= 0.0031308, 12.92 * linear, 1.055 * np.power(linear, 1.0 / 2.4) - 0.055)

    rgb = np.stack(
        [
            np.clip(srgb_gamma(r_lin), 0.0, 1.0),
            np.clip(srgb_gamma(g_lin), 0.0, 1.0),
            np.clip(srgb_gamma(b_lin), 0.0, 1.0),
        ],
        axis=2,
    )
    return np.rint(rgb * 255.0).astype(np.uint8)


def rasterize_envi_xml(xml_path: Path, width: int, height: int) -> np.ndarray:
    root = ET.parse(xml_path).getroot()
    text = root.find(".//{*}Coordinates")
    if text is None or not (text.text or "").strip():
        text = root.find(".//Coordinates")
    vals = [float(x) for x in (text.text or "").split()]
    pts = [(vals[i], vals[i + 1]) for i in range(0, len(vals) - 1, 2)]
    image = Image.new("L", (width, height), 0)
    ImageDraw.Draw(image).polygon(pts, fill=255)
    return (np.array(image) > 0).astype(np.uint8)


def write_hdr(meta, out_hdr: Path, cube: np.ndarray, description: str) -> None:
    meta.data_type = 4
    write_envi_hdr(
        out_hdr,
        meta,
        samples=cube.shape[1],
        lines=cube.shape[0],
        bands=cube.shape[2],
        raw_basename=out_hdr.with_suffix(".raw").name,
        description=description,
        wavelengths_nm=meta.wavelengths_nm,
    )
    write_bil_cube(out_hdr.with_suffix(".raw"), cube)


def process_stream(
    mode: str,
    camera: str,
    plan: dict,
    prompt_override: str | None = None,
    run_gsam: bool = True,
) -> None:
    capture = SESSION / mode / camera / "capture"
    sample_hdr = next(capture.glob(f"{STEM}_*.hdr"))
    white_hdr = next(capture.glob("WHITEREF_*.hdr"))
    dark_hdr = next(capture.glob("DARKREF_*.hdr"))
    sample_meta = parse_envi_hdr(sample_hdr)
    sample = read_bil_cube(sample_meta)
    white = read_bil_cube(parse_envi_hdr(white_hdr))
    dark = read_bil_cube(parse_envi_hdr(dark_hdr))
    white_row = white.mean(axis=0)
    dark_row = dark.mean(axis=0)

    if camera == "swir3":
        mask = detect_bad_mask(white_row.T, dark_row.T)
        white_row = apply_mask_to_cube(white_row[np.newaxis, ...], mask)[0]
        dark_row = apply_mask_to_cube(dark_row[np.newaxis, ...], mask)[0]
        sample = apply_mask_to_cube(sample, mask)
        print(f"  {mode}/{camera}: SWIR ref BPR columns={int((mask.mean(0) >= 0.25).sum())}")

    ffc = ffc_cube(sample, dark_row, white_row)
    out_dir = SESSION / mode / camera / "preprocessed"
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    ffc_hdr = out_dir / f"{STEM}_ffc.hdr"
    write_hdr(sample_meta, ffc_hdr, ffc, f"HyperFusion preprocessed {mode}")
    wl = np.array(sample_meta.wavelengths_nm, dtype=np.float64)

    if camera == "swir3":
        rgb = false_color(ffc, wl, [(1300, 1400), (1050, 1150), (980, 1050)])
    else:
        rgb = fx_srgb(ffc, wl)

    stream_key = f"{mode}/{camera}"
    stream_plan = plan["streams"][stream_key]
    rgb_path = out_dir / f"{STEM}_rgb.png"
    Image.fromarray(rgb, mode="RGB").save(rgb_path)
    if stream_plan.get("invert_rgb"):
        Image.fromarray(255 - rgb, mode="RGB").save(rgb_path)
        print(f"  {mode}/{camera}: inverted RGB for GSAM")

    if not run_gsam:
        print(f"  {mode}/{camera}: FFC/RGB written, GSAM deferred")
        return

    segment_stream_from_plan(
        SESSION,
        mode,
        camera,
        plan,
        stem=STEM,
        gsam_url=GSAM_URL,
        prompt_override=prompt_override,
    )


def _mask_iou(a: np.ndarray, b: np.ndarray) -> float:
    aa = a > 0
    bb = b > 0
    inter = np.logical_and(aa, bb).sum()
    union = np.logical_or(aa, bb).sum()
    return float(inter) / float(union) if union else 0.0


def _swap_roi_files(seg_dir: Path, roi_a: int, roi_b: int) -> None:
    pairs = [
        (seg_dir / "masks" / f"mask_{roi_a:03d}.npy", seg_dir / "masks" / f"mask_{roi_b:03d}.npy"),
        (seg_dir / "masks" / f"mask_{roi_a:03d}.png", seg_dir / "masks" / f"mask_{roi_b:03d}.png"),
        (
            seg_dir / "segmented_rgb" / f"roi_{roi_a:03d}.png",
            seg_dir / "segmented_rgb" / f"roi_{roi_b:03d}.png",
        ),
    ]
    tmp = seg_dir / "_swap_tmp.bin"
    for left, right in pairs:
        if not left.is_file() or not right.is_file():
            continue
        shutil.move(str(left), str(tmp))
        shutil.move(str(right), str(left))
        shutil.move(str(tmp), str(right))


def apply_manual_tx_roi20() -> None:
    for camera, xml_path in (("fx10e", XML_FX), ("swir3", XML_SW)):
        if not xml_path.is_file():
            xml_path = BACKUP / xml_path.name
        if not xml_path.is_file():
            print(f"  TX {camera}: manual XML missing ({XML_FX if camera=='fx10e' else XML_SW}), skip ROI20 restore")
            continue
        pre = SESSION / "transmittance" / camera / "preprocessed"
        rgb = np.array(Image.open(pre / f"{STEM}_rgb.png"))
        height, width = rgb.shape[:2]
        mask = rasterize_envi_xml(xml_path, width, height)
        pix = int(mask.sum())
        print(f"  TX {camera} manual #20 pixels={pix} size={width}x{height}")

        seg_dir = pre / "segmentation"
        man_path = seg_dir / "segmentation_results.json"
        man = json.loads(man_path.read_text(encoding="utf-8"))
        detections = man.get("detections", [])

        best_roi = 20
        best_iou = 0.0
        for det in detections:
            roi = int(det["roi"])
            path = seg_dir / "masks" / f"mask_{roi:03d}.npy"
            if not path.is_file():
                continue
            iou = _mask_iou(np.load(path), mask)
            if iou > best_iou:
                best_iou = iou
                best_roi = roi
        print(f"  TX {camera}: XML vs GSAM best IoU={best_iou:.3f} at ROI {best_roi}")
        if best_roi != 20 and best_iou >= 0.15:
            _swap_roi_files(seg_dir, 20, best_roi)
            for det in detections:
                roi = int(det["roi"])
                if roi == 20:
                    det["roi"] = best_roi
                elif roi == best_roi:
                    det["roi"] = 20
            detections.sort(key=lambda item: int(item["roi"]))
            print(f"  TX {camera}: swapped GSAM ROI {best_roi} with 20 before overlaying XML")

        npy_path = seg_dir / "masks" / "mask_020.npy"
        png_path = seg_dir / "masks" / "mask_020.png"
        np.save(npy_path, mask)
        Image.fromarray((mask * 255).astype(np.uint8), mode="L").save(png_path)

        ys, xs = np.nonzero(mask)
        bounds = {
            "x": int(xs.min()),
            "y": int(ys.min()),
            "width": int(xs.max() - xs.min() + 1),
            "height": int(ys.max() - ys.min() + 1),
        }
        crop = rgb[bounds["y"] : bounds["y"] + bounds["height"], bounds["x"] : bounds["x"] + bounds["width"]]
        alpha = mask[bounds["y"] : bounds["y"] + bounds["height"], bounds["x"] : bounds["x"] + bounds["width"]] * 255
        rgba = np.dstack((crop, alpha.astype(np.uint8)))
        (seg_dir / "segmented_rgb").mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgba, mode="RGBA").save(seg_dir / "segmented_rgb" / "roi_020.png")

        stack = np.zeros((height, width), dtype=np.uint8)
        for path in sorted((seg_dir / "masks").glob("mask_*.npy")):
            stack = np.maximum(stack, (np.load(path) > 0).astype(np.uint8) * 255)
        Image.fromarray(stack, mode="L").save(seg_dir / "masks" / "stack_mask.png")

        found = False
        for det in detections:
            if int(det.get("roi", 0)) == 20:
                det["pixel_count"] = pix
                det["label"] = "manual"
                det["score"] = 1.0
                det["segmented_rgb_bounds"] = bounds
                found = True
        if not found:
            detections.append(
                {
                    "roi": 20,
                    "label": "manual",
                    "score": 1.0,
                    "pixel_count": pix,
                    "mask_png": win_to_wsl(png_path),
                    "mask_npy": win_to_wsl(npy_path),
                    "segmented_rgb_png": win_to_wsl(seg_dir / "segmented_rgb" / "roi_020.png"),
                    "segmented_rgb_bounds": bounds,
                }
            )
            detections.sort(key=lambda item: int(item["roi"]))
        man["detections"] = detections
        man_path.write_text(json.dumps(man, indent=2), encoding="utf-8")

        cube = read_bil_cube(parse_envi_hdr(pre / f"{STEM}_ffc.hdr"))
        wl = np.array(parse_envi_hdr(pre / f"{STEM}_ffc.hdr").wavelengths_nm)
        write_roi_csv(seg_dir, f"{STEM}_rgb.png", cube, wl, detections)


def resegment_stream(
    mode: str,
    camera: str,
    plan: dict,
    *,
    max_dets: int | None = None,
    nms_iou: float | None = None,
    prompt_override: str | None = None,
    force_two_stage: bool = False,
    stage1_prompt: str | None = None,
    stage2_prompt: str | None = None,
) -> None:
    segment_stream_from_plan(
        SESSION,
        mode,
        camera,
        plan,
        stem=STEM,
        gsam_url=GSAM_URL,
        force_two_stage=force_two_stage,
        prompt_override=prompt_override,
        stage1_prompt=stage1_prompt,
        stage2_prompt=stage2_prompt,
        max_dets=max_dets,
        nms_iou=nms_iou,
    )


def refresh_rgb_from_ffc(mode: str, camera: str, invert: bool) -> Path:
    out_dir = SESSION / mode / camera / "preprocessed"
    ffc_hdr = out_dir / f"{STEM}_ffc.hdr"
    meta = parse_envi_hdr(ffc_hdr)
    cube = read_bil_cube(meta)
    wl = np.array(meta.wavelengths_nm, dtype=np.float64)
    if camera == "swir3":
        rgb = false_color(cube, wl, [(1300, 1400), (1050, 1150), (980, 1050)])
    else:
        rgb = fx_srgb(cube, wl)
    rgb_path = out_dir / f"{STEM}_rgb.png"
    if invert:
        rgb = 255 - rgb
        print(f"  {mode}/{camera}: inverted RGB")
    Image.fromarray(rgb, mode="RGB").save(rgb_path)
    print(f"  {mode}/{camera}: wrote RGB from FFC invert={invert}")
    return rgb_path


def session_refresh(plan: dict, restore_tx_roi20: bool) -> None:
    print("GSAM health", require_gsam_ready(GSAM_URL))

    for mode in ("reflectance", "transmittance"):
        fusion = SESSION / mode / "fusion"
        if fusion.exists():
            shutil.rmtree(fusion)
            print(f"removed {mode}/fusion")

    print("== transmittance RGB+GSAM ==")
    for camera in ("fx10e", "swir3"):
        stream = plan["streams"][f"transmittance/{camera}"]
        refresh_rgb_from_ffc("transmittance", camera, bool(stream.get("invert_rgb")))
        raw = SESSION / "transmittance" / camera / "preprocessed" / "segmentation" / "masks_gsam_raw"
        if raw.exists():
            shutil.rmtree(raw)
        resegment_stream("transmittance", camera, plan)
    if restore_tx_roi20:
        print("== apply transmittance ROI 20 manual masks ==")
        apply_manual_tx_roi20()
    print("done session refresh")


def main() -> None:
    global SESSION, PLAN_PATH, STEM, BACKUP
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, default=SESSION)
    parser.add_argument("--plan", type=Path, default=PLAN_PATH)
    parser.add_argument("--stem", default=None)
    parser.add_argument("--manual-tx-roi20", action="store_true")
    parser.add_argument("--only", default=None, help="mode/camera: wipe and re-FFC that stream only")
    parser.add_argument("--resegment", default=None, help="mode/camera, keep existing FFC/RGB")
    parser.add_argument(
        "--two-stage",
        action="store_true",
        help="force two-stage even if the plan stream has no two_stage block",
    )
    parser.add_argument("--stage1-prompt", default=None, help="override plan two_stage.stage1_prompt")
    parser.add_argument("--stage2-prompt", default=None, help="override plan two_stage.stage2_prompt")
    parser.add_argument("--prompt", default=None, help="override GSAM prompt")
    parser.add_argument("--gsam-max-dets", type=int, default=None, help="override plan two_stage.max_dets / stage1 max_dets")
    parser.add_argument("--nms-iou", type=float, default=None, help="override plan nms_iou")
    parser.add_argument("--session-refresh", action="store_true")
    args = parser.parse_args()
    SESSION = args.session
    PLAN_PATH = args.plan
    STEM = args.stem or SESSION.name
    BACKUP = SESSION / "_roi20_manual_backup"

    plan = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
    if args.session_refresh:
        session_refresh(plan, restore_tx_roi20=args.manual_tx_roi20 or STEM.lower().startswith("unripe"))
        return

    if args.only:
        print("GSAM health", require_gsam_ready(GSAM_URL))
        mode, camera = args.only.split("/", 1)
        pre = SESSION / mode / camera / "preprocessed"
        if pre.exists():
            shutil.rmtree(pre)
        fusion = SESSION / mode / "fusion"
        if fusion.exists():
            shutil.rmtree(fusion)
        print(f"removed old {mode}/{camera} preprocessed (+ {mode} fusion)")
        print(f"== {mode}/{camera} ==")
        process_stream(mode, camera, plan, prompt_override=args.prompt, run_gsam=False)
        resegment_stream(
            mode,
            camera,
            plan,
            max_dets=args.gsam_max_dets,
            nms_iou=args.nms_iou,
            prompt_override=args.prompt,
            force_two_stage=args.two_stage,
            stage1_prompt=args.stage1_prompt,
            stage2_prompt=args.stage2_prompt,
        )
        print("done FFC/GSAM")
        return

    if args.resegment:
        print("GSAM health", require_gsam_ready(GSAM_URL))
        mode, camera = args.resegment.split("/", 1)
        print(f"== resegment {mode}/{camera} ==")
        resegment_stream(
            mode,
            camera,
            plan,
            max_dets=args.gsam_max_dets,
            nms_iou=args.nms_iou,
            prompt_override=args.prompt,
            force_two_stage=args.two_stage,
            stage1_prompt=args.stage1_prompt,
            stage2_prompt=args.stage2_prompt,
        )
        print("done resegment")
        return

    if args.manual_tx_roi20:
        BACKUP.mkdir(parents=True, exist_ok=True)
        for camera in ("fx10e", "swir3"):
            src = SESSION / "transmittance" / camera / "preprocessed" / "segmentation" / "masks" / "mask_020.npy"
            if src.is_file():
                shutil.copy2(src, BACKUP / f"{camera}_mask_020.npy")
                png = src.with_suffix(".png")
                if png.is_file():
                    shutil.copy2(png, BACKUP / f"{camera}_mask_020.png")
        if XML_FX.is_file():
            shutil.copy2(XML_FX, BACKUP / XML_FX.name)
        if XML_SW.is_file():
            shutil.copy2(XML_SW, BACKUP / XML_SW.name)

    wiped_fusion: set[str] = set()
    for mode, camera in STREAMS:
        pre = SESSION / mode / camera / "preprocessed"
        if pre.exists():
            shutil.rmtree(pre)
        if mode not in wiped_fusion:
            fusion = SESSION / mode / "fusion"
            if fusion.exists():
                shutil.rmtree(fusion)
            wiped_fusion.add(mode)
        print(f"removed old {mode}/{camera} preprocessed (+ fusion)")

    print("GSAM health", require_gsam_ready(GSAM_URL))

    for mode, camera in STREAMS:
        print(f"== {mode}/{camera} ==")
        process_stream(mode, camera, plan, prompt_override=args.prompt, run_gsam=False)
        resegment_stream(
            mode,
            camera,
            plan,
            max_dets=args.gsam_max_dets,
            nms_iou=args.nms_iou,
            prompt_override=args.prompt,
            force_two_stage=args.two_stage,
            stage1_prompt=args.stage1_prompt,
            stage2_prompt=args.stage2_prompt,
        )

    if args.manual_tx_roi20:
        print("== apply transmittance ROI 20 manual masks ==")
        apply_manual_tx_roi20()
    print("done FFC/GSAM")


if __name__ == "__main__":
    main()
