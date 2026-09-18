# Plan-driven GSAM: single-prompt NMS, two-stage hole-box → object crop, or reuse masks
# from another stream (typically transmittance ← reflectance, same camera).
# Stage 2 runs inside each stage-1 DINO box crop. Do not AND with the stage-1 hole mask.
# If no stage-2 box is above stage2_box_threshold, keep the highest-confidence candidate.
from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from src.gsam_client import post_json, win_to_wsl
from src.gsam_overlay_sheet import write_session_qa_sheets
from src.gsam_segment_io import (
    mask_bbox_xyxy,
    nms_keep_indices,
    rewrite_segmentation_from_masks,
    z_order_sort_indices,
)
from src.utils.envi import parse_envi_hdr, read_bil_cube

DEFAULT_NMS_IOU = 0.40
DEFAULT_STAGE1_PROMPT = "hole"
DEFAULT_STAGE2_BOX_THRESHOLD = 0.20
# Query DINO below the plan threshold so below-threshold boxes still exist for max-confidence fallback.
STAGE2_QUERY_BOX_FLOOR = 0.01
DEFAULT_MAX_DETS = 50


@dataclass(frozen=True)
class StreamSegmentResult:
    stream: str
    detection_count: int
    two_stage: bool
    prompt: str
    segmentation_dir: Path
    reused_from: str = ""


def load_gsam_plan(path: Path) -> dict:
    plan = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(plan, dict) or not isinstance(plan.get("streams"), dict):
        raise ValueError(f"GSAM plan missing streams object: {path}")
    return plan


def two_stage_settings(stream_plan: dict) -> dict | None:
    ts = stream_plan.get("two_stage")
    if ts is True:
        return {}
    if not isinstance(ts, dict):
        return None
    if ts.get("enabled") is False:
        return None
    return ts


def resolve_reuse_masks_from(stream_plan: dict, mode: str, camera: str) -> str | None:
    """Return source stream key (mode/camera) when this stream should copy masks, else None."""
    explicit = stream_plan.get("reuse_masks_from")
    if isinstance(explicit, str) and explicit.strip():
        key = explicit.strip().replace("\\", "/").lower()
        if "/" not in key:
            raise ValueError(f"reuse_masks_from must be mode/camera, got {explicit!r}")
        return key
    if stream_plan.get("reuse_reflectance_masks") is True:
        if mode.strip().lower() != "transmittance":
            raise ValueError(
                f"reuse_reflectance_masks only applies to transmittance streams, not {mode}/{camera}"
            )
        return f"reflectance/{camera.strip().lower()}"
    return None


def stream_needs_gsam(stream_plan: dict, mode: str, camera: str) -> bool:
    return resolve_reuse_masks_from(stream_plan, mode, camera) is None


def _resize_mask(mask: np.ndarray, height: int, width: int) -> np.ndarray:
    if mask.shape[0] == height and mask.shape[1] == width:
        return (mask > 0).astype(np.uint8)
    img = Image.fromarray((mask > 0).astype(np.uint8) * 255, mode="L")
    resized = img.resize((width, height), resample=Image.Resampling.NEAREST)
    return (np.array(resized) > 0).astype(np.uint8)


def _load_ordered_source_masks(source_seg: Path) -> tuple[list[np.ndarray], list[float], list[str]]:
    man_path = source_seg / "segmentation_results.json"
    if not man_path.is_file():
        raise FileNotFoundError(f"Missing source segmentation_results.json: {man_path}")
    man = json.loads(man_path.read_text(encoding="utf-8"))
    dets = list(man.get("detections") or [])
    if not dets:
        raise RuntimeError(f"No detections in {man_path}")
    dets.sort(key=lambda det: int(det["roi"]))
    masks: list[np.ndarray] = []
    scores: list[float] = []
    labels: list[str] = []
    for det in dets:
        roi = int(det["roi"])
        path = source_seg / "masks" / f"mask_{roi:03d}.npy"
        if not path.is_file():
            raise FileNotFoundError(f"Missing source mask: {path}")
        masks.append(np.load(path) > 0)
        scores.append(float(det.get("score", 1.0)))
        labels.append(str(det.get("label") or "reused"))
    return masks, scores, labels


def resolve_stream_paths(session: Path, mode: str, camera: str, stem: str) -> tuple[Path, Path, Path]:
    pre = session / mode / camera / "preprocessed"
    rgb = pre / f"{stem}_rgb.png"
    if not rgb.is_file():
        matches = sorted(pre.glob("*_rgb.png"))
        if not matches:
            raise FileNotFoundError(f"No RGB PNG under {pre}")
        rgb = matches[0]
    ffc = pre / f"{stem}_ffc.hdr"
    if not ffc.is_file():
        matches = sorted(pre.glob("*_ffc.hdr"))
        if not matches:
            raise FileNotFoundError(f"No FFC HDR under {pre}")
        ffc = matches[0]
    return rgb, ffc, pre / "segmentation"


def segment_stream_reuse_masks(
    session: Path,
    mode: str,
    camera: str,
    plan: dict,
    *,
    stem: str,
    source_stream: str,
) -> StreamSegmentResult:
    stream_key = f"{mode}/{camera}"
    source_key = source_stream.strip().replace("\\", "/").lower()
    if "/" not in source_key:
        raise ValueError(f"Invalid reuse source stream: {source_stream}")
    src_mode, src_camera = source_key.split("/", 1)
    source_seg = session / src_mode / src_camera / "preprocessed" / "segmentation"
    if not source_seg.is_dir():
        raise FileNotFoundError(
            f"{stream_key}: reuse source missing ({source_key}). Segment {source_key} first."
        )

    rgb_path, ffc_hdr, seg_dir = resolve_stream_paths(session, mode, camera, stem)
    cube = read_bil_cube(parse_envi_hdr(ffc_hdr))
    wl = np.array(parse_envi_hdr(ffc_hdr).wavelengths_nm)
    rgb = np.array(Image.open(rgb_path).convert("RGB"))
    height, width = rgb.shape[:2]

    src_masks, scores, labels = _load_ordered_source_masks(source_seg)
    resized_note = ""
    if src_masks and (src_masks[0].shape[0] != height or src_masks[0].shape[1] != width):
        resized_note = f" (resized {src_masks[0].shape[1]}x{src_masks[0].shape[0]} -> {width}x{height})"
    masks = [_resize_mask(mask, height, width) for mask in src_masks]

    prompt = f"reused from {source_key}"
    print(f"  reuse {stream_key}: copy {len(masks)} masks from {source_key}{resized_note}")
    rewrite_segmentation_from_masks(
        seg_dir,
        rgb,
        masks,
        scores,
        labels,
        rgb_path.name,
        prompt,
        cube,
        wl,
    )
    (seg_dir / "reused_from.txt").write_text(f"{source_key}\n", encoding="utf-8")
    print(f"  wrote {len(masks)} reused ROIs (no GSAM)")
    return StreamSegmentResult(stream_key, len(masks), False, prompt, seg_dir, reused_from=source_key)


def _pick_stage2_mask(
    crop_seg: Path,
    crop_hw: tuple[int, int],
    min_score: float,
) -> tuple[np.ndarray | None, str]:
    man_path = crop_seg / "segmentation_results.json"
    if not man_path.is_file():
        return None, "no-manifest"
    man = json.loads(man_path.read_text(encoding="utf-8"))
    dets = man.get("detections") or []
    if not dets:
        return None, "no-det"
    crop_area = int(crop_hw[0] * crop_hw[1])
    ranked = []
    for det in dets:
        roi = int(det["roi"])
        path = crop_seg / "masks" / f"mask_{roi:03d}.npy"
        if not path.is_file():
            continue
        mask = np.load(path) > 0
        n = int(mask.sum())
        if n == 0:
            continue
        ranked.append((float(det.get("score", 0.0)), n, mask))
    if not ranked:
        return None, "empty"
    ranked.sort(key=lambda item: item[0], reverse=True)
    above = [item for item in ranked if item[0] >= min_score]
    if above:
        compact = [item for item in above if item[1] < 0.70 * crop_area]
        chosen = compact[0] if compact else above[0]
        note = f"score={chosen[0]:.3f} n={chosen[1]}"
        if not compact:
            note += " fallback-large"
        return chosen[2], note
    chosen = ranked[0]
    return chosen[2], f"score={chosen[0]:.3f} n={chosen[1]} below-thr-max"


def segment_stream_single(
    session: Path,
    mode: str,
    camera: str,
    plan: dict,
    *,
    stem: str,
    gsam_url: str,
    prompt_override: str | None = None,
    max_dets_override: int | None = None,
    nms_iou_override: float | None = None,
) -> StreamSegmentResult:
    stream_key = f"{mode}/{camera}"
    stream_plan = plan["streams"][stream_key]
    rgb_path, ffc_hdr, seg_dir = resolve_stream_paths(session, mode, camera, stem)
    cube = read_bil_cube(parse_envi_hdr(ffc_hdr))
    wl = np.array(parse_envi_hdr(ffc_hdr).wavelengths_nm)
    rgb = np.array(Image.open(rgb_path).convert("RGB"))
    keep_count = int(plan.get("sample_count", 25))
    nms_iou = float(stream_plan["nms_iou"]) if "nms_iou" in stream_plan else DEFAULT_NMS_IOU
    if nms_iou_override is not None:
        nms_iou = float(nms_iou_override)
    max_dets = int(max_dets_override) if max_dets_override is not None else max(DEFAULT_MAX_DETS, keep_count)
    prompt = prompt_override if prompt_override else str(stream_plan["prompt"])
    body = {
        "input_rgb": win_to_wsl(rgb_path),
        "out_dir": win_to_wsl(seg_dir),
        "image_name": rgb_path.name,
        "prompt": prompt,
        "max_dets": max(max_dets, keep_count),
        "box_threshold": float(stream_plan.get("box_threshold", 0.30)),
    }
    if float(stream_plan.get("max_box_area_frac", 0) or 0) > 0:
        body["max_box_area_frac"] = float(stream_plan["max_box_area_frac"])
    print(f"  single-stage {stream_key}: prompt={body['prompt']!r} max_dets={body['max_dets']} nms={nms_iou}")
    result = post_json(gsam_url, "/segment", body)
    if not result.get("ok"):
        raise RuntimeError(f"GSAM failed for {stream_key}: {result}")

    man = json.loads((seg_dir / "segmentation_results.json").read_text(encoding="utf-8"))
    raw = []
    for det in man.get("detections", []):
        roi = int(det["roi"])
        mask = np.load(seg_dir / "masks" / f"mask_{roi:03d}.npy")
        raw.append((mask, float(det.get("score", 0.0)), str(det.get("label", ""))))
    print(f"  raw detections={len(raw)}")
    keep = nms_keep_indices([m for m, _, _ in raw], [s for _, s, _ in raw], nms_iou)
    kept = [raw[i] for i in keep]
    print(f"  after NMS={len(kept)}")
    if len(kept) > keep_count:
        kept.sort(key=lambda item: item[1], reverse=True)
        kept = kept[:keep_count]
    if len(kept) < keep_count:
        raise RuntimeError(f"{stream_key}: {len(kept)} unique masks after NMS, need {keep_count}")

    boxes = []
    for mask, _, _ in kept:
        ys, xs = np.nonzero(mask > 0)
        boxes.append([float(xs.min()), float(ys.min()), float(xs.max()) + 1, float(ys.max()) + 1])
    order = z_order_sort_indices(np.asarray(boxes, dtype=np.float32))
    ordered = [kept[i] for i in order]
    rewrite_segmentation_from_masks(
        seg_dir,
        rgb,
        [m for m, _, _ in ordered],
        [s for _, s, _ in ordered],
        [lab for _, _, lab in ordered],
        rgb_path.name,
        prompt,
        cube,
        wl,
    )
    print(f"  wrote {keep_count} z-ordered unique ROIs")
    return StreamSegmentResult(stream_key, keep_count, False, prompt, seg_dir)


def segment_stream_two_stage(
    session: Path,
    mode: str,
    camera: str,
    plan: dict,
    *,
    stem: str,
    gsam_url: str,
    stage1_prompt: str | None = None,
    stage2_prompt: str | None = None,
    max_dets_override: int | None = None,
    nms_iou_override: float | None = None,
) -> StreamSegmentResult:
    stream_key = f"{mode}/{camera}"
    stream_plan = plan["streams"][stream_key]
    ts = two_stage_settings(stream_plan) or {}
    rgb_path, ffc_hdr, seg_dir = resolve_stream_paths(session, mode, camera, stem)
    cube = read_bil_cube(parse_envi_hdr(ffc_hdr))
    wl = np.array(parse_envi_hdr(ffc_hdr).wavelengths_nm)
    rgb = np.array(Image.open(rgb_path).convert("RGB"))
    height, width = rgb.shape[:2]
    keep_count = int(plan.get("sample_count", 25))
    prompt1 = stage1_prompt or str(ts.get("stage1_prompt") or DEFAULT_STAGE1_PROMPT)
    prompt2 = stage2_prompt or str(ts.get("stage2_prompt") or stream_plan.get("prompt") or "green ball")
    nms_iou = float(ts["nms_iou"]) if "nms_iou" in ts else float(stream_plan.get("nms_iou", DEFAULT_NMS_IOU))
    if nms_iou_override is not None:
        nms_iou = float(nms_iou_override)
    max_dets = int(ts["max_dets"]) if "max_dets" in ts else max(DEFAULT_MAX_DETS, keep_count)
    if max_dets_override is not None:
        max_dets = int(max_dets_override)
    crop_margin_px = int(ts.get("crop_margin_px", 0))
    stage2_box_threshold = float(ts.get("stage2_box_threshold", DEFAULT_STAGE2_BOX_THRESHOLD))
    print(f"  two-stage {stream_key}: DINO '{prompt1}' boxes patchify '{prompt2}'")

    body = {
        "input_rgb": win_to_wsl(rgb_path),
        "out_dir": win_to_wsl(seg_dir),
        "image_name": rgb_path.name,
        "prompt": prompt1,
        "max_dets": max(max_dets, keep_count),
        "box_threshold": float(stream_plan.get("box_threshold", 0.30)),
    }
    if float(stream_plan.get("max_box_area_frac", 0) or 0) > 0:
        body["max_box_area_frac"] = float(stream_plan["max_box_area_frac"])
    result = post_json(gsam_url, "/segment", body)
    if not result.get("ok"):
        raise RuntimeError(f"GSAM stage1 failed for {stream_key}: {result}")
    if (seg_dir / "overlay.png").is_file():
        shutil.copy2(seg_dir / "overlay.png", seg_dir / "overlay_stage1_holes.png")

    raw = []
    for det in result.get("detections") or []:
        roi = int(det["roi"])
        mask = np.load(seg_dir / "masks" / f"mask_{roi:03d}.npy") > 0
        box = det.get("box_xyxy")
        if not box:
            bb = mask_bbox_xyxy(mask)
            if bb is None:
                continue
            box = [float(v) for v in bb]
        raw.append((mask, float(det.get("score", 0.0)), str(det.get("label", "")), np.asarray(box, dtype=np.float32)))
    box_src = "yes" if (result.get("detections") or [{}])[0].get("box_xyxy") else "mask-bbox-fallback"
    print(f"  stage1 raw={len(raw)} box_xyxy={box_src}")
    keep = nms_keep_indices([m for m, _, _, _ in raw], [s for _, s, _, _ in raw], nms_iou)
    kept = [raw[i] for i in keep]
    print(f"  stage1 after NMS={len(kept)}")
    if len(kept) > keep_count:
        kept.sort(key=lambda item: item[1], reverse=True)
        kept = kept[:keep_count]
    if len(kept) < keep_count:
        raise RuntimeError(f"{stream_key}: {len(kept)} unique hole boxes after NMS, need {keep_count}")
    order = z_order_sort_indices(np.stack([item[3] for item in kept], axis=0))
    kept = [kept[i] for i in order]

    tmp_root = seg_dir / "_stage2_tmp"
    if tmp_root.exists():
        shutil.rmtree(tmp_root)
    tmp_root.mkdir()

    final_masks: list[np.ndarray] = []
    scores: list[float] = []
    labels: list[str] = []
    for roi, (hole, score, _lab, box) in enumerate(kept, start=1):
        x0 = int(np.floor(box[0])) - crop_margin_px
        y0 = int(np.floor(box[1])) - crop_margin_px
        x1 = int(np.ceil(box[2])) + crop_margin_px
        y1 = int(np.ceil(box[3])) + crop_margin_px
        x0 = max(0, x0)
        y0 = max(0, y0)
        x1 = min(width, max(x0 + 1, x1))
        y1 = min(height, max(y0 + 1, y1))
        crop = rgb[y0:y1, x0:x1]
        crop_dir = tmp_root / f"roi_{roi:03d}"
        crop_dir.mkdir()
        crop_rgb = crop_dir / "crop.png"
        Image.fromarray(crop, mode="RGB").save(crop_rgb)
        stage2_body = {
            "input_rgb": win_to_wsl(crop_rgb),
            "out_dir": win_to_wsl(crop_dir / "seg"),
            "image_name": crop_rgb.name,
            "prompt": prompt2,
            "max_dets": 8,
            "box_threshold": min(STAGE2_QUERY_BOX_FLOOR, float(stage2_box_threshold)),
            "max_box_area_frac": 0.90,
        }
        stage2_result = post_json(gsam_url, "/segment", stage2_body)
        stage2, note = _pick_stage2_mask(
            crop_dir / "seg",
            crop.shape[:2],
            min_score=float(stage2_box_threshold),
        )
        full = np.zeros((height, width), dtype=bool)
        if stage2 is None or not stage2_result.get("ok"):
            full[y0:y1, x0:x1] = hole[y0:y1, x0:x1]
            print(f"  ROI {roi:02d}: stage2 miss ({note}), hole inside box n={int(full.sum())}")
        else:
            full[y0:y1, x0:x1] = stage2
            print(f"  ROI {roi:02d}: box {x1 - x0}x{y1 - y0} hole {int(hole.sum())} -> ball {int(full.sum())} ({note})")
        final_masks.append(full.astype(np.uint8))
        scores.append(float(score))
        labels.append(prompt2)

    shutil.rmtree(tmp_root, ignore_errors=True)
    combined_prompt = f"{prompt1} box -> {prompt2}"
    rewrite_segmentation_from_masks(
        seg_dir,
        rgb,
        final_masks,
        scores,
        labels,
        rgb_path.name,
        combined_prompt,
        cube,
        wl,
    )
    print(f"  wrote two-stage {keep_count} ROIs from stage1 grounding boxes")
    return StreamSegmentResult(stream_key, keep_count, True, combined_prompt, seg_dir)


def segment_stream_from_plan(
    session: Path,
    mode: str,
    camera: str,
    plan: dict,
    *,
    stem: str,
    gsam_url: str,
    force_two_stage: bool = False,
    prompt_override: str | None = None,
    stage1_prompt: str | None = None,
    stage2_prompt: str | None = None,
    max_dets: int | None = None,
    nms_iou: float | None = None,
) -> StreamSegmentResult:
    stream_key = f"{mode}/{camera}"
    if stream_key not in plan["streams"]:
        raise KeyError(f"Plan has no stream {stream_key}")
    stream_plan = plan["streams"][stream_key]
    reuse_from = resolve_reuse_masks_from(stream_plan, mode, camera)
    if reuse_from is not None:
        result = segment_stream_reuse_masks(
            session,
            mode,
            camera,
            plan,
            stem=stem,
            source_stream=reuse_from,
        )
    else:
        use_two_stage = force_two_stage or two_stage_settings(stream_plan) is not None
        if use_two_stage:
            result = segment_stream_two_stage(
                session,
                mode,
                camera,
                plan,
                stem=stem,
                gsam_url=gsam_url,
                stage1_prompt=stage1_prompt,
                stage2_prompt=stage2_prompt or prompt_override,
                max_dets_override=max_dets,
                nms_iou_override=nms_iou,
            )
        else:
            result = segment_stream_single(
                session,
                mode,
                camera,
                plan,
                stem=stem,
                gsam_url=gsam_url,
                prompt_override=prompt_override,
                max_dets_override=max_dets,
                nms_iou_override=nms_iou,
            )
    try:
        sheets = write_session_qa_sheets(session)
        for name, path in sheets.items():
            print(f"  wrote {path}")
    except Exception as exc:
        print(f"  session QA sheets failed: {exc}")
    return result


def plan_stream_keys(plan: dict) -> list[str]:
    return [str(key) for key in plan["streams"].keys()]


def _stream_process_order(mode: str, camera: str) -> tuple[int, str]:
    # Reflectance before transmittance so reuse_reflectance_masks can find source masks.
    mode_rank = 0 if mode.lower() == "reflectance" else 1 if mode.lower() == "transmittance" else 2
    return (mode_rank, camera.lower())


def _order_streams_for_reuse(plan: dict, streams: list[tuple[str, str]]) -> list[tuple[str, str]]:
    remaining = list(streams)
    ordered: list[tuple[str, str]] = []
    while remaining:
        progressed = False
        for item in list(remaining):
            mode, camera = item
            stream_plan = plan["streams"].get(f"{mode}/{camera}") or {}
            source = resolve_reuse_masks_from(stream_plan, mode, camera)
            if source is not None:
                src_mode, src_camera = source.split("/", 1)
                if (src_mode, src_camera) in remaining:
                    continue
            ordered.append(item)
            remaining.remove(item)
            progressed = True
        if not progressed:
            remaining.sort(key=lambda item: _stream_process_order(item[0], item[1]))
            ordered.extend(remaining)
            break
    return ordered


def streams_with_rgb(session: Path, plan: dict, stem: str) -> list[tuple[str, str]]:
    found: list[tuple[str, str]] = []
    for key in plan_stream_keys(plan):
        if "/" not in key:
            continue
        mode, camera = key.split("/", 1)
        pre = session / mode / camera / "preprocessed"
        rgb = pre / f"{stem}_rgb.png"
        if rgb.is_file() or any(pre.glob("*_rgb.png")):
            found.append((mode, camera))
    return _order_streams_for_reuse(plan, found)

