"""GSAM2 semantic masks for FPP fusion, deferred until pose registration."""

from __future__ import annotations

import json
import urllib.request
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .frames import DepthFrame


def _win_to_wsl(path: Path) -> str:
    text = str(path.resolve()).replace("\\", "/")
    if len(text) >= 2 and text[1] == ":":
        return f"/mnt/{text[0].lower()}/{text[2:].lstrip('/')}"
    return text


def _post_json(url: str, path: str, body: dict[str, Any], timeout_s: int) -> dict[str, Any]:
    request = urllib.request.Request(
        url.rstrip("/") + path,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=int(timeout_s)) as response:
        return json.loads(response.read().decode("utf-8"))


def apply_gsam_masks(
    frames: list[DepthFrame],
    out_dir: Path,
    *,
    server_url: str,
    prompt: str,
    box_threshold: float = 0.25,
    max_detections: int = 8,
    dilation_px: int = 4,
    timeout_s: int = 900,
) -> dict[str, Any]:
    """Prepare exact SAM masks without changing full-scene registration geometry."""
    root = Path(out_dir) / "gsam"
    root.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    total_before = 0
    total_after = 0

    for frame in frames:
        view_dir = root / frame.stem
        view_dir.mkdir(parents=True, exist_ok=True)
        input_path = view_dir / "input.png"
        bgr = cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(input_path), bgr):
            raise RuntimeError(f"Could not write GSAM input image: {input_path}")

        result = _post_json(
            server_url,
            "/segment",
            {
                "input_rgb": _win_to_wsl(input_path),
                "out_dir": _win_to_wsl(view_dir),
                "image_name": frame.stem,
                "prompt": str(prompt),
                "max_dets": int(max_detections),
                "box_threshold": float(box_threshold),
            },
            timeout_s,
        )
        mask_path = view_dir / "masks" / "stack_mask.png"
        mask_u8 = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask_u8 is None:
            raise RuntimeError(f"GSAM did not produce a stack mask for {frame.stem}")
        if mask_u8.shape != frame.mask.shape:
            mask_u8 = cv2.resize(
                mask_u8,
                (frame.mask.shape[1], frame.mask.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        if int(dilation_px) > 0:
            radius = int(dilation_px)
            size = 2 * radius + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
            mask_u8 = cv2.dilate(mask_u8, kernel, iterations=1)
        object_mask = mask_u8 > 0
        frame.object_mask = object_mask
        before = int(frame.mask.sum())
        after = int((frame.mask & object_mask).sum())
        if after < 1000:
            raise RuntimeError(
                f"GSAM mask for {frame.stem} retains only {after} depth pixels; "
                "inspect its overlay or adjust the prompt/threshold"
            )
        prepared_path = view_dir / "prepared_object_mask.png"
        cv2.imwrite(str(prepared_path), object_mask.astype(np.uint8) * 255)
        total_before += before
        total_after += after
        rows.append(
            {
                "stem": frame.stem,
                "detections": int(result.get("detection_count", 0)),
                "valid_before": before,
                "valid_after": after,
                "keep_ratio": float(after / max(before, 1)),
                "overlay": str(view_dir / "overlay.png"),
                "mask": str(prepared_path),
            }
        )

    return {
        "enabled": True,
        "server_url": server_url,
        "prompt": prompt,
        "box_threshold": float(box_threshold),
        "max_detections": int(max_detections),
        "mask_mode": "sam_exact_deferred",
        "dilation_px": int(dilation_px),
        "application": "after_pose_refine",
        "valid_before": total_before,
        "valid_after": total_after,
        "keep_ratio": float(total_after / max(total_before, 1)),
        "views": rows,
        "output_dir": str(root),
    }


def apply_prepared_object_masks(frames: list[DepthFrame]) -> dict[str, Any]:
    """Apply prepared semantic masks after full-scene pose registration."""
    rows: list[dict[str, Any]] = []
    total_before = 0
    total_after = 0
    for frame in frames:
        if frame.object_mask is None or frame.object_mask.shape != frame.mask.shape:
            raise RuntimeError(f"No prepared GSAM object mask for {frame.stem}")
        before = int(frame.mask.sum())
        frame.mask &= frame.object_mask
        frame.depth_m[~frame.mask] = np.nan
        after = int(frame.mask.sum())
        if after < 1000:
            raise RuntimeError(f"GSAM object mask for {frame.stem} retains only {after} pixels")
        total_before += before
        total_after += after
        rows.append({"stem": frame.stem, "valid_before": before, "valid_after": after})
    return {
        "valid_before": total_before,
        "valid_after": total_after,
        "keep_ratio": float(total_after / max(total_before, 1)),
        "views": rows,
    }