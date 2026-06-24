#!/usr/bin/env python3
"""
GSAM2 HTTP server for HyperFusion (WSL sidecar).

Endpoints:
  GET  /health   -> {"status":"ok","model_loaded":true}
  POST /segment  -> run GSAM2, write mask PNG/NPY + overlay to out_dir
  POST /shutdown -> stop server
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parent
# Package layout: resources/gsam2/ is the `sam2` Python package.
sys.path.insert(0, str(ROOT.parent))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import supervision as sv  # noqa: E402

from sam2.gsam2_segmenter import GSAM2_Segmenter  # noqa: E402

_segmenter: Optional[GSAM2_Segmenter] = None
_segmenter_lock = threading.Lock()


def get_segmenter(args: argparse.Namespace) -> GSAM2_Segmenter:
    global _segmenter
    with _segmenter_lock:
        if _segmenter is None:
            ckpt_path = args.sam2_checkpoint
            if not Path(ckpt_path).is_absolute():
                ckpt_path = str(ROOT / ckpt_path)
            _segmenter = GSAM2_Segmenter(
                hf_model_id=args.hf_model_id,
                detector_device=args.detector_device,
                sam2_cfg_path=args.sam2_config,
                sam2_ckpt_path=ckpt_path,
                sam2_device=args.sam2_device,
                box_threshold=args.box_threshold,
                max_dets=args.max_dets_default,
                multimask_output=args.multimask_output,
            )
        return _segmenter


def load_rgb(path: Path) -> np.ndarray:
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def write_segmented_rgb_patch(rgb: np.ndarray, mask: np.ndarray, path: Path) -> Dict[str, int]:
    """Write a tight RGBA PNG crop: transparent outside the mask, sized to mask bounds."""
    ys, xs = np.nonzero(mask)
    if ys.size == 0 or xs.size == 0:
        empty = np.zeros((1, 1, 4), dtype=np.uint8)
        cv2.imwrite(str(path), cv2.cvtColor(empty, cv2.COLOR_RGBA2BGRA))
        return {"x": 0, "y": 0, "width": 0, "height": 0}

    y0, y1 = int(ys.min()), int(ys.max()) + 1
    x0, x1 = int(xs.min()), int(xs.max()) + 1

    crop_rgb = rgb[y0:y1, x0:x1]
    crop_mask = mask[y0:y1, x0:x1]
    alpha = (crop_mask.astype(np.uint8) * 255)
    rgba = np.dstack((crop_rgb, alpha))
    cv2.imwrite(str(path), cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))

    return {"x": x0, "y": y0, "width": x1 - x0, "height": y1 - y0}

def run_segment(body: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    input_rgb = Path(str(body["input_rgb"]))
    out_dir = Path(str(body["out_dir"]))
    prompt = str(body.get("prompt", "sample."))
    max_dets = int(body.get("max_dets", args.max_dets_default))
    box_threshold = float(body.get("box_threshold", args.box_threshold))
    image_name = str(body.get("image_name", input_rgb.name))

    out_dir.mkdir(parents=True, exist_ok=True)
    rgb = load_rgb(input_rgb)

    segmenter = get_segmenter(args)
    det = segmenter._gdino_detect_hf(rgb, prompt, box_threshold, max_dets)
    boxes = det["boxes_xyxy"]
    scores = det["scores"]
    phrases = det["phrases"]
    masks = segmenter._sam2_masks_from_boxes(
        segmenter.sam2_predictor, rgb, boxes, args.multimask_output
    )
    mask_images = [(m.astype(np.uint8) * 255) for m in masks]
    stack_mask = (
        (np.max(np.stack(masks, axis=0), axis=0).astype(np.uint8) * 255)
        if masks
        else np.zeros(rgb.shape[:2], np.uint8)
    )

    detections_sv = sv.Detections(
        xyxy=boxes,
        mask=np.stack(masks, axis=0) if masks else None,
        class_id=np.arange(len(phrases)),
    )
    draw_labels = [
        f"{idx}: {phr} {sc:.2f}" for idx, (phr, sc) in enumerate(zip(phrases, scores), start=1)
    ]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    annotated_bgr = sv.MaskAnnotator().annotate(scene=bgr.copy(), detections=detections_sv)
    annotated_bgr = sv.BoxAnnotator().annotate(scene=annotated_bgr, detections=detections_sv)
    annotated_bgr = sv.LabelAnnotator().annotate(
        scene=annotated_bgr, detections=detections_sv, labels=draw_labels
    )

    overlay_path = out_dir / "overlay.png"
    stack_path = out_dir / "stack_mask.png"
    cv2.imwrite(str(overlay_path), annotated_bgr)

    masks_dir = out_dir / "masks"
    masks_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(masks_dir / stack_path.name), stack_mask)

    segmented_rgb_dir = out_dir / "segmented_rgb"
    segmented_rgb_dir.mkdir(parents=True, exist_ok=True)

    detections: List[Dict[str, Any]] = []
    for idx, (mask, mask_u8, phrase, score) in enumerate(
        zip(masks, mask_images, phrases, scores),
        start=1,
    ):
        stem = f"mask_{idx:03d}"
        mask_png = masks_dir / f"{stem}.png"
        mask_npy = masks_dir / f"{stem}.npy"
        cv2.imwrite(str(mask_png), mask_u8)
        np.save(str(mask_npy), mask.astype(np.uint8))

        segmented_rgb_path = segmented_rgb_dir / f"roi_{idx:03d}.png"
        patch_bounds = write_segmented_rgb_patch(rgb, mask, segmented_rgb_path)

        detections.append(
            {
                "roi": idx,
                "label": phrase,
                "score": float(score),
                "pixel_count": int(mask.sum()),
                "mask_png": str(mask_png),
                "mask_npy": str(mask_npy),
                "segmented_rgb_png": str(segmented_rgb_path),
                "segmented_rgb_bounds": patch_bounds,
            }
        )

    result = {
        "ok": True,
        "image": image_name,
        "prompt": prompt,
        "detection_count": len(detections),
        "overlay_png": str(overlay_path),
        "stack_mask_png": str(masks_dir / stack_path.name),
        "masks_dir": str(masks_dir),
        "segmented_rgb_dir": str(segmented_rgb_dir),
        "detections": detections,
    }

    manifest_path = out_dir / "segmentation_results.json"
    manifest_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


class Handler(BaseHTTPRequestHandler):
    server_version = "Gsam2Server/1.0"
    args: argparse.Namespace

    def log_message(self, fmt: str, *log_args) -> None:
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % log_args))

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def _send_json(self, code: int, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        if self.path.rstrip("/") == "/health":
            self._send_json(
                200,
                {
                    "status": "ok",
                    "model_loaded": _segmenter is not None,
                },
            )
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:
        if self.path.rstrip("/") == "/shutdown":
            self._send_json(200, {"ok": True, "shutting_down": True})
            threading.Thread(target=self.server.shutdown, daemon=True).start()
            return

        if self.path.rstrip("/") == "/segment":
            try:
                body = self._read_json()
                result = run_segment(body, self.args)
                self._send_json(200, result)
            except Exception as exc:
                self._send_json(500, {"ok": False, "error": str(exc)})
            return

        self._send_json(404, {"ok": False, "error": "not found"})


def main() -> None:
    parser = argparse.ArgumentParser(description="GSAM2 HTTP server for HyperFusion")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--hf-model-id", default="IDEA-Research/grounding-dino-base")
    parser.add_argument("--detector-device", default="cuda")
    parser.add_argument("--sam2-device", default="cuda")
    parser.add_argument("--sam2-config", default="configs/sam2.1/sam2.1_hiera_l.yaml")
    parser.add_argument("--sam2-checkpoint", default="checkpoints/sam2.1_hiera_large.pt")
    parser.add_argument("--box-threshold", type=float, default=0.30)
    parser.add_argument("--max-dets-default", type=int, default=100)
    parser.add_argument("--multimask-output", action="store_true")
    parser.add_argument("--warmup", action="store_true", help="Load models at startup")
    args = parser.parse_args()

    Handler.args = args
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    sys.stderr.write(f"GSAM2 server listening on {args.host}:{args.port}\n")
    if args.warmup:
        threading.Thread(target=lambda: get_segmenter(args), daemon=True, name="gsam2-warmup").start()
    httpd.serve_forever()


if __name__ == "__main__":
    main()
