"""
GSAM2: GroundingDINO (HF Transformers) + SAM2 segmentation wrapper.

Author: Jinhong Yu
Institution: Cornell University, CALS
Lab: Postharvest Technology Lab & CAIR Lab
Email: jy773@cornell.edu

Pipeline
--------
1) GroundingDINO (Hugging Face) does zero-shot *grounding*:
   text prompt -> bounding boxes + scores + phrases (labels)

2) SAM2 (local) turns those boxes into segmentation masks.

Returns (segment)
-----------------
masks              : List[np.ndarray bool]  # each (H, W)
annotated_image    : np.ndarray uint8 BGR   # boxes + masks + labels drawn
mask_images        : List[np.ndarray uint8] # 0/255, each (H, W)
stack_mask_image   : np.ndarray uint8       # 0/255, union/max over all masks (H, W)

Notes
-----
- Prompts work best lowercased; each concept ending with '.' helps (e.g., "leaf.")
- `multimask_output=True` requests multiple mask candidates per box from SAM2,
  but this wrapper still selects the best (highest-score) mask per box.
"""

from typing import Any, Dict, List, Optional, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
import supervision as sv
import torch


class GSAM2_Segmenter:
    """
    HF GroundingDINO (Transformers) + local SAM2 wrapper.

    Args
    ----
    hf_model_id: str
        Hugging Face repo id for GroundingDINO (e.g. "IDEA-Research/grounding-dino-base").
    detector_device: str
        "cuda" or "cpu" for the GroundingDINO model.
    sam2_predictor: Any | None
        Prebuilt SAM2ImagePredictor. If None, provide both `sam2_cfg_path` and `sam2_ckpt_path`.
    sam2_cfg_path: str | None
        SAM2 config yaml path (used if `sam2_predictor` is None).
    sam2_ckpt_path: str | None
        SAM2 checkpoint path (used if `sam2_predictor` is None).
    sam2_device: str
        "cuda" or "cpu" for SAM2 model.
    box_threshold: float
        Confidence cutoff for GroundingDINO scores (lower -> more boxes).
    max_dets: int
        Max number of detections to keep (top-K by score).
    multimask_output: bool
        If True, SAM2 proposes multiple masks per box; we still pick the best one.
    use_bf16_autocast: bool
        If True, try bfloat16 autocast for detector (if supported).
    enable_tf32_if_ampere: bool
        If True and on Ampere+, enable TF32 for faster matmuls.
    """

    def __init__(
        self,
        *,
        # ---- HF GroundingDINO ----
        hf_model_id: str = "IDEA-Research/grounding-dino-base",
        detector_device: str = "cuda",
        # ---- SAM2 ----
        sam2_predictor: Any = None,
        sam2_cfg_path: Optional[str] = "configs/sam2.1/sam2.1_hiera_l.yaml",
        sam2_ckpt_path: Optional[str] = "./checkpoints/sam2.1_hiera_large.pt",
        sam2_device: str = "cuda",
        # ---- Defaults / thresholds ----
        box_threshold: float = 0.30,
        max_dets: int = 100,
        multimask_output: bool = False,
        # Numeric opt
        use_bf16_autocast: bool = True,
        enable_tf32_if_ampere: bool = True,
    ):
        self.detector_device = detector_device
        self.hf_model_id = hf_model_id

        # Defaults used if not overridden in segment()
        self.default_box_thr = float(box_threshold)
        self.default_max_dets = int(max_dets)
        self.default_multimask = bool(multimask_output)

        # Build GroundingDINO (HF Transformers)
        self._build_gdino_hf_transformers()

        # Build or accept SAM2 predictor
        self.sam2_predictor = sam2_predictor
        if self.sam2_predictor is None:
            if not (sam2_cfg_path and sam2_ckpt_path):
                raise ValueError(
                    "Provide either `sam2_predictor` OR both `sam2_cfg_path` and `sam2_ckpt_path`."
                )
            self.sam2_predictor = self._build_sam2_predictor(
                sam2_cfg_path, sam2_ckpt_path, sam2_device
            )

        # Numeric preferences
        if enable_tf32_if_ampere and torch.cuda.is_available():
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True

        self._use_bf16 = bool(use_bf16_autocast)

    # -------------------- Public API --------------------

    def segment(
        self,
        rgb: np.ndarray,
        text_prompt: str,
        *,
        box_threshold: Optional[float] = None,
        max_dets: Optional[int] = None,
        multimask_output: Optional[bool] = None,
        visualize: bool = False,
    ) -> Tuple[List[np.ndarray], np.ndarray, List[np.ndarray], np.ndarray]:
        """
        Run detection + segmentation.

        Args
        ----
        rgb: np.ndarray uint8 (H, W, 3)
            Input image (RGB).
        text_prompt: str
            Zero-shot prompt, e.g. "dog." or "grape cluster. leaf."
        box_threshold: float | None
            Override the detector score cutoff; default from __init__ if None.
        max_dets: int | None
            Override max detections; default from __init__ if None.
        multimask_output: bool | None
            Request multiple mask proposals per box from SAM2; best one is kept.
        visualize: bool
            If True, show the annotated image with matplotlib (crisp, no smoothing).

        Returns
        -------
        masks: List[np.ndarray bool]        # list of (H, W) boolean masks
        annotated_image: np.ndarray uint8    # BGR image with masks+boxes+labels
        mask_images: List[np.ndarray uint8]  # list of 0/255 grayscale masks
        stack_mask_image: np.ndarray uint8   # 0/255 stacked (union) grayscale mask
        """
        self._assert_rgb(rgb)

        # Resolve effective params
        box_thr = self.default_box_thr if box_threshold is None else float(box_threshold)
        max_det = self.default_max_dets if max_dets is None else int(max_dets)
        mmask = self.default_multimask if multimask_output is None else bool(multimask_output)

        # ---- Detection (GroundingDINO) ----
        det = self._gdino_detect_hf(rgb, text_prompt, box_thr, max_det)
        boxes, scores, phrases = det["boxes_xyxy"], det["scores"], det["phrases"]

        # ---- Segmentation (SAM2) ----
        masks = self._sam2_masks_from_boxes(self.sam2_predictor, rgb, boxes, mmask)

        # ---- Convert masks to requested formats ----
        mask_images = [(m.astype(np.uint8) * 255) for m in masks]
        stack_mask_image = (
            (np.max(np.stack(masks, axis=0), axis=0).astype(np.uint8) * 255)
            if masks else np.zeros(rgb.shape[:2], np.uint8)
        )

        # ---- Build supervision.Detections for drawing ----
        detections = sv.Detections(
            xyxy=boxes,
            mask=np.stack(masks, axis=0) if masks else None,
            class_id=np.arange(len(phrases)),
        )

        # Labels: "<phrase> <score>"
        labels = [f"{idx}: {phr} {sc:.2f}" for idx, (phr, sc) in enumerate(zip(phrases, scores), start=1)]

        # ---- Compose annotated image (BGR) ----
        # supervision expects BGR images; `rgb` is RGB. Convert for drawing, convert back if desired.
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        annotated_bgr = sv.MaskAnnotator().annotate(scene=bgr.copy(), detections=detections)
        annotated_bgr = sv.BoxAnnotator().annotate(scene=annotated_bgr, detections=detections)
        annotated_bgr = sv.LabelAnnotator().annotate(
            scene=annotated_bgr, detections=detections, labels=labels
        )

        # Optional crisp visualization (convert to RGB for matplotlib)
        if visualize:
            vis_rgb = cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)
            h, w = vis_rgb.shape[:2]
            dpi = 100
            fig = plt.figure(figsize=(w / dpi, h / dpi), dpi=dpi)
            ax = plt.axes([0, 0, 1, 1], frameon=False)
            ax.set_axis_off()
            ax.imshow(vis_rgb, interpolation="none")
            plt.show()

        return masks, annotated_bgr, mask_images, stack_mask_image

    __call__ = segment  # convenience

    # -------------------- Internals: Detector (HF Transformers) --------------------

    def _build_gdino_hf_transformers(self) -> None:
        """Load GroundingDINO processor + model from HF and move model to device."""
        try:
            from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        except Exception as e:
            raise RuntimeError(
                "Install Hugging Face Transformers: pip install transformers"
            ) from e

        self._hf_processor = AutoProcessor.from_pretrained(self.hf_model_id)
        self._hf_model = AutoModelForZeroShotObjectDetection.from_pretrained(self.hf_model_id)
        self._hf_model.to(self.detector_device)
        self._hf_model.eval()

    def _gdino_detect_hf(
        self,
        rgb: np.ndarray,
        text_prompt: str,
        box_threshold: float,
        max_dets: int,
    ) -> Dict[str, Any]:
        """
        GroundingDINO (HF) forward + postprocess + manual threshold/top-k.
        Returns numpy xyxy boxes, scores, and label strings.
        """
        from PIL import Image

        # Normalize prompt: lowercased, ensure ending with '.'
        text = text_prompt.strip().lower()
        if not text.endswith("."):
            text += "."

        # Preprocess inputs on the right device
        image = Image.fromarray(rgb, mode="RGB")
        inputs = self._hf_processor(images=image, text=text, return_tensors="pt")
        inputs = {
            k: (v.to(self.detector_device) if isinstance(v, torch.Tensor) else v)
            for k, v in inputs.items()
        }

        # Optional bf16 autocast for detector
        autocast_ctx = (
            torch.autocast(
                device_type=("cuda" if "cuda" in self.detector_device else "cpu"),
                dtype=torch.bfloat16,
            )
            if self._use_bf16
            else nullcontext()
        )
        
        # enable fast attention kernels
        try:
            torch.backends.cuda.sdp_kernel(enable_flash=True, enable_mem_efficient=True, enable_math=False)
        except Exception:
            pass

        with autocast_ctx, torch.no_grad():
            outputs = self._hf_model(**inputs)

        # Convert to pixel boxes (target size expects (H, W))
        target_sizes = [rgb.shape[:2]]
        results = self._hf_processor.post_process_grounded_object_detection(
            outputs, inputs["input_ids"], target_sizes=target_sizes
        )
        det = results[0]
        boxes = det["boxes"]   # tensor (N,4) xyxy
        scores = det["scores"] # tensor (N,)
        labels = det["labels"] # list[str]

        if boxes.numel() == 0:
            return {
                "boxes_xyxy": np.zeros((0, 4), dtype=np.float32),
                "scores": np.zeros((0,), dtype=np.float32),
                "phrases": [],
            }

        # Score threshold
        keep = scores > float(box_threshold)
        boxes = boxes[keep]
        scores = scores[keep]
        labels = [labels[i] for i, k in enumerate(keep.tolist()) if k]

        # Top-K by score
        if boxes.shape[0] > max_dets:
            idx = torch.topk(scores, k=max_dets).indices
            boxes = boxes[idx]
            scores = scores[idx]
            labels = [labels[i.item()] for i in idx]

        # Clip to image bounds
        H, W = rgb.shape[:2]
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clamp(0, W - 1)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clamp(0, H - 1)

        return {
            "boxes_xyxy": boxes.detach().cpu().numpy().astype(np.float32),
            "scores": scores.detach().cpu().numpy().astype(np.float32),
            "phrases": labels,
        }

    # -------------------- Internals: SAM2 --------------------

    def _build_sam2_predictor(self, cfg_path: str, ckpt_path: str, device: str):
        """Construct a SAM2ImagePredictor from cfg + checkpoint."""
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        model = build_sam2(cfg_path, ckpt_path, device=device)
        return SAM2ImagePredictor(model)

    def _sam2_masks_from_boxes(
        self,
        predictor: Any,
        rgb: np.ndarray,
        boxes_xyxy: np.ndarray,
        multimask_output: bool,
    ) -> List[np.ndarray]:
        """
        For each box, ask SAM2 for masks. If multiple are returned, pick the best
        (highest SAM2 score) to keep the contract "one mask per box".
        """
        if boxes_xyxy.size == 0:
            return []
        predictor.set_image(rgb)
        out: List[np.ndarray] = []
        for i in range(boxes_xyxy.shape[0]):
            box = boxes_xyxy[i].astype(np.float32)
            masks, scores, _ = predictor.predict(box=box, multimask_output=multimask_output)
            m = masks[int(np.argmax(scores))]
            out.append((m > 0.5) if m.dtype != bool else m.astype(bool))
        return out

    # -------------------- Internals: Validation --------------------

    @staticmethod
    def _assert_rgb(rgb: np.ndarray) -> None:
        """Ensure the input is uint8 RGB HxWx3."""
        if not (
            isinstance(rgb, np.ndarray)
            and rgb.ndim == 3
            and rgb.shape[2] == 3
            and rgb.dtype == np.uint8
        ):
            raise AssertionError("`rgb` must be a uint8 array of shape (H, W, 3).")


# small no-op context manager so we can toggle autocast cleanly
from contextlib import contextmanager

@contextmanager
def nullcontext():
    yield
