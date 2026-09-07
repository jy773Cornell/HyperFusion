# HyperFusion GSAM2 Segmentation and Preprocessing Pipeline

## Purpose

This document explains how HyperFusion turns a captured hyperspectral scan into preprocessed, segmented data ready for fusion and analysis. The focus is on the **GSAM2 segmenter**, but the full preprocessing path is included so the segmentation step can be understood in context.

In short:

```text
capture -> dark/white reference -> flat-field correction -> RGB preview -> GSAM2 segmentation -> ROI spectra
```

GSAM2 is the object-detection and segmentation engine. It finds sample objects in the RGB preview image and writes one binary mask per object. Those masks are then used for:

- per-ROI spectra export
- dual-camera fusion alignment
- QA overlays and inspection

## High-Level Architecture

HyperFusion uses a **Windows app + WSL sidecar** design for GSAM2.

```text
Windows app (C++)
  -> CapturePostProcessor
  -> Gsam2SegmentationClient
  -> wsl.exe curl
  -> GSAM2 HTTP server in Ubuntu (Python)
  -> gsam2_segmenter.py
```

Why WSL:

- GroundingDINO and SAM2 run in Python/PyTorch.
- GPU acceleration is available through WSL2 + NVIDIA drivers.
- Scan folders on `D:\...` are accessible from WSL as `/mnt/d/...`.
- Models stay loaded in the sidecar, so repeated segmentation requests are much faster than reloading models each time.

Main code locations:

```text
app/sidecars/gsam2/
  gsam2_segmenter.py      # GroundingDINO + SAM2 wrapper
  gsam2_server.py         # HTTP sidecar server
  README.md               # WSL setup instructions

app/src/backend/camera/processing/
  CapturePostProcessor.cpp
  Gsam2SegmentationClient.cpp
  Gsam2ServerManager.cpp
  Gsam2RoiAnalysis.cpp
```

## Preprocessing Pipeline Overview

Post-processing runs per camera stream after a stage scan completes. A stream is one camera under one illumination mode, for example:

```text
reflectance/fx10e
reflectance/swir3
transmittance/fx10e
transmittance/swir3
```

For each stream, the pipeline writes outputs under:

```text
{session}/{mode}/{camera}/preprocessed/
```

### Step 1: Reference Plots

From the captured dark and white reference cubes, the app builds row-mean reference spectra and saves plots:

```text
DARKREF_<dataset>_ref_plot.png
WHITEREF_<dataset>_ref_plot.png
```

These are mainly for QA and do not feed directly into GSAM2.

### Step 2: Flat-Field Correction (FFC)

The sample hyperspectral cube is corrected using the dark and white references:

```text
FFC(sample) = (sample - dark) / (white - dark)
```

Outputs:

```text
<dataset>_ffc.hdr
<dataset>_ffc.raw
```

Default clamp and epsilon values come from `app/preset/hyperfusion.cfg`:

```ini
ffc_epsilon = 1e-6
ffc_clamp_min = 0.0
ffc_clamp_max = 1.0
truncate_nm = 780.0
```

The FFC cube is the scientific input used for ROI spectra. GSAM2 itself runs on the RGB preview, not directly on the raw ENVI cube.

### Step 3: RGB Preview Export

The app converts the FFC cube into a PNG preview used for segmentation:

```text
<dataset>_rgb.png
```

Rules:

- **FX10e** uses spectral-to-sRGB conversion.
- **SWIR3** uses false-color RGB from configured wavelength ranges.

Current SWIR false-color defaults:

```ini
swir_false_color_red_nm_min = 1550
swir_false_color_red_nm_max = 1700
swir_false_color_green_nm_min = 1100
swir_false_color_green_nm_max = 1300
swir_false_color_blue_nm_min = 950
swir_false_color_blue_nm_max = 1050
```

This RGB image is the direct input to GSAM2.

### Step 4: GSAM2 Segmentation

If enabled, the app sends the RGB preview to the GSAM2 WSL server and writes masks plus metadata under:

```text
preprocessed/segmentation/
```

### Step 5: ROI Spectra Export

After masks are returned, C++ reads the FFC cube and computes mean reflectance or transmittance spectra inside each mask. Outputs:

```text
roi_spectra.csv
roi_spectra_plot.png
```

This step is local C++ analysis. It does not call the Python server again.

### Step 6: Session preview folder

After per-stream preprocessing finishes, the app writes QA collages to `{session}/preview/`:

```text
preview/rgb_all.png
preview/reference_intensity.png
```

If GSAM ran and overlays / ROI spectra plots exist, those collages are added as well:

```text
preview/mask_overlaps.png
preview/roi_spectra_plots.png
```

Per-stream ROI spectra stay in `preprocessed/segmentation/` (`roi_spectra.csv`, `roi_spectra_plot.png`).

FFC-only sessions get only the RGB and reference-intensity sheets.

## GSAM2 Model Stack

GSAM2 in HyperFusion is a two-stage vision pipeline:

```text
text prompt -> GroundingDINO -> bounding boxes -> SAM2 -> segmentation masks
```

### Stage A: GroundingDINO

GroundingDINO is a zero-shot object detector from Hugging Face Transformers.

Input:

- RGB image
- text prompt, for example `cheese.` or `grape. leaf.`

Output:

- bounding boxes (`xyxy`)
- confidence scores
- text labels / phrases

Prompt handling:

- prompt is lowercased
- if it does not end with `.`, a period is added
- this follows GroundingDINO prompt conventions

Detection filtering:

- boxes below `box_threshold` are removed
- remaining boxes are limited to top `max_dets` by score

Current defaults:

```ini
hf_model_id = IDEA-Research/grounding-dino-base
box_threshold = 0.30
detector_device = cuda
```

In the app UI, **Max samples** maps to `max_dets`.

### Stage B: SAM2

SAM2 turns each bounding box into a pixel mask.

For every box:

1. SAM2 image features are set from the full RGB image.
2. SAM2 predicts one or more mask candidates for that box.
3. HyperFusion keeps the highest-scoring mask.
4. the mask is stored as a binary `(H, W)` array.

Current defaults:

```ini
sam2_config = configs/sam2.1/sam2.1_hiera_l.yaml
sam2_checkpoint = checkpoints/sam2.1_hiera_large.pt
sam2_device = cuda
multimask_output = false
```

If `multimask_output = true`, SAM2 may return multiple mask proposals per box, but the wrapper still selects only the best one.

## GSAM2 Segmenter API

The core Python class is `GSAM2_Segmenter` in `gsam2_segmenter.py`.

Main method:

```python
masks, annotated_image, mask_images, stack_mask_image = segmenter.segment(
    rgb,
    text_prompt,
    box_threshold=...,
    max_dets=...,
    multimask_output=...,
)
```

Returns:

| Output | Meaning |
|--------|---------|
| `masks` | list of boolean `(H, W)` masks |
| `annotated_image` | BGR image with boxes, labels, and masks drawn |
| `mask_images` | list of `0/255` grayscale masks |
| `stack_mask_image` | union of all masks |

The class can be used directly in Python, but HyperFusion normally calls it through `gsam2_server.py`.

## GSAM2 HTTP Server

The sidecar server exposes three endpoints:

| Endpoint | Purpose |
|----------|---------|
| `GET /health` | server and model status |
| `POST /segment` | run segmentation and write files |
| `POST /shutdown` | stop the server |

### Server Startup

The Windows app launches the server in WSL using `Gsam2ServerManager`:

```text
cd /mnt/d/.../app/sidecars/gsam2
./venv/bin/python gsam2_server.py --host 0.0.0.0 --port 8765 --warmup
```

`--warmup` loads GroundingDINO and SAM2 at startup so the first real request is faster.

The Capture tab shows server status as:

```text
GSAM server: connected
```

### Segment Request

The C++ client sends a JSON body like:

```json
{
  "input_rgb": "/mnt/d/.../preprocessed/chiptest_rgb.png",
  "out_dir": "/mnt/d/.../preprocessed/segmentation",
  "image_name": "chiptest_rgb.png",
  "prompt": "cheese.",
  "max_dets": 3,
  "box_threshold": 0.30
}
```

Windows paths are converted to WSL paths before the request is sent.

The client uses `wsl.exe curl` because WSL2 localhost networking is handled from inside WSL, not directly from Windows.

## ROI Ordering Inside the Server

This is an important detail.

GroundingDINO returns detections in model/score order. Before masks are written, `gsam2_server.py` reorders detections into a stable spatial order:

```text
top-to-bottom rows
right-to-left within each row
```

Then ROI IDs are assigned:

```text
roi 1, roi 2, roi 3, ...
```

So in current HyperFusion:

- ROI numbers are **not** raw GroundingDINO detection order
- ROI numbers are assigned after row-major spatial sorting
- mask files use those final ROI numbers:
  - `mask_001.png`
  - `mask_002.png`
  - `mask_003.png`

This matters for fusion because downstream pairing uses object position, not arbitrary detector order.

## Segmentation Outputs

For each camera stream, GSAM2 writes:

```text
preprocessed/segmentation/
  segmentation_results.json
  overlay.png
  stack_mask.png
  masks/
    mask_001.png
    mask_001.npy
    mask_002.png
    mask_002.npy
    ...
  segmented_rgb/
    roi_001.png
    roi_002.png
    ...
```

### `segmentation_results.json`

Main manifest file. Example fields per detection:

```json
{
  "roi": 1,
  "label": "cheese",
  "score": 0.42,
  "pixel_count": 12840,
  "mask_png": ".../mask_001.png",
  "mask_npy": ".../mask_001.npy",
  "segmented_rgb_png": ".../segmented_rgb/roi_001.png"
}
```

### `overlay.png`

Annotated QA image showing:

- bounding boxes
- mask outlines
- labels and scores

### `stack_mask.png`

Union of all ROI masks. Useful for quick visual inspection.

### `segmented_rgb/roi_NNN.png`

Tight RGBA crop around each mask:

- opaque inside the mask
- transparent outside the mask

These are helpful for visual QA of what GSAM2 actually segmented.

## ROI Spectra Analysis (C++)

After GSAM2 returns masks, `Gsam2RoiAnalysis.cpp` computes spectra locally.

Process:

1. Read `segmentation_results.json`.
2. Load each `mask_*.png`.
3. Verify mask size matches the FFC cube.
4. Stream through the FFC ENVI cube line by line.
5. For each ROI and each band, compute mean and standard deviation over masked pixels.
6. Write:
   - `roi_spectra.csv`
   - `roi_spectra_plot.png`

The CSV columns are GSAM-compatible:

```text
image, label, roi#, pixel_num, wavelength_1, wavelength_2, ...
```

This gives immediate scientific output even before fusion runs.

## How the App Triggers Segmentation

Segmentation can run in three ways:

1. **Automatically after scan**
   - Capture tab -> Preprocessing -> `Preprocess the image when the scanning is done`
   - also enable `Run GSAM segmentation`

2. **Before manual fusion**
   - if `preprocessed/segmentation/` is missing, the app can run post-processing + GSAM first

3. **Manual server testing**
   - run `gsam2_server.py` directly in WSL for debugging

Important UI fields:

| UI field | Effect |
|----------|--------|
| GSAM prompt | text sent to GroundingDINO |
| Max samples | maximum number of detections (`max_dets`) |
| Run GSAM segmentation | enables segmentation during post-processing |
| Run fusion on session... | may trigger preprocess + GSAM if prerequisites are missing |

GSAM prompt and Max samples remain editable even when other preprocessing controls are locked.

## Configuration

From `app/preset/hyperfusion.cfg`:

```ini
[segmentation]
wsl_distro = Ubuntu
server_port = 8765
box_threshold = 0.30
multimask_output = false
warmup_on_start = true
hf_model_id = IDEA-Research/grounding-dino-base
sam2_config = configs/sam2.1/sam2.1_hiera_l.yaml
sam2_checkpoint = checkpoints/sam2.1_hiera_large.pt
detector_device = cuda
sam2_device = cuda
```

Recommended prompt style:

```text
cheese.
grape. leaf.
sample.
```

Lowercase phrases ending with `.` work best.

## End-to-End Data Flow

Example for one stream:

```text
capture/chiptest_reflectance.hdr
capture/chiptest_reflectance.raw
        |
        v
FFC + RGB preview
        |
        +--> chiptest_ffc.hdr / chiptest_ffc.raw
        |
        +--> chiptest_rgb.png
                 |
                 v
            GSAM2 server
                 |
                 +--> overlay.png
                 +--> masks/mask_001.png
                 +--> segmentation_results.json
                 |
                 v
            C++ ROI analysis
                 |
                 +--> roi_spectra.csv
                 +--> roi_spectra_plot.png
```

For dual-camera fusion, both `fx10e` and `swir3` must produce segmentation outputs with the same object count for a given illumination mode.

## Relationship to Fusion

Fusion uses GSAM outputs as object definitions:

- masks define chip boundaries
- mask centroids define coarse object matching between FX10e and SWIR3
- `segmentation_results.json` is a required prerequisite

Fusion does **not** rerun GSAM. It consumes the masks already written during preprocessing.

See also:

- `app/sidecars/hf_fusion/hyperspectral-fusion.md`

## Performance Notes

Typical runtime behavior:

- first server start: slow, because models load into GPU memory
- warmed server: much faster per request
- per-camera segmentation: one HTTP request per stream

The log may show PyTorch attention warnings such as Flash Attention fallback. These are usually not fatal; they only mean a slower attention kernel was chosen.

GPU is strongly recommended. CPU mode is possible but much slower.

## Common Failure Modes

| Symptom | Likely cause |
|---------|--------------|
| `GSAM server: not available` | WSL not installed, venv missing, or server failed to start |
| `GSAM2 segmentation failed` | server not reachable, bad path conversion, or request timeout |
| `0 detections` | prompt too specific, threshold too high, or poor RGB contrast |
| too many false objects | lower `box_threshold`, tighten prompt, or reduce `Max samples` |
| mask size mismatch | RGB/FFC geometry mismatch; segmentation ran on wrong preview |
| fusion says ROI count mismatch | FX10e and SWIR3 found different numbers of objects |
| first run slow | model download / warmup; expected on cold start |

## Current Limitations

- Segmentation is driven by a text prompt, not trained class-specific models.
- One mask is kept per detected box; there is no instance splitting beyond SAM2's best mask.
- ROI matching across cameras is position-based, not identity-based.
- Segmentation quality depends heavily on RGB preview quality.
- The sidecar currently requires WSL + Ubuntu; it is not a native Windows inference path.

## Summary

HyperFusion preprocessing converts raw capture cubes into corrected ENVI data and RGB previews. GSAM2 then uses a text prompt to detect objects and SAM2 to segment them. The app stores masks, overlays, and a JSON manifest under `preprocessed/segmentation/`, then computes per-ROI spectra from the FFC cube. Those segmentation products become the object definitions used by the downstream fusion pipeline.
