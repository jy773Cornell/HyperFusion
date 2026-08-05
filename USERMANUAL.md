# HyperFusion — software user manual

This document guides how to use the HyperFusion software prototype.

## Contents

1. [Starting the application](#1-starting-the-application)
2. [Main window layout](#2-main-window-layout)
3. [Camera tab (FX10e / SWIR3)](#3-camera-tab-fx10e--swir3)
  - [3.1 Connection](#31-connection)
  - [3.2 Imaging parameters](#32-imaging-parameters)
  - [3.3 Stream views](#33-stream-views-fx10e--swir3-tabs)
4. [Stage tab (Zaber)](#4-stage-tab-zaber)
5. [Light tab (MCC lighthouse)](#5-light-tab-mcc-lighthouse)
6. [Capture tab (recorder)](#6-capture-tab-recorder)
  - [6.1 Recorder buttons](#61-recorder-buttons)
  - [6.2 Cameras](#62-cameras)
  - [6.3 Modes](#63-modes)
  - [6.4 Position and scanning](#64-position-and-scanning)
  - [6.5 Typical staged Record workflow](#65-typical-staged-record-workflow)
  - [6.6 Output data](#66-output-data)
  - [6.7 Post-processing and spectral fusion](#67-post-processing-and-spectral-fusion)
  - [6.8 Capture stream tab](#68-capture-stream-tab)
7. [UR3e tab](#7-ur3e-tab)
8. [Configuration — `hyperfusion.cfg`](#8-configuration--hyperfusioncfg)
  - `[[sample_stage_position]](#sample_stage_position-mm)`
  - `[[camera_calibration]](#camera_calibration)`
  - `[[scanning_settings]](#scanning_settings)`
  - `[[lighthouse]](#lighthouse)`
  - `[[preprocessing]](#preprocessing)`
  - `[[segmentation]` (GSAM)](#segmentation-optional-gsam)
  - `[[fusion]`](#fusion-dual-camera)
9. [SWIR3 image quality — NUC and column profile destripe](#9-swir3-image-quality--nuc-and-column-profile-destripe)
  - [Auto NUC](#auto-nuc-swir3_auto_nuc--true)
  - [Column profile destripe](#column-profile-destripe-swir3_column_profile_correct--true)
10. [Optional — GSAM2 segmentation](#10-optional--gsam2-segmentation)
11. [Dual-camera spectral fusion](#11-dual-camera-spectral-fusion)
12. [Quick reference — daily operator flow](#12-quick-reference--daily-operator-flow)

---

## 1. Starting the application

1. Run `**app.exe`** from your deployment folder (typically `app\build\Release\` on a dev machine, or a copied Release package on the lab PC).
2. Ensure `**hyperfusion.cfg`** sits **next to `app.exe`** (copied automatically on build).
3. On first launch, connect hardware in this order (recommended):
  - **Stage** (if used) → **Lighthouse** (if used) → **Cameras**
4. Watch the **Log** panel at the bottom for connection status and errors.

Settings in `hyperfusion.cfg` reload **on every app start**. Camera exposure, frame rate, binning, and RGB bands are saved in **app settings** and persist across sessions.

---

## 2. Main window layout

```
┌─────────────────────────────┬──────────────────────────────────────────┐
│  Settings (left)            │  Stream views (right)                    │
│  • Camera                   │  • FX10e                                 │
│  • Stage                    │  • SWIR3                                 │
│  • Light                    │  • UR3e (placeholder)                    │
│  • UR3e                     │  • Capture (recorder waterfalls)         │
│  • Capture                  │                                          │
├─────────────────────────────┴──────────────────────────────────────────┤
│  Log                                                                   │
└────────────────────────────────────────────────────────────────────────┘
```

**Left — settings tabs** control connections and parameters.  
**Right — stream tabs** show live imagery and profiles.  
**Log** records connect/disconnect, capture progress, timing adjustments, and SWIR diagnostics.

---

## 3. Camera tab (FX10e / SWIR3)

Each camera has its own sub-tab.

### 3.1 Connection


| Control              | Action                                                                                                                                                                                   |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **SSP profile**      | Select the Lumo device profile (`FX10e with Pleora` or `SWIR3 with NI`).                                                                                                                 |
| **Calibration pack** | `.scp` file for spectral calibration, radiometric correction, and BPR map. Bundled packs live under `calibration/fx10e/` and `calibration/swir/`. **Required for SWIR3** before connect. |
| **Connect**          | Opens the camera, loads calpack, initializes SDK. Streaming starts automatically when ready.                                                                                             |
| **Disconnect**       | Stops stream and closes the camera.                                                                                                                                                      |


**SWIR3 notes**

- Close **NI MAX Grab** before connecting from HyperFusion.
- Allow the SWIR head to **warm up** after power-on before critical work.
- After connect, the log may show NI channel readback and BPR/NUC status, for example:  
`SDK.BPR=…`, `ColumnProfile=…`, `AutoNUC=on`, `NUC=…`

**FX10e notes**

- Pleora eBUS may show a device picker on first connect.

### 3.2 Imaging parameters


| Parameter                      | Description                                                                                                                |
| ------------------------------ | -------------------------------------------------------------------------------------------------------------------------- |
| **Frame rate**                 | Target lines per second (Hz).                                                                                              |
| **Exposure**                   | Integration time (ms). The Lumo SDK may adjust the applied value when **auto exposure** is active to fit the frame period. |
| **Spectral / spatial binning** | 1, 2, 4, or 8. Changing spectral binning reloads the wavelength band list.                                                 |
| **RGB bands**                  | False-color and waterfall R/G/B rows (from calpack wavelength table).                                                      |
| **Shutter**                    | Open/close when supported by the sensor profile.                                                                           |


### 3.3 Stream views (FX10e / SWIR3 tabs)

Each camera stream tab is a 2×2 layout:


| Pane           | Content                                                                                                                            |
| -------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| **Detector**   | Live grayscale view (spatial × spectral). Crosshair selects profile cursor. **Red pixels** = saturation warning (not BPR markers). |
| **Waterfall**  | Scrolling time series; vertical axis = time, horizontal = spatial pixels.                                                          |
| **Wavelength** | Spectral profile at the crosshair spatial column.                                                                                  |
| **Pixel**      | Spatial profile at the crosshair band (wavelength).                                                                                |


**Tips**

- Use the pixel profile to inspect column defects and BPR effectiveness.
- FPS is shown on the detector pane when streaming.

---

## 4. Stage tab (Zaber)


| Step | Action                                                                               |
| ---- | ------------------------------------------------------------------------------------ |
| 1    | **Refresh** COM ports, select the Zaber port and baud (usually 115200).              |
| 2    | **Connect** — HyperFusion applies travel limits and lockstep from the Zaber profile. |
| 3    | **Home** (house icon) — reference the stage before absolute moves.                   |
| 4    | Use **jog**, **Go to start/end**, or **absolute position** for manual positioning.   |


The axis widget shows current position along the scan rail (mm).

Stage positions for white reference, bright reference, and sample scan start are defined in `**hyperfusion.cfg`** (see §8). Capture tab position spin boxes mirror those values for quick “Go” moves during setup.

---

## 5. Light tab (MCC lighthouse)


| Step | Action                                                                                                      |
| ---- | ----------------------------------------------------------------------------------------------------------- |
| 1    | **Connect** to the Measurement Computing USB-1208 board.                                                    |
| 2    | Set per-channel **reflectance** and **transmittance** levels (also driven from `hyperfusion.cfg` defaults). |
| 3    | During capture, HyperFusion switches lighthouse intensity for reflectance vs transmittance modes.           |


If the board is missing, camera streaming still works; staged capture workflows that need controlled illumination will fail at the hood-preparation step.

---

## 6. Capture tab (recorder)

### 6.1 Recorder buttons


| Button      | Purpose                                                                                                                           |
| ----------- | --------------------------------------------------------------------------------------------------------------------------------- |
| **Preview** | Run the **full stage scan sequence** (black ref → white ref → sample) **without saving** files. Requires stage recording enabled. |
| **Record**  | Same sequence **and save** `.raw` hyperspectral data to disk.                                                                     |
| **Stop**    | End preview/recording or continuous reflectance-only capture.                                                                     |


Status text shows per-camera frame counts during capture.

### 6.2 Cameras

- Check **FX10e** and/or **SWIR3** to include in capture.
- **Auto-sync FX10e and SWIR3 scan rate** — when both are selected, HyperFusion matches SWIR3 frame rate to FX10e so both cover the same physical distance per line (uses `hyperfusion.cfg` spatial scales).

### 6.3 Modes


| Mode              | Description                                                                                                              |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------ |
| **Reflectance**   | Standard illuminated scan.                                                                                               |
| **Transmittance** | Second pass with transmittance lighthouse level and (when configured) **transmittance exposure** from `hyperfusion.cfg`. |


Both can be selected; Record runs reflectance first, then transmittance.

### 6.4 Position and scanning


| Option                                  | Description                                                                                                                                              |
| --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Use HyperFusion Stage for recording** | **On** — Preview/Record use the full staged procedure. **Off** — Record saves **reflectance frames continuously** until Stop (no black/white ref scans). |
| **Camera position Go**                  | Jog stage to configured white-ref position per camera.                                                                                                   |
| **Target length**                       | Sample scan window length (mm).                                                                                                                          |
| **Scanning speed**                      | Stage speed during white/sample scans. **Auto** derives speed from frame rate × spatial mm/pixel (see cfg).                                              |


### 6.5 Typical staged Record workflow

1. Connect **stage**, **lighthouse**, and **camera(s)**.
2. Enable **Use HyperFusion Stage for recording**.
3. Select cameras and modes (reflectance / transmittance).
4. **Home** the stage.
5. Press **Record** — follow hood/preparation dialogs.
6. Sequence (simplified):
  - Move to references → **black reference** frames → **white reference** scan  
  - Move to sample start → **sample scan**  
  - (If transmittance selected) temp stop, exposure switch, transmittance pass
7. Log shows save path, e.g. `Capture record: saved to …`

### 6.6 Output data

Sessions are saved under **Save folder / Dataset name**:


| Mode                                         | Folder layout                                                                                        |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| **Stage scan** (Preview / Record with stage) | `dataset/reflectance/fx10e/`, `dataset/reflectance/swir3/`, and optionally `dataset/transmittance/…` |
| **Continuous record** (stage scanning off)   | `dataset/recording/fx10e/`, `dataset/recording/swir3/`, …                                            |


Each stream folder contains `**.raw`** line files (and reference files when a staged scan ran). Post-processing (FFC, ENVI export, SWIR false-color PNG) runs in the background after capture when configured. Open the session path from the log or recorder status label.

When **spectral fusion** is enabled (see §6.7), fused products are written under each illumination mode, e.g. `dataset/reflectance/fusion/` (see §11).

### 6.7 Post-processing and spectral fusion

In the **Preprocessing** group (Capture tab):


| Control | Purpose |
| ------- | ------- |
| **Preprocess the image when the scanning is done** | Master switch for post-capture FFC, export, GSAM, and fusion. Requires a **staged stage scan** with dark/white references. |
| **Save FFC image** | Write flat-field corrected ENVI cubes under `preprocessed/`. Fusion can run without this checked — FFC cubes are still produced internally when fusion is on. |
| **Run GSAM segmentation** | Chip masks via GSAM2 WSL server → `preprocessed/segmentation/`. **Required for fusion.** |
| **Run spectral fusion (FX10e + SWIR3)** | After FFC + GSAM, align and fuse VNIR + SWIR per chip ROI for **each illumination mode** in the session (reflectance, transmittance, …). Enabled only when **both cameras** are connected and selected for capture. |
| **Run fusion on session…** | Re-run fusion offline on a saved session folder (all modes that contain `fx10e` and `swir3`). Does not require a new scan. |


Fusion runs in the background after post-processing. Watch the **Log** for lines starting with `Capture fusion:`.

### 6.8 Capture stream tab

While recording, the **Capture** stream tab can show waterfalls for connected cameras to monitor scan progress.

---

## 7. UR3e tab

Placeholder for future **Universal Robots UR3e** integration (not active in current builds). No connect or motion actions yet.

---

## 8. Configuration — `hyperfusion.cfg`

Edit beside `app.exe`; **restart the app** to reload.

### `[sample_stage_position]` (mm)


| Key                                           | Meaning                                              |
| --------------------------------------------- | ---------------------------------------------------- |
| `distance_dual_camera_mm`                     | FX10e − SWIR3 offset along rail                      |
| `white_ref_fx10e_mm` / `white_ref_swir3_mm`   | White reference scan position                        |
| `bright_ref_fx10e_mm` / `bright_ref_swir3_mm` | Bright reference position                            |
| `sample_scanning_starting_position_`*         | Sample scan start                                    |
| `temp_stop_position_mm`                       | Stage position between reflectance and transmittance |


Expressions like `white_ref_swir3_mm = white_ref_fx10e_mm - distance_dual_camera_mm` are supported.

### `[camera_calibration]`


| Key                                                           | Meaning                                                              |
| ------------------------------------------------------------- | -------------------------------------------------------------------- |
| `fx10e_spatial_mm_per_pixel` / `swir3_spatial_mm_per_pixel`   | Line spacing for scan speed and dual-camera sync                     |
| `fx10e_spatial_fwhm_mm` / `swir3_spatial_fwhm_mm`             | Spatial resolution FWHM (record only — not applied yet)              |
| `fx10e_spectral_nm_per_pixel` / `swir3_spectral_nm_per_pixel` | Spectral sampling (record only)                                      |
| `fx10e_spectral_fwhm_nm` / `swir3_spectral_fwhm_nm`           | Spectral band FWHM (record only)                                     |
| `swir3_auto_nuc`                                              | Enable SDK **AutoNUC** after timing apply (recommended `true`)       |
| `swir3_column_profile_correct`                                  | **Column profile destripe** (see §9). When `true`, **SDK BPR is off**. |
| `swir3_column_profile_baseline_radius`                        | ±columns for spatial baseline median when detecting valleys          |
| `swir3_column_profile_valley_gain_min`                        | Valley threshold: profile/baseline below this marks a bad column     |
| `swir3_column_profile_min_band_dn`                            | Ignore bands below this DN when building the spatial profile         |
| `swir3_column_profile_min_hits`                               | Consecutive valley frames before marking a column                      |
| `swir3_column_profile_min_valley_dn`                          | Optional absolute valley depth (0 = off)                             |


### `[scanning_settings]`


| Key                                                  | Meaning                                 |
| ---------------------------------------------------- | --------------------------------------- |
| `operation_scanning_speed_mm_per_sec`                | Default scan speed                      |
| `acceleration_mm_per_sec2`                           | Stage acceleration                      |
| `white_reference_frames` / `black_reference_frames`  | Reference frame counts                  |
| `sample_window_max_length_mm`                        | Max sample length                       |
| `fx10e_transmittance_exp` / `swir_transmittance_exp` | Exposure (ms) during transmittance pass |


### `[lighthouse]`

Idle, reflectance, and transmittance intensity percentages.

### `[preprocessing]`


| Key                                    | Meaning                                           |
| -------------------------------------- | ------------------------------------------------- |
| `ffc_`*, `truncate_nm`, `illuminant_d` | Flat-field correction and export                  |
| `swir_false_color_*_nm_`*              | RGB channel wavelength ranges for SWIR PNG export |


### `[segmentation]` (optional GSAM)

WSL distro, port, and model paths for the GSAM2 sidecar. See `resources/gsam2/envsetup.md`.

### `[fusion]` (dual-camera)

Offline **spatial registration** and **spectral fusion** of FX10e + SWIR3 (Python subprocess). See §11 and `resources/hf_fusion/README.md`.


| Key | Meaning |
| --- | ------- |
| `fusion_margin_mm` | Crop margin around chip masks when fusing (default **5.0** mm) |
| `fusion_timeout_ms` | Max wait for one fusion subprocess (default 3600000 ms) |

Pipeline and Python venv are always `resources/hf_fusion/` and `resources/hf_fusion/.venv/` (run `setup_venv.ps1` once there).


Spatial scales for alignment come from `[camera_calibration]` (`fx10e_spatial_mm_per_pixel`, `swir3_spatial_mm_per_pixel`).

---

## 9. SWIR3 image quality — NUC and column profile destripe

### Auto NUC (`swir3_auto_nuc = true`)

The Lumo SDK selects the best **NUC table** for the current exposure. Helps stabilize **column offset/gain drift** as the sensor temperature changes. Not a substitute for adequate **warm-up** time.

### Column profile destripe (`swir3_column_profile_correct = true`)

HyperFusion’s **column comb** correction for vertical striping:

1. Each frame, computes the **median DN across all bands** at each spatial column → spatial profile `P[x]`.
2. Compares `P[x]` to a local baseline (median of `P` within ±`baseline_radius` columns).
3. Columns where `P[x] / baseline` falls below `valley_gain_min` are marked bad.
4. All bands at bad columns are replaced with the mean of the nearest good left/right columns.
5. **SDK `Camera.BPR` is disabled** while this mode is on.

**When to use SDK BPR instead** (`swir3_column_profile_correct = false`)

- Stable sensor, static calpack map sufficient, prefer vendor-default processing.

**Tuning**

- More aggressive: lower `valley_gain_min` (e.g. 0.85), widen `baseline_radius`.
- More conservative: raise `valley_gain_min`, raise `min_hits`, set `min_valley_dn`.

Connect log shows `ColumnProfile=on, bad_columns=…` while streaming.

---

## 10. Optional — GSAM2 segmentation

Post-capture object segmentation via a **WSL Python server** (similar pattern to a sidecar service).

1. Set up WSL per `resources/gsam2/envsetup.md`.
2. Configure `[segmentation]` in `hyperfusion.cfg`.
3. HyperFusion can auto-start the server on launch when `warmup_on_start = true`.

Not required for camera operation or `.raw` recording.

---

## 11. Dual-camera spectral fusion

HyperFusion can combine **VNIR (FX10e)** and **SWIR (SWIR3)** into one spatially aligned, spectrally continuous dataset per detected chip. This runs **after capture** as an offline Python pipeline (`hf_fusion/`), launched by the app as a one-shot subprocess.

### What it does

1. **Spatial registration** — Upsample SWIR to the FX10e ground grid, match chip centroids, apply per-ROI shifts, then refine with overlap-band phase correlation (optional scale search).
2. **Spectral fusion** — Stitch aligned cubes at a configurable wavelength split (default 1000 nm) into a single ENVI BIL cube per ROI.

### Requirements

Per camera, under `{session}/{mode}/{camera}/preprocessed/`:

- `*_rgb.png` (from post-processing)
- `*_ffc.hdr` / `.raw`
- `segmentation/segmentation_results.json` and `segmentation/masks/` (from GSAM)

Both **FX10e** and **SWIR3** must be present under the same illumination folder (e.g. `reflectance/`). Fusion runs **per mode** — if you captured reflectance and transmittance, both can be fused when prerequisites are met.

### Automatic fusion (after Record)

1. Enable **Preprocess the image when the scanning is done**.
2. Enable **Run GSAM segmentation** (GSAM2 server connected).
3. Enable **Run spectral fusion (FX10e + SWIR3)**.
4. Record a **dual-camera staged scan**.

Post-processing order: FFC → GSAM → fusion. If post-processing fails, fusion is skipped.

### Manual fusion (saved sessions)

Use **Run fusion on session…** in the Preprocessing group to pick a session folder and re-run fusion without scanning again. Useful after tuning GSAM, fixing masks, or deploying an updated `hf_fusion` pipeline.

### Output layout

```
{session}/{mode}/fusion/
  metadata/
    alignment.json          # shifts, refine stats, per-ROI outputs
    fx10e_centroids.png
    swir3_centroids.png
  roi_{NNN}_fx10e_swir3/
    roi_{NNN}_fx10e_swir3.raw / .hdr   # fused hyperspectral cube
    roi_{NNN}_rgb_overlay.png
    roi_{NNN}_maskoverlay_outline.png
    roi_{NNN}_mask.npy / .png
```

### Troubleshooting

| Symptom | Check |
| ------- | ----- |
| Fusion checkbox greyed out | Both cameras connected and selected; preprocessing enabled; staged scan mode. |
| `fusion_cli.py not found` | Ensure the repo has `resources/hf_fusion/fusion_cli.py` (app resolves that path). |
| Python / venv missing | Run `resources/hf_fusion/setup_venv.ps1` once. |
| `Object count mismatch` | FX10e and SWIR3 must detect the **same number** of chips (sorted left-to-right pairing). |
| Prerequisites error in log | Missing RGB, FFC, or segmentation under one or both cameras for that mode. |

Developer setup: **SETUP.md** §5 and `resources/hf_fusion/README.md`.

---

## 12. Quick reference — daily operator flow

```
Power hardware → Start HyperFusion
  → Stage: Connect, Home
  → Light: Connect
  → Camera(s): Select calpack → Connect → check stream
  → Adjust exposure / FPS if needed
  → Capture: select cameras & modes → Preview (optional) → Record
  → (Optional) enable preprocessing, GSAM, spectral fusion before Record
  → Stop when done → note session path in Log
  → Check Log for post-process and fusion status; fused data under …/fusion/
```

