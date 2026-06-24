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
  - [6.7 Capture stream tab](#67-capture-stream-tab)
7. [UR3e tab](#7-ur3e-tab)
8. [Configuration — `hyperfusion.cfg](#8-configuration--hyperfusioncfg)`
  - `[[sample_stage_position]](#sample_stage_position-mm)`
  - `[[camera_calibration]](#camera_calibration)`
  - `[[scanning_settings]](#scanning_settings)`
  - `[[lighthouse]](#lighthouse)`
  - `[[preprocessing]](#preprocessing)`
  - `[[segmentation]` (GSAM)](#segmentation-optional-gsam)
9. [SWIR3 image quality — NUC and adaptive BPR](#9-swir3-image-quality--nuc-and-adaptive-bpr)
  - [Auto NUC](#auto-nuc-swir3_auto_nuc--true)
  - [Adaptive BPR](#adaptive-bpr-swir3_adaptive_bpr--true)
10. [Optional — GSAM2 segmentation](#10-optional--gsam2-segmentation)
11. [Quick reference — daily operator flow](#11-quick-reference--daily-operator-flow)

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
`SDK.BPR=…`, `AdaptiveBPR=…`, `AutoNUC=on`, `NUC=…`

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

### 6.7 Capture stream tab

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
| `swir3_adaptive_bpr`                                          | **Adaptive software BPR** (see §9). When `true`, **SDK BPR is off**. |
| `swir3_adaptive_bpr_gain_min` / `gain_max`                    | Outlier ratio thresholds (default 0.3 / 1.5)                         |
| `swir3_adaptive_bpr_min_neighbor_dn`                          | Minimum neighbor brightness to run detection                         |
| `swir3_adaptive_bpr_min_hits`                                 | Consecutive bad frames before adding a pixel                         |
| `swir3_adaptive_bpr_max_pixels`                               | Safety cap on new adaptive bad pixels                                |


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

WSL distro, port, and model paths for the GSAM2 sidecar. See `resources/sam2/envsetup.md`.

---

## 9. SWIR3 image quality — NUC and adaptive BPR

### Auto NUC (`swir3_auto_nuc = true`)

The Lumo SDK selects the best **NUC table** for the current exposure. Helps stabilize **column offset/gain drift** as the sensor temperature changes. Not a substitute for adequate **warm-up** time.

### Adaptive BPR (`swir3_adaptive_bpr = true`)

HyperFusion’s **stream-adaptive** bad-pixel replacement:

1. Loads the factory **BPR map** from the calpack (~120 pixels).
2. Each frame, compares every pixel to left/right neighbors (same spectral band).
3. If the ratio is outside `gain_min`…`gain_max` for several consecutive frames, the pixel is added to an **adaptive** mask.
4. Bad pixels (baseline + adaptive) are replaced with neighbor means in space and wavelength.
5. **SDK `Camera.BPR` is disabled** while this mode is on.

**When to use**

- Vertical striping / column comb not fully covered by the static calpack map.
- Defects that **drift** during a session.

**When to use SDK BPR instead** (`swir3_adaptive_bpr = false`)

- Stable sensor, static map sufficient, prefer vendor-default processing.

**Tuning**

- More aggressive: widen `gain_min`/`gain_max`, lower `min_hits`.
- More conservative: narrow gains, raise `min_hits`, lower `max_pixels`.

Connect log shows `baseline=`, `adaptive=`, and `active=` counts while streaming.

---

## 10. Optional — GSAM2 segmentation

Post-capture object segmentation via a **WSL Python server** (similar pattern to a sidecar service).

1. Set up WSL per `resources/sam2/envsetup.md`.
2. Configure `[segmentation]` in `hyperfusion.cfg`.
3. HyperFusion can auto-start the server on launch when `warmup_on_start = true`.

Not required for camera operation or `.raw` recording.

---

## 11. Quick reference — daily operator flow

```
Power hardware → Start HyperFusion
  → Stage: Connect, Home
  → Light: Connect
  → Camera(s): Select calpack → Connect → check stream
  → Adjust exposure / FPS if needed
  → Capture: select cameras & modes → Preview (optional) → Record
  → Stop when done → note session path in Log
```

