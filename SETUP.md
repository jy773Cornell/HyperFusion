# HyperFusion — setup guide

This guide covers **developer setup** on a build machine.

---

## Overview


| Stage                | What you install                         | What works                                    |
| -------------------- | ---------------------------------------- | --------------------------------------------- |
| **1 — Cameras only** | Lumo SDK, Pleora eBUS, NI Vision (SWIR3) | Connect FX10e and/or SWIR3; stream and record |
| **2 — Full bench**   | + Zaber Motion Library, MCC UL           | Stage scanning, lighthouse, capture workflows |
| **3 — Optional**     | WSL + GSAM (see `resources/gsam2/`)       | Post-capture segmentation                     |


Install **Stage 1** first and verify cameras in Lumo/NI MAX before adding stage and light hardware.

---

## 1. Development environment

Install on the machine where you build HyperFusion


| Tool                           | Notes                                                                                                                                                                                                                 |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Visual Studio 2022 or 2026** | Workload: *Desktop development with C++* (MSVC **x64** + Windows SDK). Installer: `docs/VisualStudioSetup.exe`                                                                                                        |
| **CMake**                      | On PATH (VS component or [cmake.org](https://cmake.org)). Version 3.16+.                                                                                                                                              |
| **Qt 6.11.1**                  | [Qt Online Installer](https://www.qt.io/download-qt-installer): Qt **6.11.1**, **MSVC 2022 64-bit**, **Qt Widgets**.                                                                                                  |
| **VC++ Redistributable (x64)** | Required on **every** PC that runs `app.exe`, including deploy targets. Install [Microsoft Visual C++ Redistributable](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist) (VS 2015–2022, x64). |


---

## 2. Camera SDKs and drivers

### 2.1 Specim Lumo Sensor SDK

- Install the **Specim Lumo Sensor SDK** (SpecSensor).
- Default install path used by the build:  
`C:\Program Files (x86)\Specim\SDKs\SpecSensor\2020_519`
- During setup, select SSP profiles for your hardware:
  - **FX10e with Pleora**
  - **SWIR3 with NI**
- See `docs/LUMO/Lumo Sensor SDK.pdf` for SDK details.

Set `LUMO_SDK_ROOT` if you install elsewhere. A valid **Specim/Lumo license** is required at runtime (usually installed with the SDK; empty path uses Specim default search).

**Calibration packs (`.scp`):** ship under `app/calibration/fx10e/` and `app/calibration/swir/`. Use the pack that matches your camera serial and lens.

### 2.2 Pleora eBUS (FX10e)

- Install **Pleora eBUS SDK/runtime** for Camera Link / GigE (FX10e).
- See `docs/pleora_ebus/`.
- After install, confirm the camera appears in Pleora tools before using HyperFusion.

### 2.3 National Instruments (SWIR3)

- Install NI software from the Specim-compatible bundle:  
`docs/SWIR/ni1433/NI - specim compatible/VAS1851`
- Configure the **PCIe-1433** frame grabber in **NI MAX** (`img0`, `Specim_SWIR3.icd`).
- **Close NI MAX Grab** before connecting from HyperFusion.
- Allow the SWIR head to **cool and stabilize** after power-on (see `docs/SWIR/spectral-camera-swir-user-manual-2.5.pdf`) before critical captures.

### 2.4 Dual-camera operation

- HyperFusion can run **FX10e and SWIR3 together**; dual-camera scan sync matches SWIR3 frame rate to FX10e geometry using `hyperfusion.cfg` spatial scales.
- Connect order in UI: Camera tab → **Connect** per camera → stream starts automatically when initialized.

At this point you can operate the **full-spectrum module** (FX10e + SWIR3) for streaming and recording without the stage.

---

## 3. Stage and lighthouse (full bench)

### 3.1 Zaber Motion Library (scan stage)

- Install **Zaber Motion Library (ZML)**
- One-time stage setup: use **Zaber Launcher** (LC40B profile, motor orientation). HyperFusion re-applies travel limits and lockstep on each connect.
- Docs: `docs/Zaber/documentation_index.md`

### 3.2 Measurement Computing Universal Library (lighthouse)

- Hardware: **Measurement Computing USB-1208FS-Plus** (or compatible 1208 FS Plus family).
- Install **MCC UL** from Measurement Computing (provides `cbw64.dll`).
- Run **InstaCal** if the lighthouse board is new or not detected.

---

## 4. Optional — GSAM segmentation

- Requires **WSL2 + Ubuntu** and the Python env under `resources/gsam2/`.
- See `resources/gsam2/envsetup.md`.
- Tune `[segmentation]` in `hyperfusion.cfg` (WSL distro, port, model paths).
- Not required for camera streaming or capture.

---

## 5. Configuration — `hyperfusion.cfg`

Copied next to `app.exe` on build. **Reloaded on every app start.** Edit for each bench:


| Section                   | Purpose                                                                                                             |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `[camera_calibration]`    | Spatial scale, spectral metadata (record-only FWHM keys), SWIR3 AutoNUC and adaptive BPR                            |
| `[sample_stage_position]` | White/bright/sample scan positions (mm), dual-camera offset                                                         |
| `[scanning_settings]`     | Scan speed, acceleration, ref frame counts, `fx10e_transmittance_exp`, `swir_transmittance_exp` (dual-mode capture) |
| `[lighthouse]`            | Idle / reflectance / transmittance intensity (%)                                                                    |
| `[preprocessing]`         | FFC and SWIR false-color export settings                                                                            |
| `[segmentation]`          | GSAM2 sidecar (optional)                                                                                            |


Camera exposure, frame rate, binning, and RGB band picks are stored in **app settings** (QSettings), not in this file.

---

## 6. Build HyperFusion

From the **repository root**:

```powershell
.\build_app.ps1
```

Common variants:

```powershell
.\build_app.ps1 -Config Release -NoRun    # build only, do not launch
.\build_app.ps1 -NoClean                  # reuse existing build directory
```

The script configures CMake, builds **Release**, runs `windeployqt`, and copies Lumo / MCC / Zaber runtime DLLs next to the executable.

**Output folder:** `app\build\Release\`


| Override | Environment variable |
| -------- | -------------------- |
| Qt       | `QT_PREFIX_PATH`     |
| Lumo SDK | `LUMO_SDK_ROOT`      |
| Zaber    | `ZML_ROOT`           |
| MCC UL   | `MCC_UL_ROOT`        |


Use **Release** builds for lab deployment; Debug is for development only.