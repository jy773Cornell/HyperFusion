# HyperFusion — setup guide

HyperFusion is a Windows desktop app for multimodal hyperspectral imaging with Specim FX10e + SWIR3 cameras, Zaber scan stage, and MCC lighthouse control. This guide covers **developer setup** on a build machine.

---

## Overview


| Stage                | What you install                         | What works                                    |
| -------------------- | ---------------------------------------- | --------------------------------------------- |
| **1 — Cameras only** | Lumo SDK, Pleora eBUS, NI Vision (SWIR3) | Connect FX10e and/or SWIR3; stream and record |
| **2 — Full bench**   | + Zaber Motion Library, MCC UL           | Stage scanning, lighthouse, capture workflows |
| **3 — Optional**     | WSL + GSAM (see `resources/sam2/`)       | Post-capture segmentation                     |


Install **Stage 1** first and verify cameras in Lumo/NI MAX before adding stage and light hardware.

---

## 1. Development environment

Install on the machine where you build HyperFusion


| Tool                           | Notes                                                                                                                                                                                                                 |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Visual Studio 2022 or 2026** | Workload: *Desktop development with C++* (MSVC **x64** + Windows SDK). Installer: `docs/VisualStudioSetup.exe`                                                                                                        |
| **CMake**                      | On PATH (VS component or [cmake.org](https://cmake.org)). Version 3.16+.                                                                                                                                              |
| **Qt 6.11.1**                  | Via Qt Maintenance Tool[https://doc.qt.io/qt-6/qt-online-installation.htm](https://doc.qt.io/qt-6/qt-online-installation.htm)l → **MSVC 2022 64-bit** kit.                                                            |
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

- Install **Zaber Motion Library (ZML)** → default: `C:\Program Files\Zaber Motion Library`  
(Use the ZML installer, not only the C++ headers package in `docs/Zaber/` unless you are developing the adapter.)
- One-time stage setup: use **Zaber Launcher** (LC40B profile, motor orientation). HyperFusion re-applies travel limits and lockstep on each connect.
- Docs: `docs/Zaber/documentation_index.md`

### 3.2 Measurement Computing Universal Library (lighthouse)

- Hardware: **Measurement Computing USB-1208FS-Plus** (or compatible 1208 FS Plus family).
- Install **MCC UL** from Measurement Computing (provides `cbw64.dll`).
- Default path: `C:\Program Files (x86)\Measurement Computing\DAQ`
- Run **InstaCal** if the lighthouse board is new or not detected.

---

## 4. Optional — GSAM segmentation

- Requires **WSL2 + Ubuntu** and the Python env under `resources/sam2/`.
- See `resources/sam2/envsetup.md`.
- Tune `[segmentation]` in `hyperfusion.cfg` (WSL distro, port, model paths).
- Not required for camera streaming or capture.

---

## 5. Configuration — `hyperfusion.cfg`

Copied next to `app.exe` on build. **Reloaded on every app start.** Edit for each bench:


| Section                   | Purpose                                                                                                        |
| ------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `[sample_stage_position]` | White/bright/sample scan positions (mm), dual-camera offset                                                    |
| `[scanning_settings]`     | Scan speed, acceleration, ref frame counts, `fx10e_spatial_mm_per_pixel`, `swir3_spatial_mm_per_pixel`, `fx10e_transmittance_exp`, `swir_transmittance_exp` (dual-mode capture) |
| `[lighthouse]`            | Idle / reflectance / transmittance intensity (%)                                                               |
| `[preprocessing]`         | FFC and SWIR false-color wavelength ranges                                                                     |
| `[segmentation]`          | GSAM2 sidecar (optional)                                                                                       |


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

---

## 7. Run the application

### On the build machine

After a normal build, run:

```text
app\build\Release\app.exe
```

Close any running `app.exe` before rebuilding (otherwise the linker may fail with LNK1104).

### On another lab PC (portable deploy)

1. Build with `.\build_app.ps1 -Config Release -NoRun`
2. Copy or zip the entire folder `**app\build\Release\**` (~100 MB)
3. Unzip on the target PC (e.g. `C:\HyperFusion\`) and run `**app.exe**`
4. On the target PC, still install: **VC++ Redistributable x64**, **Lumo license**, **NI drivers** (SWIR3), **Pleora/eBUS** (FX10e) as needed
5. Edit `**hyperfusion.cfg`** beside `app.exe` for that bench

Bundled with the app folder:

- `hyperfusion.cfg` — hardware parameters  
- `calibration/fx10e/` and `calibration/swir/` — `.scp` calibration packs  
- `external/NI/` — NI camera ICD files (including `Specim_SWIR3.icd`)  
- Qt, Lumo, Zaber, and MCC runtime DLLs (see build output listing)

Optional: remove `tbb_*debug*.dll` from the zip before deploy (not needed at runtime).

---

## 8. First-run checklist

- [ ] VC++ Redistributable x64 installed  
- [ ] Lumo license active  
- [ ] FX10e: Pleora/eBUS OK; SWIR3: NI MAX snap/grab OK, then **close MAX**  
- [ ] Calibration pack selected in Camera settings **before connect** (matches sensor serial)  
- [ ] `hyperfusion.cfg` matches this bench (stage positions, spatial mm/px, scan speed)  
- [ ] Stage and lighthouse connected if using Capture tab scanning  
- [ ] SWIR cooled and stable before long recordings  

---

## 9. Troubleshooting


| Symptom                            | Things to check                                                                                                  |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| Build: Lumo / SpecSensor not found | Install SDK; set `LUMO_SDK_ROOT`                                                                                 |
| Build: LNK1104 on `app.exe`        | Close running HyperFusion                                                                                        |
| SWIR3 connect fails                | NI MAX grab stopped; FX10e disconnected; ICD at `external/NI/Specim_SWIR3.icd`                                   |
| Vertical stripes on SWIR           | Correct `.scp` loaded; exposure matches cal; BPR map current                                                     |
| Dual-camera sync wrong rates       | `fx10e_spatial_mm_per_pixel` / `swir3_spatial_mm_per_pixel` in `[scanning_settings]`; restart app after cfg edit |
| Stage does not move                | ZML installed; correct COM port in Stage tab                                                                     |
| Lighthouse not found               | MCC UL + InstaCal; USB-1208FS-Plus connected                                                                     |
| GSAM fails                         | WSL running; `resources/sam2/envsetup.md`; `[segmentation]` in cfg                                               |


---

## Source layout

See `README.md` for repository structure (`adapters` / `backend` / `frontend`).