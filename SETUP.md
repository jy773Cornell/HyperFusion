# HyperFusion — setup guide

This guide covers **developer setup** on a build machine.

## Contents

1. [Overview](#overview)
2. [Development environment](#1-development-environment)
3. [Camera SDKs and drivers](#2-camera-sdks-and-drivers)
  - [2.1 Specim Lumo Sensor SDK](#21-specim-lumo-sensor-sdk)
  - [2.2 Pleora eBUS (FX10e)](#22-pleora-ebus-fx10e)
  - [2.3 National Instruments (SWIR3)](#23-national-instruments-swir3)
  - [2.4 Dual-camera operation](#24-dual-camera-operation)
4. [Stage and lighthouse (full bench)](#3-stage-and-lighthouse-full-bench)
  - [3.1 Zaber Motion Library](#31-zaber-motion-library)
  - [3.2 MCC Universal Library](#32-mcc-universal-library)
5. [Segmentation — GSAM segmentation](#4-segmentation--gsam-segmentation)
6. [Fusion — Dual-camera fusion](#5-fusion--dual-camera-fusion)
7. [Robot Arm — UR3e robot (WSL ROS 2)](#6-robot-arm--ur3e-robot-wsl-ros-2)
8. [Configuration — `hyperfusion.cfg`](#7-configuration--hyperfusioncfg)
9. [Build HyperFusion](#8-build-hyperfusion)

---



## Overview


| Stage                | What you install                         | What works                                         |
| -------------------- | ---------------------------------------- | -------------------------------------------------- |
| **1 — Cameras only** | Lumo SDK, Pleora eBUS, NI Vision (SWIR3) | Connect FX10e and/or SWIR3; stream and record      |
| **2 — Full bench**   | + Zaber Motion Library, MCC UL           | Stage scanning, lighthouse, capture workflows      |
| **3 — Optional**     | WSL + GSAM (see `app/sidecars/gsam2/`)      | Post-capture segmentation                          |
| **4 — Optional**     | Python venv + `hf_fusion` (see below)    | Dual-camera spatial registration + spectral fusion |
| **5 — Optional**     | WSL + ROS 2 + `app/sidecars/ur3e/` (see below) | UR3e robot sidecar (`use_mock_hardware` for simulation) |


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

**Calibration packs (**`.scp`**):** ship under `app/calibration/fx10e/` and `app/calibration/swir/`. Use the pack that matches your camera serial and lens.

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



### 3.1 Zaber Motion Library

- Install **Zaber Motion Library (ZML)**
- One-time stage setup: use **Zaber Launcher** (LC40B profile, motor orientation). HyperFusion re-applies travel limits and lockstep on each connect.
- Docs: `docs/Zaber/documentation_index.md`



### 3.2 MCC Universal Library

- Hardware: **Measurement Computing USB-1208FS-Plus** (or compatible 1208 FS Plus family).
- Install **MCC UL** from Measurement Computing (provides `cbw64.dll`).
- Run **InstaCal** if the lighthouse board is new or not detected.

---



## 4. Segmentation — GSAM segmentation

- Requires **WSL2 + Ubuntu** and the Python env under `app/sidecars/gsam2/`.
- See `app/sidecars/gsam2/README.md`.
- Tune `[segmentation]` in `hyperfusion.cfg` (WSL distro, port, model paths).
- Not required for camera streaming or capture.

---



## 5. Fusion — Dual-camera fusion

Offline **FX10e + SWIR3** pipeline: spatial registration (centroid match + phase correction) and spectral fusion into unified ENVI cubes per chip ROI. Lives in `app/sidecars/hf_fusion/`.

### 5.1 Python environment

The app uses **one venv only**: `app/sidecars/hf_fusion/.venv/Scripts/python.exe`.

**First-time setup:**

```powershell
cd app\sidecars\hf_fusion
.\setup_venv.ps1
```

Verify in the app Log: `Capture fusion: python=…\app\sidecars\hf_fusion\.venv\Scripts\python.exe`

Dependencies: `numpy`, `Pillow`, `opencv-python` (see `requirements.txt`).

### 5.2 Configuration

Add or edit `[fusion]` in `hyperfusion.cfg` beside `app.exe`:


| Key                 | Purpose                                            |
| ------------------- | -------------------------------------------------- |
| `fusion_margin_mm`  | Crop margin around chip masks (**5.0** mm default) |
| `fusion_timeout_ms` | Subprocess timeout (default 3600000 ms)            |


Pipeline path and Python venv are fixed at `app/sidecars/hf_fusion/` and `app/sidecars/hf_fusion/.venv/` (not configurable).

Alignment uses `fx10e_spatial_mm_per_pixel` and `swir3_spatial_mm_per_pixel` from `[camera_calibration]`.

### 5.3 Prerequisites (per capture session)

Fusion expects post-processed data under `{session}/{mode}/{camera}/preprocessed/` for **both** cameras:

- `*_rgb.png`, `*_ffc.hdr` / `.raw`
- GSAM outputs: `segmentation/segmentation_results.json`, `segmentation/masks/`

The app can run fusion automatically after capture (Capture tab → **Run spectral fusion**) or manually via **Run fusion on session…**. See **USERMANUAL.md** §11.

### 5.4 Manual CLI (debugging)

```powershell
cd app\sidecars\hf_fusion
.\.venv\Scripts\Activate.ps1
python fusion_cli.py --session E:\path\to\session --mode reflectance
```

Full pipeline docs: `app/sidecars/hf_fusion/README.md`.

---



## 6. Robot Arm — UR3e robot (WSL ROS 2)

Universal Robots **UR3e** control runs as a **WSL sidecar** (same pattern as GSAM2): a small HTTP server in Ubuntu talks to the ROS 2 driver; the Windows app will call it via `wsl.exe`.

### 6.1 Prerequisites

| Item | Notes |
|------|--------|
| **WSL 2 + Ubuntu 22.04** | Required for ROS 2 Humble |
| **Robot on LAN** | Default IP `192.168.0.10` (edit `[ur3e]` in `hyperfusion.cfg`) |
| **Lab safety** | E-stop accessible; no motion until explicit connect from the UR3e tab |

If WSL cannot ping the robot, or the teach pendant reports **cannot reach remote PC**, run the one-time network setup (mirrored WSL + inbound firewall for UR reverse ports):

```powershell
cd D:\Pototypy\HyperFusion\app\sidecars\ur3e
.\install_env.ps1 -SetupRobotNetwork -ShutdownWsl
```

Or after a build:

```powershell
cd D:\Pototypy\HyperFusion\app
.\build_app.ps1 -NoRun -SetupUrRobotNetwork -ShutdownWslAfterUrNetworkSetup
```

Firewall rules require **Administrator** PowerShell. The script is idempotent and does **not** run on every compile unless you pass those flags.

### 6.2 One-time WSL setup

From **PowerShell** (delegates to WSL):

```powershell
cd D:\Pototypy\HyperFusion\app\sidecars\ur3e   # adjust path
.\install_env.ps1
```

Or from **WSL**, in `app/sidecars/ur3e/`:

```bash
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e   # adjust path
chmod +x install_env.sh
./install_env.sh
```

Do **not** run `./install_env.sh` directly in PowerShell — it is a bash script and will not execute.

This installs **ROS 2 + `ros-*-ur`** and creates `./venv` for the HTTP sidecar.

Full details: **`app/sidecars/ur3e/README.md`**.

### 6.3 Verify sidecar (simulation)

```bash
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e
source /opt/ros/jazzy/setup.bash   # or humble on Ubuntu 22.04
./venv/bin/ur3e_server --use-mock-hardware --port 8766
```

From **Windows PowerShell**:

```powershell
wsl curl -s http://127.0.0.1:8766/health
wsl curl -s -X POST http://127.0.0.1:8766/connect -H "Content-Type: application/json" -d "{}"
```

Robot reachability from WSL:

```bash
./scripts/check_robot_network.sh 192.168.0.10
```

### 6.4 Configuration

`[ur3e]` in `hyperfusion.cfg` (reloaded on app start):


| Key | Purpose |
|-----|---------|
| `wsl_distro` | WSL distribution name (default `Ubuntu`) |
| `ur3e_repo_linux` | WSL path to sidecar; empty = auto `app/sidecars/ur3e` |
| `server_port` | HTTP port (default **8766**, GSAM uses 8765) |
| `robot_ip` | UR controller IP |
| `dashboard_port` / `rtde_port` | UR dashboard / RTDE ports |
| `use_mock_hardware` | `true` = ROS simulation (no physical robot); `false` = real arm |
| `ros_distro` / `ur_type` | ROS distro (`jazzy` / `humble`) and UR model (`ur3e`) |
| `prestart_driver` | Warm ROS driver in background when sidecar starts (default `false`; use with simulation first) |
| `max_linear_speed_m_per_s` / `max_linear_accel_m_per_s2` | Motion caps |

C++ app integration (`Ur3eServerManager`, UR3e tab wiring) is added incrementally; the sidecar can be tested standalone now.

---



## 7. Configuration — `hyperfusion.cfg`

Copied next to `app.exe` on build from `app/preset/`. **Reloaded on every app start.** Edit the copy in `app/preset/` (not the output folder) for each bench:


| Section                   | Purpose                                                                                                             |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `[camera_calibration]`    | Spatial scale, spectral metadata (record-only FWHM keys), SWIR3 AutoNUC and column profile destripe                 |
| `[sample_stage_position]` | White/bright/sample scan positions (mm), dual-camera offset                                                         |
| `[scanning_settings]`     | Scan speed, acceleration, ref frame counts, `fx10e_transmittance_exp`, `swir_transmittance_exp` (dual-mode capture) |
| `[lighthouse]`            | Idle / reflectance / transmittance intensity (%)                                                                    |
| `[preprocessing]`         | FFC, SWIR ref BPR (`swir3_ref_bpr`), and SWIR false-color export settings                                           |
| `[segmentation]`          | GSAM2 sidecar (optional)                                                                                            |
| `[fusion]`                | Dual-camera fusion Python paths, margin, timeout                                                                    |
| `[ur3e]`                  | UR3e WSL sidecar (optional)                                                                                         |


Camera exposure, frame rate, binning, and RGB band picks are stored in **app settings** (QSettings), not in this file.

---



## 8. Build HyperFusion

From the `app` folder:

```powershell
cd app
.\build_app.ps1
```

Common variants:

```powershell
.\build_app.ps1 -Config Release -NoRun    # build only, do not launch
.\build_app.ps1 -NoClean                  # reuse existing build directory
```

The script configures CMake, builds **Release**, runs `windeployqt`, and copies Lumo / MCC / Zaber runtime DLLs, `app/preset/` (including `hyperfusion.cfg`, GSAM plans, and UR3e semi scan plans), and calibration packs next to the executable. Fusion uses `app/sidecars/hf_fusion/` in the repo (not copied beside `app.exe`).

**Output folder:** `build\Release\` (under `app\`)


| Override | Environment variable |
| -------- | -------------------- |
| Qt       | `QT_PREFIX_PATH`     |
| Lumo SDK | `LUMO_SDK_ROOT`      |
| Zaber    | `ZML_ROOT`           |
| MCC UL   | `MCC_UL_ROOT`        |


Use **Release** builds for lab deployment; Debug is for development only.