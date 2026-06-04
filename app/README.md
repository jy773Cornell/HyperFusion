HyperFusion app — Windows build (MSVC)

## Source layout

| Layer | Path | Responsibility |
|--------|------|----------------|
| **adapters** | `include/adapters/`, `src/adapters/` | Vendor SDK and hardware only (Lumo, Zaber, …). No Qt widgets. |
| **backend** | `include/backend/`, `src/backend/` | Device-agnostic layer: contracts (`ICameraController`, `IStageController`), workers, orchestration, `.raw` dump I/O. |
| **frontend** | `include/frontend/`, `src/frontend/` | Qt application layer (see subfolders below). |

Dependency direction: **frontend → backend → adapters**. Adapters must not include Qt Widgets.

### Frontend subfolders

| Subfolder | Role |
|-----------|------|
| `widgets/` | `QWidget` subclasses only: `MainWindow`, plots, `StageAxisWidget`, `LumoCameraUi` |
| `processing/` | Stream pipelines: `ProfileProcessor`, `WaterfallProcessor`, extractors, frame converter |
| `controllers/` | App flow: `CameraAppController` (optional; not linked in CMake yet) |
| `settings/` | `AppSettingsStore` (QSettings) |
| `utils/` | Small helpers: `SerialPortEnumerator` |

### Backend (flat under `backend/`)

- `CameraCoordinator` — multi-camera worker routing
- `HyperspectralRawDumper` — hyperspectral `.raw` session writer
- `CameraWorker`, `StageWorker`, interface types

---

USER
  Install Lumo Sensor SDK (Specim).
  D:\Pototypy\HyperFusion\docs\SWIR\ni1433\NI - specim compatible\VAS1851
  Install Zaber Motion Library (ZML) when using the scan stage.

  Note on LC40B / orientation
The Launcher wizard (stage type LC40B, motor orientation Right/Left) writes a full peripheral configuration to the controller via ZML’s internal device database — typically done once with Save without Testing. HyperFusion re-applies the runtime settings each connect (travel limits + lockstep). Motor direction/orientation should already be stored on the X-MCC2 from that Launcher save; if axes move the wrong way on first use, save the Launcher config once, then reconnect in HyperFusion.

DEVELOPER — one-time
  Visual Studio 2022 or 2026: workload "Desktop development with C++"
    + MSVC build tools (e.g. v14.44) + Windows SDK
  Qt Maintenance Tool: Qt 6.11.1 → MSVC 2022 64-bit
    Path: C:\Qt\6.11.1\msvc2022_64
  ZML: run installer → C:\Program Files\Zaber Motion Library
  CMake on PATH (VS component or https://cmake.org)

BUILD
  From repo root:
    .\build_app.ps1
  Debug, no launch:
    .\build_app.ps1 -Config Debug -NoRun

Capture tab (Record / Preview)
  Record: fill Dataset + Save folder (Metadata), select a checked camera that is streaming
  (preview on Camera tab). Stage optional for record — without stage, press Stop when done.
  Preview: requires stage Connected on the Stage tab.

SWIR3 / NI — step-by-step debug (NI MAX works, HyperFusion -1101)
  1. Quit NI MAX completely (not only Stop Grab).
  2. Rebuild and run: .\build_app.ps1 — on Connect, log must show Grabber.Channel=img0, Scb=(none), Specim_SWIR3.icd exists=yes.
  3. Disconnect FX10e and Zaber stage in HyperFusion; connect only SWIR3, profile "SWIR3 with NI".
  4. Do not point Scb at COM4 if that port is the Zaber USB serial (wrong device → fast -1101).
  5. In Lumo Recorder: SWIR3 with NI + img0 + Specim_SWIR3.icd (SSP default; same as working 1427 PC).
  6. Paste full log from Connect through error (including readback / Grabber.Channel options if present).

SWIR3 / NI — 1427 (works) vs 1433 (fails) Lumo log
  Both: grab001+cam007+scb load; Grabber autoconnect "img0"; two "AIM SWIR: 0 bytes returned" warnings.
  1427 then: "Camera autoconnect found a camera in port: COM3" → success (Camera.Channel = COM3 for cam007).
  1433 then: OpenSerialPort -1101, no COM autoconnect → no Windows COM for the camera head on that desk (or wrong COM).
  Fix on 1433: camera head USB + Specim/FTDI driver so a COM appears (often COM3); disconnect Zaber during connect test;
  in Lumo set Camera.Channel to that COM (not img0, not Zaber COM4). HyperFusion: niCameraSerialPort in buildCameraSettings.

SWIR3 / NI IMAQdx (Communication timeout / Specim -1101 on Initialize)
  Lumo "OpenSerialPort() in CCamera007::Initialize -1101" = cam007 / Camera.Channel serial, not PCIe-1433.
  HyperFusion: Grabber.Channel=img0, Specim_SWIR3.icd; set niCameraSerialPort only when Device Manager shows the camera COM (e.g. COM3 on 1427).

FX10e / Lumo connect errors
  If the log shows SI_Open: Loading the module failed, Pleora grabber DLLs are missing
  beside app.exe. Rebuild with .\build_app.ps1 (copies all DLLs from SDK bin\x64), or copy
  everything from %LUMO_SDK_ROOT%\bin\x64 next to app.exe. Launching from Explorer without
  that folder on PATH causes the same error.

VERIFY TOOLCHAIN
  Qt:  Test-Path "C:\Qt\6.11.1\msvc2022_64\bin\qmake.exe"
  ZML:  Test-Path "C:\Program Files\Zaber Motion Library\lib\Release\zaber-motion.lib"
  VS:   Open "Developer PowerShell for VS 2026" from Start, then run:  cl

STAGE (Zaber X-MCC2 + lockstep): see docs/Zaber/links.md
