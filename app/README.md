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
  2. Rebuild and run: .\build_app.ps1 — on Connect, log must show Grabber.Channel=img0, NiImaq.CameraFile=Fenix SWIR.icd.
  3. Disconnect FX10e in HyperFusion; connect only SWIR3, profile "SWIR3 with NI".
  4. If -1101: in MainWindow.cpp set niScbSerialPort = "COM4" (match MAX ASRL/COM), rebuild, retry.
  5. In Lumo Recorder (if installed): same SSP profile + img0 + Fenix ICD — if Recorder fails too, fix NI/SCB outside HyperFusion.
  6. Paste full log from Connect through error (including "SDK Grabber.Channel options" if present).

SWIR3 / NI IMAQdx (Communication timeout / Specim -1101 on Initialize)
  The PDF "Communication timeout -1101" is mainly about USB-serial (SCB) to the camera head, not the frame grabber.
  NI MAX Grab working does not prove Lumo can open the same IMAQdx session — stop Grab and close MAX first.
  HyperFusion sets before Initialize: Grabber.Channel=img0, NiImaq.CameraFile=Fenix SWIR.icd (match NI MAX model).
  Scb serial is off by default; if -1101 persists, set niScbSerialPort to your COM port in buildCameraSettings. Match NI MAX exactly:
  IMAQdx camera name (e.g. img0 under PCIe-1433), ICD file, and serial port (ASRL/COM in MAX tree).
  Disconnect FX10e when testing SWIR alone. If -1101 persists, try empty Scb (disable COM4 default) or correct COM in MainWindow buildCameraSettings.

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
