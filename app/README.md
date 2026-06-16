HyperFusion app — Windows build (MSVC)

## Source layout


| Layer        | Path                                 | Responsibility                                                                                                       |
| ------------ | ------------------------------------ | -------------------------------------------------------------------------------------------------------------------- |
| **adapters** | `include/adapters/`, `src/adapters/` | Vendor SDK and hardware only (Lumo, Zaber, …). No Qt widgets.                                                        |
| **backend**  | `include/backend/`, `src/backend/`   | Device-agnostic layer: contracts (`ICameraController`, `IStageController`), workers, orchestration, `.raw` dump I/O. |
| **frontend** | `include/frontend/`, `src/frontend/` | Qt application layer (see subfolders below).                                                                         |


Dependency direction: **frontend → backend → adapters**. Adapters must not include Qt Widgets.

### Frontend subfolders


| Subfolder      | Role                                                                                    |
| -------------- | --------------------------------------------------------------------------------------- |
| `widgets/`     | `QWidget` subclasses only: `MainWindow`, plots, `StageAxisWidget`, `LumoCameraUi`       |
| `processing/`  | Stream pipelines: `ProfileProcessor`, `WaterfallProcessor`, extractors, frame converter |
| `controllers/` | App flow: `CameraAppController` (optional; not linked in CMake yet)                     |
| `settings/`    | `AppSettingsStore` (QSettings)                                                          |
| `utils/`       | Small helpers: `SerialPortEnumerator`                                                   |


### Backend (flat under `backend/`)

- `CameraCoordinator` — multi-camera worker routing
- `CaptureWriterWorker` — threaded Lumo-style dataset writer (`save folder/dataset/`)
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
  CMake on PATH (VS component or [https://cmake.org](https://cmake.org))

BUILD  
  From repo root:  
    .\build_app.ps1

