HyperFusion app — Windows build (MSVC)

USER
  Install Lumo Sensor SDK (Specim).
  Install Zaber Motion Library (ZML) when using the scan stage.

DEVELOPER — one-time
  Visual Studio 2022 or 2026: workload "Desktop development with C++"
    + MSVC build tools (e.g. v14.44) + Windows SDK
    Open Visual Studio Installer → Modify on Community 2026.
Check the workload: Desktop development with C++ (main checkbox, not only individual components).
On the right, ensure:
MSVC v143 (or latest) build tools — you already have this
Windows 11 SDK or Windows 10 SDK (required)
C++ CMake tools for Windows (optional but helpful)
Install / Update, then reboot if prompted.

  Qt Maintenance Tool: Qt 6.11.1 → MSVC 2022 64-bit
    Path: C:\Qt\6.11.1\msvc2022_64
  ZML: run installer → C:\Program Files\Zaber Motion Library
  CMake on PATH (VS component or https://cmake.org)

BUILD
  From repo root:
    .\build_app.ps1
  Debug, no launch:
    .\build_app.ps1 -Config Debug -NoRun

VERIFY TOOLCHAIN
  Qt:  Test-Path "C:\Qt\6.11.1\msvc2022_64\bin\qmake.exe"
  ZML:  Test-Path "C:\Program Files\Zaber Motion Library\lib\Release\zaber-motion.lib"
  VS:   Open "Developer PowerShell for VS 2026" from Start, then run:  cl
        (Must print Microsoft C/C++ Optimizing Compiler — not "not recognized")

STAGE (Zaber X-MCC2 + lockstep): see docs/Zaber/links.md — not wired in CMake yet.
