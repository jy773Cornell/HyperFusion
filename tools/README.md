# HyperFusion packaging and install

Windows-only. This folder stages a **payload** (the built app) and installs it on a PC.

| Script | Role |
| --- | --- |
| `package_release.ps1` | Builds the payload folder under `dist/HyperFusion-Setup/` |
| `installer/Install-HyperFusion.ps1` | Copies the payload to the install directory and finishes setup |
| `installer/HyperFusion.iss` | Optional Inno Setup wrapper (`HyperFusion-Setup.exe`) if Inno 6 is installed |
| `vendor_installers/` | Optional vendor SDK installers + `install_order.txt` |

Developer SDK/build steps stay in [SETUP.md](../SETUP.md). This page is how you **package and install** the app.

## What the payload is

The **payload** is the ready-to-copy app tree. It is not source and not a vendor SDK installer.

`package_release.ps1` writes it here:

`dist/HyperFusion-Setup/payload`

That folder is `app/build/Release` plus `app/sidecars` (without large `venv` / `.venv` folders): `app.exe`, Qt and vendor runtime DLLs, presets, and sidecar scripts.

`Install-HyperFusion.ps1 -PayloadDir ...\payload` copies that tree to `-InstallDir` (default `C:\HyperFusion`), then creates missing Python/WSL/venvs. Vendor SDKs that are already on the PC are skipped.

## 1. Build the app

From the repo, in PowerShell:

```powershell
cd D:\Pototypy\HyperFusion
.\app\build_app.ps1 -NoClean -NoRun
```

You need `app\build\Release\app.exe` before packaging.

## 2. Stage the payload

```powershell
cd D:\Pototypy\HyperFusion
.\tools\package_release.ps1 -SkipVendor
```

`-SkipVendor` skips copying `tools/vendor_installers` into the stage (use this when the target PC already has Lumo, Pleora, NI, Zaber, MCC, Spinnaker, etc.).

Result:

- `dist\HyperFusion-Setup\payload\` — the payload
- `dist\HyperFusion-Setup\Install-HyperFusion.ps1` — installer
- `HyperFusion-Setup.exe` only if Inno Setup 6 is installed; otherwise use the `.ps1`

## 3. Install (Administrator, this PC)

Close HyperFusion if it is running.

1. Open **Windows PowerShell as Administrator**.
2. Run:

```powershell
cd D:\Pototypy\HyperFusion\dist\HyperFusion-Setup
Set-ExecutionPolicy -Scope Process Bypass
.\Install-HyperFusion.ps1 -InstallDir "C:\HyperFusion" -PayloadDir "D:\Pototypy\HyperFusion\dist\HyperFusion-Setup\payload"
```

3. Launch `C:\HyperFusion\app.exe`.

An elevated install is **per-machine**: every Windows user gets the same `C:\HyperFusion\app.exe`, plus All Users shortcuts:

- Start Menu: `C:\ProgramData\Microsoft\Windows\Start Menu\Programs\HyperFusion\HyperFusion.lnk`
- Desktop: `C:\Users\Public\Desktop\HyperFusion.lnk`

Without Administrator, `C:\HyperFusion` is not writable. The installer then falls back to `D:\HyperFusion`, then `%LOCALAPPDATA%\HyperFusion`, and skips all-users shortcuts.

Re-run the same script anytime; it skips what is already installed.

### Optional switches

| Switch | Meaning |
| --- | --- |
| `-SkipVendor` | Do not run vendor SDK setup (already installed) |
| `-SkipWsl` | Do not install WSL/Ubuntu |
| `-SkipGsam` | Do not create the GSAM2 venv |
| `-SkipUr3e` | Do not create the UR3e ROS venv |
| `-SkipFusionVenv` | Do not create `hf_fusion\.venv` |
| `-SkipPayloadCopy` | App files already in `-InstallDir` (used by the Inno wrapper) |
| `-SkipShortcuts` | Do not create All Users Start Menu / Public Desktop shortcuts |

GSAM2 / UR3e venvs are not copied in the payload (too large). If this machine already has `app\sidecars\gsam2\venv` or `ur3e\venv` in the repo, the installer **junctions** those into the install tree instead of downloading PyTorch/ROS again. Override the repo root with `HYPERFUSION_DEV_ROOT` if needed.

## Another PC

1. Build and stage on a machine that has the Visual Studio / Qt build.
2. Copy the whole `dist\HyperFusion-Setup` folder to the target PC (include `payload` and `Install-HyperFusion.ps1`).
3. On the target PC, run step 3 as Administrator. Point `-PayloadDir` at that copy’s `payload` folder.
4. If vendor SDKs are not installed, omit `-SkipVendor` and put the installers under `vendor\` (see `vendor_installers/install_order.txt`).

## Notes

- `Install-HyperFusion.ps1` must stay **ASCII**. Windows PowerShell 5.1 misreads UTF-8 em dashes and the script will not parse.
- WSL install may require a reboot; re-run the installer after reboot to finish GSAM2 and UR3e.
- There is no `HyperFusion-Setup.exe` unless Inno Setup 6 is installed. The `.ps1` is the installer.
