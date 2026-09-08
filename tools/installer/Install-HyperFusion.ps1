#Requires -Version 5.1
# HyperFusion target-PC setup (tools/installer). Copies the staged app, installs
# missing vendor SDKs, Python + fusion venv, WSL/Ubuntu, then GSAM2 and UR3e.
# Skips anything already present. WSL usually needs a reboot, then re-run this
# script. Does not change application behavior.
#
# Keep this file ASCII-only. Windows PowerShell 5.1 reads UTF-8 as ANSI; a
# UTF-8 em dash (bytes E2 80 94) becomes a stray quote (CP1252 0x94) and the
# script fails to parse.
param(
    [string]$InstallDir = "C:\HyperFusion",
    [string]$PayloadDir = "",
    [string]$VendorDir = "",
    [switch]$SkipPayloadCopy,
    [switch]$SkipVendor,
    [switch]$SkipFusionVenv,
    [switch]$SkipWsl,
    [switch]$SkipGsam,
    [switch]$SkipUr3e,
    [switch]$SkipShortcuts
)

$ErrorActionPreference = "Stop"

function Test-IsAdmin {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    $p = New-Object Security.Principal.WindowsPrincipal($id)
    return $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Test-AnyPath([string[]]$Paths) {
    foreach ($path in $Paths) {
        if (-not [string]::IsNullOrWhiteSpace($path) -and (Test-Path -LiteralPath $path)) {
            return $true
        }
    }
    return $false
}

function Test-VcRedistInstalled {
    $keys = @(
        "HKLM:\SOFTWARE\Microsoft\VisualStudio\14.0\VC\Runtimes\x64",
        "HKLM:\SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.0\VC\Runtimes\x64"
    )
    foreach ($key in $keys) {
        if (Test-Path $key) {
            $installed = (Get-ItemProperty $key -ErrorAction SilentlyContinue).Installed
            if ($installed -eq 1) { return $true }
        }
    }
    return Test-Path "$env:SystemRoot\System32\vcruntime140.dll"
}

function Test-ApiInstalled([string]$Id) {
    $pf = ${env:ProgramFiles}
    $pf86 = ${env:ProgramFiles(x86)}
    switch ($Id.ToLowerInvariant()) {
        "vcredist" {
            return Test-VcRedistInstalled
        }
        "lumo" {
            return Test-AnyPath @(
                "$pf86\Specim\SDKs\SpecSensor\2020_519\include\SI_sensor.h",
                "$pf\Specim\SDKs\SpecSensor\2020_519\include\SI_sensor.h",
                "$env:LUMO_SDK_ROOT\include\SI_sensor.h"
            )
        }
        "zaber" {
            return Test-AnyPath @(
                "$pf\Zaber Motion Library\include\zaber\motion\ascii.h",
                "$pf86\Zaber Motion Library\include\zaber\motion\ascii.h",
                "$env:ZML_ROOT\include\zaber\motion\ascii.h"
            )
        }
        "mcc" {
            return Test-AnyPath @(
                "$pf86\Measurement Computing\DAQ\cbw64.dll",
                "$pf\Measurement Computing\DAQ\cbw64.dll"
            )
        }
        "spinnaker" {
            return Test-AnyPath @(
                "$pf\Teledyne\Spinnaker\include\Spinnaker.h",
                "$pf86\Teledyne\Spinnaker\include\Spinnaker.h",
                "$env:SPINNAKER_ROOT\include\Spinnaker.h"
            )
        }
        "pleora" {
            return Test-AnyPath @(
                "$pf\Pleora Technologies Inc\eBUS SDK",
                "$pf86\Pleora Technologies Inc\eBUS SDK",
                "$pf\Pleora Technologies Inc"
            )
        }
        "ni" {
            return Test-AnyPath @(
                "$pf86\National Instruments",
                "$pf\National Instruments",
                "$env:NIEXTCCOMPILERSUPP"
            )
        }
        "dlpc" {
            $onedrive = Join-Path $env:USERPROFILE "OneDrive - Cornell University\Documents\Texas Instruments\DLPC-API-1.12\api\dlpc34xx.h"
            return Test-AnyPath @(
                "C:\ti\DLPC-API-1.12\api\dlpc34xx.h",
                $onedrive,
                "$env:DLPC_API_ROOT\api\dlpc34xx.h"
            )
        }
        "python" {
            return [bool](Get-Command python -ErrorAction SilentlyContinue) -and
                ((& python --version 2>&1 | Out-String) -notmatch "Microsoft Store")
        }
        default { return $false }
    }
}

function Resolve-VendorId([string]$FileName, [string]$ExplicitId) {
    if (-not [string]::IsNullOrWhiteSpace($ExplicitId)) {
        return $ExplicitId.Trim().ToLowerInvariant()
    }
    $n = $FileName.ToLowerInvariant()
    if ($n -match "vc_redist|vcredist") { return "vcredist" }
    if ($n -match "python") { return "python" }
    if ($n -match "specim|lumo|specsensor") { return "lumo" }
    if ($n -match "pleora|ebus") { return "pleora" }
    if ($n -match "spinnaker|teledyne") { return "spinnaker" }
    if ($n -match "zaber") { return "zaber" }
    if ($n -match "mcc|instacal|cbw|1208") { return "mcc" }
    if ($n -match "dlpc|dlp3010") { return "dlpc" }
    if ($n -match "national|ni[_ -]|vas1851|nivision") { return "ni" }
    return ""
}

function Resolve-VendorPath([string]$VendorRoot, [string]$Rel) {
    $path = Join-Path $VendorRoot $Rel
    if (Test-Path -LiteralPath $path) { return $path }
    return $null
}

function Get-VendorJobs([string]$VendorRoot) {
    $jobs = @()
    if ([string]::IsNullOrWhiteSpace($VendorRoot) -or -not (Test-Path -LiteralPath $VendorRoot)) {
        return $jobs
    }
    $orderFile = Join-Path $VendorRoot "install_order.txt"
    if (Test-Path -LiteralPath $orderFile) {
        Get-Content -LiteralPath $orderFile | ForEach-Object {
            $line = $_.Trim()
            if ([string]::IsNullOrWhiteSpace($line) -or $line.StartsWith("#")) { return }
            $parts = $line.Split("|")
            $name = $parts[0].Trim()
            $id = if ($parts.Count -gt 1) { $parts[1].Trim() } else { "" }
            $path = Resolve-VendorPath $VendorRoot $name
            if ($path) {
                $jobs += [pscustomobject]@{ Path = $path; Id = (Resolve-VendorId $name $id) }
            }
            else {
                Write-Warning "Vendor package not found: $name"
            }
        }
    }
    return $jobs
}

function Install-VendorPackage([string]$Path, [string]$Id) {
    $launch = $Path
    if ((Get-Item -LiteralPath $Path).PSIsContainer) {
        $setup = Join-Path $Path "setup.exe"
        if (-not (Test-Path -LiteralPath $setup)) {
            Write-Warning "No setup.exe in $Path"
            return 1
        }
        $launch = $setup
    }
    $name = [IO.Path]::GetFileName($launch)
    Write-Host "==> Installing $name"
    if ($Id -eq "vcredist") {
        $p = Start-Process -FilePath $launch -ArgumentList "/install", "/quiet", "/norestart" -Wait -PassThru
        return $p.ExitCode
    }
    if ([IO.Path]::GetExtension($launch) -ieq ".msi") {
        $p = Start-Process -FilePath "msiexec.exe" -ArgumentList "/i", "`"$launch`"", "/norestart" -Wait -PassThru
        return $p.ExitCode
    }
    $p = Start-Process -FilePath $launch -Wait -PassThru
    return $p.ExitCode
}

function ConvertTo-WslPath([string]$WindowsPath) {
    $full = [IO.Path]::GetFullPath($WindowsPath)
    if ($full -match '^([A-Za-z]):\\') {
        $drive = $Matches[1].ToLowerInvariant()
        $rest = ($full.Substring(2) -replace '\\', '/').TrimStart('/')
        return "/mnt/$drive/$rest"
    }
    return $WindowsPath -replace '\\', '/'
}

function Test-PythonUsable {
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if (-not $cmd) { return $false }
    try {
        $ver = & python --version 2>&1 | Out-String
        if ($ver -match "Microsoft Store") { return $false }
        return $LASTEXITCODE -eq 0
    }
    catch {
        return $false
    }
}

function Refresh-ProcessPath {
    $machine = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $user = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machine;$user"
}

function Install-PythonIfMissing {
    Refresh-ProcessPath
    if (Test-PythonUsable) {
        Write-Host "==> Skip Python - already on PATH"
        return $true
    }
    $winget = Get-Command winget -ErrorAction SilentlyContinue
    if (-not $winget) {
        Write-Warning "Python is missing and winget is not available. Install Python 3.12+, then re-run this script."
        return $false
    }
    Write-Host "==> Installing Python 3.12 (winget)"
    & winget install -e --id Python.Python.3.12 --accept-package-agreements --accept-source-agreements
    Refresh-ProcessPath
    if (Test-PythonUsable) { return $true }
    Write-Warning "Python install finished but python is not on PATH yet. Open a new terminal or re-run this script."
    return $false
}

function Test-WslUbuntuReady {
    $wsl = Get-Command wsl.exe -ErrorAction SilentlyContinue
    if (-not $wsl) { return $false }
    try {
        $probe = & wsl.exe -d Ubuntu -- bash -lc "echo ok" 2>$null
        $text = ($probe | ForEach-Object { $_ -replace "`0", "" }) -join ""
        return $text -match "ok"
    }
    catch {
        return $false
    }
}

function Install-WslUbuntuIfMissing {
    if (Test-WslUbuntuReady) {
        Write-Host "==> Skip WSL - Ubuntu is already installed"
        return "ready"
    }
    if (-not (Test-IsAdmin)) {
        Write-Warning "WSL install needs Administrator. Re-run elevated after reboot if needed."
        return "missing"
    }
    Write-Host "==> Installing WSL + Ubuntu (a reboot is usually required)"
    & wsl.exe --install -d Ubuntu
    if (Test-WslUbuntuReady) { return "ready" }
    return "reboot"
}

function Invoke-WslBash([string]$LinuxDir, [string]$Command, [string]$User = "") {
    $cmd = "cd '$LinuxDir'; $Command"
    Write-Host "==> wsl: $cmd"
    if ([string]::IsNullOrWhiteSpace($User)) {
        & wsl.exe -d Ubuntu -- bash -lc $cmd
    }
    else {
        & wsl.exe -d Ubuntu -u $User -- bash -lc $cmd
    }
    return $LASTEXITCODE
}

function Test-DirectoryWritable([string]$Path) {
    try {
        New-Item -ItemType Directory -Force -Path $Path -ErrorAction Stop | Out-Null
        $probe = Join-Path $Path ".hf_write_probe"
        [IO.File]::WriteAllText($probe, "ok")
        Remove-Item -LiteralPath $probe -Force
        return $true
    }
    catch {
        return $false
    }
}

function Resolve-WritableInstallDir([string]$Requested) {
    if (Test-DirectoryWritable $Requested) {
        return $Requested
    }
    $fallbacks = New-Object System.Collections.Generic.List[string]
    if (Test-Path -LiteralPath "D:\") {
        [void]$fallbacks.Add("D:\HyperFusion")
    }
    [void]$fallbacks.Add((Join-Path $env:LOCALAPPDATA "HyperFusion"))
    foreach ($alt in $fallbacks) {
        if ($alt -eq $Requested) { continue }
        if (Test-DirectoryWritable $alt) {
            Write-Warning "Cannot write $Requested (C:\ needs Administrator). Installing to $alt"
            return $alt
        }
    }
    Write-Error "Cannot write $Requested. Re-run elevated or pass -InstallDir to a writable folder."
}

function Get-DevSidecarRoot {
    $candidates = New-Object System.Collections.Generic.List[string]
    if (-not [string]::IsNullOrWhiteSpace($env:HYPERFUSION_DEV_ROOT)) {
        [void]$candidates.Add((Join-Path $env:HYPERFUSION_DEV_ROOT "app\sidecars"))
    }
    if (-not [string]::IsNullOrWhiteSpace($PSScriptRoot)) {
        $here = $PSScriptRoot
        [void]$candidates.Add((Join-Path (Split-Path (Split-Path $here -Parent) -Parent) "app\sidecars"))
        [void]$candidates.Add((Join-Path (Split-Path $here -Parent) "app\sidecars"))
    }
    if (-not [string]::IsNullOrWhiteSpace($script:PayloadDirResolved)) {
        $payloadParent = Split-Path $script:PayloadDirResolved -Parent
        [void]$candidates.Add((Join-Path (Split-Path (Split-Path $payloadParent -Parent) -Parent) "app\sidecars"))
    }
    foreach ($root in $candidates) {
        if ([string]::IsNullOrWhiteSpace($root)) { continue }
        if (Test-Path -LiteralPath (Join-Path $root "gsam2")) {
            return $root
        }
    }
    return $null
}

function Link-SidecarVenv([string]$DestVenv, [string]$SourceVenv, [string]$Label, [string]$MarkerRel) {
    $destMarker = Join-Path $DestVenv $MarkerRel
    if (Test-Path -LiteralPath $destMarker) {
        return $true
    }
    $srcMarker = Join-Path $SourceVenv $MarkerRel
    if (-not (Test-Path -LiteralPath $srcMarker)) {
        return $false
    }
    $destParent = Split-Path $DestVenv -Parent
    New-Item -ItemType Directory -Force -Path $destParent | Out-Null
    if (Test-Path -LiteralPath $DestVenv) {
        try {
            Remove-Item -LiteralPath $DestVenv -Recurse -Force -ErrorAction Stop
        }
        catch {
            Write-Warning "Could not replace $Label venv folder: $($_.Exception.Message)"
            return $false
        }
    }
    $null = cmd /c mklink /J "$DestVenv" "$SourceVenv"
    if ($LASTEXITCODE -eq 0 -and (Test-Path -LiteralPath $destMarker)) {
        Write-Host "==> Reusing $Label venv (junction) $SourceVenv"
        return $true
    }
    return $false
}

function New-AllUsersShortcut([string]$LinkPath, [string]$TargetPath, [string]$WorkingDir) {
    $folder = Split-Path $LinkPath -Parent
    New-Item -ItemType Directory -Force -Path $folder | Out-Null
    $wsh = New-Object -ComObject WScript.Shell
    $lnk = $wsh.CreateShortcut($LinkPath)
    $lnk.TargetPath = $TargetPath
    $lnk.WorkingDirectory = $WorkingDir
    $lnk.WindowStyle = 1
    if (Test-Path -LiteralPath $TargetPath) {
        $lnk.IconLocation = "$TargetPath,0"
    }
    $lnk.Save()
}

function Install-AllUsersShortcuts([string]$AppDir) {
    $exe = Join-Path $AppDir "app.exe"
    if (-not (Test-Path -LiteralPath $exe)) {
        Write-Warning "Cannot create shortcuts; missing $exe"
        return
    }
    $startDir = Join-Path $env:ProgramData "Microsoft\Windows\Start Menu\Programs\HyperFusion"
    $startLnk = Join-Path $startDir "HyperFusion.lnk"
    $desktopLnk = Join-Path $env:PUBLIC "Desktop\HyperFusion.lnk"
    New-AllUsersShortcut $startLnk $exe $AppDir
    New-AllUsersShortcut $desktopLnk $exe $AppDir
    Write-Host "==> All-users shortcuts:"
    Write-Host "    $startLnk"
    Write-Host "    $desktopLnk"
}

$here = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($PayloadDir)) {
    $beside = Join-Path $here "payload"
    if (Test-Path -LiteralPath (Join-Path $beside "app.exe")) {
        $PayloadDir = $beside
    }
    elseif (Test-Path -LiteralPath (Join-Path $here "app.exe")) {
        $PayloadDir = $here
    }
    else {
        $PayloadDir = $beside
    }
}
if ([string]::IsNullOrWhiteSpace($VendorDir)) {
    $VendorDir = Join-Path $here "vendor"
    if (-not (Test-Path -LiteralPath (Join-Path $VendorDir "install_order.txt"))) {
        $sibling = Join-Path (Split-Path $here -Parent) "vendor_installers"
        if (Test-Path -LiteralPath (Join-Path $sibling "install_order.txt")) {
            $VendorDir = $sibling
        }
    }
}

$script:PayloadDirResolved = [IO.Path]::GetFullPath($PayloadDir)
$InstallDir = Resolve-WritableInstallDir $InstallDir

Write-Host "==> HyperFusion installer"
Write-Host "    InstallDir  $InstallDir"
Write-Host "    PayloadDir  $PayloadDir"
Write-Host "    VendorDir   $VendorDir"

if (-not (Test-IsAdmin)) {
    Write-Warning "Not running as Administrator. Vendor SDK installers may fail. Re-run in an elevated PowerShell if they do."
}

if (-not $SkipPayloadCopy) {
    $payloadExe = Join-Path $PayloadDir "app.exe"
    if (-not (Test-Path -LiteralPath $payloadExe)) {
        Write-Error "Payload app.exe not found in $PayloadDir. Run tools\package_release.ps1 first."
    }
    $srcFull = [IO.Path]::GetFullPath($PayloadDir).TrimEnd('\')
    $dstFull = [IO.Path]::GetFullPath($InstallDir).TrimEnd('\')
    if ($srcFull -eq $dstFull) {
        Write-Host "==> App already in $InstallDir (skip copy)"
    }
    else {
        Write-Host "==> Copying HyperFusion to $InstallDir"
        New-Item -ItemType Directory -Force -Path $InstallDir | Out-Null
        Copy-Item -Path (Join-Path $PayloadDir '*') -Destination $InstallDir -Recurse -Force
    }
}

$installerCopy = Join-Path $InstallDir "Install-HyperFusion.ps1"
if ($PSCommandPath -and ($PSCommandPath -ne $installerCopy)) {
    Copy-Item -LiteralPath $PSCommandPath -Destination $installerCopy -Force
}

if (-not $SkipVendor) {
    $jobs = @(Get-VendorJobs $VendorDir)
    if ($jobs.Count -eq 0) {
        Write-Host "==> No vendor installers in $VendorDir (skipped)."
    }
    $already = @($jobs | Where-Object { $_.Id -and (Test-ApiInstalled $_.Id) })
    if ($jobs.Count -gt 0 -and $already.Count -eq $jobs.Count) {
        Write-Host "==> All listed vendor SDKs already installed (skipped)."
    }
    foreach ($job in $jobs) {
        $label = if ($job.Id) { $job.Id } else { [IO.Path]::GetFileName($job.Path) }
        if ($job.Id -and (Test-ApiInstalled $job.Id)) {
            Write-Host "==> Skip $label - already installed"
            continue
        }
        $code = Install-VendorPackage $job.Path $job.Id
        if ($code -ne 0 -and $code -ne 3010) {
            Write-Warning "Installer exited $code : $([IO.Path]::GetFileName($job.Path))"
        }
        elseif ($job.Id -and -not (Test-ApiInstalled $job.Id)) {
            Write-Warning "$label installer finished, but the API was not detected yet (license/reboot/path)."
        }
        else {
            Write-Host "==> Done $label"
        }
    }
}

if (-not $SkipFusionVenv) {
    $setup = Join-Path $InstallDir "sidecars\hf_fusion\setup_venv.ps1"
    $venvPy = Join-Path $InstallDir "sidecars\hf_fusion\.venv\Scripts\python.exe"
    if (Test-Path -LiteralPath $venvPy) {
        Write-Host "==> Skip fusion venv - already present"
    }
    elseif (Test-Path -LiteralPath $setup) {
        if (Install-PythonIfMissing) {
            Write-Host "==> Creating hf_fusion venv"
            & $setup
        }
    }
}

$needReboot = $false
if (-not $SkipWsl) {
    $wslState = Install-WslUbuntuIfMissing
    if ($wslState -eq "reboot") {
        $needReboot = $true
        Write-Host "==> Reboot this PC, then run Install-HyperFusion.ps1 again to finish GSAM2 and UR3e."
    }
}

$wslReady = (-not $needReboot) -and (Test-WslUbuntuReady)
$devSidecars = Get-DevSidecarRoot
if ($wslReady -and -not $SkipGsam) {
    $gsamWin = Join-Path $InstallDir "sidecars\gsam2"
    $gsamVenv = Join-Path $gsamWin "venv\bin\python"
    if ($devSidecars) {
        [void](Link-SidecarVenv (Join-Path $gsamWin "venv") (Join-Path $devSidecars "gsam2\venv") "GSAM2" "bin\python")
    }
    if (Test-Path -LiteralPath $gsamVenv) {
        Write-Host "==> Skip GSAM2 venv - already present"
    }
    elseif (Test-Path -LiteralPath (Join-Path $gsamWin "install_venv.sh")) {
        $gsamWsl = ConvertTo-WslPath $gsamWin
        Write-Host "==> Setting up GSAM2 in WSL (PyTorch download can take a long time)"
        $code = Invoke-WslBash $gsamWsl "chmod +x install_venv.sh; ./install_venv.sh"
        if ($code -ne 0) {
            Write-Warning "GSAM2 setup exited $code. Need Ubuntu user, NVIDIA driver, and internet."
        }
    }
    else {
        Write-Warning "GSAM2 sidecar not in $gsamWin"
    }
}

if ($wslReady -and -not $SkipUr3e) {
    $ur3eWin = Join-Path $InstallDir "sidecars\ur3e"
    $ur3eVenv = Join-Path $ur3eWin "venv\bin\ur3e_server"
    if ($devSidecars) {
        [void](Link-SidecarVenv (Join-Path $ur3eWin "venv") (Join-Path $devSidecars "ur3e\venv") "UR3e" "bin\ur3e_server")
    }
    if (Test-Path -LiteralPath $ur3eVenv) {
        Write-Host "==> Skip UR3e sidecar venv - already present"
    }
    elseif (Test-Path -LiteralPath (Join-Path $ur3eWin "scripts\install_env.sh")) {
        $ur3eWsl = ConvertTo-WslPath $ur3eWin
        Write-Host "==> Setting up UR3e ROS 2 sidecar in WSL (apt as root; may take a while)"
        $code = Invoke-WslBash $ur3eWsl "chmod +x scripts/install_env.sh scripts/*.sh; ./scripts/install_env.sh" "root"
        if ($code -ne 0) {
            Write-Warning "UR3e setup exited $code."
        }
    }
    else {
        Write-Warning "UR3e sidecar not in $ur3eWin"
    }

    $netScript = Join-Path $ur3eWin "scripts\setup_wsl_robot_network.ps1"
    if ((Test-Path -LiteralPath $netScript) -and (Test-IsAdmin)) {
        Write-Host "==> UR3e WSL mirrored network + firewall"
        & $netScript
    }
}

if ($wslReady) {
    $nv = & wsl.exe -d Ubuntu -- bash -lc "if command -v nvidia-smi >/dev/null; then nvidia-smi -L; fi"
    if ([string]::IsNullOrWhiteSpace($nv)) {
        Write-Warning "No NVIDIA GPU visible inside WSL. Install the Windows NVIDIA driver, then re-run GSAM setup if segmentation needs CUDA."
    }
    else {
        Write-Host "==> WSL GPU: $nv"
    }
}

$exe = Join-Path $InstallDir "app.exe"
if (-not $SkipShortcuts) {
    if (Test-IsAdmin) {
        Install-AllUsersShortcuts $InstallDir
    }
    else {
        Write-Warning "All-users Start Menu / Public Desktop shortcuts need Administrator. Re-run elevated."
    }
}
Write-Host "==> Finished. App: $exe"
Write-Host "    Re-run this script anytime (skips what is already installed):"
Write-Host "      powershell -ExecutionPolicy Bypass -File `"$(Join-Path $InstallDir 'Install-HyperFusion.ps1')`" -InstallDir `"$InstallDir`""
if ($needReboot) {
    Write-Host "    Reboot, then re-run that command to complete GSAM2 and UR3e."
}
