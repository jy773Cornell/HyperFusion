#Requires -Version 5.1
<#
.SYNOPSIS
  Configure, build, and optionally run the HyperFusion Qt app on Windows (MSVC + Qt 6.11 MSVC kit).

.EXAMPLE
  cd app
  .\build_app.ps1

.EXAMPLE
  .\build_app.ps1 -Config Debug -NoRun

.EXAMPLE
  $env:QT_PREFIX_PATH = "C:\Qt\6.11.1\msvc2022_64"
  .\build_app.ps1
#>
param(
    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$Config = "Release",

    [string]$QtPrefixPath = $(if ($env:QT_PREFIX_PATH) { $env:QT_PREFIX_PATH } else { "C:\Qt\6.11.1\msvc2022_64" }),

    [string]$LumoSdkRoot = $(if ($env:LUMO_SDK_ROOT) { $env:LUMO_SDK_ROOT } else { "C:\Program Files (x86)\Specim\SDKs\SpecSensor\2020_519" }),

    [string]$ZmlRoot = $(if ($env:ZML_ROOT) { $env:ZML_ROOT } else { "C:\Program Files\Zaber Motion Library" }),

    [string]$DlpcApiRoot = $(if ($env:DLPC_API_ROOT) { $env:DLPC_API_ROOT } else { "" }),

    [switch]$NoRun,

    [switch]$NoClean,

    # One-time WSL mirrored networking + UR reverse-port firewall (not run on every build by default).
    [switch]$SetupUrRobotNetwork,

    [switch]$ShutdownWslAfterUrNetworkSetup
)

$ErrorActionPreference = "Stop"

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) {
    Write-Error @"
Visual Studio Installer (vswhere) not found.

Install Visual Studio with workload: Desktop development with C++
(including MSVC build tools and Windows SDK).
"@
}

$vsPath = & $vswhere -latest -property installationPath
$vsName = & $vswhere -latest -property displayName
if (-not $vsPath) {
    Write-Error "No Visual Studio installation found."
}

$msvcToolsRoot = Join-Path $vsPath "VC\Tools\MSVC"
if (-not (Test-Path -LiteralPath $msvcToolsRoot)) {
    Write-Error "Visual Studio is installed but MSVC toolset is missing at: $msvcToolsRoot`nAdd workload 'Desktop development with C++' in Visual Studio Installer."
}

# VS 2026 (internal v18) vs VS 2022 (v17) — pick a CMake generator that exists locally.
$cmakeHelp = & cmake --help 2>&1 | Out-String
if ($vsPath -match '[\\/]18[\\/]' -and $cmakeHelp -match 'Visual Studio 18 2026') {
    $CmakeGenerator = 'Visual Studio 18 2026'
} elseif ($cmakeHelp -match 'Visual Studio 17 2022') {
    $CmakeGenerator = 'Visual Studio 17 2022'
} else {
    Write-Error "No supported Visual Studio CMake generator found. Install VS 2022/2026 with C++ and CMake 3.24+."
}

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    Write-Error "cmake not found on PATH. Install CMake (VS component or standalone) and reopen the terminal."
}

$qtConfig = Join-Path $QtPrefixPath "lib\cmake\Qt6\Qt6Config.cmake"
if (-not (Test-Path -LiteralPath $qtConfig)) {
    Write-Error "Qt6Config.cmake not found at: $qtConfig`nSet -QtPrefixPath or QT_PREFIX_PATH (e.g. C:\Qt\6.11.1\msvc2022_64)."
}

$AppDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $AppDir "build"

$qtBin = Join-Path $QtPrefixPath "bin"
$lumoBin = Join-Path $LumoSdkRoot "bin\x64"
$zmlBin = Join-Path $ZmlRoot "bin\Release"
$niVisionBins = @(
    "C:\Program Files\National Instruments\Vision\Bin"
    "C:\Program Files (x86)\National Instruments\Vision\Bin"
    "C:\Program Files\National Instruments\Shared\LabVIEW Runtime\2024"
)
$niPathExtra = ""
foreach ($niBin in $niVisionBins) {
    if (Test-Path -LiteralPath $niBin) {
        $niPathExtra = "$niBin;"
        break
    }
}
$env:PATH = "$qtBin;$lumoBin;$zmlBin;$niPathExtra$env:PATH"
$env:LUMO_SDK_ROOT = $LumoSdkRoot
$env:ZML_ROOT = $ZmlRoot

Write-Host "==> Visual Studio: $vsName"
Write-Host "==> VS path:       $vsPath"
Write-Host "==> CMake generator: $CmakeGenerator"
Write-Host "==> Qt prefix:    $QtPrefixPath"
Write-Host "==> Lumo SDK:     $LumoSdkRoot"
Write-Host "==> ZML root:     $ZmlRoot"
if ($DlpcApiRoot) {
    Write-Host "==> DLPC-API:     $DlpcApiRoot"
}
Write-Host "==> Config:       $Config"
if ($NoClean) {
    Write-Host "==> Keeping existing build dir: $BuildDir"
} else {
    Write-Host "==> Removing old build dir: $BuildDir"
    if (Test-Path -LiteralPath $BuildDir) {
        try {
            Remove-Item -Recurse -Force -LiteralPath $BuildDir
        } catch {
            Write-Warning "Could not remove $BuildDir (in use?). Reconfiguring in place."
        }
    }
}

Write-Host "==> Configuring ($CmakeGenerator, x64)"
$cmakeArgs = @(
    "-S", $AppDir
    "-B", $BuildDir
    "-G", $CmakeGenerator
    "-A", "x64"
    "-DCMAKE_GENERATOR_INSTANCE=$vsPath"
    "-DCMAKE_PREFIX_PATH=$QtPrefixPath"
    "-DLUMO_SDK_ROOT=$LumoSdkRoot"
    "-DZML_ROOT=$ZmlRoot"
)
if ($DlpcApiRoot) {
    $cmakeArgs += "-DDLPC_API_ROOT=$DlpcApiRoot"
}
& cmake @cmakeArgs

if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configure failed (exit $LASTEXITCODE)."
}

Write-Host "==> Building"
& cmake --build $BuildDir --config $Config
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake build failed (exit $LASTEXITCODE)."
}

$exeDir = Join-Path $BuildDir $Config
$exe = Join-Path $exeDir "app.exe"
if (-not (Test-Path -LiteralPath $exe)) {
    Write-Error "Expected binary not found: $exe"
}

$windeployqt = Join-Path $QtPrefixPath "bin\windeployqt.exe"
if (Test-Path -LiteralPath $windeployqt) {
    Write-Host "==> Deploying Qt runtime DLLs (windeployqt)"
    & $windeployqt --no-translations $exe
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "windeployqt exited with code $LASTEXITCODE (app may still run if Qt bin is on PATH)."
    }
}

# Lumo / Pleora runtime DLLs next to app.exe (SI_Open loads grab*.dll / Pv*.dll from here).
$lumoBinDir = Join-Path $LumoSdkRoot "bin\x64"
if (Test-Path -LiteralPath $lumoBinDir) {
    $lumoDlls = Get-ChildItem -LiteralPath $lumoBinDir -Filter "*.dll" -File
    foreach ($dll in $lumoDlls) {
        Copy-Item -LiteralPath $dll.FullName -Destination $exeDir -Force
    }
    Write-Host "==> Copied $($lumoDlls.Count) Lumo SDK DLL(s) from $lumoBinDir to $exeDir"
}

$lumoNiIcd = Join-Path $LumoSdkRoot "external\NI"
$destNiIcd = Join-Path $exeDir "external\NI"
if (Test-Path -LiteralPath $lumoNiIcd) {
    New-Item -ItemType Directory -Force -Path $destNiIcd | Out-Null
    Copy-Item -LiteralPath (Join-Path $lumoNiIcd "*") -Destination $destNiIcd -Force
    Write-Host "==> Copied Lumo NI ICD files to $destNiIcd"
}

if (-not (Test-Path -LiteralPath (Join-Path $exeDir "SpecSensor.dll")) -and (Test-Path -LiteralPath (Join-Path $LumoSdkRoot "bin\x64\SpecSensor.dll"))) {
    Copy-Item -LiteralPath (Join-Path $LumoSdkRoot "bin\x64\SpecSensor.dll") -Destination $exeDir -Force
    Write-Host "==> Copied SpecSensor.dll to $exeDir"
}

$mccDaqDir = ${env:ProgramFiles(x86)} + "\Measurement Computing\DAQ"
if (-not (Test-Path -LiteralPath $mccDaqDir)) {
    $mccDaqDir = Join-Path $env:ProgramFiles "Measurement Computing\DAQ"
}
if (Test-Path -LiteralPath $mccDaqDir) {
    $mccDlls = Get-ChildItem -LiteralPath $mccDaqDir -Filter "*.dll" -File
    foreach ($dll in $mccDlls) {
        Copy-Item -LiteralPath $dll.FullName -Destination $exeDir -Force
    }
    Write-Host "==> Copied $($mccDlls.Count) MCC UL DLL(s) from $mccDaqDir to $exeDir"
}

if ($SetupUrRobotNetwork) {
    $networkScript = Join-Path $AppDir "sidecars\ur3e\scripts\setup_wsl_robot_network.ps1"
    if (-not (Test-Path -LiteralPath $networkScript)) {
        Write-Error "UR network setup script not found: $networkScript"
    }
    Write-Host "==> UR3e robot network setup (WSL mirrored + firewall)"
    $netArgs = @()
    if ($ShutdownWslAfterUrNetworkSetup) { $netArgs += "-ShutdownWsl" }
    & $networkScript @netArgs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if ($NoRun) {
    Write-Host "==> Built: $exe"
    exit 0
}

Write-Host "==> Running $exe"
& $exe
