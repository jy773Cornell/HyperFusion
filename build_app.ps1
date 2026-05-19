#Requires -Version 5.1
<#
.SYNOPSIS
  Configure, build, and run the HyperFusion Qt app on native Windows using Qt's MinGW kit.

.EXAMPLE
  .\build_app.ps1

.EXAMPLE
  .\build_app.ps1 -QtPrefixPath "C:\Qt\6.11.0\mingw_64" -Config Debug

.EXAMPLE
  $env:QT_PREFIX_PATH = "C:\Qt\6.11.0\mingw_64"
  $env:MINGW_PATH = "C:\Qt\Tools\mingw1310_64"
  .\build_app.ps1
#>
param(
    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$Config = "Release",

    [string]$QtPrefixPath = $(if ($env:QT_PREFIX_PATH) { $env:QT_PREFIX_PATH } else { "C:\Qt\6.11.0\mingw_64" }),

    # Optional override if auto-detection fails:
    # Example: C:\Qt\Tools\mingw1310_64
    [string]$MingwPath = $env:MINGW_PATH,

    # Specim Lumo Sensor SDK (FX10e). Override if installed elsewhere.
    [string]$LumoSdkRoot = $(if ($env:LUMO_SDK_ROOT) { $env:LUMO_SDK_ROOT } else { "C:\Program Files (x86)\Specim\SDKs\SpecSensor\2020_519" })
)

$ErrorActionPreference = "Stop"

function Find-DefaultMingwFromQtLayout {
    param(
        [Parameter(Mandatory = $true)]
        [string]$QtPrefix
    )

    # Typical layout:
    #   C:\Qt\6.11.0\mingw_64
    #   C:\Qt\Tools\mingwXXXX_64
    $qtRoot = Split-Path -Parent (Split-Path -Parent $QtPrefix)
    $toolsDir = Join-Path $qtRoot "Tools"
    if (-not (Test-Path -LiteralPath $toolsDir)) {
        return $null
    }

    $candidates = Get-ChildItem -LiteralPath $toolsDir -Directory -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -like "mingw*_64" } |
    Sort-Object Name -Descending

    foreach ($d in $candidates) {
        $gcc = Join-Path $d.FullName "bin\gcc.exe"
        $gpp = Join-Path $d.FullName "bin\g++.exe"
        if ((Test-Path -LiteralPath $gcc) -and (Test-Path -LiteralPath $gpp)) {
            return $d.FullName
        }
    }

    return $null
}

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$AppDir = Join-Path $Root "app"
$BuildDir = Join-Path $AppDir "build"

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    Write-Error "cmake not found on PATH. Install CMake and reopen the terminal."
}

$qtConfig = Join-Path $QtPrefixPath "lib\cmake\Qt6\Qt6Config.cmake"
if (-not (Test-Path -LiteralPath $qtConfig)) {
    Write-Error "Qt6Config.cmake not found at: $qtConfig`nCheck -QtPrefixPath / QT_PREFIX_PATH."
}

if (-not $MingwPath -or $MingwPath.Trim().Length -eq 0) {
    $MingwPath = Find-DefaultMingwFromQtLayout -QtPrefix $QtPrefixPath
}

if (-not $MingwPath) {
    Write-Error @"
Could not auto-detect MinGW toolchain under C:\Qt\Tools\mingw*_64.

Fix: install Qt's MinGW component in Qt Maintenance Tool, or set MINGW_PATH explicitly:

  `$env:MINGW_PATH = 'C:\Qt\Tools\mingw1310_64'
  .\build_app.ps1
"@
}

$gcc = Join-Path $MingwPath "bin\gcc.exe"
$gpp = Join-Path $MingwPath "bin\g++.exe"
$mingwMake = Join-Path $MingwPath "bin\mingw32-make.exe"
if (-not (Test-Path -LiteralPath $gcc) -or -not (Test-Path -LiteralPath $gpp)) {
    Write-Error "MinGW bin tools not found. Expected:`n  $gcc`n  $gpp"
}

$mingwBin = Join-Path $MingwPath "bin"
$qtBin = Join-Path $QtPrefixPath "bin"
$lumoBin = Join-Path $LumoSdkRoot "bin\x64"
$lumoProfiles = Join-Path $LumoSdkRoot "profiles"
# Ensure MinGW/Qt DLLs win over other tools on PATH (notably Git's sh.exe, which can break MinGW Makefiles).
$env:PATH = "$mingwBin;$qtBin;$lumoBin;$env:PATH"
$env:LUMO_SDK_ROOT = $LumoSdkRoot
$env:HF_LUMO_PROFILES_DIR = $lumoProfiles

Write-Host "==> Qt prefix: $QtPrefixPath"
Write-Host "==> MinGW:   $MingwPath"
Write-Host "==> Lumo SDK: $LumoSdkRoot"
Write-Host "==> Removing old build dir: $BuildDir"
if (Test-Path -LiteralPath $BuildDir) {
    Remove-Item -Recurse -Force -LiteralPath $BuildDir
}

$useNinja = $false
if (Get-Command ninja -ErrorAction SilentlyContinue) {
    $useNinja = $true
}

if ($useNinja) {
    Write-Host "==> Configuring (Ninja, MinGW)"
    & cmake -S $AppDir -B $BuildDir -G Ninja `
        "-DCMAKE_PREFIX_PATH=$QtPrefixPath" `
        "-DCMAKE_SH=CMAKE_SH-NOTFOUND" `
        "-DCMAKE_C_COMPILER=$gcc" `
        "-DCMAKE_CXX_COMPILER=$gpp" `
        "-DLUMO_SDK_ROOT=$LumoSdkRoot" `
        "-DCMAKE_BUILD_TYPE=$Config"
} else {
    Write-Warning "ninja not found on PATH; falling back to 'MinGW Makefiles'."
    if (-not (Test-Path -LiteralPath $mingwMake)) {
        Write-Error @"
mingw32-make.exe not found at:
  $mingwMake

Fix options (pick one):
1) Install Qt's MinGW toolchain component (Qt Maintenance Tool) so mingw32-make exists under Tools\mingw*_64\bin
2) Put ninja.exe on PATH and rerun (preferred): then this script uses Ninja instead
"@
    }

    Write-Host "==> Configuring (MinGW Makefiles, MinGW)"
    & cmake -S $AppDir -B $BuildDir -G "MinGW Makefiles" `
        "-DCMAKE_PREFIX_PATH=$QtPrefixPath" `
        "-DCMAKE_SH=CMAKE_SH-NOTFOUND" `
        "-DCMAKE_C_COMPILER=$gcc" `
        "-DCMAKE_CXX_COMPILER=$gpp" `
        "-DCMAKE_MAKE_PROGRAM=$mingwMake" `
        "-DLUMO_SDK_ROOT=$LumoSdkRoot" `
        "-DCMAKE_BUILD_TYPE=$Config"
}

if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake configure failed (exit $LASTEXITCODE)."
}

Write-Host "==> Building"
& cmake --build $BuildDir
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake build failed (exit $LASTEXITCODE)."
}

$exe = Join-Path $BuildDir "app.exe"
if (-not (Test-Path -LiteralPath $exe)) {
    Write-Error "Expected binary not found: $exe"
}

Write-Host "==> Running $exe"
& $exe
