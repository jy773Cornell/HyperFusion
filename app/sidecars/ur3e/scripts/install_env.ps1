# Runs install_env.sh inside WSL (bash scripts do not run in PowerShell).
# Usage: .\install_env.ps1 [--skip-ros] [--skip-venv] [-SetupRobotNetwork] [-ShutdownWsl]
param(
    [switch]$SkipRos,
    [switch]$SkipVenv,
    [switch]$SetupRobotNetwork,
    [switch]$ShutdownWsl
)

$ErrorActionPreference = "Stop"
# This file lives in scripts/; the sidecar root is the parent (setup.py, venv).
$sidecarRoot = if (Test-Path -LiteralPath (Join-Path $PSScriptRoot "setup.py")) {
    $PSScriptRoot
} else {
    Split-Path -Parent $PSScriptRoot
}
$here = $sidecarRoot -replace '\\', '/'
if ($here -match '^([A-Za-z]):') {
    $drive = $Matches[1].ToLower()
    $rest = $here.Substring(2)
    $wslPath = "/mnt/$drive$rest"
} else {
    throw "Could not map path to WSL: $sidecarRoot"
}

$argsList = @()
if ($SkipRos) { $argsList += "--skip-ros" }
if ($SkipVenv) { $argsList += "--skip-venv" }
$bashArgs = ($argsList -join " ").Trim()

Write-Host "==> Running install_env.sh in WSL: $wslPath"
$cmd = "cd '$wslPath' && chmod +x scripts/install_env.sh scripts/*.sh && ./scripts/install_env.sh $bashArgs"
wsl.exe -d Ubuntu bash -lc $cmd
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($SetupRobotNetwork) {
    $networkScript = Join-Path $sidecarRoot "scripts\setup_wsl_robot_network.ps1"
    if (-not (Test-Path -LiteralPath $networkScript)) {
        Write-Error "Network setup script not found: $networkScript"
    }
    $netArgs = @()
    if ($ShutdownWsl) { $netArgs += "-ShutdownWsl" }
    & $networkScript @netArgs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
