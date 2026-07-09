# Runs install_env.sh inside WSL (bash scripts do not run in PowerShell).
# Usage: .\install_env.ps1 [--skip-ros] [--skip-venv] [-SetupRobotNetwork] [-ShutdownWsl]
param(
    [switch]$SkipRos,
    [switch]$SkipVenv,
    [switch]$SetupRobotNetwork,
    [switch]$ShutdownWsl
)

$ErrorActionPreference = "Stop"
$here = $PSScriptRoot -replace '\\', '/'
if ($here -match '^([A-Za-z]):') {
    $drive = $Matches[1].ToLower()
    $rest = $here.Substring(2)
    $wslPath = "/mnt/$drive$rest"
} else {
    throw "Could not map path to WSL: $PSScriptRoot"
}

$argsList = @()
if ($SkipRos) { $argsList += "--skip-ros" }
if ($SkipVenv) { $argsList += "--skip-venv" }
$bashArgs = ($argsList -join " ").Trim()

Write-Host "==> Running install_env.sh in WSL: $wslPath"
$cmd = "cd '$wslPath' && chmod +x install_env.sh scripts/*.sh && ./install_env.sh $bashArgs"
wsl.exe -d Ubuntu bash -lc $cmd
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($SetupRobotNetwork) {
    $networkScript = Join-Path $PSScriptRoot "scripts\setup_wsl_robot_network.ps1"
    if (-not (Test-Path -LiteralPath $networkScript)) {
        Write-Error "Network setup script not found: $networkScript"
    }
    $netArgs = @()
    if ($ShutdownWsl) { $netArgs += "-ShutdownWsl" }
    & $networkScript @netArgs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
