#Requires -Version 5.1
# Create or refresh the hf_fusion Python venv beside fusion_cli.py (deploy: {app}/hf_fusion/.venv).
param(
    [switch]$Force
)

$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
Set-Location -LiteralPath $here

if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    Write-Error "python not found on PATH. Install Python 3.10+ and reopen the terminal."
}

$venvPython = Join-Path $here ".venv\Scripts\python.exe"
if ($Force -and (Test-Path -LiteralPath (Join-Path $here ".venv"))) {
    Write-Host "==> Removing existing .venv"
    Remove-Item -Recurse -Force -LiteralPath (Join-Path $here ".venv")
}

if (-not (Test-Path -LiteralPath $venvPython)) {
    Write-Host "==> Creating venv in $here\.venv"
    & python -m venv (Join-Path $here ".venv")
}

Write-Host "==> Installing requirements"
& $venvPython -m pip install --upgrade pip
& $venvPython -m pip install -r (Join-Path $here "requirements.txt")

Write-Host "==> hf_fusion venv ready: $venvPython"
