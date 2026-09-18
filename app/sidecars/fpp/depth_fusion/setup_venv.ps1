#Requires -Version 5.1
# Create or refresh the classical FPP depth_fusion venv (no torch).
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
if ($Force -and (Test-Path (Join-Path $here ".venv"))) {
    Remove-Item -LiteralPath (Join-Path $here ".venv") -Recurse -Force
}
if (-not (Test-Path $venvPython)) {
    $parentVenv = Join-Path $here "..\.venv\Scripts\python.exe"
    if (Test-Path $parentVenv) {
        Write-Host "Using parent fpp venv: $parentVenv"
        $venvPython = $parentVenv
    } else {
        python -m venv .venv
        $venvPython = Join-Path $here ".venv\Scripts\python.exe"
    }
}

& $venvPython -m pip install --upgrade pip
& $venvPython -m pip install -r (Join-Path $here "requirements.txt")
& $venvPython -c "import open3d, numpy, yaml; print('ok open3d', open3d.__version__)"
Write-Host "Venv ready: $venvPython"
