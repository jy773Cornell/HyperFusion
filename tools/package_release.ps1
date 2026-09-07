#Requires -Version 5.1
# Stage HyperFusion + optional vendor SDK installers. Does not change app code.
# Output: dist/HyperFusion-Setup/  and HyperFusion-Setup.exe if Inno Setup is installed.
param(
    [string]$Config = "Release",
    [string]$OutDir = "",
    [switch]$SkipVendor,
    [switch]$SkipInno
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$exeDir = Join-Path $repoRoot "app\build\$Config"
$exe = Join-Path $exeDir "app.exe"
if (-not (Test-Path -LiteralPath $exe)) {
    Write-Error "Build first: app\build_app.ps1 -NoClean -NoRun. Missing $exe"
}

if ([string]::IsNullOrWhiteSpace($OutDir)) {
    $OutDir = Join-Path $repoRoot "dist\HyperFusion-Setup"
}

$payload = Join-Path $OutDir "payload"
$vendorOut = Join-Path $OutDir "vendor"
Write-Host "==> Staging $OutDir"
if (Test-Path -LiteralPath $OutDir) {
    Remove-Item -LiteralPath $OutDir -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $payload | Out-Null
New-Item -ItemType Directory -Force -Path $vendorOut | Out-Null

function Copy-Tree([string]$From, [string]$To, [string[]]$ExcludeDirNames) {
    if (-not (Test-Path -LiteralPath $From)) {
        Write-Warning "Skip missing: $From"
        return
    }
    $exclude = [System.Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
    foreach ($name in $ExcludeDirNames) { [void]$exclude.Add($name) }
    function Copy-Filtered([string]$Src, [string]$Dst) {
        New-Item -ItemType Directory -Force -Path $Dst | Out-Null
        Get-ChildItem -LiteralPath $Src -Force | ForEach-Object {
            if ($_.PSIsContainer) {
                if ($exclude.Contains($_.Name)) { return }
                Copy-Filtered $_.FullName (Join-Path $Dst $_.Name)
            }
            else {
                Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $Dst $_.Name) -Force
            }
        }
    }
    Copy-Filtered $From $To
}

Copy-Tree $exeDir $payload @("CMakeFiles", "app_autogen", ".qt")
Get-ChildItem -LiteralPath $payload -Filter "_*.png" -File -ErrorAction SilentlyContinue |
    Remove-Item -Force

$sidecarsSrc = Join-Path $repoRoot "app\sidecars"
Copy-Tree $sidecarsSrc (Join-Path $payload "sidecars") @(
    "venv", ".venv", "__pycache__", ".pytest_cache", ".cache", "logs"
)

$ckpt = Join-Path $sidecarsSrc "gsam2\checkpoints\sam2.1_hiera_large.pt"
if (-not (Test-Path -LiteralPath $ckpt)) {
    Write-Warning "GSAM checkpoint not in the source tree (gitignored). Target PC will download it during GSAM setup."
}

$vendorSrc = Join-Path $PSScriptRoot "vendor_installers"
$orderFile = Join-Path $vendorSrc "install_order.txt"
if (-not $SkipVendor -and (Test-Path -LiteralPath $orderFile)) {
    Copy-Item -LiteralPath $orderFile -Destination (Join-Path $vendorOut "install_order.txt") -Force
    $copied = 0
    Get-Content -LiteralPath $orderFile | ForEach-Object {
        $line = $_.Trim()
        if ([string]::IsNullOrWhiteSpace($line) -or $line.StartsWith("#")) { return }
        $rel = $line.Split("|")[0].Trim()
        $from = Join-Path $vendorSrc $rel
        $to = Join-Path $vendorOut $rel
        if (-not (Test-Path -LiteralPath $from)) {
            Write-Warning "Vendor source missing: $from"
            return
        }
        $toDir = Split-Path -Parent $to
        New-Item -ItemType Directory -Force -Path $toDir | Out-Null
        if ((Get-Item -LiteralPath $from).PSIsContainer) {
            Write-Host "==> Copying vendor folder $rel"
            Copy-Item -LiteralPath $from -Destination $to -Recurse -Force
        }
        else {
            Copy-Item -LiteralPath $from -Destination $to -Force
        }
        $copied += 1
    }
    Write-Host "==> Vendor packages staged: $copied"
}

$installPs1 = Join-Path $PSScriptRoot "installer\Install-HyperFusion.ps1"
Copy-Item -LiteralPath $installPs1 -Destination (Join-Path $OutDir "Install-HyperFusion.ps1") -Force
Copy-Item -LiteralPath $installPs1 -Destination (Join-Path $payload "Install-HyperFusion.ps1") -Force

$issSrc = Join-Path $PSScriptRoot "installer\HyperFusion.iss"
$issOut = Join-Path $OutDir "HyperFusion.iss"
if (Test-Path -LiteralPath $issSrc) {
    Copy-Item -LiteralPath $issSrc -Destination $issOut -Force
}

Write-Host "==> Staged folder: $OutDir"
Write-Host "    Elevated:  .\Install-HyperFusion.ps1"

if (-not $SkipInno) {
    $iscc = @(
        "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe",
        "$env:ProgramFiles\Inno Setup 6\ISCC.exe"
    ) | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    if ($iscc) {
        Write-Host "==> Compiling setup exe with Inno Setup"
        & $iscc $issOut
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "ISCC failed (exit $LASTEXITCODE). Use Install-HyperFusion.ps1 from the staged folder."
        }
    }
    else {
        Write-Host "==> Inno Setup not installed; no .exe wrapper yet."
        Write-Host "    Install Inno Setup 6, then re-run this script to emit HyperFusion-Setup.exe."
    }
}
