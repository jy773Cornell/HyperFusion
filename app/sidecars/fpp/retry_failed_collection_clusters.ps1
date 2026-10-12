$ErrorActionPreference = 'Continue'
$env:PYTHONUTF8 = '1'
$root = 'D:\Data_JY\2026_Grape_Data_Collection'
$py = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\.venv\Scripts\python.exe'
$cli = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\fpp_mvs_cli.py'
$niagaraRoot = Join-Path $root '2026_CLEREL_Niagara_DM\clusters'
$niagaraStereo = Join-Path $niagaraRoot 'camera_cal\dlp_cal\results\camera_projector_stereo.yaml'
$niagaraHandEye = Join-Path $niagaraRoot 'camera_cal\bfs_cal\results\flange_T_camera.yaml'
while ($true) {
  $active = Get-ChildItem -LiteralPath $root -Recurse -File -Filter 'batch_tsdf.log' | Where-Object {
    $text = Get-Content -LiteralPath $_.FullName -Raw -ErrorAction SilentlyContinue
    $text -match 'START' -and $text -notmatch 'END .+ exit='
  }
  if (-not $active) { break }
  Start-Sleep -Seconds 60
}
$failedLogs = Get-ChildItem -LiteralPath $root -Recurse -File -Filter 'batch_tsdf.log' | Where-Object {
  (Get-Content -LiteralPath $_.FullName -Raw -ErrorAction SilentlyContinue) -match 'END .+ exit=([1-9]\d*)'
}
foreach ($logFile in $failedLogs) {
  $multiview = Split-Path (Split-Path $logFile.DirectoryName -Parent) -Parent
  $cluster = Split-Path $multiview -Parent
  $processed = Join-Path $multiview 'processed'
  if (Test-Path -LiteralPath $processed) { Remove-Item -LiteralPath $processed -Recurse -Force }
  $fusion = Join-Path $processed 'fusion'
  New-Item -ItemType Directory -Force -Path $fusion | Out-Null
  $retryLog = Join-Path $fusion 'retry_tsdf.log'
  "[$(Get-Date -Format s)] RETRY START $cluster" | Set-Content -LiteralPath $retryLog
  $args = @($cli, '--input', $cluster, '--dense-voxel-mm', '0.25', '--enable-tsdf', '--tsdf-voxel-mm', '0.25', '--tsdf-trunc-mm', '2.5')
  if ($cluster.StartsWith($niagaraRoot, [System.StringComparison]::OrdinalIgnoreCase)) { $args += @('--stereo', $niagaraStereo, '--hand-eye', $niagaraHandEye, '--pose-mode', 'flange_camera') }
  & $py @args *>> $retryLog
  "[$(Get-Date -Format s)] RETRY END $cluster exit=$LASTEXITCODE" | Add-Content -LiteralPath $retryLog
}