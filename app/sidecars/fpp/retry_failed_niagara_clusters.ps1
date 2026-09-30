$ErrorActionPreference = 'Continue'
$env:PYTHONUTF8 = '1'
$batchPid = 43984
while (Get-Process -Id $batchPid -ErrorAction SilentlyContinue) { Start-Sleep -Seconds 30 }
$root = 'D:\Data_JY\2026_Grape_Data_Collection\2026_CLEREL_Niagara\clusters'
$py = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\.venv\Scripts\python.exe'
$cli = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\fpp_mvs_cli.py'
$stereo = Join-Path $root 'camera_cal\dlp_cal\results\camera_projector_stereo.yaml'
$handEye = Join-Path $root 'camera_cal\bfs_cal\results\flange_T_camera.yaml'
$clusters = Get-ChildItem -LiteralPath $root -Directory | Where-Object { $_.Name -ne 'camera_cal' }
foreach ($cluster in $clusters) {
  $processed = Join-Path $cluster.FullName 'multiview\processed'
  $log = Join-Path $processed 'fusion\batch_tsdf.log'
  if (-not (Test-Path -LiteralPath $log)) { continue }
  $text = Get-Content -LiteralPath $log -Raw
  if ($text -notmatch 'END .+ exit=([1-9]\d*)') { continue }
  if (Test-Path -LiteralPath $processed) { Remove-Item -LiteralPath $processed -Recurse -Force }
  $fusion = Join-Path $processed 'fusion'
  New-Item -ItemType Directory -Force -Path $fusion | Out-Null
  $retryLog = Join-Path $fusion 'retry_tsdf.log'
  "[$(Get-Date -Format s)] RETRY START $($cluster.Name)" | Set-Content -LiteralPath $retryLog
  & $py $cli --input $cluster.FullName --stereo $stereo --hand-eye $handEye --pose-mode flange_camera --dense-voxel-mm 0.25 --enable-tsdf --tsdf-voxel-mm 0.25 --tsdf-trunc-mm 2.5 *>> $retryLog
  "[$(Get-Date -Format s)] RETRY END $($cluster.Name) exit=$LASTEXITCODE" | Add-Content -LiteralPath $retryLog
}