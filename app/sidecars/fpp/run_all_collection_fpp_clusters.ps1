$ErrorActionPreference = 'Continue'
$env:PYTHONUTF8 = '1'
$root = 'D:\Data_JY\2026_Grape_Data_Collection'
$py = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\.venv\Scripts\python.exe'
$cli = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\fpp_mvs_cli.py'
$niagaraRoot = Join-Path $root '2026_CLEREL_Niagara_DM\clusters'
$niagaraStereo = Join-Path $niagaraRoot 'camera_cal\dlp_cal\results\camera_projector_stereo.yaml'
$niagaraHandEye = Join-Path $niagaraRoot 'camera_cal\bfs_cal\results\flange_T_camera.yaml'
$multiviews = Get-ChildItem -LiteralPath $root -Directory -Recurse | Where-Object { $_.Name -eq 'multiview' } | Where-Object { (Get-ChildItem -LiteralPath $_.FullName -File -Filter '*.json' | Where-Object { $_.Name -ne 'transforms.json' } | Select-Object -First 1) -ne $null } | Sort-Object FullName
$failed = $false
foreach ($mv in $multiviews) {
  $cluster = $mv.Parent
  $fusion = Join-Path $mv.FullName 'processed\fusion'
  New-Item -ItemType Directory -Force -Path $fusion | Out-Null
  $log = Join-Path $fusion 'batch_tsdf.log'
  "[$(Get-Date -Format s)] START $($cluster.FullName)" | Set-Content -LiteralPath $log
  $common = @($cli, '--input', $cluster.FullName, '--dense-voxel-mm', '0.25', '--enable-tsdf', '--tsdf-voxel-mm', '0.25', '--tsdf-trunc-mm', '2.5')
  if ($cluster.FullName.StartsWith($niagaraRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    $common += @('--stereo', $niagaraStereo, '--hand-eye', $niagaraHandEye, '--pose-mode', 'flange_camera')
  }
  & $py @common *>> $log
  $code = $LASTEXITCODE
  "[$(Get-Date -Format s)] END $($cluster.Name) exit=$code" | Add-Content -LiteralPath $log
  if ($code -ne 0) { $failed = $true }
}
if ($failed) { exit 1 }