$ErrorActionPreference = 'Continue'
$env:PYTHONUTF8 = '1'
$root = 'D:\Data_JY\2026_Grape_Data_Collection\2026_CLEREL_Niagara\clusters'
$py = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\.venv\Scripts\python.exe'
$cli = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\fpp_mvs_cli.py'
$stereo = Join-Path $root 'camera_cal\dlp_cal\results\camera_projector_stereo.yaml'
$handEye = Join-Path $root 'camera_cal\bfs_cal\results\flange_T_camera.yaml'
$names = @(
  'Nagara_DM_Cluster_1_B', 'Nagara_DM_Cluster_1_T', 'Nagara_DM_Cluster_2_B',
  'Nagara_DM_Cluster_2_T', 'Nagara_DM_Cluster_3_B', 'Nagara_DM_Cluster_3_T',
  'Nagara_DM_Cluster_4_B', 'Nagara_DM_Cluster_4_T', 'Nagara_DM_Cluster_5_B',
  'Nagara_DM_Cluster_5_T'
)
$failed = $false
foreach ($name in $names) {
  $folder = Join-Path $root $name
  $fusion = Join-Path $folder 'multiview\processed\fusion'
  New-Item -ItemType Directory -Force -Path $fusion | Out-Null
  $log = Join-Path $fusion 'batch_tsdf.log'
  "[$(Get-Date -Format s)] START $name" | Set-Content -LiteralPath $log
  & $py $cli --input $folder --stereo $stereo --hand-eye $handEye --pose-mode flange_camera --dense-voxel-mm 0.25 --enable-tsdf --tsdf-voxel-mm 0.25 --tsdf-trunc-mm 2.5 *>> $log
  $code = $LASTEXITCODE
  "[$(Get-Date -Format s)] END $name exit=$code" | Add-Content -LiteralPath $log
  if ($code -ne 0) { $failed = $true }
}
if ($failed) { exit 1 }