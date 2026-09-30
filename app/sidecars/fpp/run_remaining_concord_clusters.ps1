$ErrorActionPreference = 'Continue'
$env:PYTHONUTF8 = '1'
$root = 'D:\Data_JY\2026_Grape_Data_Collection\2026_CLEREL_Concord\clusters'
$py = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\.venv\Scripts\python.exe'
$cli = 'D:\Pototypy\HyperFusion\app\sidecars\fpp\fpp_mvs_cli.py'
$names = @(
  'Concord_BMD_Cluster_1_T', 'Concord_BMD_Cluster_2_B', 'Concord_BMD_Cluster_2_T',
  'Concord_BMD_Cluster_3_B', 'Concord_BMD_Cluster_3_T', 'Concord_BMD_Cluster_4_B',
  'Concord_BMD_Cluster_4_T', 'Concord_BMD_Cluster_5_B', 'Concord_BMD_Cluster_5_T'
)
$failed = $false
foreach ($name in $names) {
  $folder = Join-Path $root $name
  $fusion = Join-Path $folder 'multiview\processed\fusion'
  New-Item -ItemType Directory -Force -Path $fusion | Out-Null
  $log = Join-Path $fusion 'batch_tsdf.log'
  "[$(Get-Date -Format s)] START $name" | Set-Content -LiteralPath $log
  & $py $cli --input $folder --dense-voxel-mm 0.25 --enable-tsdf --tsdf-voxel-mm 0.25 --tsdf-trunc-mm 2.5 *>> $log
  $code = $LASTEXITCODE
  "[$(Get-Date -Format s)] END $name exit=$code" | Add-Content -LiteralPath $log
  if ($code -ne 0) { $failed = $true }
}
if ($failed) { exit 1 }