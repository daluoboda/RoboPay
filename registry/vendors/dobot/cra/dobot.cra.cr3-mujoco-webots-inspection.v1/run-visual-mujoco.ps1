[CmdletBinding()]
param([double]$HoldSeconds = 180)

$profileRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$env:PYTHONPATH = Join-Path $profileRoot 'bridge'
py -3 (Join-Path $profileRoot 'bridge\run_mujoco_inspection.py') --hold-seconds $HoldSeconds
