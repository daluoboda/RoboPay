[CmdletBinding()]
param()

$profileRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$env:PYTHONPATH = Join-Path $profileRoot 'bridge'
py -3 -m pytest -q (Join-Path $profileRoot 'tests')
