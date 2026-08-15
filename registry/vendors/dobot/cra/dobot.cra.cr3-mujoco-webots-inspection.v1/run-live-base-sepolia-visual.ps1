[CmdletBinding()]
param(
  [ValidateRange(0, 20)][int]$StartHoldSeconds = 6,
  [ValidateRange(0, 20)][int]$TargetHoldSeconds = 2,
  [ValidateRange(0, 20)][int]$FinalHoldSeconds = 3,
  [switch]$DryRun,
  [switch]$OpenBaseScan,
  [switch]$PauseAfter,
  [Parameter(Mandatory = $false)][string]$TunnelBin = $env:TUNNEL_BIN
)

$ErrorActionPreference = 'Stop'
$profileRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$payee = if (-not [string]::IsNullOrWhiteSpace($env:ROBO_PAYEE_ADDRESS)) {
  $env:ROBO_PAYEE_ADDRESS
} else {
  $env:ROBOT_PAYEE_ADDRESS
}

foreach ($name in 'ROBOT_PAYEE_ADDRESS', 'TUNNEL_BIN') {
  if ($name -eq 'TUNNEL_BIN' -and -not [string]::IsNullOrWhiteSpace($TunnelBin)) { continue }
  if ($name -eq 'ROBOT_PAYEE_ADDRESS' -and -not [string]::IsNullOrWhiteSpace($payee)) { continue }
  if ([string]::IsNullOrWhiteSpace([Environment]::GetEnvironmentVariable($name))) {
    throw "Missing $name in the current process environment. Do not put credentials in this file."
  }
}
if (-not $DryRun -and [string]::IsNullOrWhiteSpace($env:BASE_SEPOLIA_PRIVATE_KEY)) {
  throw 'Missing BASE_SEPOLIA_PRIVATE_KEY in the current process environment. It is never stored by this launcher.'
}
if (-not (Test-Path -LiteralPath $TunnelBin)) {
  throw "Tunnel binary was not found: '$TunnelBin'. Supply -TunnelBin or TUNNEL_BIN from a hardened catalog-aware Tunnel build."
}

$env:TUNNEL_BIN = (Resolve-Path -LiteralPath $TunnelBin).Path
$env:ROBOT_PAYEE_ADDRESS = $payee
$env:ROBO_PAYEE_ADDRESS = $payee
$env:PYTHONPATH = Join-Path $profileRoot 'bridge'
$commitSha = (& git -C $profileRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $commitSha -notmatch '^[0-9a-f]{40}$') {
  throw 'Unable to resolve the exact Git commit for the visual evidence run.'
}
$env:ROBO_PAY_COMMIT_SHA = $commitSha
$env:DOBOT_CR3_MUJOCO_VIEWER_WAIT_FOR_ENTER = 'true'
$env:DOBOT_CR3_MUJOCO_VIEWER_START_HOLD_SECONDS = [string]$StartHoldSeconds
$env:DOBOT_CR3_MUJOCO_VIEWER_TARGET_HOLD_SECONDS = [string]$TargetHoldSeconds
$env:DOBOT_CR3_MUJOCO_VIEWER_HOLD_SECONDS = [string]$FinalHoldSeconds

Write-Host 'Arrange this terminal for a stable split-screen recording, then press Enter to begin.'
Write-Host ''
Write-Host 'OBS sequence: bridge ready -> discovery -> unpaid 402 -> first paid 202/action_id -> CR3 MuJoCo three-tag motion -> correlated result -> settlement -> BaseScan'
Write-Host "Evidence commit: $commitSha"
Write-Host "Visual holds: initial ${StartHoldSeconds}s; each tag ${TargetHoldSeconds}s; final ${FinalHoldSeconds}s."
Write-Host 'After HTTP 202, the MuJoCo viewer will pause once so it can be placed beside this terminal before motion starts.'
Write-Host 'Secrets are loaded from the current process and will not be printed or written.'
[void](Read-Host 'Press Enter to begin the current-head recording')
$arguments = @((Join-Path $profileRoot 'bridge\run_live_base_sepolia_e2e.py'), '--visual')
if ($DryRun) { $arguments += '--dry-run' }
if ($OpenBaseScan) { $arguments += '--open-basescan' }
py -3 @arguments
$exitCode = $LASTEXITCODE
Remove-Item Env:ROBO_PAY_COMMIT_SHA -ErrorAction SilentlyContinue
Remove-Item Env:DOBOT_CR3_MUJOCO_VIEWER_WAIT_FOR_ENTER -ErrorAction SilentlyContinue
Remove-Item Env:DOBOT_CR3_MUJOCO_VIEWER_START_HOLD_SECONDS -ErrorAction SilentlyContinue
Remove-Item Env:DOBOT_CR3_MUJOCO_VIEWER_TARGET_HOLD_SECONDS -ErrorAction SilentlyContinue
Remove-Item Env:DOBOT_CR3_MUJOCO_VIEWER_HOLD_SECONDS -ErrorAction SilentlyContinue
if ($PauseAfter) {
  [void](Read-Host 'Recording complete. Press Enter to close this window')
}
exit $exitCode
