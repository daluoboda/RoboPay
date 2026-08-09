[CmdletBinding()]
param(
  [switch]$DryRun,
  [switch]$OpenBaseScan,
  [Parameter(Mandatory = $false)][string]$TunnelBin = $env:TUNNEL_BIN,
  [Parameter(Mandatory = $false)][string]$ZenohdBin = $env:ZENOH_ROUTER_BIN
)

$ErrorActionPreference = 'Stop'
$profileRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $profileRoot '..\..\..\..\..')).Path
$routerStartedHere = $false
$routerProcess = $null

if ([string]::IsNullOrWhiteSpace($ZenohdBin)) {
  $ZenohdBin = Join-Path $repoRoot '.zenoh-router\zenohd'
}
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
if (-not (Test-Path -LiteralPath $ZenohdBin)) {
  throw "Zenoh router was not found: '$ZenohdBin'. Supply -ZenohdBin or ZENOH_ROUTER_BIN."
}

$env:TUNNEL_BIN = (Resolve-Path -LiteralPath $TunnelBin).Path
$env:ROBOT_PAYEE_ADDRESS = $payee
$env:ROBO_PAYEE_ADDRESS = $payee
$env:PYTHONPATH = Join-Path $profileRoot 'bridge'
$arguments = @((Join-Path $profileRoot 'bridge\run_live_base_sepolia_e2e.py'), '--visual')
if ($DryRun) { $arguments += '--dry-run' }
if ($OpenBaseScan) { $arguments += '--open-basescan' }

try {
  $routerListening = Test-NetConnection -ComputerName 127.0.0.1 -Port 7447 -InformationLevel Quiet -WarningAction SilentlyContinue
  if (-not $routerListening) {
    $zenohdWindowsPath = (Resolve-Path -LiteralPath $ZenohdBin).Path
    $zenohdDrive = $zenohdWindowsPath.Substring(0, 1).ToLowerInvariant()
    $zenohdRest = $zenohdWindowsPath.Substring(2).Replace('\', '/')
    $zenohdWslPath = "/mnt/$zenohdDrive$zenohdRest"
    if ([string]::IsNullOrWhiteSpace($zenohdWslPath)) {
      throw 'Could not translate the Zenoh router path for WSL.'
    }
    $routerProcess = Start-Process -FilePath 'wsl.exe' -ArgumentList @('-d', 'Ubuntu-22.04', '--exec', $zenohdWslPath, '-l', 'tcp/0.0.0.0:7447') -WindowStyle Hidden -PassThru
    $routerStartedHere = $true
    $deadline = (Get-Date).AddSeconds(15)
    do {
      Start-Sleep -Milliseconds 250
      if ($routerProcess.HasExited) {
        throw "Zenoh router exited before listening on tcp/127.0.0.1:7447 (exit $($routerProcess.ExitCode))."
      }
      $routerListening = Test-NetConnection -ComputerName 127.0.0.1 -Port 7447 -InformationLevel Quiet -WarningAction SilentlyContinue
    } until ($routerListening -or (Get-Date) -ge $deadline)
    if (-not $routerListening) {
      throw 'Zenoh router did not listen on tcp/127.0.0.1:7447 within 15 seconds.'
    }
    Write-Host '[visual-e2e] Zenoh router ready on tcp/127.0.0.1:7447.' -ForegroundColor Cyan
  } else {
    Write-Host '[visual-e2e] Reusing Zenoh router on tcp/127.0.0.1:7447.' -ForegroundColor Cyan
  }

  py -3 @arguments
  $runnerExitCode = $LASTEXITCODE
  if (-not $DryRun) {
    [void](Read-Host 'Visual proof finished. Keep OBS recording as needed, then press Enter to close this terminal')
  }
  exit $runnerExitCode
} finally {
  if ($routerStartedHere -and $null -ne $routerProcess -and -not $routerProcess.HasExited) {
    Stop-Process -Id $routerProcess.Id -Force -ErrorAction SilentlyContinue
  }
}
