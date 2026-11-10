param(
    [Parameter(Mandatory)][ValidatePattern('^[A-Za-z0-9_-]+$')][string]$RecordingName,
    [ValidateRange(3, 30)][int]$Seconds = 8,
    [int[]]$Ports = @(8770, 8771),
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$')][string]$Interface = 'eth0',
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$')][string]$Distro = 'Ubuntu-22.04'
)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
if ($root -notmatch '^([A-Za-z]):[\\/]') { throw 'A local Windows drive is required' }
$linuxRoot = '/mnt/' + $Matches[1].ToLowerInvariant() + $root.Substring(2).Replace('\', '/')
$destination = Join-Path $root "recordings/$RecordingName"
if (Test-Path -LiteralPath $destination) { throw 'Use a new recording directory' }
$binary = "$linuxRoot/build/as2-probe/as2_record_sensors"
if (-not (Test-Path -LiteralPath (Join-Path $root 'build/as2-probe/as2_record_sensors'))) {
    throw 'Build as2_record_sensors first'
}
$states = @{}
$restore = [System.Collections.Generic.List[int]]::new()
$restoreErrors = [System.Collections.Generic.List[string]]::new()
$captureExit = $null

function Read-State([int]$Port) {
    Invoke-RestMethod -Uri "http://127.0.0.1:$Port/api/state" -TimeoutSec 10
}
function Set-Simulation([int]$Port, [bool]$Enabled) {
    $origin = "http://127.0.0.1:$Port"
    $reply = Invoke-RestMethod -Uri "$origin/api/mode" -Method Post -TimeoutSec 45 `
        -Headers @{Origin = $origin} -ContentType 'application/json' `
        -Body (@{simulation = $Enabled} | ConvertTo-Json -Compress)
    if ($reply.ok -ne $true) { throw "Panel ${Port}: $($reply.reason)" }
}

# Validate every panel before changing any mode. Native motion transports are excluded.
foreach ($portNumber in ($Ports | Select-Object -Unique)) {
    if ($portNumber -lt 1 -or $portNumber -gt 65535) { throw 'Invalid panel port' }
    $state = Read-State $portNumber
    if ($state.sensors.shared_acquisition -eq $true) {
        throw 'Shared acquisition is active. Use record-shared-sensors.ps1; panel switching does not stop the hub.'
    }
    if ($state.mode -ne 'mock' -and (
        $state.read_only -ne $true -or $state.telemetry.source -ne 'as2_cpp_readonly' -or
        $state.telemetry.motion_enabled -ne $false -or $state.telemetry.armed -ne $false)) {
        throw "Panel ${portNumber} is not a verified read-only transport"
    }
    if ($state.state -ne 'IDLE') { throw "Panel ${portNumber} is not idle" }
    $states[[string]$portNumber] = $state.mode
}

try {
    foreach ($portNumber in $states.Keys) {
        if ($states[$portNumber] -eq 'as2') {
            $restore.Add($portNumber)
            Set-Simulation $portNumber $true
        }
    }
    foreach ($portNumber in $states.Keys) {
        if ((Read-State $portNumber).mode -ne 'mock') { throw 'A sensor panel is still live' }
    }
    $bridges = & wsl -d $Distro -- pgrep -x as2_sensor_stre
    if ($LASTEXITCODE -eq 0) { throw "Sensor bridge still running: $bridges" }
    if ($LASTEXITCODE -ne 1) { throw 'Could not inspect remaining sensor processes' }
    Write-Output 'Panel sensor subscriptions stopped; starting read-only capture.'
    & wsl -d $Distro -- timeout ($Seconds + 15) $binary $Interface "$linuxRoot/recordings/$RecordingName" $Seconds 256
    $captureExit = $LASTEXITCODE
    if ($captureExit -ne 0) { throw "Recorder exited with code $captureExit" }
} finally {
    foreach ($portNumber in $restore) {
        try {
            Set-Simulation $portNumber $false
            $state = Read-State $portNumber
            if ($state.mode -ne 'as2' -or $state.read_only -ne $true -or
                $state.telemetry.source -ne 'as2_cpp_readonly' -or
                $state.telemetry.motion_enabled -ne $false -or $state.telemetry.armed -ne $false) {
                throw 'Read-only restoration verification failed'
            }
            Write-Output "Panel $portNumber restored to read-only live sensors."
        } catch {
            $restoreErrors.Add("Panel ${portNumber}: $_")
            Write-Warning "Could not restore panel ${portNumber}: $_"
        }
    }
    if (Test-Path -LiteralPath $destination) {
        @{schema = 1; read_only = $true; panel_modes_before = $states;
          panels_temporarily_disconnected = @($restore.ToArray());
          restoration_errors = @($restoreErrors.ToArray()); recorder_exit_code = $captureExit;
          note = 'Sequential stationary load diagnostic; not a controlled benchmark'} |
            ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $destination 'load-experiment.json') -Encoding utf8
    }
}
if ($restoreErrors.Count) { throw ($restoreErrors -join '; ') }
