param([switch]$SimulationOnly)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$python = $env:AS2_VLM_PYTHON
if (-not $python) { $python = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' }
if (-not (Test-Path -LiteralPath $python)) { $python = (Get-Command python -ErrorAction Stop).Source }
$null = New-Item -ItemType Directory -Force -Path (Join-Path $root 'logs')
function Probe([string]$Url) {
    try { Invoke-RestMethod -Uri $Url -TimeoutSec 2 } catch { return $null }
}
function Wait-Ready([string]$Url) {
    $until = [DateTime]::UtcNow.AddSeconds(45)
    do { $data = Probe $Url; if ($data) { return $data }; Start-Sleep -Milliseconds 300 } while ([DateTime]::UtcNow -lt $until)
    throw "Service not ready: $Url. Inspect logs/."
}
function Python-Service([string]$Arguments, [string]$Name) {
    $p = Start-Process -FilePath $python -ArgumentList $Arguments -WorkingDirectory $root -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $root "logs/$Name.stdout.log") -RedirectStandardError (Join-Path $root "logs/$Name.stderr.log") -PassThru
    Write-Host "$Name PID: $($p.Id)"
}
if (-not (Probe 'http://127.0.0.1:11434/api/tags')) {
    $ollama = Join-Path $root '.runtime/ollama-v0.40.2/ollama.exe'
    if (-not (Test-Path -LiteralPath $ollama)) { $ollama = (Get-Command ollama -ErrorAction Stop).Source }
    $env:OLLAMA_HOST = '127.0.0.1:11434'; $env:OLLAMA_MODELS = Join-Path $root '.models/ollama'
    $p = Start-Process -FilePath $ollama -ArgumentList 'serve' -WorkingDirectory $root -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $root 'logs/model-services.stdout.log') -RedirectStandardError (Join-Path $root 'logs/model-services.stderr.log') -PassThru
    Write-Host "Ollama PID: $($p.Id)"
}
$model = Wait-Ready 'http://127.0.0.1:11434/api/tags'
if ('qwen2.5vl:3b' -notin @($model.models | ForEach-Object {$_.name})) { throw 'Install qwen2.5vl:3b before starting language services' }
$sim = Probe 'http://127.0.0.1:8770/api/state'
if (-not $sim) {
    Python-Service '-m as2_vlm.agent --sim --port 8770 --planner ollama --relative-language' 'simulation-service'
    $sim = Wait-Ready 'http://127.0.0.1:8770/api/state'
}
$simConfig = Probe 'http://127.0.0.1:8770/api/config'
if ($sim.mode -ne 'mock' -or $simConfig.relative_language_planner -ne $true) { throw 'Port 8770 is occupied by another profile; not stopped automatically' }
Write-Host 'Simulation: http://127.0.0.1:8770/'
if ($SimulationOnly) { return }
if (-not (Test-Path -LiteralPath (Join-Path $root 'build/as2-probe/as2_relative_move'))) { throw 'Build as2_relative_move before starting the real robot service' }
$health = Probe 'http://127.0.0.1:8780/health'
if (-not $health) {
    Python-Service '-m as2_vlm.agent.sensor_hub --config configs/agent.as2-sensors.json --port 8780' 'sensor-service'
    $health = Wait-Ready 'http://127.0.0.1:8780/health'
}
if ($health.read_only -ne $true) { throw 'Unexpected service on sensor hub port' }
$live = Probe 'http://127.0.0.1:8771/api/state'
if (-not $live) {
    Python-Service '-m as2_vlm.agent --real --config configs/agent.as2-sensors.json --port 8771 --manual-control' 'robot-service'
    $live = Wait-Ready 'http://127.0.0.1:8771/api/state'
}
if ($live.mode -ne 'as2' -or $live.manual_control.language_planning -ne $true) { throw 'Port 8771 is occupied by another profile; not stopped automatically' }
Write-Host 'Real robot: http://127.0.0.1:8771/'
Write-Host 'Services ready. No physical commands were sent. Review plans before execution.'
