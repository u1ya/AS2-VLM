param([int]$Port = 8780, [string]$Config = 'configs/agent.as2-sensors.json')
$ErrorActionPreference = 'Stop'
$python = $env:AS2_VLM_PYTHON
if (-not $python) {
    $python = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
    if (-not (Test-Path -LiteralPath $python)) { $python = (Get-Command python -ErrorAction Stop).Source }
}
$env:PYTHONUTF8 = '1'
Push-Location $PSScriptRoot
try { & $python -m as2_vlm.agent.sensor_hub --config $Config --port $Port; $result = $LASTEXITCODE }
finally { Pop-Location }
exit $result
