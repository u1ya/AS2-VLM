param([switch]$Sim, [int]$Port = 8770, [switch]$ManualControl)
$ErrorActionPreference = 'Stop'
$python = $env:AS2_VLM_PYTHON
if (-not $python) {
    $python = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
    if (-not (Test-Path -LiteralPath $python)) { $python = (Get-Command python -ErrorAction Stop).Source }
}
$env:PYTHONUTF8 = '1'
Push-Location $PSScriptRoot
try {
    if ($Sim) {
        & $python -m as2_vlm.agent --sim --port $Port
    } else {
        try {
            $health = Invoke-RestMethod http://127.0.0.1:8780/health -TimeoutSec 2
            if ($health.read_only -ne $true) { throw 'Unexpected service on sensor hub port' }
        } catch {
            throw 'Start ./sensor-hub.ps1 in a separate terminal before the live panel.'
        }
        $arguments = @('-m', 'as2_vlm.agent', '--real', '--config', 'configs/agent.as2-sensors.json', '--port', "$Port")
        if ($ManualControl) { $arguments += '--manual-control' }
        & $python @arguments
    }
    $result = $LASTEXITCODE
} finally { Pop-Location }
exit $result
