param([switch]$Sim, [int]$Port = 8770)
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
        & $python -m as2_vlm.agent --real --config configs/agent.as2-sensors.json --port $Port
    }
    $result = $LASTEXITCODE
} finally { Pop-Location }
exit $result
