param([int]$Port = 8765)
$ErrorActionPreference = "Stop"
$pythonExecutable = $env:AS2_VLM_PYTHON
if (-not $pythonExecutable) {
    $bundled = Join-Path $env:USERPROFILE ".cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe"
    if (Test-Path -LiteralPath $bundled) {
        $pythonExecutable = $bundled
    } else {
        $pythonExecutable = (Get-Command python -ErrorAction Stop).Source
    }
}
$env:PYTHONUTF8 = "1"
Push-Location $PSScriptRoot
try {
    & $pythonExecutable -m as2_vlm.demo --port $Port
} finally {
    Pop-Location
}
