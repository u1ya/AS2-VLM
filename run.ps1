$ErrorActionPreference = "Stop"
$pythonExecutable = $env:AS2_VLM_PYTHON
if (-not $pythonExecutable) {
    $bundledPython = Join-Path $env:USERPROFILE ".cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe"
    if (Test-Path -LiteralPath $bundledPython) {
        $pythonExecutable = $bundledPython
    } else {
        $pythonExecutable = (Get-Command python -ErrorAction Stop).Source
    }
}
$env:PYTHONUTF8 = "1"
Push-Location $PSScriptRoot
try {
    & $pythonExecutable -m as2_vlm @args
    $result = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $result
