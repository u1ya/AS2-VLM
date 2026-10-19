param(
    [int]$Port = 8765,
    [switch]$RobotState,
    [string]$RobotInterface = 'eth1',
    [string]$WslDistro = 'Ubuntu-22.04'
)
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
    $demoArgs = @('-m', 'as2_vlm.demo', '--port', "$Port")
    if ($RobotState) {
        $demoArgs += @('--robot-state', '--robot-interface', $RobotInterface, '--wsl-distro', $WslDistro)
    }
    & $pythonExecutable @demoArgs
} finally {
    Pop-Location
}
