param([int]$Port = 8773, [double]$SquareMm = 25, [string]$Session = '', [string]$Intrinsics = '')
$ErrorActionPreference = 'Stop'
$python = Join-Path $PSScriptRoot '.runtime/calibration-python/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $python)) {
    throw 'Install the optional calibration environment first; see docs/on_site_calibration.md'
}
Push-Location $PSScriptRoot
try {
    $arguments = @('-m', 'as2_vlm.agent.camera_calibration', '--port', "$Port", '--square-mm', $SquareMm.ToString([Globalization.CultureInfo]::InvariantCulture))
    if ($Session) { $arguments += @('--session', $Session) }
    if ($Intrinsics) { $arguments += @('--intrinsics', $Intrinsics) }
    & $python @arguments
    $result = $LASTEXITCODE
}
finally { Pop-Location }
exit $result
