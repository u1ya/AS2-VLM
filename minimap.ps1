param([switch]$Replay, [int]$Port = 8772, [switch]$NoFastPerson,
    [string]$Recording = 'recordings/slam-shared-cli-20261025-06',
    [string]$Image = 'logs/camera-20261025-videohub.jpg')
$ErrorActionPreference = 'Stop'
$python = $env:AS2_VLM_PYTHON
$fastPersonModel = Join-Path $PSScriptRoot '.models/yolox/object_detection_yolox_2022nov_int8.onnx'
$visionPython = Join-Path $PSScriptRoot '.runtime/calibration-python/Scripts/python.exe'
$fastPerson = -not $NoFastPerson -and (Test-Path -LiteralPath $fastPersonModel) -and (Test-Path -LiteralPath $visionPython)
if (-not $python) {
    if ($fastPerson) { $python = $visionPython }
    else { $python = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' }
    if (-not (Test-Path -LiteralPath $python)) { $python = (Get-Command python -ErrorAction Stop).Source }
}
$env:PYTHONUTF8 = '1'
Push-Location $PSScriptRoot
try {
    $model = $null
    try { $model = Invoke-RestMethod http://127.0.0.1:11434/api/tags -TimeoutSec 2 } catch {}
    if (-not $model) {
        $ollama = Join-Path $PSScriptRoot '.runtime/ollama-v0.40.2/ollama.exe'
        if (-not (Test-Path -LiteralPath $ollama)) { $ollama = (Get-Command ollama -ErrorAction Stop).Source }
        $env:OLLAMA_HOST = '127.0.0.1:11434'
        $env:OLLAMA_MODELS = Join-Path $PSScriptRoot '.models/ollama'
        $null = New-Item -ItemType Directory -Force -Path (Join-Path $PSScriptRoot 'logs')
        $null = Start-Process -FilePath $ollama -ArgumentList 'serve' -WorkingDirectory $PSScriptRoot -WindowStyle Hidden `
            -RedirectStandardOutput 'logs/minimap-model.stdout.log' -RedirectStandardError 'logs/minimap-model.stderr.log'
        $until = [DateTime]::UtcNow.AddSeconds(20)
        do {
            Start-Sleep -Milliseconds 300
            try { $model = Invoke-RestMethod http://127.0.0.1:11434/api/tags -TimeoutSec 2 } catch {}
        } while (-not $model -and [DateTime]::UtcNow -lt $until)
    }
    if ('qwen2.5vl:3b' -notin @($model.models | ForEach-Object { $_.name })) {
        throw 'The local qwen2.5vl:3b model is unavailable. No model is downloaded automatically.'
    }
    $arguments = @('-m', 'as2_vlm.agent.minimap', '--port', "$Port")
    if ($fastPerson) { $arguments += @('--fast-person-model', $fastPersonModel) }
    if ($Replay) { $arguments += @('--recording', $Recording, '--image', $Image) }
    & $python @arguments
    $result = $LASTEXITCODE
} finally { Pop-Location }
exit $result
