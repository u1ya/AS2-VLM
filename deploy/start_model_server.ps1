param([string]$OllamaPath = "ollama", [switch]$PullModel, [switch]$Detached)
$ErrorActionPreference = "Stop"
$model = "qwen2.5vl:3b"
$env:OLLAMA_HOST = "127.0.0.1:11434"
$env:OLLAMA_MODELS = Join-Path $PSScriptRoot "../.models/ollama"
if (-not (Get-Command $OllamaPath -ErrorAction SilentlyContinue)) {
    throw "Ollama is not installed. Install from https://ollama.com/download/windows or provide -OllamaPath."
}
if ($PullModel) {
    try { $null = Invoke-RestMethod "http://127.0.0.1:11434/api/version" -TimeoutSec 3 }
    catch { throw "Start this script without -PullModel in another terminal first." }
    & $OllamaPath pull $model
    if ($LASTEXITCODE -ne 0) { throw "Model download failed." }
} else {
    if ($Detached) {
        $logRoot = Join-Path $PSScriptRoot "../logs"
        $null = New-Item -ItemType Directory -Path $logRoot -Force
        $process = Start-Process -FilePath (Get-Command $OllamaPath).Source -ArgumentList "serve" -WindowStyle Hidden -PassThru `
            -RedirectStandardOutput (Join-Path $logRoot "ollama.stdout.log") `
            -RedirectStandardError (Join-Path $logRoot "ollama.stderr.log")
        Write-Output "Ollama process started: $($process.Id). Check CLI --healthcheck before inference."
        exit 0
    }
    # Foreground process: closing the terminal stops this server.
    & $OllamaPath serve
    if ($LASTEXITCODE -ne 0) { throw "Ollama server failed; check whether the port is already occupied." }
}
