param([string]$Version = "v0.40.2")
$ErrorActionPreference = "Stop"
if ($Version -notmatch '^v\d+\.\d+\.\d+$') { throw "Expected a version such as v0.40.2." }
$projectRoot = Split-Path $PSScriptRoot -Parent
$runtimeRoot = Join-Path $projectRoot ".runtime"
$installDir = Join-Path $runtimeRoot "ollama-$Version"
$executable = Join-Path $installDir "ollama.exe"
if (Test-Path -LiteralPath $executable) {
    Write-Output $executable
    exit 0
}
$release = Invoke-RestMethod "https://api.github.com/repos/ollama/ollama/releases/tags/$Version" -TimeoutSec 30
$assets = @($release.assets | Where-Object { $_.name -eq "ollama-windows-amd64.zip" })
if ($assets.Count -ne 1) { throw "Expected exactly one official Windows amd64 asset." }
$asset = $assets[0]
if ($asset.browser_download_url -notlike "https://github.com/ollama/ollama/releases/download/*") {
    throw "Unexpected download source."
}
$null = New-Item -ItemType Directory -Path $runtimeRoot -Force
$archive = Join-Path $runtimeRoot "ollama-$Version.zip"
$parts = @()
$curlArgs = @("--parallel", "--parallel-immediate", "--parallel-max", "8")
$chunkSize = [long](16 * 1024 * 1024)
$partCount = [int][Math]::Ceiling($asset.size / [double]$chunkSize)
for ($index = 0; $index -lt $partCount; $index++) {
    $start = [long]$index * $chunkSize
    $end = [Math]::Min($start + $chunkSize - 1, $asset.size - 1)
    $part = Join-Path $runtimeRoot "ollama-$Version.part$index"
    $parts += $part
    if ((Test-Path -LiteralPath $part) -and (Get-Item -LiteralPath $part).Length -eq ($end - $start + 1)) { continue }
    if ($curlArgs.Count -gt 4) { $curlArgs += "--next" }
    $curlArgs += @("--http1.1", "--fail", "--location", "--silent", "--show-error", "--retry", "2",
        "--connect-timeout", "20", "--max-time", "120", "--range", "$start-$end",
        "--output", $part, $asset.browser_download_url)
}
if ($curlArgs.Count -gt 4) {
    & curl.exe @curlArgs
    if ($LASTEXITCODE -ne 0) { throw "Portable runtime download failed; partial files retained." }
}
for ($index = 0; $index -lt $partCount; $index++) {
    $expected = [Math]::Min($chunkSize, $asset.size - ([long]$index * $chunkSize))
    if ((Get-Item -LiteralPath $parts[$index]).Length -ne $expected) { throw "Archive part size mismatch." }
}
$output = [System.IO.File]::Create($archive)
try {
    foreach ($part in $parts) {
        $inputStream = [System.IO.File]::OpenRead($part)
        try { $inputStream.CopyTo($output) } finally { $inputStream.Dispose() }
    }
} finally { $output.Dispose() }
if ((Get-Item -LiteralPath $archive).Length -ne $asset.size) { throw "Archive size mismatch." }
if ($asset.digest -and $asset.digest.StartsWith("sha256:")) {
    $actual = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $asset.digest.Substring(7)) { throw "Archive digest mismatch." }
}
Expand-Archive -LiteralPath $archive -DestinationPath $installDir -Force
if (-not (Test-Path -LiteralPath $executable)) { throw "Ollama executable missing from archive." }
Write-Output $executable
