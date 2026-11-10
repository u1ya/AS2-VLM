param(
    [Parameter(Mandatory)][ValidatePattern('^[A-Za-z0-9_-]{1,80}$')][string]$Name,
    [ValidateRange(3,30)][int]$Seconds = 8
)
$ErrorActionPreference = 'Stop'
$url = 'http://127.0.0.1:8780'
$health = Invoke-RestMethod "$url/health" -TimeoutSec 5
if ($health.read_only -ne $true -or $health.connected -ne $true) { throw 'Read-only acquisition is not connected' }
$reply = Invoke-RestMethod "$url/record" -Method Post -Headers @{Origin=$url} `
    -ContentType application/json -Body (@{name=$Name;seconds=$Seconds} | ConvertTo-Json -Compress) -TimeoutSec 5
if ($reply.accepted -ne $true) { throw 'Recording was not accepted' }
Write-Output "Recording $Name from shared acquisition; panels remain connected."
$deadline = [DateTime]::UtcNow.AddSeconds($Seconds + 30)
$finished = $false
while ([DateTime]::UtcNow -lt $deadline) {
    Start-Sleep -Milliseconds 300
    $status = (Invoke-RestMethod "$url/snapshot" -TimeoutSec 5).recording
    if ($status.command_error) { throw "Acquisition rejected recording: $($status.command_error)" }
    if ($status.name -eq $Name -and $status.finished -eq $true) {
        if ($status.ok -ne $true) { throw "Recording incomplete: $($status | ConvertTo-Json -Compress)" }
        $finished = $true
        break
    }
}
if (-not $finished) { throw 'Recording completion not confirmed; inspect /snapshot and private manifest before retrying' }
$python = $env:AS2_VLM_PYTHON
if (-not $python) {
    $python = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
    if (-not (Test-Path -LiteralPath $python)) { $python = (Get-Command python -ErrorAction Stop).Source }
}
Push-Location $PSScriptRoot
try {
    & $python -m as2_vlm.agent.sensor_audit "recordings/$Name" --replay-check
    if ($LASTEXITCODE -ne 0) { throw 'Offline audit failed' }
} finally { Pop-Location }
Write-Output 'Recording and decoding completed. Audit completion is not SLAM readiness.'
