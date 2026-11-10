$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$destination = Join-Path $root '.models/yolox'
$commit = '48b10fa505aeb51fbd127fc60a5c660040e823b7'
$name = 'object_detection_yolox_2022nov_int8.onnx'
$expected = '01a3b0f400b30bc1e45230e991b2e499ab42622485a330021947333fbaf03935'
$null = New-Item -ItemType Directory -Force -Path $destination
$path = Join-Path $destination $name
if (-not (Test-Path -LiteralPath $path) -or (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
    $temporary = Join-Path $destination ($name + '.download')
    Invoke-WebRequest -Uri "https://media.githubusercontent.com/media/opencv/opencv_zoo/$commit/models/object_detection_yolox/$name" -OutFile $temporary -TimeoutSec 60
    if ((Get-FileHash -LiteralPath $temporary -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
        throw 'Downloaded person model checksum differs from pinned OpenCV Zoo data; model was not installed'
    }
    Move-Item -LiteralPath $temporary -Destination $path -Force
}
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/opencv/opencv_zoo/$commit/models/object_detection_yolox/LICENSE" -OutFile (Join-Path $destination 'LICENSE') -TimeoutSec 30
@{repository='https://github.com/opencv/opencv_zoo';commit=$commit;model=$name;sha256=$expected;license='Apache-2.0'} |
    ConvertTo-Json | Set-Content -Encoding utf8 (Join-Path $destination 'provenance.json')
Write-Host 'Pinned YOLOX person model installed locally. No robot commands or service restart performed.'
