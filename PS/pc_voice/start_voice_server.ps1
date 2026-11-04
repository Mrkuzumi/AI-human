param(
    [string]$Python = "python",
    [string]$Bind = "0.0.0.0",
    [int]$Port = 8766,
    [switch]$Check
)
$ErrorActionPreference = "Stop"
$env:PYTHONUTF8 = "1"
$env:PYTHONUNBUFFERED = "1"
$voiceConfig = Get-Content -LiteralPath (Join-Path $PSScriptRoot "voice.local.json") -Raw -Encoding UTF8 | ConvertFrom-Json
$env:VOICE_UPLOAD_TOKEN = [string]$voiceConfig.upload_token
$env:TENCENTCLOUD_APP_ID = [string]$voiceConfig.app_id
$env:TENCENTCLOUD_SECRET_ID = [string]$voiceConfig.secret_id
$env:TENCENTCLOUD_SECRET_KEY = [string]$voiceConfig.secret_key
$venvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $venvPython)) {
    & $Python -m venv (Join-Path $PSScriptRoot ".venv")
    if ($LASTEXITCODE -ne 0) { throw "Failed to create the PC voice virtual environment." }
}
& $venvPython -m pip install --disable-pip-version-check -r (Join-Path $PSScriptRoot "requirements.txt")
if ($LASTEXITCODE -ne 0) { throw "Failed to install the PC voice dependency." }
$voiceArgs = @("-u", (Join-Path $PSScriptRoot "stream_receiver.py"),
    "--host", $Bind, "--port", "$Port", "--voice-mode-control")
if ($Check) { $voiceArgs += "--check" }
& $venvPython @voiceArgs
exit $LASTEXITCODE
