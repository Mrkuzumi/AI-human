param(
    [string]$VmcHost = "127.0.0.1",
    [int]$VmcPort = 39539,
    [int]$FingerPort = 39542,
    [double]$ArmDepthScale = 1.0,
    [string]$FaceLog = "",
    [string]$FingerLog = "",
    [string]$Hand3DLog = "",
    [switch]$Hand3DDrive,
    [switch]$EnableRgb3D,
    [switch]$WristOrientation,
    [switch]$ArmDepthOnly,
    [switch]$BodyTurn,
    [ValidateRange(20, 500)]
    [double]$RgbMaxSkewMs = 150,
    [switch]$DisableBodyTurn,
    [ValidateSet("follow", "interact")]
    [string]$Mode = "follow",
    [ValidateRange(0, 65535)]
    [int]$ModeControlPort = 39544,
    [string]$ModeControlHost = "",
    [string]$PythonExe = "",
    [switch]$Gesture,
    [string]$GestureLog = ""
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$venvPython = Join-Path $projectRoot ".venv-3d\Scripts\python.exe"
$python = $null

function Test-PythonExecutable([string]$candidate) {
    if ([string]::IsNullOrWhiteSpace($candidate)) { return $false }
    try {
        & $candidate -c "import sys; print(sys.executable)" 2>$null | Out-Null
        return $LASTEXITCODE -eq 0
    }
    catch { return $false }
}

function Test-RgbPython([string]$candidate) {
    if ([string]::IsNullOrWhiteSpace($candidate)) { return $false }
    try {
        & $candidate -c "import mediapipe, cv2, numpy" 2>$null | Out-Null
        return $LASTEXITCODE -eq 0
    }
    catch { return $false }
}

if ($PythonExe) {
    if (Test-PythonExecutable $PythonExe) { $python = $PythonExe }
    else { throw "PythonExe cannot start: $PythonExe" }
}
elseif (Test-PythonExecutable $venvPython) {
    $python = $venvPython
}
else {
    # A copied venv keeps the absolute interpreter path from the source PC.
    # The normal DMV/FNG/Face bridge uses only the Python standard library, so
    # the installed base interpreter is a valid fallback when that venv is stale.
    $systemPython = (Get-Command python.exe -ErrorAction SilentlyContinue).Source
    if (Test-PythonExecutable $systemPython) {
        $python = $systemPython
        Write-Warning "Ignoring stale .venv-3d; using system Python: $python"
    }
}
$bridge = Join-Path $PSScriptRoot "reference\motion_bridge.py"
$model = Join-Path $PSScriptRoot "models\pose_landmarker_full.task"
$handModel = Join-Path $PSScriptRoot "models\hand_landmarker.task"
$useRgb3D = $EnableRgb3D -or $Hand3DDrive -or $Hand3DLog -or $WristOrientation -or $ArmDepthOnly -or $BodyTurn
$useHand3DDrive = $Hand3DDrive
if ($WristOrientation -and $Hand3DDrive) {
    throw "Use -WristOrientation or legacy -Hand3DDrive, not both."
}
if ($BodyTurn -and $DisableBodyTurn) {
    throw "Use -BodyTurn or -DisableBodyTurn, not both."
}
if ($BodyTurn) { $ArmDepthOnly = $true }

if (-not $python) {
    throw "No usable Python was found. Install Python 3.10+ or pass -PythonExe C:\path\to\python.exe."
}
& $python -c "import sys; print('Bridge Python: ' + sys.executable)"
if ($LASTEXITCODE -ne 0) {
    throw "Python cannot start: $python"
}
if ($useRgb3D -and -not (Test-Path -LiteralPath $model)) {
    throw "Missing MediaPipe Pose Landmarker bundle: $model"
}
if ($useRgb3D) {
    & $python -c "import mediapipe, cv2, numpy; print('RGB 3D dependencies: OK')"
    if ($LASTEXITCODE -ne 0) {
        throw "RGB 3D requires mediapipe and numpy in $python. Run 'python -m pip install -r .\pc\requirements-3d.txt' or omit -EnableRgb3D."
    }
}

$bridgeArgs = @(
    $bridge, "--bind", "0.0.0.0", "--port", "39540", "--vmc-host", $VmcHost, "--vmc-port", $VmcPort,
    "--finger-port", $FingerPort,
    "--arm-depth-scale", $ArmDepthScale, "--mode", $Mode,
    "--mode-control-port", $ModeControlPort
)
if ($useRgb3D) {
    $bridgeArgs += @("--rgb-pose-model", $model, "--rgb-port", "39541")
}
if ($ModeControlHost) {
    $bridgeArgs += @("--mode-control-host", $ModeControlHost)
}
if ($FaceLog) {
    $bridgeArgs += @("--face-log", $FaceLog)
}
if ($FingerLog) {
    $bridgeArgs += @("--finger-log", $FingerLog)
}
if ($Hand3DLog -or $useHand3DDrive -or $WristOrientation) {
    if (-not (Test-Path -LiteralPath $handModel)) {
        throw "Missing $handModel. Download the MediaPipe Hand Landmarker bundle first."
    }
    $bridgeArgs += @("--rgb-hand-model", $handModel)
}
if ($Hand3DLog) {
    $bridgeArgs += @("--rgb-hand-log", $Hand3DLog)
}
if ($useHand3DDrive) {
    $bridgeArgs += "--rgb-hand-drive"
}
if ($WristOrientation) {
    $bridgeArgs += "--rgb-hand-orientation-only"
}
if ($ArmDepthOnly -or $BodyTurn) {
    $bridgeArgs += "--rgb-arm-depth-only"
}
if ($BodyTurn) {
    $bridgeArgs += "--rgb-body-turn"
}
$bridgeArgs += @("--rgb-max-skew-ms", $RgbMaxSkewMs)
if ($DisableBodyTurn) {
    $bridgeArgs += "--no-body-turn"
}
if ($Gesture) {
    $bridgeArgs += "--gesture"
}
if ($GestureLog) {
    $bridgeArgs += @("--gesture-log", $GestureLog)
}

Write-Host "Mapping build: wrist-depth-20261022"
Write-Host "Mapping build: body-turn-20261022 BodyTurn=$BodyTurn"
Write-Host "Bridge source: $bridge"
Write-Host "WristOrientation=$WristOrientation ArmDepthOnly=$ArmDepthOnly (RGB capture skew <= ${RgbMaxSkewMs}ms)"
& $python @bridgeArgs
if ($LASTEXITCODE -ne 0) {
    throw "motion_bridge.py exited with code $LASTEXITCODE. See the Python error above."
}
