#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

exe='sdicamera+yolov8-pose+yolov8-face+yolov8-finger+hdmi'
config="${1:-configs/ZG/yolov8-pose+yolov8-face+yolov8-finger+hdmi-192.168.126.100.yaml}"
uart_device="${2-${UART_DEVICE-/dev/ttyUSB0}}"
mode_host="${3:-${MODE_HOST:-192.168.126.100}}"
if [[ -n "$uart_device" && "$EUID" -ne 0 ]]; then
    command -v sudo >/dev/null || { printf 'ERROR: UART access requires root; run this script as root.\n' >&2; exit 1; }
    # Keep the listener and its cleanup under the same UID, even after sudo expires.
    printf 'Requesting sudo for UART %s -> %s:39544\n' "$uart_device" "$mode_host"
    exec sudo --preserve-env=VOICE_ENABLED,VOICE_HOST,VOICE_PORT,AUDIO_DEVICE,VOICE_UPLOAD_TOKEN -- bash "$PWD/scripts/run_board.sh" "$config" "$uart_device" "$mode_host"
fi
if [[ -n "$uart_device" ]]; then
    command -v python3 >/dev/null || { printf 'ERROR: UART control requires Python 3 (no pip packages).\n' >&2; exit 1; }
    [[ -f tools/uart_mode_bridge.py ]] || { printf 'ERROR: missing tools/uart_mode_bridge.py\n' >&2; exit 1; }
    if [[ "$uart_device" == /dev/ttyUSB* ]]; then
        bash scripts/load_ch340_driver.sh || printf '[UART] CH340 driver load failed; listener will retry.\n' >&2
    fi
fi
case "$(uname -m)" in
    aarch64|arm64) ;;
    *) printf 'ERROR: this binary requires an AArch64 board. Current machine: %s\n' "$(uname -m)" >&2; exit 1 ;;
esac
for file in "$exe" "$config" \
    imodel/ZG/yolov8-pose_ZG.json imodel/ZG/yolov8-pose_ZG.raw \
    imodel/ZG/yolov8-face_ZG.json imodel/ZG/yolov8-face_ZG.raw \
    imodel/ZG/yolov8-finger_ZG.json imodel/ZG/yolov8-finger_ZG.raw \
    names/yolov8-pose.names names/yolov8-face.names names/yolov8-finger.names; do
    if [[ ! -f "$file" ]]; then printf 'ERROR: missing %s\n' "$file" >&2; exit 1; fi
done
chmod +x -- "$exe"
if command -v ldd >/dev/null 2>&1; then
    libs=$(ldd "./$exe" 2>&1)
    printf '%s\n' "$libs"
    if [[ "$libs" == *'not found'* ]]; then
        printf 'ERROR: missing board libraries. Keep the existing working iCraft 3.36.1 environment.\n' >&2
        exit 1
    fi
fi
mkdir -p logs
log="logs/trinet-$(date +%Y%m%d-%H%M%S).log"
printf 'Runtime: %s\nConfig: %s\nLog: %s\n' "$exe" "$config" "$log"
uart_pid=''
voice_pid=''
cleanup_helpers() {
    for pid in "$voice_pid" "$uart_pid"; do
        if [[ -n "$pid" ]]; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    for pid in "$voice_pid" "$uart_pid"; do
        if [[ -n "$pid" ]]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
}
trap cleanup_helpers EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ -n "$uart_device" ]]; then
    python3 -u tools/uart_mode_bridge.py --serial "$uart_device" --host "$mode_host" > >(tee -a "$log") 2>&1 &
    uart_pid=$!
    printf 'UART control PID: %s (UDP 39544)\n' "$uart_pid"
fi
if [[ "${VOICE_ENABLED:-1}" != 0 ]]; then
    if command -v python3 >/dev/null && [[ -f tools/board_voice_stream.py && -f configs/voice.json ]]; then
        python3 -u tools/board_voice_stream.py --config configs/voice.json --host "${VOICE_HOST:-$mode_host}" > >(tee -a "$log") 2>&1 &
        voice_pid=$!
        printf 'Voice upload PID: %s (TCP host %s; port/device: configs/voice.json or overrides)\n' "$voice_pid" "${VOICE_HOST:-$mode_host}"
    else
        printf '[VOICE] disabled: Python 3 or voice files missing; video continues.\n' >&2
    fi
fi
"./$exe" "$config" 2>&1 | tee -a "$log"
