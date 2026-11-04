#!/usr/bin/env bash
set -euo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

# Keep voice and UART consistent with the teammate's four UDP YAML targets.
export VOICE_HOST=192.168.126.10
exec bash "$root_dir/scripts/run_board.sh" \
    "$root_dir/configs/ZG/yolov8-pose+yolov8-face+yolov8-finger+hdmi-192.168.126.10.yaml" \
    "${UART_DEVICE-/dev/ttyACM0}" \
    "192.168.126.10"
