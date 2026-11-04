#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if (( EUID != 0 )); then
    printf 'ERROR: load CH340 driver as root.\n' >&2
    exit 1
fi
driver_dir="drivers/$(uname -r)"
for module in usbserial ch341; do
    if ! grep -q "^${module} " /proc/modules; then
        insmod "$driver_dir/$module.ko"
    fi
done
printf '[UART] CH340 driver loaded for %s\n' "$(uname -r)"
