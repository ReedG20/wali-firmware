#!/usr/bin/env bash
# Build and flash the Mind pendant firmware.  Usage: tools/flash.sh [port]   (build only: BUILD_ONLY=1)
set -euo pipefail
cd "$(dirname "$0")/.."
PORT="${1:-${PORT:-/dev/cu.usbmodem1101}}"
FQBN="esp32:esp32:XIAO_ESP32S3:PSRAM=opi"
arduino-cli compile --fqbn "$FQBN" --build-path build mind_pendant
[ "${BUILD_ONLY:-0}" = 1 ] && exit 0
pkill -f "tools/serial_log.py|serial_log.py" 2>/dev/null && sleep 0.5 || true   # free the port
arduino-cli upload --fqbn "$FQBN" --input-dir build -p "$PORT" mind_pendant
