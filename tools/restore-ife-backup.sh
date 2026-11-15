#!/usr/bin/env bash
# Restore Ife's firmware 1.2 (full 8 MB image taken before this firmware was first flashed).
set -euo pipefail
cd "$(dirname "$0")/.."
PORT="${1:-/dev/cu.usbmodem1101}"
IMG=backups/ife-fw-1.2-backup.bin
[ -f "$IMG" ] || { echo "missing $IMG"; exit 1; }
# expected sha256: 1defd666fffe0ddb91cf1c3a96757947cfa2a30337f6095c0f1d4fc4b8781eea
shasum -a 256 "$IMG"
tools/.venv/bin/esptool --port "$PORT" --baud 921600 write-flash 0 "$IMG"
tools/.venv/bin/esptool --port "$PORT" --baud 921600 verify-flash 0 "$IMG"
