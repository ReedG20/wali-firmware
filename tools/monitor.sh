#!/usr/bin/env bash
# Interactive serial console (115200). Type commands like: status, levels, mode all, mute.  Ctrl-] to quit.
cd "$(dirname "$0")"
PORT="${1:-/dev/cu.usbmodem1101}"
exec .venv/bin/python -m serial.tools.miniterm --eol LF --raw "$PORT" 115200
