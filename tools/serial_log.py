#!/usr/bin/env python3
"""Headless serial logger for tests: prints every line with a timestamp, and sends any line written to a
command FIFO (default /tmp/mind-serial-cmd) to the board. Reconnects if the port goes away (reset/flash)."""
import argparse, os, sys, time, select
import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="/dev/cu.usbmodem1101")
ap.add_argument("--fifo", default="/tmp/mind-serial-cmd")
a = ap.parse_args()
if not os.path.exists(a.fifo):
    os.mkfifo(a.fifo)
fd = os.open(a.fifo, os.O_RDONLY | os.O_NONBLOCK)
buf = b""
while True:
    try:
        s = serial.Serial(a.port, 115200, timeout=0.05)
        print(f"{time.strftime('%H:%M:%S')} [log] port open", flush=True)
        while True:
            data = s.read(4096)
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    print(f"{time.strftime('%H:%M:%S')} {line.decode(errors='replace').rstrip()}", flush=True)
            r, _, _ = select.select([fd], [], [], 0)
            if r:
                cmd = os.read(fd, 1024)
                if cmd:
                    s.write(cmd if cmd.endswith(b"\n") else cmd + b"\n")
    except (serial.SerialException, OSError) as e:
        print(f"{time.strftime('%H:%M:%S')} [log] port error: {e}", flush=True)
        time.sleep(0.5)
