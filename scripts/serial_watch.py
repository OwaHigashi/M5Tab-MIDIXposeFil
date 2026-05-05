"""Stream a serial port to stdout with timestamps and crash highlights.

Reads lines from the M5Tab USB-CDC port (default COM7) and prints them
with millisecond timestamps. Reset / panic / watchdog signatures are
flagged so the human running the test can spot them at a glance.

Auto-reopens the port if the device disappears (which is what a USB-CDC
reset looks like from the host side) so a crash + reboot does not end
the watch — instead, you see the gap and the fresh boot banner.
"""

from __future__ import annotations
import argparse
import sys
import time

import serial


CRASH_HINTS = (
    "guru meditation",
    "panic",
    "watchdog",
    "wdt",
    "abort()",
    "rst:0x",      # esp boot prefix
    "rst cause",
    "stack canary",
    "loadprohibited",
    "storeprohibited",
    "instrfetchprohibited",
    "illegalinstruction",
    "cpu halt",
    "backtrace:",
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM7")
    p.add_argument("--baud", type=int, default=115200)
    return p.parse_args()


def open_with_retry(port: str, baud: int) -> serial.Serial:
    while True:
        try:
            s = serial.Serial(port, baud, timeout=0.2)
            return s
        except serial.SerialException as e:
            sys.stdout.write(f"[watch] open {port} failed: {e} — retry in 1s\n")
            sys.stdout.flush()
            time.sleep(1.0)


def main() -> int:
    args = parse_args()
    s = open_with_retry(args.port, args.baud)
    sys.stdout.write(f"[watch] streaming {args.port} @ {args.baud}\n")
    sys.stdout.flush()
    buf = b""
    while True:
        try:
            chunk = s.read(4096)
        except serial.SerialException as e:
            sys.stdout.write(f"[watch] !!! port lost: {e} — reopening\n")
            sys.stdout.flush()
            try:
                s.close()
            except Exception:
                pass
            s = open_with_retry(args.port, args.baud)
            sys.stdout.write("[watch] !!! port back\n")
            sys.stdout.flush()
            continue
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, _, buf = buf.partition(b"\n")
            text = line.decode("utf-8", errors="replace").rstrip("\r")
            stamp = time.strftime("%H:%M:%S") + f".{int(time.time()*1000)%1000:03d}"
            tag = ""
            low = text.lower()
            for hint in CRASH_HINTS:
                if hint in low:
                    tag = " <<<CRASH"
                    break
            if "[boot]" in low:
                tag = " <<<BOOT"
            sys.stdout.write(f"{stamp}  {text}{tag}\n")
            sys.stdout.flush()


if __name__ == "__main__":
    sys.exit(main())
