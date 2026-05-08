"""Quick chord-drop diagnostic.

Snapshots the M5Tab's STATUS counters, prompts the user to play a chord,
then snapshots again and prints the delta. The two key fields are
``midi_in_real`` / ``midi_out_real`` (heartbeat-filtered byte counters).

If midi_in_real grows by N but midi_out_real grows by less (or 0),
something between the parser and Serial2.write is suppressing notes —
look at FILTER / MAPPER / Transpose range checks.

If midi_in_real does NOT grow as expected, bytes never reached the parser
— Serial2 RX overflow or USB-MIDI ring overflow is likely.

Usage:
    python scripts/chord_diag.py [--port COM7] [--baud 115200]
"""

from __future__ import annotations
import argparse
import re
import sys
import time

import serial


COUNT_RE = re.compile(
    r"midi_in=(?P<in_>\d+)\s+midi_in_real=(?P<in_r>\d+)\s+"
    r"midi_out=(?P<out_>\d+)\s+midi_out_real=(?P<out_r>\d+)"
)


def read_status(port: serial.Serial, timeout_s: float = 2.0) -> dict[str, int]:
    """Send STATUS, read until we see the response line, parse counters."""
    port.reset_input_buffer()
    port.write(b"STATUS\n")
    port.flush()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = port.readline().decode(errors="replace").strip()
        if not line:
            continue
        if "midi_in=" in line and "midi_out=" in line:
            pairs = dict(re.findall(r"(\w+)=(\d+)", line))
            try:
                return {
                    "midi_in":       int(pairs["midi_in"]),
                    "midi_in_real":  int(pairs.get("midi_in_real", pairs["midi_in"])),
                    "midi_out":      int(pairs["midi_out"]),
                    "midi_out_real": int(pairs.get("midi_out_real", pairs["midi_out"])),
                    "usb_drop":      int(pairs.get("usb_drop", 0)),
                }
            except KeyError:
                raise RuntimeError(f"unparseable STATUS line: {line!r}")
    raise TimeoutError("STATUS response not received")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--baud", default=115200, type=int)
    ap.add_argument("--wait", default=0.0, type=float,
                    help="seconds to sleep instead of waiting for Enter (0 = interactive)")
    args = ap.parse_args()

    print(f"[diag] opening {args.port} @ {args.baud}")
    with serial.Serial(args.port, args.baud, timeout=0.5) as port:
        time.sleep(0.5)
        port.reset_input_buffer()
        before = read_status(port)
        print(f"[diag] BEFORE: in={before['midi_in']:>6}  in_real={before['midi_in_real']:>6}  "
              f"out={before['midi_out']:>6}  out_real={before['midi_out_real']:>6}  "
              f"usb_drop={before['usb_drop']:>6}")
        print()
        if args.wait > 0:
            print(f"[diag] play your chord now — sampling again in {args.wait}s")
            time.sleep(args.wait)
        else:
            print("Now play one chord (e.g., C-E-G-B four notes simultaneously)")
            print("then RELEASE all keys. Press Enter when done.")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                return 1
            time.sleep(0.5)
        after = read_status(port)
        print(f"[diag] AFTER : in={after['midi_in']:>6}  in_real={after['midi_in_real']:>6}  "
              f"out={after['midi_out']:>6}  out_real={after['midi_out_real']:>6}  "
              f"usb_drop={after['usb_drop']:>6}")

    d_in       = after["midi_in"]       - before["midi_in"]
    d_in_real  = after["midi_in_real"]  - before["midi_in_real"]
    d_out      = after["midi_out"]      - before["midi_out"]
    d_out_real = after["midi_out_real"] - before["midi_out_real"]
    d_usb_drop = after["usb_drop"]      - before["usb_drop"]

    print()
    print(f"[delta] in_real    = +{d_in_real}  (expected ~24 for a 4-note chord on+off,")
    print(f"                       18 if running status is used)")
    print(f"[delta] out_real   = +{d_out_real}  (should match in_real if FILTER/MAPPER pass-through)")
    print(f"[delta] in (raw)   = +{d_in}")
    print(f"[delta] out (raw)  = +{d_out}")
    print(f"[delta] usb_drop   = +{d_usb_drop}  (USB MIDI ring overflow — bytes silently dropped)")

    if d_in_real == 0:
        print("\nVERDICT: nothing reached the MIDI parser. Check cabling / input source / RX wire.")
    elif d_out_real == 0 and d_in_real > 0:
        print("\nVERDICT: parser saw bytes but NONE were emitted — FILTER, MAPPER, or transpose")
        print("         range check is dropping every event. Check FILTER bypass and transpose.")
    elif d_out_real < d_in_real:
        print(f"\nVERDICT: {d_in_real - d_out_real} byte(s) lost between parser and Serial2 TX.")
        print("         Possible causes: FILTER rule, transpose pushing notes out of range,")
        print("         or sendMIDIMessage's `if (transposed >= 0 && transposed <= 127)` skipping.")
    else:
        print("\nVERDICT: in == out byte-for-byte. If notes still stick on the synth, the bug")
        print("         is downstream (synth side / cabling) rather than in MIDI processing.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
