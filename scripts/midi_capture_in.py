"""Capture MIDI bytes from a host input port (e.g. UM-ONE 0) and report
total bytes seen. Used to verify that a device-under-test's MIDI OUT is
actually emitting bytes by counting them on the loop back to the host.
"""
from __future__ import annotations
import argparse
import sys
import time

import mido


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True, help="mido input port name (e.g. 'UM-ONE 0')")
    p.add_argument("--duration", type=float, required=True)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    print(f"[cap] opening {args.port!r}", flush=True)
    inp = mido.open_input(args.port)
    bytes_total = 0
    msgs_total = 0
    notes = pb = cc = clock = active = other = 0
    deadline = time.time() + args.duration
    last_report = time.time()
    while time.time() < deadline:
        for msg in inp.iter_pending():
            msgs_total += 1
            bytes_total += len(msg.bytes())
            t = msg.type
            if t in ("note_on", "note_off"): notes += 1
            elif t == "pitchwheel":          pb += 1
            elif t == "control_change":      cc += 1
            elif t == "clock":               clock += 1
            elif t == "active_sensing":      active += 1
            else:                            other += 1
        now = time.time()
        if now - last_report >= 5:
            print(f"[cap] msgs={msgs_total} bytes={bytes_total} notes={notes} pb={pb} cc={cc} clock={clock} active={active} other={other}", flush=True)
            last_report = now
        time.sleep(0.005)
    print(f"[done] msgs={msgs_total} bytes={bytes_total} notes={notes} pb={pb} cc={cc} clock={clock} active={active} other={other}", flush=True)
    inp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
