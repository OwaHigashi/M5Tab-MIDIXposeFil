"""One-shot Tab5 MIDI OUT continuity check.

Sends a short burst of MIDI to UM-ONE 1 (host -> device IN) while reading
UM-ONE 0 (device OUT -> host) in the background. Reports PASS / FAIL based
on whether the host actually received the device's outgoing bytes.

Usage from the M5Tab-MIDIXposeFil directory:
    python -X utf8 scripts/check_out_path.py
or with explicit ports / duration:
    python -X utf8 scripts/check_out_path.py --duration 5 --in-port "UM-ONE 0" --out-port "UM-ONE 1"
"""
from __future__ import annotations
import argparse
import random
import sys
import threading
import time

import mido


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--in-port",  default="UM-ONE 0", help="mido input port (device OUT capture)")
    p.add_argument("--out-port", default="UM-ONE 1", help="mido output port (host -> device IN)")
    p.add_argument("--duration", type=float, default=4.0,  help="seconds of burst")
    p.add_argument("--notes-per-sec", type=float, default=60.0)
    return p.parse_args()


def find_port(name_substr: str, names: list[str]) -> str | None:
    for n in names:
        if name_substr.lower() in n.lower():
            return n
    return None


def main() -> int:
    args = parse_args()
    in_name  = find_port(args.in_port,  mido.get_input_names())
    out_name = find_port(args.out_port, mido.get_output_names())
    if in_name is None:
        print(f"!! No input port matching {args.in_port!r}. Available: {mido.get_input_names()}")
        return 2
    if out_name is None:
        print(f"!! No output port matching {args.out_port!r}. Available: {mido.get_output_names()}")
        return 2
    print(f"[check] capture <- {in_name}")
    print(f"[check] send    -> {out_name}")
    print(f"[check] running {args.duration:.1f}s burst at ~{args.notes_per_sec:.0f} notes/s")

    # Capture thread
    cap_bytes = [0]
    cap_msgs  = [0]
    stop = [False]
    def reader():
        with mido.open_input(in_name) as inp:
            while not stop[0]:
                for msg in inp.iter_pending():
                    cap_msgs[0]  += 1
                    cap_bytes[0] += len(msg.bytes())
                time.sleep(0.005)
    t = threading.Thread(target=reader, daemon=True)
    t.start()

    # Stress burst
    rng = random.Random(42)
    note_period = 1.0 / max(args.notes_per_sec, 1.0)
    sent_msgs = 0
    sent_bytes = 0
    deadline = time.monotonic() + args.duration
    last_note = time.monotonic()
    held: list[tuple[int, int, float]] = []
    with mido.open_output(out_name) as out:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_note >= note_period:
                last_note = now
                ch = rng.randrange(4)
                note = rng.randint(48, 84)
                vel = rng.randint(60, 100)
                out.send(mido.Message("note_on", channel=ch, note=note, velocity=vel))
                sent_msgs += 1; sent_bytes += 3
                held.append((ch, note, now + 0.20))
            still: list[tuple[int, int, float]] = []
            for (ch, note, rel) in held:
                if now >= rel:
                    out.send(mido.Message("note_off", channel=ch, note=note, velocity=0))
                    sent_msgs += 1; sent_bytes += 3
                else:
                    still.append((ch, note, rel))
            held = still
            time.sleep(0.001)
        for (ch, note, _) in held:
            out.send(mido.Message("note_off", channel=ch, note=note, velocity=0))
            sent_msgs += 1; sent_bytes += 3
    # Allow tail to arrive
    time.sleep(1.0)
    stop[0] = True
    t.join(timeout=2.0)

    print()
    print(f"sent     {sent_msgs:6d} messages / {sent_bytes:6d} bytes  -> UM-ONE 1 (device IN)")
    print(f"captured {cap_msgs[0]:6d} messages / {cap_bytes[0]:6d} bytes  <- UM-ONE 0 (device OUT)")
    print()
    if cap_bytes[0] == 0:
        print("==== FAIL: no bytes received on UM-ONE 0 ====")
        print("MIDI OUT path is broken. Likely cause: dead Unit MIDI module")
        print("(TX driver / DIN OUT solder joint), or cable not plugged into")
        print("the correct UM-ONE jack (MIDI IN 5-pin DIN, usually labelled IN).")
        return 1
    ratio = cap_bytes[0] / max(sent_bytes, 1)
    if ratio < 0.5:
        print(f"==== WARN: only {ratio*100:.0f}% of sent bytes returned ====")
        print("Some bytes pass through but path may be flaky.")
        return 1
    print(f"==== PASS: {cap_bytes[0]} bytes captured ({ratio*100:.0f}% of sent) ====")
    print("Tab5 MIDI OUT -> Unit MIDI -> UM-ONE 0 path is healthy.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
