"""Stress-tester for the M5Tab MIDI transposer.

Sends a sustained, moderately dense stream of MIDI events to a hardware
output port (Roland UM-ONE on this rig) so the device under test can be
exercised under realistic playing load. Mixes:

  - Note-on / note-off pairs across multiple channels with varied velocities
  - Running-status chord stacks
  - Pitch bend sweeps
  - Control change (modulation, expression, sustain on/off)
  - 24 ppq MIDI Clock at the configured BPM

The mix is deliberately heavier than a single keyboard player to make any
backpressure / heap / watchdog problem reproduce within minutes rather than
hours. Adjust `notes_per_sec` and `bpm` to taste.
"""

from __future__ import annotations
import argparse
import random
import sys
import time

import mido


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="UM-ONE 1",
                   help="MIDI output port name (substring match)")
    p.add_argument("--bpm", type=float, default=140.0,
                   help="MIDI clock BPM (24 ppq pulses sent at this rate)")
    p.add_argument("--notes-per-sec", type=float, default=60.0,
                   help="Average notes (on/off pair) per second")
    p.add_argument("--duration", type=float, default=0.0,
                   help="Run for this many seconds (0 = run until Ctrl-C)")
    p.add_argument("--no-clock", action="store_true",
                   help="Suppress 0xF8 clock pulses")
    p.add_argument("--seed", type=int, default=42,
                   help="RNG seed for repeatability")
    return p.parse_args()


def find_port(name_substr: str) -> str:
    available = mido.get_output_names()
    for n in available:
        if name_substr.lower() in n.lower():
            return n
    raise SystemExit(
        f"No output port matching {name_substr!r}. Available: {available!r}")


def main() -> int:
    args = parse_args()
    port_name = find_port(args.port)
    print(f"Sending to: {port_name}", flush=True)
    rng = random.Random(args.seed)

    clock_period = 60.0 / args.bpm / 24.0 if not args.no_clock else 0.0
    note_period = 1.0 / max(args.notes_per_sec, 1.0)

    stats = {"notes": 0, "cc": 0, "pb": 0, "clock": 0}
    deadline = time.monotonic() + args.duration if args.duration > 0 else None

    last_clock = time.monotonic()
    last_note = time.monotonic()
    last_report = time.monotonic()
    last_chord_burst = time.monotonic()
    last_pb_step = time.monotonic()
    pb_value = 0
    pb_dir = 1
    held_notes: list[tuple[int, int, float]] = []  # (ch, note, release_at)

    with mido.open_output(port_name) as out:
        # Sane starting state on each channel.
        for ch in range(4):
            out.send(mido.Message("control_change", channel=ch, control=121, value=0))
            out.send(mido.Message("program_change", channel=ch, program=0))

        try:
            while True:
                now = time.monotonic()
                if deadline is not None and now >= deadline:
                    break

                # 1) MIDI clock pulses.
                if clock_period > 0.0:
                    while now - last_clock >= clock_period:
                        out.send(mido.Message("clock"))
                        last_clock += clock_period
                        stats["clock"] += 1

                # 2) Note on / note off scheduler.
                if now - last_note >= note_period:
                    last_note = now
                    ch = rng.randrange(4)
                    note = rng.randint(36, 84)
                    vel = rng.randint(50, 110)
                    out.send(mido.Message("note_on",
                                          channel=ch, note=note, velocity=vel))
                    stats["notes"] += 1
                    duration = rng.uniform(0.10, 0.40)
                    held_notes.append((ch, note, now + duration))

                # 3) Release notes whose hold-time has expired.
                still: list[tuple[int, int, float]] = []
                for (ch, note, release_at) in held_notes:
                    if now >= release_at:
                        out.send(mido.Message("note_off",
                                              channel=ch, note=note, velocity=0))
                    else:
                        still.append((ch, note, release_at))
                held_notes = still

                # 4) Periodic chord-stack burst — five rapid note-ons within
                # ~5 ms, each individually released later. This is the worst
                # case for the TX FIFO, and the whole point of the test.
                if now - last_chord_burst > 1.5:
                    last_chord_burst = now
                    base = rng.randint(40, 70)
                    ch = rng.randrange(4)
                    burst_vel = rng.randint(80, 110)
                    for offset in (0, 4, 7, 11, 14):
                        n = base + offset
                        out.send(mido.Message("note_on",
                                              channel=ch, note=n, velocity=burst_vel))
                        held_notes.append((ch, n, now + 0.6))
                        stats["notes"] += 1

                # 5) Pitch bend sweep.
                if now - last_pb_step >= 0.020:
                    last_pb_step = now
                    pb_value += pb_dir * 512
                    if pb_value >= 8000:
                        pb_value = 8000; pb_dir = -1
                    elif pb_value <= -8000:
                        pb_value = -8000; pb_dir = 1
                    out.send(mido.Message("pitchwheel", channel=0, pitch=pb_value))
                    stats["pb"] += 1

                # 6) Modulation / expression / sustain CCs.
                if rng.random() < 0.02:
                    cc = rng.choice([1, 11, 64, 7])
                    val = rng.randint(0, 127)
                    out.send(mido.Message("control_change",
                                          channel=rng.randrange(4),
                                          control=cc, value=val))
                    stats["cc"] += 1

                # 7) Status report once per second.
                if now - last_report >= 1.0:
                    last_report = now
                    print(f"[gen] notes={stats['notes']} cc={stats['cc']} "
                          f"pb={stats['pb']} clock={stats['clock']} "
                          f"held={len(held_notes)}",
                          flush=True)

                # Tight enough to handle 24 ppq clock at 200 BPM cleanly
                # without burning a core.
                time.sleep(0.001)
        finally:
            # Flush note offs so the synth on the receiving end does not
            # ring out on shutdown.
            for (ch, note, _) in held_notes:
                out.send(mido.Message("note_off", channel=ch, note=note, velocity=0))
            for ch in range(4):
                out.send(mido.Message("control_change", channel=ch,
                                       control=123, value=0))  # all notes off
                out.send(mido.Message("control_change", channel=ch,
                                       control=120, value=0))  # all sound off
            print(f"[done] {stats}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
