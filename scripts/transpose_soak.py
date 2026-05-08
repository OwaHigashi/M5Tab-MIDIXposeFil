"""Two-hour transpose-mode soak test with periodic transpose changes.

This orchestrator owns the Tab5 USB-CDC port and drives the existing
`midi_stress.py` generator on the UM-ONE output port while periodically
injecting `SET TRANSPOSE` commands on COM7.

The goal is to exercise the transpose path under sustained polyphonic load,
including high-resolution velocity prefix traffic and sustain-pedal pulses,
and to keep serial logs that make reboot / panic / watchdog causes visible.

Usage:
    python -X utf8 scripts/transpose_soak.py --com COM7 --out-port "UM-ONE 1" --duration 7200
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import serial
import mido


CRASH_HINTS = (
    "guru meditation",
    "panic",
    "watchdog",
    "wdt",
    "abort()",
    "brownout",
    "brown out",
    "brown-out",
    "brownout detector was triggered",
    "rst:0x",
    "rst cause",
    "stack canary",
    "loadprohibited",
    "storeprohibited",
    "instrfetchprohibited",
    "illegalinstruction",
    "backtrace:",
)

STATUS_RE = re.compile(r"OK STATUS .*transpose=(-?\d+).*midi_in=(\d+) midi_out=(\d+)")


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser()
    p.add_argument("--com", default="COM7", help="USB-CDC port for the Tab5")
    p.add_argument("--out-port", default="UM-ONE 1",
                   help="MIDI output port that feeds the device input")
    p.add_argument("--duration", type=int, default=7200,
                   help="run length in seconds")
    p.add_argument("--notes-per-sec", type=float, default=90.0)
    p.add_argument("--bpm", type=float, default=160.0)
    p.add_argument("--chord-burst-interval", type=float, default=0.75)
    p.add_argument("--chord-voices", type=int, default=8)
    p.add_argument("--chord-hold-min", type=float, default=0.9)
    p.add_argument("--chord-hold-max", type=float, default=1.8)
    p.add_argument("--hr-velocity", action="store_true", default=True,
                   help="emit CC 88 High Resolution Velocity Prefix before notes")
    p.add_argument("--mute-note-rate", type=float, default=0.10,
                   help="fraction of note releases sent as note_on velocity 0")
    p.add_argument("--pedal-period", type=float, default=6.0,
                   help="seconds between sustain pedal presses")
    p.add_argument("--pedal-hold", type=float, default=0.8,
                   help="seconds to hold sustain pedal once pressed")
    p.add_argument("--pedal-channel", type=int, default=0,
                   help="channel used for sustain pedal CC messages")
    p.add_argument("--transpose-interval", type=int, default=120,
                   help="seconds between transpose changes")
    p.add_argument("--transpose-seq", default="0,5,-5,12,-12,7,-7,3,-3",
                   help="comma-separated transpose values to cycle through")
    p.add_argument("--stress-script", default=str(here / "midi_stress.py"))
    p.add_argument("--capture-script", default=str(here / "midi_capture_in.py"))
    p.add_argument("--stress-log", default="logs/transpose_soak_stress.log")
    p.add_argument("--capture-log", default="logs/transpose_soak_capture.log")
    p.add_argument("--watch-log", default="logs/transpose_soak_watch.log")
    return p.parse_args()


class SerialMonitor:
    def __init__(self, port: str, baud: int = 115200):
        self.s = serial.Serial(port, baud, timeout=0.2)
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.lines: list[str] = []
        self._stop = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self) -> None:
        while not self._stop:
            try:
                chunk = self.s.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            self.buf.extend(chunk)
            while b"\n" in self.buf:
                idx = self.buf.index(b"\n")
                line = bytes(self.buf[:idx]).decode("utf-8", errors="replace").rstrip("\r")
                del self.buf[: idx + 1]
                stamp = time.strftime("%H:%M:%S")
                low = line.lower()
                tag = " <<<CRASH" if any(h in low for h in CRASH_HINTS) else ""
                with self.lock:
                    self.lines.append(line)
                print(f"{stamp}  {line}{tag}", flush=True)

    def send(self, cmd: str, timeout: float = 4.0) -> str:
        with self.lock:
            self.lines.clear()
        self.s.write((cmd + "\n").encode("utf-8"))
        self.s.flush()
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                for line in self.lines:
                    if line.startswith("OK ") or line.startswith("ERR "):
                        return line
            time.sleep(0.02)
        return f"ERR TIMEOUT {cmd}"

    def query_status(self, timeout: float = 5.0) -> str:
        with self.lock:
            self.lines.clear()
        self.s.write(b"STATUS\n")
        self.s.flush()
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                for line in self.lines:
                    if line.startswith("OK STATUS"):
                        return line
            time.sleep(0.02)
        return "ERR TIMEOUT STATUS"

    def close(self) -> None:
        self._stop = True
        try:
            self.s.close()
        except Exception:
            pass


def build_env() -> dict[str, str]:
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    return env


def start_process(args: list[str], log_path: str, env: dict[str, str]) -> subprocess.Popen:
    log_file = open(log_path, "a", encoding="utf-8", buffering=1)
    return subprocess.Popen(
        args,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )


def parse_transpose_seq(text: str) -> list[int]:
    values: list[int] = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(int(item))
    if not values:
        raise ValueError("transpose sequence must not be empty")
    return values


def main() -> int:
    args = parse_args()
    env = build_env()
    Path(args.stress_log).parent.mkdir(parents=True, exist_ok=True)
    Path(args.capture_log).parent.mkdir(parents=True, exist_ok=True)
    Path(args.watch_log).parent.mkdir(parents=True, exist_ok=True)

    transposes = parse_transpose_seq(args.transpose_seq)
    monitor = SerialMonitor(args.com)
    print(f"[soak] serial {args.com} opened")

    try:
        print(f"[soak] {monitor.send('MODE DIRECT')}")
        print(f"[soak] {monitor.send('SET TRANSPOSE 0')}")
        print(f"[soak] {monitor.query_status()}")

        capture_proc = start_process(
            [sys.executable, "-X", "utf8", args.capture_script,
             "--port", "UM-ONE 0", "--duration", str(args.duration + 10)],
            args.capture_log, env)
        stress_args = [
            sys.executable, "-X", "utf8", args.stress_script,
            "--port", args.out_port,
            "--duration", str(args.duration),
            "--notes-per-sec", str(args.notes_per_sec),
            "--bpm", str(args.bpm),
            "--chord-burst-interval", str(args.chord_burst_interval),
            "--chord-voices", str(args.chord_voices),
            "--chord-hold-min", str(args.chord_hold_min),
            "--chord-hold-max", str(args.chord_hold_max),
            "--mute-note-rate", str(args.mute_note_rate),
            "--pedal-period", str(args.pedal_period),
            "--pedal-hold", str(args.pedal_hold),
            "--pedal-channel", str(args.pedal_channel),
        ]
        if args.hr_velocity:
            stress_args.append("--hr-velocity")
        stress_proc = start_process(
            stress_args,
            args.stress_log, env)

        start = time.monotonic()
        next_change = start + args.transpose_interval
        idx = 0
        while True:
            now = time.monotonic()
            if now >= start + args.duration:
                break

            if now >= next_change:
                idx = (idx + 1) % len(transposes)
                value = transposes[idx]
                print(f"[soak] transpose -> {value}")
                print(f"[soak] {monitor.send(f'SET TRANSPOSE {value}')}")
                print(f"[soak] {monitor.query_status()}")
                next_change += args.transpose_interval

            if stress_proc.poll() is not None:
                print(f"[soak] stress exited code={stress_proc.returncode}")
                break

            time.sleep(1.0)

        print("[soak] restoring transpose=0")
        print(f"[soak] {monitor.send('SET TRANSPOSE 0')}")
        print(f"[soak] {monitor.query_status()}")

        try:
            stress_proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            stress_proc.terminate()

        try:
            capture_proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            capture_proc.terminate()

        print(f"[soak] stress return={stress_proc.returncode}")
        print(f"[soak] capture return={capture_proc.returncode}")
        return 0
    finally:
        monitor.close()


if __name__ == "__main__":
    raise SystemExit(main())
