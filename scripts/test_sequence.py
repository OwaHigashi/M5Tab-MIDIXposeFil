"""6-phase regression test for the M5 MIDI Transposer family.

Drives the device through six 60-second phases via USB-serial commands,
sending a deterministic MIDI stream on each, while:
  - capturing the device's MIDI OUT on a host input port
  - reading the firmware's [mem] heap/MIDI-counter monitor (M5TAB_DIAG)
  - querying STATUS to read midi_in / midi_out deltas

Phases (each 60 s, 6 min total):
  1. passthrough (F.bypass=1, M.bypass=0+empty)        OUT == IN expected
  2. transpose +5 (F.bypass=0+empty, M.bypass=1)       OUT count == IN count
  3. filter x1   (rule1 only — drops PitchBend)        OUT < IN
  4. filter x2   (rule1+rule2 — drops PB and CC)       OUT < IN x1
  5. mapper x1   (rule1 only — Ch1 NoteOn -> Ch2)      OUT count == IN count
  6. mapper x2   (rule1+rule2 — also halves Ch3 vel)   OUT count == IN count

Prerequisite: the device firmware must be built with -DM5TAB_DIAG so that
`[mem] ... midi_in=N midi_out=N ...` is printed every 5 s, and the firmware
must support the `LOAD TESTRULES` and `SET FILTER|MAPPER ENABLED` commands.

Run with the Roland UM-ONE rig (or equivalent) wired in IN/OUT loop:
  python -X utf8 scripts/test_sequence.py \
      --com COM4 --out-port "UM-ONE 1" --in-port "UM-ONE 0"
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import serial


# Match the firmware's [mem] line. midi_in / midi_out fields are required —
# the older format without them is rejected here so we fail loudly if the
# wrong firmware is on the device.
MEM_RE = re.compile(
    r"\[mem\] heap=(\d+) win_min=(\d+) all_min=(\d+) "
    r"psram=(\d+) psram_min=(\d+) stack_hw=(\d+) "
    r"midi_in=(\d+) midi_out=(\d+) uptime_ms=(\d+)"
)
STATUS_MIDI_RE = re.compile(r"midi_in=(\d+) midi_out=(\d+)")
CAP_DONE_RE = re.compile(
    r"\[done\] msgs=(\d+) bytes=(\d+) notes=(\d+) pb=(\d+) cc=(\d+) "
    r"clock=(\d+) active=(\d+) other=(\d+)"
)


class SerialMonitor:
    """Single owner of the COM port. Background reader thread continuously
    drains the serial stream, parsing [mem] samples into a list and capturing
    OK/ERR command acks for the synchronous command path."""

    def __init__(self, port: str, baud: int = 115200):
        self.s = serial.Serial(port, baud, timeout=0.2)
        self.buf = bytearray()
        self.mem_samples: list[dict] = []
        self.acks: list[str] = []
        self.lock = threading.Lock()
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
                self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        with self.lock:
            if line.startswith("[mem]"):
                m = MEM_RE.search(line)
                if m:
                    self.mem_samples.append({
                        "t": time.time(),
                        "heap": int(m.group(1)),
                        "win_min": int(m.group(2)),
                        "all_min": int(m.group(3)),
                        "psram": int(m.group(4)),
                        "psram_min": int(m.group(5)),
                        "stack_hw": int(m.group(6)),
                        "midi_in": int(m.group(7)),
                        "midi_out": int(m.group(8)),
                        "uptime_ms": int(m.group(9)),
                    })
                return
            if line.startswith("OK ") or line.startswith("ERR "):
                self.acks.append(line)

    def close(self) -> None:
        self._stop = True
        try:
            self.s.close()
        except Exception:
            pass

    def send_command(self, cmd: str, timeout: float = 4.0) -> str:
        """Send a command, wait for a single OK/ERR ack."""
        with self.lock:
            self.acks.clear()
        self.s.write((cmd + "\n").encode())
        self.s.flush()
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if self.acks:
                    return self.acks.pop(0)
            time.sleep(0.01)
        return f"ERR TIMEOUT {cmd}"

    def query_status(self, timeout: float = 6.0, retries: int = 2) -> tuple[int | None, int | None, str]:
        """Send STATUS, parse midi_in / midi_out from the OK STATUS reply.
        Retries on timeout — under heavy load right after a stress run, the
        firmware can take longer than usual to drain its serial work queue."""
        for attempt in range(retries + 1):
            with self.lock:
                self.acks.clear()
            self.s.write(b"STATUS\n")
            self.s.flush()
            deadline = time.time() + timeout
            while time.time() < deadline:
                with self.lock:
                    for line in list(self.acks):
                        if line.startswith("OK STATUS"):
                            m = STATUS_MIDI_RE.search(line)
                            if m:
                                return int(m.group(1)), int(m.group(2)), line
                time.sleep(0.01)
            # timeout — small backoff before retry
            if attempt < retries:
                time.sleep(0.5)
        return None, None, "TIMEOUT"

    def all_min_window(self, since_t: float) -> tuple[int | None, int | None, int]:
        with self.lock:
            window = [s for s in self.mem_samples if s["t"] >= since_t]
        if not window:
            return None, None, 0
        mins = [s["all_min"] for s in window]
        return min(mins), max(mins), len(window)

    def stack_hw_min(self, since_t: float) -> int | None:
        with self.lock:
            window = [s for s in self.mem_samples if s["t"] >= since_t]
        if not window:
            return None
        return min(s["stack_hw"] for s in window)


PHASES = [
    {
        "name": "1_passthrough_FbypassON",
        "config": [
            "SET TRANSPOSE 0",
            "SET FILTER BYPASS 1",
            "SET FILTER ENABLED 1 0",
            "SET FILTER ENABLED 2 0",
            "SET MAPPER BYPASS 0",
            "SET MAPPER ENABLED 1 0",
            "SET MAPPER ENABLED 2 0",
        ],
        "verify": "passthrough",
    },
    {
        "name": "2_transpose+5_MbypassON",
        "config": [
            "SET TRANSPOSE 5",
            "SET FILTER BYPASS 0",
            "SET FILTER ENABLED 1 0",
            "SET FILTER ENABLED 2 0",
            "SET MAPPER BYPASS 1",
            "SET MAPPER ENABLED 1 0",
            "SET MAPPER ENABLED 2 0",
        ],
        "verify": "passthrough",  # count match, content shifted (we don't decode)
    },
    {
        "name": "3_filter_x1_PB",
        "config": [
            "SET TRANSPOSE 0",
            "SET FILTER BYPASS 0",
            "SET FILTER ENABLED 1 1",
            "SET FILTER ENABLED 2 0",
            "SET MAPPER BYPASS 0",
            "SET MAPPER ENABLED 1 0",
            "SET MAPPER ENABLED 2 0",
        ],
        "verify": "filter_x1",
    },
    {
        "name": "4_filter_x2_PB+CC",
        "config": [
            "SET TRANSPOSE 0",
            "SET FILTER BYPASS 0",
            "SET FILTER ENABLED 1 1",
            "SET FILTER ENABLED 2 1",
            "SET MAPPER BYPASS 0",
            "SET MAPPER ENABLED 1 0",
            "SET MAPPER ENABLED 2 0",
        ],
        "verify": "filter_x2",
    },
    {
        "name": "5_mapper_x1_Ch1to2",
        "config": [
            "SET TRANSPOSE 0",
            "SET FILTER BYPASS 0",
            "SET FILTER ENABLED 1 0",
            "SET FILTER ENABLED 2 0",
            "SET MAPPER BYPASS 0",
            "SET MAPPER ENABLED 1 1",
            "SET MAPPER ENABLED 2 0",
        ],
        "verify": "mapper",
    },
    {
        "name": "6_mapper_x2_Ch1to2+Ch3vel",
        "config": [
            "SET TRANSPOSE 0",
            "SET FILTER BYPASS 0",
            "SET FILTER ENABLED 1 0",
            "SET FILTER ENABLED 2 0",
            "SET MAPPER BYPASS 0",
            "SET MAPPER ENABLED 1 1",
            "SET MAPPER ENABLED 2 1",
        ],
        "verify": "mapper",
    },
]


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser()
    p.add_argument("--com", required=True, help="device USB-CDC COM port")
    p.add_argument("--out-port", required=True,
                   help="mido OUTPUT port that drives the device's MIDI IN")
    p.add_argument("--in-port", required=True,
                   help="mido INPUT port that captures the device's MIDI OUT")
    p.add_argument("--phase-duration", type=int, default=60,
                   help="seconds per phase (default 60)")
    p.add_argument("--notes-per-sec", type=int, default=100)
    p.add_argument("--bpm", type=int, default=140)
    p.add_argument("--seed", type=int, default=42,
                   help="RNG seed for the stress generator (kept identical "
                        "across phases so each phase sees the same byte stream)")
    p.add_argument("--stress-script", default=str(here / "midi_stress.py"))
    p.add_argument("--capture-script", default=str(here / "midi_capture_in.py"))
    p.add_argument("--out-csv", default="test_sequence_report.csv")
    return p.parse_args()


def fmt_int(n) -> str:
    if n is None:
        return "?"
    return f"{n:,}"


def verdict(phase: dict, in_d: int, out_d: int, cap_bytes: int, cap_pb: int,
            cap_cc: int) -> str:
    """Apply phase-specific output check. Tolerances are loose because of
    Active Sensing, host-side capture jitter and loopback feedback."""
    kind = phase["verify"]
    if in_d == 0:
        return "FAIL no MIDI received"

    if kind == "passthrough":
        ratio = abs(out_d - in_d) / in_d
        return "OK" if ratio < 0.10 else f"FAIL out/in skew {ratio:.2%}"

    if kind == "filter_x1":
        # PitchBend should be dropped: capture should see ~0 PB events.
        if cap_pb > 50:
            return f"FAIL cap_pb={cap_pb} (expected ~0)"
        return "OK PB dropped"

    if kind == "filter_x2":
        if cap_pb > 50:
            return f"FAIL cap_pb={cap_pb} (expected ~0)"
        if cap_cc > 50:
            return f"FAIL cap_cc={cap_cc} (expected ~0)"
        return "OK PB+CC dropped"

    if kind == "mapper":
        # Mapper preserves message count (just rewrites channel / velocity).
        ratio = abs(out_d - in_d) / in_d
        return "OK count preserved" if ratio < 0.10 else f"FAIL out/in skew {ratio:.2%}"

    return "?"


def run_phase(mon: SerialMonitor, phase: dict, args, env) -> dict:
    print(f"\n=== {phase['name']} ===")

    # Apply config commands. Bail early if any returns ERR.
    for cmd in phase["config"]:
        ack = mon.send_command(cmd)
        ok = ack.startswith("OK")
        print(f"  {cmd:<40s} -> {ack}")
        if not ok:
            print(f"  !! aborting phase config at {cmd!r}")
            return {"phase": phase["name"], "config_error": ack}

    time.sleep(0.5)

    in0, out0, _ = mon.query_status()
    if in0 is None:
        print("  !! STATUS query failed at baseline")
        return {"phase": phase["name"], "status_error": "baseline"}
    print(f"  baseline midi_in={fmt_int(in0)} midi_out={fmt_int(out0)}")

    t0 = time.time()

    # Capture starts slightly before stress and runs slightly past it.
    cap_args = [sys.executable, "-X", "utf8", args.capture_script,
                "--port", args.in_port,
                "--duration", str(args.phase_duration + 2)]
    cap_proc = subprocess.Popen(
        cap_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=env, encoding="utf-8")
    time.sleep(0.5)

    stress_args = [sys.executable, "-X", "utf8", args.stress_script,
                   "--port", args.out_port,
                   "--duration", str(args.phase_duration),
                   "--notes-per-sec", str(args.notes_per_sec),
                   "--bpm", str(args.bpm),
                   "--seed", str(args.seed)]
    stress_proc = subprocess.Popen(
        stress_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, env=env, encoding="utf-8")

    stress_stdout, _ = stress_proc.communicate()
    cap_stdout, _ = cap_proc.communicate()

    time.sleep(0.5)

    in1, out1, status_line = mon.query_status()
    if in1 is None:
        print("  !! STATUS query failed at finish")
        return {"phase": phase["name"], "status_error": "finish"}

    in_d = in1 - in0
    out_d = out1 - out0

    cap_bytes = cap_pb = cap_cc = cap_notes = cap_clock = 0
    cm = CAP_DONE_RE.search(cap_stdout or "")
    if cm:
        cap_bytes = int(cm.group(2))
        cap_notes = int(cm.group(3))
        cap_pb = int(cm.group(4))
        cap_cc = int(cm.group(5))
        cap_clock = int(cm.group(6))

    all_min_low, all_min_high, n = mon.all_min_window(t0)
    all_min_drop = (all_min_high - all_min_low) if all_min_low is not None else None
    stack_hw = mon.stack_hw_min(t0)

    v = verdict(phase, in_d, out_d, cap_bytes, cap_pb, cap_cc)
    print(f"  in_d={fmt_int(in_d)} out_d={fmt_int(out_d)} "
          f"cap_bytes={fmt_int(cap_bytes)} cap_pb={cap_pb} cap_cc={cap_cc}")
    print(f"  all_min_low={fmt_int(all_min_low)} drop={fmt_int(all_min_drop)} "
          f"stack_hw_min={fmt_int(stack_hw)} mem_samples={n}")
    print(f"  verdict: {v}")

    return {
        "phase": phase["name"],
        "midi_in_delta": in_d,
        "midi_out_delta": out_d,
        "cap_bytes": cap_bytes,
        "cap_notes": cap_notes,
        "cap_pb": cap_pb,
        "cap_cc": cap_cc,
        "cap_clock": cap_clock,
        "all_min_low": all_min_low,
        "all_min_high": all_min_high,
        "all_min_drop": all_min_drop,
        "stack_hw_min": stack_hw,
        "mem_samples": n,
        "verdict": v,
    }


def main() -> int:
    args = parse_args()
    env = {**os.environ, "PYTHONIOENCODING": "utf-8"}

    print(f"[seq] opening serial {args.com}")
    mon = SerialMonitor(args.com)
    time.sleep(2.0)  # let any boot chatter settle

    # Initial setup: load the test rules and force DIRECT mode so PLAY-mode
    # log spam doesn't crowd the [mem] stream.
    for cmd in ("MODE DIRECT", "LOAD TESTRULES"):
        ack = mon.send_command(cmd, timeout=3.0)
        print(f"[seq] {cmd:<20s} -> {ack}")
        if not ack.startswith("OK"):
            print(f"[seq] aborting: {cmd} returned {ack}")
            mon.close()
            return 2

    rows: list[dict] = []
    for phase in PHASES:
        rows.append(run_phase(mon, phase, args, env))

    # Restore safe state: filter & mapper bypassed, transpose 0.
    print("\n[seq] restoring safe state")
    for cmd in ("SET FILTER BYPASS 1", "SET MAPPER BYPASS 1", "SET TRANSPOSE 0"):
        print(f"[seq]   {cmd:<24s} -> {mon.send_command(cmd)}")

    # Write CSV with whatever fields we accumulated.
    if rows:
        keys = sorted({k for r in rows for k in r.keys()})
        with open(args.out_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print(f"\n[seq] CSV: {args.out_csv}")

    print("\n=== SUMMARY ===")
    print(f"  {'phase':<32s} {'in':>10s} {'out':>10s} {'cap':>10s} "
          f"{'all_min_drop':>14s} {'stack_hw':>10s}  verdict")
    for r in rows:
        print(f"  {r.get('phase',''):<32s} "
              f"{fmt_int(r.get('midi_in_delta')):>10s} "
              f"{fmt_int(r.get('midi_out_delta')):>10s} "
              f"{fmt_int(r.get('cap_bytes')):>10s} "
              f"{fmt_int(r.get('all_min_drop')):>14s} "
              f"{fmt_int(r.get('stack_hw_min')):>10s}  "
              f"{r.get('verdict','?')}")

    mon.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
