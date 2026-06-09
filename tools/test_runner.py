#!/usr/bin/env python3
"""
Elcano test runner — PC-driven CSV test for the closed-loop simulator.

Reads a test CSV (Stage 2 command rows + `# assert` lines), pushes
commands to Sensor_Hub over USB serial at the scheduled times, captures
the LOG telemetry Sensor_Hub prints back, evaluates the assertions, and
prints a pass/fail summary.

Usage:
    python test_runner.py <test.csv> <serial_port>

Examples:
    python test_runner.py ../tests/test_steering_response.csv COM22
    python test_runner.py ../tests/test_steering_response.csv /dev/ttyACM0

CSV format (extends the existing Stage 2 CSVs — backwards compatible):
    Command row:    time_ms, CANID, nbytes, speed_cmPs, brake, mode, angle_tenths
    Assertion line: # assert t=<ms>: <python expression using LOG field names>
    Comment:        # any other line starting with #

Sensor_Hub serial protocol (matching the NavigateTestRunner sketch in
NavigateTestRunner/NavigateTestRunner.ino):
    Inbound  (PC -> SH):  CMD,<id>,<a>,<b>,<c>,<d>\\n
    Outbound (SH -> PC):  LOG,<t_ms>,<key1>=<v1>,<key2>=<v2>,...\\n
                          plus ACK,<t_ms>\\n per accepted CMD

Dependencies:
    pip install pyserial
"""

from __future__ import annotations

import re
import sys
import time
import threading
from dataclasses import dataclass

try:
    import serial
except ImportError:
    sys.stderr.write("Missing pyserial — install with: pip install pyserial\n")
    sys.exit(2)


# -- Patterns ----------------------------------------------------------------

CMD_PATTERN = re.compile(
    r"^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,"
    r"\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*$"
)
ASSERT_PATTERN = re.compile(r"^\s*#\s*assert\s+t=(\d+)\s*:\s*(.+?)\s*$")
LOG_PATTERN = re.compile(r"^LOG,(\d+),(.+)$")


# -- Data --------------------------------------------------------------------

@dataclass
class Command:
    time_ms: int
    can_id: int
    speed: int
    brake: int
    mode: int
    angle: int


@dataclass
class Assertion:
    time_ms: int
    expression: str


# -- CSV parsing -------------------------------------------------------------

def parse_csv(path: str) -> tuple[list[Command], list[Assertion]]:
    commands: list[Command] = []
    asserts: list[Assertion] = []
    with open(path) as f:
        for raw in f:
            line = raw.rstrip("\r\n")
            stripped = line.strip()
            if not stripped:
                continue
            m = ASSERT_PATTERN.match(line)
            if m:
                asserts.append(Assertion(int(m.group(1)), m.group(2)))
                continue
            if stripped.startswith("#"):
                continue
            m = CMD_PATTERN.match(line)
            if m:
                commands.append(
                    Command(
                        time_ms=int(m.group(1)),
                        can_id=int(m.group(2)),
                        speed=int(m.group(4)),
                        brake=int(m.group(5)),
                        mode=int(m.group(6)),
                        angle=int(m.group(7)),
                    )
                )
            else:
                sys.stderr.write(f"  [warn] unparseable line: {line!r}\n")
    commands.sort(key=lambda c: c.time_ms)
    asserts.sort(key=lambda a: a.time_ms)
    return commands, asserts


# -- Serial reader thread ----------------------------------------------------

class LogReader(threading.Thread):
    """Background thread: drain serial, capture LOG lines, echo others."""

    def __init__(self, ser: "serial.Serial", t0: float, verbose: bool = False):
        super().__init__(daemon=True)
        self.ser = ser
        self.t0 = t0
        self.verbose = verbose
        self.log: list[tuple[int, dict[str, int]]] = []
        self._stop_flag = threading.Event()

    def stop(self) -> None:
        self._stop_flag.set()

    def run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").rstrip("\r\n")
            if not line:
                continue
            elapsed_ms = int((time.monotonic() - self.t0) * 1000)
            m = LOG_PATTERN.match(line)
            if m:
                # Index LOG entries by PC-side elapsed time, not the device's
                # millis() — the device has been running long before the test
                # started, so its millis values won't match our assertion times.
                try:
                    fields: dict[str, int] = {}
                    for kv in m.group(2).split(","):
                        if "=" in kv:
                            k, v = kv.split("=", 1)
                            fields[k.strip()] = int(v.strip())
                    self.log.append((elapsed_ms, fields))
                    if self.verbose:
                        print(f"  [{elapsed_ms:6d} ms] {line}")
                except Exception:
                    pass
            else:
                # Echo human-readable lines as they happen
                print(f"  [{elapsed_ms:6d} ms] {line}")


# -- Assertion evaluation ----------------------------------------------------

def evaluate(assertion: Assertion, log: list[tuple[int, dict[str, int]]]) -> tuple[bool, str]:
    if not log:
        return False, "no LOG lines captured"
    nearest = min(log, key=lambda e: abs(e[0] - assertion.time_ms))
    drift_ms = nearest[0] - assertion.time_ms
    if abs(drift_ms) > 500:
        return False, f"no LOG within 500 ms of t={assertion.time_ms} (nearest: t={nearest[0]})"
    fields = nearest[1]
    # Substitute field names with values in the expression.
    expr = assertion.expression
    for k in sorted(fields.keys(), key=len, reverse=True):
        expr = re.sub(rf"\b{re.escape(k)}\b", str(fields[k]), expr)
    try:
        result = eval(expr, {"__builtins__": {}}, {})  # noqa: S307 — restricted builtins
    except Exception as e:
        return False, f"eval error: {e!r}  (expanded expr: {expr!r})"
    return bool(result), f"@ t={nearest[0]} (Δ {drift_ms:+d} ms) → {expr}"


# -- Main --------------------------------------------------------------------

def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    verbose = any(a in ("--verbose", "-v") for a in sys.argv[1:])
    if len(args) != 2:
        sys.stderr.write(f"Usage: {sys.argv[0]} [--verbose] <test.csv> <serial_port>\n")
        return 2
    csv_path, port = args[0], args[1]

    commands, asserts = parse_csv(csv_path)
    print(f"Loaded {len(commands)} commands and {len(asserts)} assertions from {csv_path}")

    ser = serial.Serial(port, 115200, timeout=0.1)
    # Let the Due settle (Native USB CDC takes a moment to come up after open).
    time.sleep(0.5)

    t0 = time.monotonic()
    reader = LogReader(ser, t0, verbose=verbose)
    reader.start()

    # Schedule and send commands.
    for cmd in commands:
        target = t0 + cmd.time_ms / 1000.0
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        line = f"CMD,{cmd.can_id},{cmd.speed},{cmd.brake},{cmd.mode},{cmd.angle}\n"
        ser.write(line.encode("ascii"))
        ser.flush()
        elapsed_ms = int((time.monotonic() - t0) * 1000)
        print(f"  [{elapsed_ms:6d} ms] -> {line.strip()}")

    # Wait until the last assertion's time + slack so all LOG lines arrive.
    if asserts:
        slack_s = max(a.time_ms for a in asserts) / 1000.0 + 1.0
        target = t0 + slack_s
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    reader.stop()
    reader.join(timeout=1)
    ser.close()

    # Evaluate.
    print()
    print("=" * 60)
    print(f"Assertions ({len(asserts)}):")
    passed = failed = 0
    for a in asserts:
        ok, detail = evaluate(a, reader.log)
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] t={a.time_ms:>5d} ms: {a.expression}")
        print(f"           {detail}")
        if ok:
            passed += 1
        else:
            failed += 1
    print()
    print(f"Summary: {passed} passed, {failed} failed, {len(reader.log)} LOG samples captured")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
