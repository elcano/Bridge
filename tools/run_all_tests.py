#!/usr/bin/env python3
"""
Batch runner — runs every CSV test directly in the tests/ folder against the
Router gateway (simulator_router_gateway) over one serial port, then prints a
combined pass/fail summary.

Only the CSVs that live *directly* in tests/ are run; subfolders
(MotorTurn, MoveMotor, WireTest_*) are skipped.

Usage:
    python run_all_tests.py <serial_port> [--verbose]

Examples:
    python run_all_tests.py COM24
    python run_all_tests.py COM24 --verbose
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
TESTS_DIR = (HERE / ".." / "tests").resolve()
RUNNER = HERE / "test_runner.py"


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    verbose = "--verbose" in sys.argv[1:] or "-v" in sys.argv[1:]
    if len(args) != 1:
        sys.stderr.write(f"Usage: {sys.argv[0]} <serial_port> [--verbose]\n")
        return 2
    port = args[0]

    csvs = sorted(p for p in TESTS_DIR.glob("*.csv") if p.is_file())
    if not csvs:
        sys.stderr.write(f"No CSV files found directly in {TESTS_DIR}\n")
        return 1

    print(f"Running {len(csvs)} test CSV(s) from {TESTS_DIR} on {port}\n")

    results: list[tuple[str, int]] = []
    for csv in csvs:
        print("#" * 70)
        print(f"# {csv.name}")
        print("#" * 70)
        cmd = [sys.executable, str(RUNNER), str(csv), port]
        if verbose:
            cmd.insert(2, "--verbose")
        proc = subprocess.run(cmd)
        results.append((csv.name, proc.returncode))
        # Let the Native USB port fully close/re-enumerate between tests.
        time.sleep(1.0)
        print()

    print("=" * 70)
    print("OVERALL SUMMARY")
    print("=" * 70)
    passed = failed = errored = 0
    for name, rc in results:
        if rc == 0:
            status = "PASS"
            passed += 1
        elif rc == 1:
            status = "FAIL"
            failed += 1
        else:
            status = f"ERR({rc})"
            errored += 1
        print(f"  [{status:>6}] {name}")
    print()
    print(f"Total: {len(results)}  |  passed: {passed}  failed: {failed}  errored: {errored}")
    return 0 if failed == 0 and errored == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
