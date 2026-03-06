#!/usr/bin/env python3
"""
tests/test_serial.py
────────────────────
Interactive bench test for all SCARA firmware serial commands.
Run with the Arduino connected via USB before the robot body is assembled.

Usage:
    python tests/test_serial.py --port COM3          (Windows)
    python tests/test_serial.py --port /dev/ttyUSB0  (Linux)

Each test sends a command and checks the response against the expected
OK:<CMD> string defined in the protocol.  Results are printed as
PASS / FAIL with a summary at the end.
"""

import argparse
import sys
import time

# Allow running from project root or from tests/ directory
sys.path.insert(0, "pc_software")
from serial_comms import connect, send_command, disconnect

# ─── COLOUR HELPERS ──────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"

def passed(msg): print(f"  {GREEN}PASS{RESET}  {msg}")
def failed(msg): print(f"  {RED}FAIL{RESET}  {msg}")
def info(msg):   print(f"  {YELLOW}INFO{RESET}  {msg}")

# ─── TEST RUNNER ─────────────────────────────────────────────────────────────
results = []

def run_test(name, cmd, expected, ser, timeout=10):
    """Send cmd, check response starts with expected. Log result."""
    print(f"\n{'─'*55}")
    print(f"TEST: {name}")
    print(f"      cmd={cmd!r}   expect={expected!r}")
    try:
        resp = send_command(ser, cmd, timeout=timeout)
        ok = resp.startswith(expected)
        if ok:
            passed(f"Got: {resp!r}")
        else:
            failed(f"Got: {resp!r}  (expected starts with {expected!r})")
        results.append((name, ok, resp))
        return ok
    except RuntimeError as e:
        failed(str(e))
        results.append((name, False, "TIMEOUT"))
        return False


def run_nack_test(name, cmd, ser):
    """Expect a NACK response."""
    print(f"\n{'─'*55}")
    print(f"TEST: {name}  [expect NACK]")
    print(f"      cmd={cmd!r}")
    try:
        resp = send_command(ser, cmd, timeout=5)
        ok = resp.startswith("NACK:")
        if ok:
            passed(f"Got NACK as expected: {resp!r}")
        else:
            failed(f"Got: {resp!r}  (expected NACK:...)")
        results.append((name, ok, resp))
        return ok
    except RuntimeError as e:
        failed(str(e))
        results.append((name, False, "TIMEOUT"))
        return False


# ─── MAIN ────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="SCARA serial command tests")
    parser.add_argument("--port", required=True,
                        help="Serial port e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--skip-home", action="store_true",
                        help="Skip HOME test (firmware already homed)")
    args = parser.parse_args()

    print(f"\n{'='*55}")
    print(f"  SCARA Firmware Serial Test Suite")
    print(f"  Port: {args.port}  Baud: {args.baud}")
    print(f"{'='*55}\n")

    # ── CONNECT ──────────────────────────────────────────────────────────────
    try:
        ser = connect(args.port, args.baud, wait_ready=True)
    except RuntimeError as e:
        print(f"{RED}CONNECT FAILED:{RESET} {e}")
        sys.exit(1)

    # ── TEST 1: STATUS (should be IDLE after boot) ────────────────────────────
    run_test("STATUS after boot", "STATUS", "STATUS:IDLE", ser)

    # ── TEST 2: HOME ─────────────────────────────────────────────────────────
    if not args.skip_home:
        info("HOME will drive all axes to limit switches — ensure clearance")
        input("  Press ENTER when ready...")
        run_test("HOME all axes", "HOME", "OK:HOME", ser, timeout=30)
    else:
        info("HOME skipped (--skip-home flag)")

    # ── TEST 3: STATUS after home ─────────────────────────────────────────────
    run_test("STATUS after HOME", "STATUS", "STATUS:IDLE", ser)

    # ── TEST 4: MOVE — small safe move ───────────────────────────────────────
    info("MOVE:10,0,0,0  →  J1 rotates 10 degrees, others stay")
    run_test("MOVE J1=10deg Z=0", "MOVE:10,0,0,0", "OK:MOVE", ser, timeout=15)

    # ── TEST 5: MOVE — return to zero ────────────────────────────────────────
    run_test("MOVE all back to 0", "MOVE:0,0,0,0", "OK:MOVE", ser, timeout=15)

    # ── TEST 6: MOVE — multi-axis ────────────────────────────────────────────
    info("MOVE:20,10,-5,5  →  all four axes move simultaneously")
    run_test("MOVE multi-axis", "MOVE:20,10,-5,5", "OK:MOVE", ser, timeout=15)
    run_test("MOVE return home", "MOVE:0,0,0,0",   "OK:MOVE", ser, timeout=15)

    # ── TEST 7: MOVE with Z ──────────────────────────────────────────────────
    info("MOVE:0,0,0,10  →  Z descends 10 mm")
    run_test("MOVE Z=10mm", "MOVE:0,0,0,10", "OK:MOVE", ser, timeout=15)
    run_test("MOVE Z back to 0", "MOVE:0,0,0,0", "OK:MOVE", ser, timeout=15)

    # ── TEST 8: GRIP ─────────────────────────────────────────────────────────
    info("Gripper will close")
    run_test("GRIP", "GRIP", "OK:GRIP", ser)

    # ── TEST 9: RELEASE ──────────────────────────────────────────────────────
    info("Gripper will open")
    run_test("RELEASE", "RELEASE", "OK:RELEASE", ser)

    # ── TEST 10: REHOME_Z ────────────────────────────────────────────────────
    info("REHOME_Z:19.2  →  Z homes then descends to layer 1 height")
    run_test("REHOME_Z layer 1", "REHOME_Z:19.2", "OK:REHOME_Z", ser, timeout=20)
    run_test("MOVE Z back to 0", "MOVE:0,0,0,0",  "OK:MOVE",     ser, timeout=15)

    # ── TEST 11: SET_SPEED ───────────────────────────────────────────────────
    run_test("SET_SPEED valid",   "SET_SPEED:800",  "OK:SPEED", ser)
    run_test("SET_SPEED restore", "SET_SPEED:1200", "OK:SPEED", ser)

    # ── TEST 12: SET_ACCEL ───────────────────────────────────────────────────
    run_test("SET_ACCEL valid",   "SET_ACCEL:600", "OK:ACCEL", ser)
    run_test("SET_ACCEL restore", "SET_ACCEL:800", "OK:ACCEL", ser)

    # ── TEST 13: NACK cases ───────────────────────────────────────────────────
    run_nack_test("NACK unknown command",   "BLAH",           ser)
    run_nack_test("NACK bad MOVE args",     "MOVE:abc",       ser)
    run_nack_test("NACK speed too low",     "SET_SPEED:10",   ser)
    run_nack_test("NACK invalid accel",     "SET_ACCEL:-1",   ser)

    # ── TEST 14: STATUS:IDLE final check ─────────────────────────────────────
    run_test("STATUS final check", "STATUS", "STATUS:IDLE", ser)

    # ── DISCONNECT ───────────────────────────────────────────────────────────
    disconnect(ser)

    # ── SUMMARY ──────────────────────────────────────────────────────────────
    total  = len(results)
    passed_n = sum(1 for _, ok, _ in results if ok)
    failed_n = total - passed_n

    print(f"\n{'='*55}")
    print(f"  RESULTS:  {passed_n}/{total} passed   {failed_n} failed")
    print(f"{'='*55}")

    for name, ok, resp in results:
        status = f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"
        print(f"  {status}  {name:<40}  {resp}")

    print()
    if failed_n:
        print(f"{RED}  {failed_n} test(s) FAILED — check firmware and wiring{RESET}\n")
        sys.exit(1)
    else:
        print(f"{GREEN}  All tests passed{RESET}\n")


if __name__ == "__main__":
    main()
