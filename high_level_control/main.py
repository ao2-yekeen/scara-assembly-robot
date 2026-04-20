"""
main.py  —  SCARA Duplo House Floor Plan Builder
─────────────────────────────────────────────────
Entry point. Parses arguments, orchestrates editor → validate → build.

Usage:
  python main.py --port COM5
  python main.py --port COM5 --dry-run
  python main.py --port COM5 --floor floor_plan.txt --dry-run
  python main.py --port COM5 --layers 1
"""

import argparse
import sys

from config import DEFAULT_PORT, DEFAULT_BAUD, MAX_LAYERS, Z_CLEARANCE, DUPLO_H_MM, HOME_TIMEOUT, REHOME_TIMEOUT
from display import ok, fail, info, header
from grid import load_floor_plan, positions_to_grid, save_floor_plan
from display import print_grid
from kinematics import validate_positions
from editor import run_editor
from serial_comms import make_serial
from builder import run_pickup_place


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SCARA House Floor Plan Builder")
    parser.add_argument("--port",    default=DEFAULT_PORT,
                        help="Serial port e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud",    type=int, default=DEFAULT_BAUD)
    parser.add_argument("--floor",   default=None,
                        help="Load floor plan from file — skips interactive editor")
    parser.add_argument("--layers",  type=int, default=MAX_LAYERS,
                        choices=[1, 2],
                        help="Number of layers to build (1 or 2)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Simulate all serial commands — no Arduino required")
    parser.add_argument("--zones", action="store_true",
                        help="Use measured pickup/place zone positions instead of grid editor")
    return parser.parse_args()


def _get_positions(args: argparse.Namespace) -> list[tuple[int, int]]:
    """Load positions from file or run the interactive editor."""
    if args.floor:
        positions = load_floor_plan(args.floor)
        info(f"Loaded {len(positions)} block(s) from {args.floor}")
        print_grid(positions_to_grid(positions), args.floor)
        return positions
    return run_editor()


def _validate(positions: list[tuple[int, int]]) -> None:
    """Validate all positions are reachable. Exit if any are not."""
    info(f"Validating {len(positions)} position(s)...")
    valid, invalid = validate_positions(positions)
    if invalid:
        for col, row, reason in invalid:
            fail(f"Grid ({col},{row}): {reason}")
        fail("Fix unreachable positions before running.")
        sys.exit(1)
    ok("All positions reachable")


def _home(ser, dry_run: bool) -> None:
    """Home all axes. Exit if homing fails."""
    info("Homing all axes...")
    resp = ser.send("HOME") if dry_run else ser.send("HOME", timeout=HOME_TIMEOUT)
    if not resp.startswith("OK:HOME"):
        fail(f"Homing failed — got: {resp}")
        sys.exit(1)
    ok("All axes homed")


def _rehome_z(ser, layer: int, dry_run: bool) -> bool:
    """Rehome Z axis to the correct height for the given layer."""
    z_mm = layer * DUPLO_H_MM
    cmd  = f"REHOME_Z:{z_mm:.2f}"
    resp = ser.send(cmd) if dry_run else ser.send(cmd, timeout=REHOME_TIMEOUT)
    if resp.startswith("OK:REHOME_Z"):
        ok(f"Z rehomed to {z_mm:.1f}mm")
        return True
    fail(f"Z rehome failed — got: {resp}")
    return False


def main() -> None:
    args = _parse_args()

    header("SCARA House Floor Plan Builder")

    positions = _get_positions(args)
    if not positions:
        fail("No blocks selected — exiting.")
        sys.exit(1)
    _validate(positions)

    try:
        ser = make_serial(args.port, args.baud, args.dry_run)
    except RuntimeError as e:
        fail(f"Connection failed: {e}")
        sys.exit(1)

    _home(ser, args.dry_run)

    total_placed = 0

    for layer in range(args.layers):
        print(f"\n  Layer {layer + 1} / {args.layers}")

        if layer > 0:
            if not _rehome_z(ser, layer, args.dry_run):
                break

        success = run_pickup_place(ser, positions, layer, args.dry_run)
        n = len(positions)

        if success:
            total_placed += n
            ok(f"Layer {layer + 1} complete")
        else:
            fail(f"Layer {layer + 1} aborted")
            break

    info("Returning home...")
    ser.send(f"MOVE:0.00,0.00,0.00,{Z_CLEARANCE:.2f}")

    if hasattr(ser, 'close'):
        ser.close()

    print(f"\n  Build complete — {total_placed} block(s) placed across {args.layers} layer(s)\n")


if __name__ == "__main__":
    main()