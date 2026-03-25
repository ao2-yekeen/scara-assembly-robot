"""
main.py  —  SCARA Duplo Block Placement Controller
───────────────────────────────────────────────────
Modes:
  Interactive  : draw floor plan on a terminal grid, then run
  File         : load floor plan from .txt file, then run
  Dry-run      : simulate without Arduino connected (--dry-run)

Usage:
  python main.py --port COM5
  python main.py --port COM5 --floor floor_plan.txt
  python main.py --port COM5 --dry-run
  python main.py --port COM5 --floor floor_plan.txt --dry-run
"""

import argparse
import sys
import time
from math import sqrt, acos, atan2, sin, cos, pi

# ─── ROBOT CONFIG ─────────────────────────────────────────────────────────────
L1 = 0.150          # upper arm length (m)
L2 = 0.100          # forearm length (m)

DUPLO_PITCH  = 0.0154   # stud pitch centre-to-centre (m)
DUPLO_H_MM   = 19.2     # brick height (mm)

SUPPLY_X     = 0.120    # supply tray X from robot base (m)
SUPPLY_Y     = 0.000    # supply tray Y from robot base (m)

ORIGIN_X     = 0.120    # grid origin X offset from robot base (m)
ORIGIN_Y     = 0.000    # grid origin Y offset from robot base (m)

Z_CLEARANCE  = 40.0     # travel height between moves (mm)
Z_PICK       = 5.0      # pick height above supply (mm)
Z_PLACE      = 5.0      # place height above current layer surface (mm)

NUM_LAYERS   = 3        # number of layers to stack

# ─── COLOUR HELPERS ──────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def ok(msg):   print(f"  {GREEN}OK{RESET}    {msg}")
def err(msg):  print(f"  {RED}FAIL{RESET}  {msg}")
def info(msg): print(f"  {YELLOW}--{RESET}    {msg}")

# ─── INVERSE KINEMATICS ──────────────────────────────────────────────────────
def ik(x: float, y: float):
    """
    Two-link planar IK for SCARA (elbow-up solution).
    x, y in metres from robot base.
    Returns (q1_deg, q2_deg, q3_deg).
    q3 = wrist constraint keeping end-effector orientation fixed: q3 = -(q1+q2).
    Raises ValueError if position is outside reachable workspace.
    """
    r = sqrt(x**2 + y**2)

    if r > (L1 + L2) or r < abs(L1 - L2):
        raise ValueError(
            f"({x:.3f}, {y:.3f})m unreachable — "
            f"r={r:.3f}m, workspace [{abs(L1-L2):.3f}, {L1+L2:.3f}]m"
        )

    cos_q2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))   # clamp floating-point drift
    q2 = acos(cos_q2)

    beta  = atan2(L2 * sin(q2), L1 + L2 * cos(q2))
    gamma = atan2(y, x)
    q1    = gamma - beta
    q3    = -(q1 + q2)

    return (
        round(q1 * 180 / pi, 2),
        round(q2 * 180 / pi, 2),
        round(q3 * 180 / pi, 2),
    )

# ─── GRID HELPERS ────────────────────────────────────────────────────────────
def grid_to_world(col: int, row: int):
    """Convert grid indices to world coordinates (m)."""
    x = col * DUPLO_PITCH + DUPLO_PITCH / 2 + ORIGIN_X
    y = row * DUPLO_PITCH + DUPLO_PITCH / 2 + ORIGIN_Y
    return x, y


def grid_to_positions(grid: list):
    """Convert 2D grid to list of (col, row) tuples for cells == 1."""
    return [
        (col, row)
        for row, cells in enumerate(grid)
        for col, val in enumerate(cells)
        if val == 1
    ]

# ─── FLOOR PLAN I/O ──────────────────────────────────────────────────────────
def load_floor_plan(filename: str):
    """Load binary floor plan from .txt file."""
    positions = []
    try:
        with open(filename, 'r') as f:
            for row, line in enumerate(f):
                for col, val in enumerate(line.strip().split()):
                    if val == '1':
                        positions.append((col, row))
    except FileNotFoundError:
        err(f"File not found: {filename}")
        sys.exit(1)
    return positions


def save_floor_plan(grid: list, filename: str = 'floor_plan.txt'):
    """Save 2D grid to .txt file."""
    with open(filename, 'w') as f:
        for row in grid:
            f.write(' '.join(str(c) for c in row) + '\n')
    ok(f"Saved to {filename}")

# ─── INTERACTIVE GRID EDITOR ─────────────────────────────────────────────────
def interactive_grid():
    """
    Terminal grid editor.
    User sets dimensions, toggles cells, then confirms.
    Returns list of (col, row) tuples.
    """
    print(f"\n{BOLD}{CYAN}  Floor Plan Editor{RESET}\n")

    while True:
        try:
            cols = int(input("  Columns (width)  [2-12]: ").strip())
            rows = int(input("  Rows    (height) [2-12]: ").strip())
            if 2 <= cols <= 12 and 2 <= rows <= 12:
                break
            print(f"  {RED}Enter values between 2 and 12{RESET}")
        except ValueError:
            print(f"  {RED}Numbers only{RESET}")

    grid = [[0] * cols for _ in range(rows)]

    def print_grid():
        print(f"\n       ", end="")
        for c in range(cols):
            print(f"{c:<2}", end="")
        print()
        print("      " + "─" * (cols * 2 + 1))
        for r in range(rows):
            print(f"  {r:<4}│ ", end="")
            for c in range(cols):
                print(f"{GREEN}■{RESET} " if grid[r][c] else f"{YELLOW}·{RESET} ", end="")
            print()
        n = sum(v for row in grid for v in row)
        print(f"\n  {n} block(s) selected\n")

    print(f"\n  Commands:")
    print(f"    {BOLD}row,col{RESET}  toggle a cell  e.g.  0,2")
    print(f"    {BOLD}fill{RESET}     fill all cells")
    print(f"    {BOLD}clear{RESET}    clear all cells")
    print(f"    {BOLD}save{RESET}     save to floor_plan.txt")
    print(f"    {BOLD}done{RESET}     confirm and continue\n")

    print_grid()

    while True:
        try:
            cmd = input("  > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)

        if cmd == 'done':
            positions = grid_to_positions(grid)
            if not positions:
                print(f"  {RED}Select at least one cell{RESET}")
                continue
            ok(f"{len(positions)} position(s) confirmed")
            return positions

        elif cmd == 'fill':
            for r in range(rows):
                for c in range(cols):
                    grid[r][c] = 1
            print_grid()

        elif cmd == 'clear':
            for r in range(rows):
                for c in range(cols):
                    grid[r][c] = 0
            print_grid()

        elif cmd == 'save':
            save_floor_plan(grid)

        elif ',' in cmd:
            try:
                r, c = [int(v.strip()) for v in cmd.split(',')]
                if 0 <= r < rows and 0 <= c < cols:
                    grid[r][c] ^= 1
                    print_grid()
                else:
                    print(f"  {RED}Out of range — rows 0-{rows-1}, cols 0-{cols-1}{RESET}")
            except (ValueError, IndexError):
                print(f"  {RED}Format: row,col{RESET}")
        else:
            print(f"  {RED}Unknown command{RESET}")

# ─── VALIDATE POSITIONS ───────────────────────────────────────────────────────
def validate_positions(positions):
    """Check all positions are reachable. Returns True if all valid."""
    info(f"Validating {len(positions)} position(s)...")
    invalid = []
    for col, row in positions:
        x, y = grid_to_world(col, row)
        try:
            ik(x, y)
        except ValueError as e:
            invalid.append(f"  Grid ({col},{row}): {e}")

    if invalid:
        for msg in invalid:
            err(msg)
        return False

    ok("All positions reachable")
    return True

# ─── SERIAL LAYER ─────────────────────────────────────────────────────────────
class DryRunSerial:
    """Simulates serial responses without hardware."""
    def send(self, cmd):
        time.sleep(0.03)
        print(f"    {YELLOW}[DRY]{RESET} >> {cmd}")
        if cmd.startswith("MOVE:"):   resp = "OK:MOVE"
        elif cmd == "GRIP":           resp = "OK:GRIP"
        elif cmd == "RELEASE":        resp = "OK:RELEASE"
        elif cmd == "HOME":           resp = "OK:HOME"
        elif cmd.startswith("REHOME_Z:"): resp = "OK:REHOME_Z"
        else:                         resp = "NACK:UNKNOWN"
        print(f"    {YELLOW}[DRY]{RESET} << {resp}")
        return resp


def make_connection(port, baud, dry_run):
    if dry_run:
        info("Dry-run mode — no Arduino required")
        return DryRunSerial()
    from comms import connect
    return connect(port, baud, wait_ready=True)


def send_cmd(ser, cmd, dry_run, timeout=15):
    if dry_run:
        return ser.send(cmd)
    from comms import send_command
    return send_command(ser, cmd, timeout=timeout)


def close_connection(ser, dry_run):
    if not dry_run:
        from comms import disconnect
        disconnect(ser)

# ─── PICK AND PLACE ───────────────────────────────────────────────────────────
def pick_and_place(ser, positions, layer, dry_run):
    """Execute one full layer of block placements."""
    z_layer = Z_PLACE + layer * DUPLO_H_MM

    # Nearest-first sorting minimises arm travel distance
    positions_sorted = sorted(
        positions,
        key=lambda p: sum(v**2 for v in grid_to_world(p[0], p[1]))
    )

    for i, (col, row) in enumerate(positions_sorted):
        wx, wy = grid_to_world(col, row)
        q1p, q2p, q3p = ik(wx, wy)
        q1s, q2s, q3s = ik(SUPPLY_X, SUPPLY_Y)

        info(f"Block {i+1}/{len(positions_sorted)} — "
             f"grid({col},{row}) → ({wx:.3f},{wy:.3f})m — "
             f"J1={q1p}° J2={q2p}°")

        sequence = [
            (f"MOVE:{q1s:.2f},{q2s:.2f},{q3s:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Supply clearance"),
            (f"MOVE:{q1s:.2f},{q2s:.2f},{q3s:.2f},{Z_PICK:.2f}",      "OK:MOVE",    "Lower to pick"),
            ("GRIP",                                                     "OK:GRIP",    "Grip block"),
            (f"MOVE:{q1s:.2f},{q2s:.2f},{q3s:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Lift from supply"),
            (f"MOVE:{q1p:.2f},{q2p:.2f},{q3p:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Move to target"),
            (f"MOVE:{q1p:.2f},{q2p:.2f},{q3p:.2f},{z_layer:.2f}",     "OK:MOVE",    "Lower to place"),
            ("RELEASE",                                                  "OK:RELEASE", "Release block"),
            (f"MOVE:{q1p:.2f},{q2p:.2f},{q3p:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Retract"),
        ]

        for cmd, expected, label in sequence:
            resp = send_cmd(ser, cmd, dry_run)
            if resp.startswith(expected):
                ok(label)
            else:
                err(f"{label} — expected {expected}, got: {resp}")
                return False

    return True

# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="SCARA block placement controller")
    parser.add_argument("--port",    default="COM5")
    parser.add_argument("--baud",    type=int, default=115200)
    parser.add_argument("--floor",   default=None,
                        help="Floor plan .txt file — omit to use interactive editor")
    parser.add_argument("--layers",  type=int, default=NUM_LAYERS)
    parser.add_argument("--dry-run", action="store_true",
                        help="Simulate without Arduino")
    args = parser.parse_args()

    print(f"\n{BOLD}{CYAN}{'═'*50}{RESET}")
    print(f"{BOLD}{CYAN}  SCARA Block Placement Controller{RESET}")
    print(f"{BOLD}{CYAN}{'═'*50}{RESET}\n")

    # Get floor plan
    if args.floor:
        positions = load_floor_plan(args.floor)
        info(f"Loaded {len(positions)} block(s) from {args.floor}")
    else:
        positions = interactive_grid()

    if not positions:
        err("No positions — exiting")
        sys.exit(1)

    # Validate reachability before connecting to Arduino
    if not validate_positions(positions):
        err("Fix unreachable positions before running")
        sys.exit(1)

    # Connect
    try:
        ser = make_connection(args.port, args.baud, args.dry_run)
    except RuntimeError as e:
        err(f"Connection failed: {e}")
        sys.exit(1)

    # Home
    info("Homing all axes...")
    resp = send_cmd(ser, "HOME", args.dry_run, timeout=30)
    if not resp.startswith("OK:HOME"):
        err(f"Homing failed — got: {resp}")
        close_connection(ser, args.dry_run)
        sys.exit(1)
    ok("All axes homed")

    # Layer loop
    total_placed = 0
    for layer in range(args.layers):
        print(f"\n  {BOLD}Layer {layer + 1} / {args.layers}{RESET}")

        # Rehome Z for each layer after the first
        if layer > 0:
            z_mm = layer * DUPLO_H_MM
            resp = send_cmd(ser, f"REHOME_Z:{z_mm:.2f}", args.dry_run, timeout=20)
            if not resp.startswith("OK:REHOME_Z"):
                err(f"Z rehome failed — got: {resp}")
                break
            ok(f"Z rehomed to {z_mm:.1f}mm")

        if pick_and_place(ser, positions, layer, args.dry_run):
            total_placed += len(positions)
            ok(f"Layer {layer + 1} complete")
        else:
            err(f"Layer {layer + 1} aborted")
            break

    # Return to home position
    info("Returning home...")
    send_cmd(ser, f"MOVE:0.00,0.00,0.00,{Z_CLEARANCE:.2f}", args.dry_run)
    close_connection(ser, args.dry_run)

    print(f"\n{BOLD}{CYAN}{'═'*50}{RESET}")
    print(f"  Build complete — {total_placed} block(s) placed")
    print(f"{BOLD}{CYAN}{'═'*50}{RESET}\n")


if __name__ == "__main__":
    main()