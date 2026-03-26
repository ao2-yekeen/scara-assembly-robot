# editor.py
# ─────────────────────────────────────────────────────────────────────────────
# Interactive terminal floor plan editor.
# User picks a shape preset, optionally toggles cells, then presses
# Enter on an empty line to confirm.

import sys
from config import GRID_ROWS, GRID_COLS
from display import ok, fail, info, header, print_grid, BOLD, CYAN, RED, RESET
from grid import grid_to_positions, toggle_cell
from presets import PRESETS, apply_preset


def _prompt_preset_choice() -> str:
    """Show preset menu and return the user's valid choice."""
    print(f"  {BOLD}Choose a floor plan shape:{RESET}\n")
    for key, (name, _) in PRESETS.items():
        print(f"    {BOLD}{key}{RESET}  {name}")
    print()

    while True:
        try:
            choice = input("  Shape [1-7]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)

        if choice in PRESETS:
            return choice

        print(f"  {RED}Enter a number between 1 and 7{RESET}")


def _parse_toggle_input(cmd: str) -> tuple[int, int] | None:
    """
    Parse 'row,col' string into (row, col) integers.
    Returns None if the input is not valid.
    """
    if ',' not in cmd:
        return None
    parts = cmd.split(',')
    if len(parts) != 2:
        return None
    try:
        return int(parts[0].strip()), int(parts[1].strip())
    except ValueError:
        return None


def _is_in_bounds(row: int, col: int) -> bool:
    return 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS


def run_editor() -> list[tuple[int, int]]:
    """
    Run the interactive floor plan editor.

    Flow:
      1. User picks a preset shape.
      2. User types row,col to toggle individual cells.
      3. User presses Enter on an empty line to confirm.

    Returns list of (col, row) positions where grid == 1.
    """
    header("SCARA House Floor Plan Builder")
    print(f"  Grid {GRID_COLS}×{GRID_ROWS}  |  Max 2 layers\n")

    choice = _prompt_preset_choice()
    preset_name = PRESETS[choice][0]
    grid = apply_preset(choice)
    print_grid(grid, preset_name)

    print(f"  Type {BOLD}row,col{RESET} to toggle a cell  (e.g. 2,3)")
    print(f"  Press {BOLD}Enter{RESET} on an empty line when done.\n")

    while True:
        try:
            cmd = input("  > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)

        if cmd == '':
            positions = grid_to_positions(grid)
            if not positions:
                print(f"  {RED}No blocks selected — add at least one{RESET}")
                continue
            ok(f"{len(positions)} block(s) confirmed")
            return positions

        parsed = _parse_toggle_input(cmd)
        if parsed is None:
            print(f"  {RED}Format: row,col  e.g. 2,3  — or press Enter to confirm{RESET}")
            continue

        row, col = parsed
        if not _is_in_bounds(row, col):
            print(f"  {RED}Out of range — row 0-{GRID_ROWS-1}, col 0-{GRID_COLS-1}{RESET}")
            continue

        toggle_cell(grid, row, col)
        print_grid(grid, preset_name)
