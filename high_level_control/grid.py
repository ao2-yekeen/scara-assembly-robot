# grid.py
# ─────────────────────────────────────────────────────────────────────────────
# Grid data model and floor plan file I/O.
# A grid is a list-of-lists: grid[row][col] = 0 or 1.

import sys
from config import GRID_ROWS, GRID_COLS


def empty_grid() -> list[list[int]]:
    """Return a blank GRID_ROWS × GRID_COLS grid of zeros."""
    return [[0] * GRID_COLS for _ in range(GRID_ROWS)]


def grid_to_positions(grid: list[list[int]]) -> list[tuple[int, int]]:
    """Return (col, row) tuples for every cell set to 1."""
    return [
        (col, row)
        for row, cells in enumerate(grid)
        for col, value in enumerate(cells)
        if value == 1
    ]


def positions_to_grid(positions: list[tuple[int, int]]) -> list[list[int]]:
    """
    Build a grid from a list of (col, row) positions.
    Out-of-bounds positions are silently ignored.
    """
    grid = empty_grid()
    for col, row in positions:
        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            grid[row][col] = 1
    return grid


def toggle_cell(grid: list[list[int]], row: int, col: int) -> None:
    """Toggle a single cell between 0 and 1 in-place."""
    grid[row][col] ^= 1


def load_floor_plan(filename: str) -> list[tuple[int, int]]:
    """
    Load a binary floor plan from a space-separated text file.
    Each line is a row; 1 = block, 0 = empty.
    Returns list of (col, row) tuples.
    Exits with an error message if the file is not found.
    """
    positions = []
    try:
        with open(filename) as f:
            for row, line in enumerate(f):
                for col, value in enumerate(line.strip().split()):
                    if value == '1':
                        positions.append((col, row))
    except FileNotFoundError:
        print(f"  Error: file not found — {filename}")
        sys.exit(1)
    return positions


def save_floor_plan(grid: list[list[int]], filename: str = 'floor_plan.txt') -> None:
    """Save grid to a space-separated text file."""
    with open(filename, 'w') as f:
        for row in grid:
            f.write(' '.join(str(v) for v in row) + '\n')
