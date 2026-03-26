# display.py
# ─────────────────────────────────────────────────────────────────────────────
# Terminal output helpers — colours, grid rendering, status messages.
# All functions are pure output with no return values.

from config import GRID_ROWS, GRID_COLS, MAX_LAYERS

# ─── ANSI COLOURS ────────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"


def ok(message: str) -> None:
    print(f"  {GREEN}OK{RESET}    {message}")


def fail(message: str) -> None:
    print(f"  {RED}FAIL{RESET}  {message}")


def info(message: str) -> None:
    print(f"  {YELLOW}--{RESET}    {message}")


def header(title: str) -> None:
    bar = '═' * 50
    print(f"\n{BOLD}{CYAN}{bar}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{bar}{RESET}\n")


def print_grid(grid: list[list[int]], title: str = "Floor Plan") -> None:
    """Render the grid to the terminal with column/row labels."""
    print(f"\n  {BOLD}{CYAN}{title}{RESET}")

    # Column numbers header
    print(f"  {DIM}      ", end="")
    for c in range(GRID_COLS):
        print(f"{c:<2}", end="")
    print(f"{RESET}")

    print(f"        {'─' * (GRID_COLS * 2)}")

    for r, row in enumerate(grid):
        print(f"  {DIM}{r:<4}{RESET}  │ ", end="")
        for value in row:
            if value:
                print(f"{GREEN}█{RESET} ", end="")
            else:
                print(f"{DIM}·{RESET} ", end="")
        print()

    block_count = sum(v for row in grid for v in row)
    total = block_count * MAX_LAYERS
    print(f"\n  {block_count} block(s) per layer  │  {total} total (×{MAX_LAYERS} layers)\n")
