# config.py
# ─────────────────────────────────────────────────────────────────────────────
# All physical constants for the SCARA Duplo robot.
#
# ── HOW TO CHANGE BLOCK SIZE ─────────────────────────────────────────────────
# Edit only BLOCK_STUDS and STUD_PITCH below.
# GRID_COLS and GRID_ROWS are computed automatically from the arm geometry
# and block pitch — they always reflect the largest safe rectangular workspace.
# Preset shapes in presets.py scale with the grid — no other file changes.
#
# Common values:
#   1×1 stud  →  BLOCK_STUDS = 1
#   2×2 stud  →  BLOCK_STUDS = 2  (default)
#   4×2 stud  →  BLOCK_STUDS = 2  (same pitch in each axis)

from math import sqrt


# ─── ROBOT SETTINGS ──────────────────────────────────────────────────────────
OPTIMIZATION_THRESHOLD = 5e-3  # 5mm
STEPPER_STEP_DEG = 1.8

# ─── Joint 1 (Base Revolute) ─────────────────────────────────────────────────
J1_MOTOR_TEETH  = 20
J1_GEAR1_BIG    = 72
J1_GEAR1_SMALL  = 40
J1_GEAR2        = 100
J1_HOME_ANGLE   = -80
J1_LIMIT        = 80

# ─── Joint 2 (Elbow Revolute) ────────────────────────────────────────────────
J2_MOTOR_TEETH  = 20
J2_GEAR1_BIG    = 50
J2_GEAR1_SMALL  = 20
J2_GEAR2        = 48
J2_HOME_ANGLE   = -165
J2_LIMIT        = 165

# ─── BLOCK SIZE — only edit these two values ─────────────────────────────────
BLOCK_STUDS = 2       # studs per block side (1 for 1×1, 2 for 2×2, etc.)
STUD_PITCH  = 0.0154  # stud centre-to-centre pitch (m) — standard Duplo/Lego

# ─── ARM GEOMETRY ────────────────────────────────────────────────────────────
L1 = 0.20026  # upper arm length (m)
L2 = 0.08391  # forearm length (m)

# ─── DERIVED: block centre-to-centre pitch ────────────────────────────────────
DUPLO_PITCH = BLOCK_STUDS * STUD_PITCH  # e.g. 2 × 0.0154 = 0.0308m

# ─── DUPLO HEIGHT ─────────────────────────────────────────────────────────────
DUPLO_H_MM = 19.2  # brick height (mm) — same for all standard Duplo

# ─── SUPPLY POSITION (robot base frame) ──────────────────────────────────────
SUPPLY_X = 0.120  # m
SUPPLY_Y = 0.000  # m

# ─── GRID ORIGIN (robot base frame) ──────────────────────────────────────────
ORIGIN_X = 0.120  # m
ORIGIN_Y = 0.000  # m

# ─── DERIVED: safe grid dimensions ───────────────────────────────────────────
# Finds the largest rectangular grid where EVERY cell is reachable by the arm.
# A cell is reachable if its world position falls within [L1-L2, L1+L2] radius.
# This updates automatically whenever BLOCK_STUDS, STUD_PITCH, L1, L2,
# ORIGIN_X, or ORIGIN_Y change.

def _safe_rows_for_col(col: int) -> int:
    """Return how many consecutive rows starting at row 0 are safe for a col."""
    r_min = abs(L1 - L2)
    r_max = L1 + L2
    count = 0
    for row in range(50):  # scan generously
        x = col * DUPLO_PITCH + DUPLO_PITCH / 2 + ORIGIN_X
        y = row * DUPLO_PITCH + DUPLO_PITCH / 2 + ORIGIN_Y
        if r_min <= sqrt(x**2 + y**2) <= r_max:
            count = row + 1
        else:
            break  # rows are monotonically increasing in radius — stop here
    return count


def _compute_safe_grid_size() -> tuple[int, int]:
    """
    Find the largest rectangle (cols × rows) where all cells are reachable.
    Scans columns left to right, stopping when a column has zero safe rows.
    Returns (cols, rows) of the best rectangle found.
    """
    best_cols, best_rows, best_area = 0, 0, 0
    min_rows_so_far = None

    for col in range(50):
        safe = _safe_rows_for_col(col)
        if safe == 0:
            break  # no safe cells in this col — stop

        min_rows_so_far = safe if min_rows_so_far is None else min(min_rows_so_far, safe)
        area = (col + 1) * min_rows_so_far
        if area > best_area:
            best_area = area
            best_cols = col + 1
            best_rows = min_rows_so_far

    return best_cols, best_rows


GRID_COLS, GRID_ROWS = _compute_safe_grid_size()

# ─── Z HEIGHTS ───────────────────────────────────────────────────────────────
Z_CLEARANCE = 40.0  # travel height between moves (mm)
Z_PICK      = 5.0   # lower to this height to pick a block (mm)
Z_PLACE     = 5.0   # lower to this height above layer surface to place (mm)

# ─── BUILD PARAMETERS ────────────────────────────────────────────────────────
MAX_LAYERS = 2  # maximum number of layers

# ─── SERIAL ──────────────────────────────────────────────────────────────────
DEFAULT_PORT   = "COM5"
DEFAULT_BAUD   = 115200
HOME_TIMEOUT   = 30
MOVE_TIMEOUT   = 15
REHOME_TIMEOUT = 20
