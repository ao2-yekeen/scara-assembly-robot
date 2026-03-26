# presets.py
# ─────────────────────────────────────────────────────────────────────────────
# House floor plan shape presets.
#
# Shapes are defined as fractions of the safe grid so they scale automatically
# when BLOCK_STUDS or STUD_PITCH changes in config.py.
#
# Each shape function receives (cols, rows) — the safe grid dimensions —
# and returns (col, row) positions scaled to fit within them.

from config import GRID_ROWS, GRID_COLS
from grid import empty_grid


# ─── SHAPE PRIMITIVES ────────────────────────────────────────────────────────

def _hollow_rect(c0: int, r0: int, w: int, h: int) -> list[tuple[int, int]]:
    """Outline (walls only) of a rectangle."""
    return [
        (c0 + c, r0 + r)
        for r in range(h)
        for c in range(w)
        if r == 0 or r == h - 1 or c == 0 or c == w - 1
    ]


def _merge(*shapes) -> list[tuple[int, int]]:
    """Merge shape lists, removing duplicates."""
    return list(set(pt for shape in shapes for pt in shape))


def _safe(w: int, h: int) -> tuple[int, int]:
    """
    Clamp (w, h) to fit within the safe grid.
    Shapes use this so they never exceed the reachable workspace
    regardless of block size.
    """
    return min(w, GRID_COLS), min(h, GRID_ROWS)


# ─── HOUSE SHAPES ─────────────────────────────────────────────────────────────
# All dimensions expressed as fractions of GRID_COLS / GRID_ROWS
# so they automatically resize when block pitch changes.

def studio() -> list[tuple[int, int]]:
    """Single room — fills most of the safe grid."""
    w, h = _safe(GRID_COLS, GRID_ROWS)
    return _hollow_rect(0, 0, w, h)


def two_room() -> list[tuple[int, int]]:
    """Two rooms stacked vertically."""
    w        = GRID_COLS
    h_each   = max(2, GRID_ROWS // 2 + 1)   # overlap by 1 row = shared wall
    h_each   = min(h_each, GRID_ROWS)
    return _merge(
        _hollow_rect(0, 0,          w, h_each),
        _hollow_rect(0, h_each - 1, w, h_each),
    )


def l_shape() -> list[tuple[int, int]]:
    """L-shaped house — wide base, narrow upper wing."""
    w_full  = GRID_COLS
    w_wing  = max(2, GRID_COLS // 2)
    h_base  = max(2, GRID_ROWS // 2)
    h_wing  = GRID_ROWS - h_base + 1
    return _merge(
        _hollow_rect(0, 0,      w_full, h_base),
        _hollow_rect(0, h_base - 1, w_wing, h_wing),
    )


def u_shape() -> list[tuple[int, int]]:
    """U-shaped courtyard — two side walls with connecting base."""
    w_wall  = max(2, GRID_COLS // 3)
    h_wall  = GRID_ROWS
    w_base  = GRID_COLS
    h_base  = max(2, GRID_ROWS // 3)
    r_base  = GRID_ROWS - h_base
    return _merge(
        _hollow_rect(0,              0, w_wall, h_wall),
        _hollow_rect(GRID_COLS - w_wall, 0, w_wall, h_wall),
        _hollow_rect(0,          r_base, w_base, h_base),
    )


def t_shape() -> list[tuple[int, int]]:
    """T-shaped house — wide top bar, narrow stem."""
    w_bar   = GRID_COLS
    h_bar   = max(2, GRID_ROWS // 3)
    w_stem  = max(2, GRID_COLS // 3)
    h_stem  = GRID_ROWS - h_bar + 1
    c_stem  = (GRID_COLS - w_stem) // 2
    return _merge(
        _hollow_rect(0,      0,         w_bar,  h_bar),
        _hollow_rect(c_stem, h_bar - 1, w_stem, h_stem),
    )


def bungalow() -> list[tuple[int, int]]:
    """Bungalow — single wide shallow room."""
    w, h = _safe(GRID_COLS, max(2, GRID_ROWS // 2))
    return _hollow_rect(0, 0, w, h)


# ─── PRESET REGISTRY ─────────────────────────────────────────────────────────

PRESETS: dict[str, tuple[str, callable]] = {
    '1': ("Studio",         studio),
    '2': ("Two rooms",      two_room),
    '3': ("L-shaped house", l_shape),
    '4': ("U-shaped house", u_shape),
    '5': ("T-shaped house", t_shape),
    '6': ("Bungalow",       bungalow),
    '7': ("Custom (blank)", None),
}


def apply_preset(key: str) -> list[list[int]]:
    """
    Return a grid with the chosen preset applied.
    Key '7' returns an empty grid.
    Out-of-bounds points are silently dropped.
    """
    grid = empty_grid()
    fn = PRESETS[key][1]
    if fn is None:
        return grid
    for col, row in fn():
        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            grid[row][col] = 1
    return grid