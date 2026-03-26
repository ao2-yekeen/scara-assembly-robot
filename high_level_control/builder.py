# builder.py
# ─────────────────────────────────────────────────────────────────────────────
# Pick-and-place sequence logic.
# Translates grid positions into robot motion commands and executes them.

from config import (
    SUPPLY_X, SUPPLY_Y,
    Z_CLEARANCE, Z_PICK, Z_PLACE,
    DUPLO_H_MM, MOVE_TIMEOUT,
)
from kinematics import ik, grid_to_world
from display import ok, fail, info


def _build_pick_place_sequence(
    q1s: float, q2s: float, q3s: float,
    q1p: float, q2p: float, q3p: float,
    z_layer: float,
) -> list[tuple[str, str, str]]:
    """
    Build the 8-step command sequence for one pick-and-place cycle.
    Returns list of (command, expected_response, label).
    """
    return [
        (f"MOVE:{q1s:.2f},{q2s:.2f},{q3s:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Supply clearance"),
        (f"MOVE:{q1s:.2f},{q2s:.2f},{q3s:.2f},{Z_PICK:.2f}",      "OK:MOVE",    "Lower to pick"),
        ("GRIP",                                                     "OK:GRIP",    "Grip"),
        (f"MOVE:{q1s:.2f},{q2s:.2f},{q3s:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Lift"),
        (f"MOVE:{q1p:.2f},{q2p:.2f},{q3p:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Move to target"),
        (f"MOVE:{q1p:.2f},{q2p:.2f},{q3p:.2f},{z_layer:.2f}",     "OK:MOVE",    "Lower to place"),
        ("RELEASE",                                                  "OK:RELEASE", "Release"),
        (f"MOVE:{q1p:.2f},{q2p:.2f},{q3p:.2f},{Z_CLEARANCE:.2f}", "OK:MOVE",    "Retract"),
    ]


def _execute_sequence(ser, sequence: list[tuple[str, str, str]], dry_run: bool) -> bool:
    """
    Send each command in sequence, check response, abort on failure.
    Returns True if all steps succeed.
    """
    for cmd, expected, label in sequence:
        resp = ser.send(cmd) if dry_run else ser.send(cmd, timeout=MOVE_TIMEOUT)
        if resp.startswith(expected):
            ok(label)
        else:
            fail(f"{label} — expected {expected}, got {resp}")
            return False
    return True


def place_layer(ser, positions: list[tuple[int, int]], layer: int, dry_run: bool) -> bool:
    """
    Execute one complete layer of block placements.

    Sorts positions nearest-first to minimise arm travel.
    Returns True if all placements succeed, False if any step fails.
    """
    z_layer = Z_PLACE + layer * DUPLO_H_MM

    q1s, q2s, q3s = ik(SUPPLY_X, SUPPLY_Y)

    sorted_positions = sorted(
        positions,
        key=lambda p: sum(v**2 for v in grid_to_world(*p)),
    )

    for index, (col, row) in enumerate(sorted_positions):
        wx, wy = grid_to_world(col, row)
        q1p, q2p, q3p = ik(wx, wy)

        info(
            f"Block {index + 1}/{len(sorted_positions)}  "
            f"grid({col},{row})  ({wx:.3f},{wy:.3f})m  "
            f"J1={q1p}°  J2={q2p}°"
        )

        sequence = _build_pick_place_sequence(
            q1s, q2s, q3s,
            q1p, q2p, q3p,
            z_layer,
        )

        if not _execute_sequence(ser, sequence, dry_run):
            return False

    return True
