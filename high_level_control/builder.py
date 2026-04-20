# builder.py
# ─────────────────────────────────────────────────────────────────────────────
# Pick-and-place sequence logic.
# Translates grid positions into robot motion commands and executes them.

from config import (
    Z_CLEARANCE, Z_PICK, Z_PLACE,
    DUPLO_H_MM, MOVE_TIMEOUT,
    PICKUP_POSITIONS, PLACE_POSITIONS, ZONE_COLS,
)
from kinematics import ik
from display import ok, fail, info

_JOINT_LIMITS = [(0.0, 160.0, "J1"), (0.0, 330.0, "J2"), (0.0, 330.0, "J3")]


def _check_ik(q1: float, q2: float, q3: float, label: str) -> bool:
    """Return False and log an error if any joint angle is outside Arduino limits."""
    for val, lo, hi, name in zip((q1, q2, q3), *zip(*_JOINT_LIMITS)):
        if val < lo or val > hi:
            fail(f"{label}: {name}={val:.2f}° outside [{lo:.0f}, {hi:.0f}]")
            return False
    return True


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


def run_pickup_place(ser, positions: list[tuple[int, int]], layer: int, dry_run: bool) -> bool:
    """
    Execute pick-and-place for the given floor-plan positions.

    For each (col, row) in positions:
      - place target  → PLACE_POSITIONS[row * ZONE_COLS + col]
      - supply pick   → PICKUP_POSITIONS cycled in order

    Returns True if all placements succeed, False if any step fails.
    """
    z_layer = Z_PLACE + layer * DUPLO_H_MM

    for i, (col, row) in enumerate(positions):
        place_idx = row * ZONE_COLS + col
        if place_idx >= len(PLACE_POSITIONS):
            fail(f"grid({col},{row}) → index {place_idx} out of range for PLACE_POSITIONS")
            return False

        sx, sy = PICKUP_POSITIONS[i % len(PICKUP_POSITIONS)]
        px, py = PLACE_POSITIONS[place_idx]

        q1s, q2s, q3s = ik(sx, sy)
        if not _check_ik(q1s, q2s, q3s, f"pick({sx:.4f},{sy:.4f})"):
            return False

        q1p, q2p, q3p = ik(px, py)
        if not _check_ik(q1p, q2p, q3p, f"place({px:.4f},{py:.4f})"):
            return False

        info(
            f"Block {i + 1}/{len(positions)}  "
            f"grid({col},{row})  pick({sx:.4f},{sy:.4f})  place({px:.4f},{py:.4f})  "
            f"J1={q1p:.2f}°  J2={q2p:.2f}°"
        )

        sequence = _build_pick_place_sequence(
            q1s, q2s, q3s,
            q1p, q2p, q3p,
            z_layer,
        )

        if not _execute_sequence(ser, sequence, dry_run):
            return False

    return True
