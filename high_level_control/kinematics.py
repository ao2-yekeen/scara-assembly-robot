# kinematics.py
# ─────────────────────────────────────────────────────────────────────────────
# Inverse kinematics and coordinate transforms for the SCARA arm.

from math import sqrt, acos, atan2, sin, cos, pi
from config import L1, L2, DUPLO_PITCH, ORIGIN_X, ORIGIN_Y


def ik(x: float, y: float) -> tuple[float, float, float]:
    """
    Two-link planar inverse kinematics (elbow-up solution).

    x, y : target position in metres, robot base frame.

    Returns (q1_deg, q2_deg, q3_deg) where:
      q1 = shoulder joint angle
      q2 = elbow joint angle
      q3 = wrist constraint = -(q1 + q2), keeps end-effector pointing down

    Raises ValueError if the position is outside the reachable workspace.
    """
    r = sqrt(x**2 + y**2)

    r_max = L1 + L2
    r_min = abs(L1 - L2)
    if not (r_min <= r <= r_max):
        raise ValueError(
            f"Position ({x:.3f}, {y:.3f})m is unreachable. "
            f"Distance r={r:.3f}m, workspace [{r_min:.3f}, {r_max:.3f}]m."
        )

    cos_q2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))  # clamp floating-point drift

    q2 = acos(cos_q2)
    q1 = atan2(y, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2))
    q3 = -(q1 + q2)

    return (
        round(q1 * 180 / pi, 2),
        round(q2 * 180 / pi, 2),
        round(q3 * 180 / pi, 2),
    )


def grid_to_world(col: int, row: int) -> tuple[float, float]:
    """
    Convert grid cell indices to world coordinates (metres, robot base frame).
    Cell centre is at half a pitch offset from the cell origin.
    """
    x = col * DUPLO_PITCH + DUPLO_PITCH / 2 + ORIGIN_X
    y = row * DUPLO_PITCH + DUPLO_PITCH / 2 + ORIGIN_Y
    return x, y


def validate_positions(positions: list[tuple[int, int]]) -> tuple[list, list]:
    """
    Check every grid position is reachable by the arm.

    Returns:
      valid   — list of (col, row) that pass IK
      invalid — list of (col, row, reason_str) that fail IK
    """
    valid, invalid = [], []
    for col, row in positions:
        x, y = grid_to_world(col, row)
        try:
            ik(x, y)
            valid.append((col, row))
        except ValueError as e:
            invalid.append((col, row, str(e)))
    return valid, invalid
