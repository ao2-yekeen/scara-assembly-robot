# kinematics.py
# ─────────────────────────────────────────────────────────────────────────────
# Inverse kinematics and coordinate transforms for the SCARA arm.
# Pure functions — no side effects, no I/O.
import numpy as np
from math import sqrt, acos, atan2, sin, cos, pi
from config import (
    L1, L2, DUPLO_PITCH, ORIGIN_X, ORIGIN_Y,
    J1_MOTOR_TEETH, J1_GEAR1_BIG, J1_GEAR1_SMALL, J1_GEAR2, J1_HOME_ANGLE, J1_LIMIT,
    J2_MOTOR_TEETH, J2_GEAR1_BIG, J2_GEAR1_SMALL, J2_GEAR2, J2_HOME_ANGLE, J2_LIMIT,
    OPTIMIZATION_THRESHOLD, STEPPER_STEP_DEG
)

def calculate_deg_per_step(motor_teeth, g1_big, g1_small, g2, base_step=1.8):
    """Calculates the degrees moved per motor step through the gear train."""
    # Ratio = (Driving / Driven) * (Driving / Driven)
    return base_step * (motor_teeth * g1_small) / (g1_big * g2)

def ik(x: float, y: float, L1: float, L2: float) -> tuple:
    """
    Advanced Inverse Kinematics using discretized workspace optimization.
    Returns (steps_J1, steps_J2, q1_deg, q2_deg)
    """
    
    # 1. Calculate Resolution based on Gear Ratios
    j1_res = calculate_deg_per_step(
        CONFIG["J1"]["MOTOR_TEETH"], CONFIG["J1"]["GEAR1_BIG"], 
        CONFIG["J1"]["GEAR1_SMALL"], CONFIG["J1"]["GEAR2"]
    )
    j2_res = calculate_deg_per_step(
        CONFIG["J2"]["MOTOR_TEETH"], CONFIG["J2"]["GEAR1_BIG"], 
        CONFIG["J2"]["GEAR1_SMALL"], CONFIG["J2"]["GEAR2"]
    )

    # 2. Generate Reachable Workspace Grid
    q1_range = np.arange(CONFIG["J1"]["HOME_ANGLE"], CONFIG["J1"]["LIMIT"] + j1_res, j1_res)
    q2_range = np.arange(CONFIG["J2"]["HOME_ANGLE"], CONFIG["J2"]["LIMIT"] + j2_res, j2_res)
    Q1G, Q2G = np.meshgrid(q1_range, q2_range)

    # Forward Kinematics for every possible stepper position
    X_work = L1 * np.cos(np.radians(Q1G)) + L2 * np.cos(np.radians(Q1G + Q2G))
    Y_work = L1 * np.sin(np.radians(Q1G)) + L2 * np.sin(np.radians(Q1G + Q2G))

    # 3. Distance Matrix Optimization
    dist_matrix = np.sqrt((X_work - x)**2 + (Y_work - y)**2)
    
    # Prefer Elbow-Up (similar to your old function)
    # In this new coordinate system, Elbow-Up is typically Q2 > 0
    pref_dist_matrix = dist_matrix.copy()
    pref_dist_matrix[Q2G < 0] = np.inf 

    idx_pref = np.argmin(pref_dist_matrix)
    best_err = pref_dist_matrix.flat[idx_pref]

    # Fallback if preferred config is mechanically unreachable
    if best_err > CONFIG["OPTIMIZATION_THRESHOLD"]:
        idx = np.argmin(dist_matrix)
    else:
        idx = idx_pref

    # 4. Extract Results
    q1_best_deg = Q1G.flat[idx]
    q2_best_deg = Q2G.flat[idx]
    
    # Calculate steps relative to home position
    steps_j1 = round(abs(q1_best_deg - CONFIG["J1"]["HOME_ANGLE"]) / j1_res)
    steps_j2 = round(abs(q2_best_deg - CONFIG["J2"]["HOME_ANGLE"]) / j2_res)

    return steps_j1, steps_j2, round(q1_best_deg, 2), round(q2_best_deg, 2)


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
