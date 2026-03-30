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
    J3_MOTOR_TEETH, J3_GEAR2, J3_HOME_ANGLE, J3_LIMIT, OPTIMIZATION_THRESHOLD, STEPPER_STEP_DEG
)
KinematicsListX = []
KinematicsListY = []
VerticalGrip = []


def calculate_deg_per_step(motor_teeth, g1_big, g1_small, g2, base_step=1.8):
    """Calculates the degrees moved per motor step through the gear train."""
    # Ratio = (Driving / Driven) * (Driving / Driven)
    return base_step * (motor_teeth * g1_small) / (g1_big * g2)

def SortPositions(positions):
    KinematicsListX = []
    KinematicsListY = []
    VerticalGrip = []
    Vertical = False
    for x in range(len(positions)):# Defining Corners
        CurrentPosition = positions[x]
        SecondCheck = False
        FirstCheck = False
        print(CurrentPosition)
        for y in range(len(positions)):
            if CurrentPosition[1] == (positions[y][1]-1) and CurrentPosition[0] == (positions[y][0]):
                FirstCheck = True
            if CurrentPosition[1] == (positions[y][1]) and CurrentPosition[0] == (positions[y][0]-1):
                SecondCheck = True
            if FirstCheck and SecondCheck:
                VerticalGrip.append(False)
                KinematicsListX.append(positions[y][0])
                KinematicsListY.append(positions[y][1])
                FirstCheck = False
                SecondCheck = False
        for y in range(len(positions)):
            if CurrentPosition[1] == (positions[y][1]-1) and CurrentPosition[0] == (positions[y][0]):
                Vertical = True
        if Vertical:
            VerticalGrip.append(True)
        KinematicsListX.append(positions[y][0])
        KinematicsListY.append(positions[y][1])
        return (KinematicsListX,KinematicsListY,VerticalGrip)

def ik(x: float, y: float) -> tuple:
    """
    Advanced Inverse Kinematics using discretized workspace optimization.
    Uses constants imported directly from config.py.
    """
    # 1. Calculate Resolution based on Gear Ratios
    j1_res = calculate_deg_per_step(
        J1_MOTOR_TEETH, J1_GEAR1_BIG, J1_GEAR1_SMALL, J1_GEAR2, STEPPER_STEP_DEG
    )
    j2_res = calculate_deg_per_step(
        J2_MOTOR_TEETH, J2_GEAR1_BIG, J2_GEAR1_SMALL, J2_GEAR2, STEPPER_STEP_DEG
    )
    j3_res = STEPPER_STEP_DEG * (J3_MOTOR_TEETH / J3_GEAR2)
    print(f"IK Debug: Resolutions → J1={j1_res:.4f}°, J2={j2_res:.4f}°, J3={j3_res:.4f}°")

    # 2. Generate Reachable Workspace Grid
    q1_range = np.arange(J1_HOME_ANGLE, J1_LIMIT + j1_res, j1_res)
    q2_range = np.arange(J2_HOME_ANGLE, J2_LIMIT + j2_res, j2_res)
    Q1G, Q2G = np.meshgrid(q1_range, q2_range)

    # Forward Kinematics for every possible stepper position
    # Uses L1 and L2 imported from config
    X_work = L1 * np.cos(np.radians(Q1G)) + L2 * np.cos(np.radians(Q1G + Q2G))
    Y_work = L1 * np.sin(np.radians(Q1G)) + L2 * np.sin(np.radians(Q1G + Q2G))

    # 3. Distance Matrix Optimization
    dist_matrix = np.sqrt((X_work - x)**2 + (Y_work - y)**2)
    
    # Prefer Elbow-Up (Q2 > 0)
    pref_dist_matrix = dist_matrix.copy()
    pref_dist_matrix[Q2G < 0] = np.inf 

    idx_pref = np.argmin(pref_dist_matrix)
    best_err = pref_dist_matrix.flat[idx_pref]

    # Fallback if preferred config is mechanically unreachable
    if best_err > OPTIMIZATION_THRESHOLD:
        idx = np.argmin(dist_matrix)
        best_err = dist_matrix.flat[idx] # Update error for final check
    else:
        idx = idx_pref

    # FINAL REACHABILITY CHECK: Required for validate_positions() to work
    if best_err > OPTIMIZATION_THRESHOLD:
         raise ValueError(f"Target ({x:.3f}, {y:.3f}) is outside reachable step-grid.")

    # Get the actual X/Y reached by the chosen stepper positions
    x_best = X_work.flat[idx]
    y_best = Y_work.flat[idx]

    # 4. Extract Results
    q1_best_deg = Q1G.flat[idx]
    q2_best_deg = Q2G.flat[idx]

    pickup = 45
    place = -45

    # Determine position based on the angle of the target
    if atan2(y_best, x_best) > 0:
        position = pickup
    else:
        position = place

    # Calculate ideal angles
    q3_ideal = position - q1_best_deg  - q2_best_deg
    q3_relative_ideal = q3_ideal - J3_HOME_ANGLE

    print(f"IK Debug: q3_ideal={q3_ideal:.2f}°, q3_relative_ideal={q3_relative_ideal:.2f}°")
    # Calculate steps and the resulting actual relative angle
    stepsJ3 = round(q3_relative_ideal / j3_res)
    relative_q3 = stepsJ3 * j3_res
    print(f"IK Debug: stepsJ3={stepsJ3}, relative_q3={relative_q3:.2f}°")

    if True:
        pass
    else:
        relative_q3 = relative_q3 - 90       
    
    # Normalize the angle within the joint limits
    if relative_q3 < 0:
        relative_q3 = relative_q3 + 180
        
    if relative_q3 > -(J3_HOME_ANGLE) * 2:
        relative_q3 = relative_q3 - 180

    q1 = q1_best_deg - J1_HOME_ANGLE
    q2 = q2_best_deg - J2_HOME_ANGLE
    q3 = relative_q3
    print(f"IK: Target ({x:.3f}, {y:.3f}) → q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}° (err={best_err:.4f}m)")

    return  (q1, q2, q3)

# def ik(x: float, y: float) -> tuple:
#     """
#     Advanced Inverse Kinematics using discretized workspace optimization.
#     Uses constants imported directly from config.py.
#     """
    
#     # 1. Calculate Resolution based on Gear Ratios
#     j1_res = calculate_deg_per_step(
#         J1_MOTOR_TEETH, J1_GEAR1_BIG, J1_GEAR1_SMALL, J1_GEAR2, STEPPER_STEP_DEG
#     )
#     j2_res = calculate_deg_per_step(
#         J2_MOTOR_TEETH, J2_GEAR1_BIG, J2_GEAR1_SMALL, J2_GEAR2, STEPPER_STEP_DEG
#     )
#     j3_res = STEPPER_STEP_DEG * (J3_MOTOR_TEETH / J3_GEAR2)

#     # 2. Generate Reachable Workspace Grid
#     q1_range = np.arange(J1_HOME_ANGLE, J1_LIMIT + j1_res, j1_res)
#     q2_range = np.arange(J2_HOME_ANGLE, J2_LIMIT + j2_res, j2_res)
#     Q1G, Q2G = np.meshgrid(q1_range, q2_range)

#     # Forward Kinematics for every possible stepper position
#     # Uses L1 and L2 imported from config
#     X_work = L1 * np.cos(np.radians(Q1G)) + L2 * np.cos(np.radians(Q1G + Q2G))
#     Y_work = L1 * np.sin(np.radians(Q1G)) + L2 * np.sin(np.radians(Q1G + Q2G))

#     # 3. Distance Matrix Optimization
#     dist_matrix = np.sqrt((X_work - x)**2 + (Y_work - y)**2)
    
#     # Prefer Elbow-Up (Q2 > 0)
#     pref_dist_matrix = dist_matrix.copy()
#     pref_dist_matrix[Q2G < 0] = np.inf 

#     idx_pref = np.argmin(pref_dist_matrix)
#     best_err = pref_dist_matrix.flat[idx_pref]

#     # Fallback if preferred config is mechanically unreachable
#     if best_err > OPTIMIZATION_THRESHOLD:
#         idx = np.argmin(dist_matrix)
#         best_err = dist_matrix.flat[idx] # Update error for final check
#     else:
#         idx = idx_pref

#     # FINAL REACHABILITY CHECK: Required for validate_positions() to work
#     if best_err > OPTIMIZATION_THRESHOLD:
#          raise ValueError(f"Target ({x:.3f}, {y:.3f}) is outside reachable step-grid.")

#     # Get the actual X/Y reached by the chosen stepper positions
#     x_best = X_work.flat[idx]
#     y_best = Y_work.flat[idx]

#     # 4. Extract Results
#     q1_best_deg = Q1G.flat[idx]
#     q2_best_deg = Q2G.flat[idx]

#      # --- Q3 (J4) CALCULATION USING REACHED COORDINATES ---
#     q3_ideal = np.degrees(np.arctan2(y_best, x_best)) - q1_best_deg - q2_best_deg + 180
#     steps_j3 = round(q3_ideal / j3_res)
    
#     q3_best_deg = steps_j3 * j3_res

#     q1 = q1_best_deg - J1_HOME_ANGLE
#     q2 = q2_best_deg - J2_HOME_ANGLE
#     q3 = q3_best_deg - (180-J3_HOME_ANGLE)
#     print(f"IK: Target ({x:.3f}, {y:.3f}) → q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}° (err={best_err:.4f}m)")

#     return  (q1, q2, q3)




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
    SortCol, SortRow, VerticalGrip = SortPositions(positions)
    for x in range(len(SortCol)):
        x, y = grid_to_world(SortCol[x], SortRow[x])
        try:
            ik(x, y)
            valid.append((SortCol, SortRow))
        except ValueError as e:
            invalid.append((SortCol, SortRow, str(e)))
    return valid, invalid
