"""
Robot Configuration Module

This module contains all configuration parameters for the LEGO robotic arm system,
including arm geometry, stepper motor conversion factors, motion constraints, 
serial communication settings, and joint limits.
"""

# Arm geometry (mm)
L1 = 150.0
L2 = 100.0
L3 = 50.0

# Steps conversion
STEPS_PER_DEG_J1 = 17.78
STEPS_PER_DEG_J2 = 17.78
STEPS_PER_DEG_J3 = 17.78
STEPS_PER_MM_Z   = 80.0

# Motion
DUPLO_W         = 8 # Duplo block width (mm)
CLEARANCE_HEIGHT = 40.0
NUM_LAYERS       = 3

# Supply tray position (mm)
SUPPLY_X = 120.0
SUPPLY_Y = 0.0

# Serial
PORT       = "COM3"
BAUD       = 115200
TIMEOUT_MS = 5000
MAX_RETRIES = 3

# Joint limits (degrees / mm)
J1_MIN, J1_MAX = -150.0, 150.0
J2_MIN, J2_MAX = -120.0, 120.0
J3_MIN, J3_MAX = -150.0, 150.0
Z_MIN,  Z_MAX  =    0.0, 60.0