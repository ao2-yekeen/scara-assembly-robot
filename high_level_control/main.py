#Software to send command to Duplo firmware
#robot workspace in binary format. 0 = no block, 1 = 2x2 block.

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

def load_floor_plan(filename='floor_plan.txt'):
    pairs = []
    with open(filename, 'r') as file:
        for row, line in enumerate(file):
            for column, x in enumerate(line.strip().split()):   # column = 0,1.. ; x = '0' or '1'
                if int(x):
                    print("Found block at row %d, column %d\n", row, column)
                    pairs.append((column*DUPLO_W+(DUPLO_W/2),row*DUPLO_W+(DUPLO_W/2)))
    return pairs

coordinates = load_floor_plan()
print("Floor plan loaded:")
for coord in coordinates:
    print(coord)
