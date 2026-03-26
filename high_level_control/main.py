
from math import sqrt, pi, acos, cos, tan, atan2, sin
import serial
import time
MovementMatrix = [[0, 1, 0, 0, 0, 0],
[0, 1, 0, 0, 0, 0],
[0, 1, 0, 0, 0, 0],
[0, 1, 1, 1, 1, 1]]
MovementInstructions = [[0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0]]
PlacementsX = []
PlacementsY = []
MotorAngle1=[]
MotorAngle2=[]
L1 = 0.150
L2 = 0.100
L3 = 0.50
Redogrid = True
O1X = 0.120
O1Y = 0

BAUD         = 115200
TIMEOUT_S    = 10.0   # seconds to wait for any response
READY_WAIT_S = 15.0   # seconds to wait for READY on connect

OK_RESPONSES = {
    "HOME":     "OK:HOME",
    "MOVE":     "OK:MOVE",
    "GRIP":     "OK:GRIP",
    "RELEASE":  "OK:RELEASE",
    "REHOME_Z": "OK:REHOME_Z",
    "SPEED":    "OK:SPEED",
    "ACCEL":    "OK:ACCEL",
}

def MovementAddition():
    if MovementMatrix[row][col] == 0:
        MovementMatrix[row][col] = 1;
    else:
        MovementMatrix[row][col] = 0
def CloseGrid():

    for x in range(3):

        for y in range(5):

            if MovementMatrix[x][y] == 1 and MovementMatrix[x + 1][y] == 1 and MovementMatrix[x][y + 1] == 1 and \
                    MovementMatrix[x + 1][y + 1] == 1:

                print("There is a 2x2 grid");

                RedoGrid = True;

            else:

                RedoGrid = False;



def CheckGrid():
    for y in range(1, 5):  # Defining Corners
        for x in range(1, 3):

            if MovementMatrix[x][y] == 1:

                if MovementMatrix[x + 1][y] == 1:

                    if MovementInstructions[x][y] == 0:
                        MovementInstructions[x][y] = 1  # 1 Represent horizontal gripper without priority

                    if MovementMatrix[x + 1][y - 1] == 1:
                        MovementInstructions[x + 1][y] = 3;

                    MovementInstructions[x + 1][y - 1] = 4;

def connect(port, baud=BAUD, wait_ready=True):
    """
    Open serial port and optionally wait for READY from firmware.
    Returns open serial.Serial object.
    Raises RuntimeError if READY not received within READY_WAIT_S.
    """
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # allow Arduino to reset after DTR toggle
    ser.reset_input_buffer()

    # if wait_ready:
    #     deadline = time.time() + READY_WAIT_S
    #     while time.time() < deadline:
    #         line = ser.readline().decode("utf-8", errors="ignore").strip()
    #         if line:
    #             print(f"  << {line}")
    #         if line == "READY":
    #             print("[connect] READY received")
    #             return ser
    #     raise RuntimeError(f"[connect] READY not received within {READY_WAIT_S}s")

    return ser


def send_command(ser, cmd, timeout=TIMEOUT_S):
    """
    Send one command string (no newline needed) and block until response.

    Returns the response string.
    Raises RuntimeError on timeout.
    Prints ERR/NACK lines as warnings but still returns them — caller decides.
    """
    full_cmd = cmd.strip() + "\n"
    ser.write(full_cmd.encode("utf-8"))
    ser.flush()
    print(f"  >> {cmd}")

    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        print(f"  << {line}")
        # Accept any non-empty line as the response
        # (firmware may emit intermediate lines like HOMED Jx during HOME)
        if (line.startswith("OK:")
                or line.startswith("NACK:")
                or line.startswith("ERR:")
                or line.startswith("STATUS:")):
            return line

    raise RuntimeError(f"[send_command] Timeout waiting for response to: {cmd}")

def disconnect(ser):
    if ser and ser.is_open:
        ser.close()
        print("[disconnect] Port closed")

for y in range(3):  # Defining Corners

    for x in range(3):

        if MovementMatrix[x][y] == 1:

            if MovementMatrix[x + 1][y] == 1:

                if MovementMatrix[x + 1][y + 1] == 1:
                    MovementInstructions[x + 1][y] = 3;  # 3 represents horizontal gripper with priority

                    MovementInstructions[x + 1][y + 1] = 4;  # 4 represents vertical gripper with priority

for x in range(3):

    for y in range(5):

        TargetPoint = MovementInstructions[x][y];

        if TargetPoint == 3 or TargetPoint == 4:

            if x < 8 and y < 8:

                if MovementInstructions[x][y + 1] == 4:
                    PlacementsX.append(x);

                    PlacementsY.append(y + 1);

                    PlacementsX.append(x);

                    PlacementsY.append(y);

                    y = y + 1

            else:

                PlacementsX.append(x);

                PlacementsY.append(y);

for x in range(3):

    for y in range(5):

        TargetPoint = MovementInstructions[x][y];

        if TargetPoint == 1 or TargetPoint == 2:
            PlacementsX.append(x);

            PlacementsY.append(y);



def triangle1(x, y) -> float:
    return sqrt(x ** 2 + y ** 2)


def triangle2(x: float, y: float, r: float) -> float:
    """ Finds the value of angle Alpha"""
    alpha = acos((L1 ** 2 + L2 ** 2 - r ** 2) / (2 * L1 * L2))

    return pi - alpha  # Radians


def triangle3(q2: float) -> float:
    """ Returns the value of a2_sin_q2 """
    a2_sin_q2 = sqrt(cos(q2 ** 2))

    return a2_sin_q2  # Radians


def triangle4(q2) -> float:
    """ Returns the angle Beta """
    beta = atan2(L2 * sin(q2), L1 + L2 * cos(q2))
    return beta  # Radians

for x in range(len(PlacementsX)):
    global_x = ((PlacementsX[x])*0.0318+0.0159) + O1X
    global_y = ((PlacementsY[x])*0.0318+0.0159) + O1Y
    print(PlacementsX[x], PlacementsY[x], global_x, global_y);
    r = triangle1(global_x, global_y)

    q2 = round(triangle2(global_x, global_y, r), 1)
    print(q2);
    q2_degrees = round(q2 * (180 / pi), 1)
    alpha = round(triangle3(q2), 1)
    beta = round(triangle4(q2), 1)
    gamma = round(atan2(y, x), 1)
    alpha_in_degrees = round(alpha * (180 / pi), 1)
    beta_in_degrees = round(beta * (180 / pi), 1)
    gamma_in_degrees = round(gamma * (180 / pi), 1)
    q1 = beta - gamma
    MotorAngle1.append (alpha);
    MotorAngle2.append (beta);

ser = serial.open('COM3')
