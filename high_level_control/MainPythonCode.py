
from math import sqrt, pi, acos, cos, tan, atan2, sin
import serial
import argparse
import sys
import time
from comms import connect, send_command, disconnect
MovementMatrix = [[0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1],
                  [1, 1, 1, 1, 1, 1],
                  [0, 0, 0, 0, 0, 0]]
MovementInstructions = [[0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0]]
PlacementsX = []
PlacementsY = []
MotorAngle 1 =[]
MotorAngle 2 =[]
MotorAngle 3 =[]
HomeMotorAngle1 =[]
HomeMotorAngle2 = []
HomeMotorAngle3 = []
L1 = 0.150
L2 = 0.100
L3 = 0.50
Redogrid = True
O1X = 0.120
O1Y = 0


def passed(msg): print(f"  {GREEN}PASS{RESET}  {msg}")


def failed(msg): print(f"  {RED}FAIL{RESET}  {msg}")


def info(msg):   print(f"  {YELLOW}INFO{RESET}  {msg}")


results = []


def CloseGrid():
    for x in range(3):

        for y in range(5):

            if MovementMatrix[x][y] == 1 and MovementMatrix[x + 1][y] == 1 and MovementMatrix[x][y + 1] == 1 and \
                    MovementMatrix[x + 1][y + 1] == 1:

                print("There is a 2x2 grid");

                RedoGrid = True;

            else:

                RedoGrid = False;


for y in range(5):  # Defining Corners
    for x in range(3):
        if MovementMatrix[x][y] == 1:
            if MovementMatrix[x][y - 1] == 1:
                MovementInstructions[x][y] = 2;
            else:
                MovementInstructions[x][y] = 1

for y in range(5):  # Defining Corners
    for x in range(3):
        if MovementMatrix[x][y] == 1:
            if MovementMatrix[x - 1][y] == 1:
                if MovementMatrix[x - 1][y - 1] == 1:
                    MovementInstructions[x][y - 1] = 3;  # 3 represents horizontal gripper with priority
                    MovementInstructions[x][y] = 4;  # 4 represents vertical gripper with priority

for x in range(len(MovementInstructions)):
    print(MovementInstructions[x]);
for x in range(3):
    for y in range(6):
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
            PlacementsX.append(x)

            PlacementsY.append(y)


def triangle1(x, y) -> float:
    return sqrt(x ** 2 + y ** 2)


def triangle2(x: float, y: float, r: float) -> float:
    """ Finds the value of angle Alpha"""
    alpha = acos((L1 ** 2 + L2 ** 2 - r ** 2) / (2 * L1 * L2))

    return pi - alpha  # Radians


def triangle3(q2: float) -> float:
    """ Returns the value of a2_sin_q2 """
    a2_sin_q2 = sqrt(- cos(q2 ** 2))

    return a2_sin_q2  # Radians


def triangle4(q2) -> float:
    """ Returns the angle Beta """
    beta = atan2(L2 * sin(q2), L1 + L2 * cos(q2))
    return beta  # Radians


for x in range(len(PlacementsX)):
    global_x = ((PlacementsX[x]) * 0.0154 + 0.0077) + O1X
    global_y = ((PlacementsY[x]) * 0.0154 + 0.0077) + O1Y
    print(PlacementsX[x], PlacementsY[x], global_x, global_y);
    r = triangle1(global_x, global_y)

    q2 = round(triangle2(global_x, global_y, r), 1)
    q2_degrees = round(q2 * (180 / pi), 1)
    alpha = round(triangle3(q2), 1)
    beta = round(triangle4(q2), 1)
    gamma = round(atan2(y, x), 1)
    alpha_in_degrees = round(alpha * (180 / pi), 1)
    beta_in_degrees = round(beta * (180 / pi), 1)
    gamma_in_degrees = round(gamma * (180 / pi), 1)
    q1 = beta - gamma
    MotorAngle1.append(alpha);
    MotorAngle2.append(beta);
    MotorAngle3.append(gamma)
connect()
for x in range(len(MotorAngle1)):
    Target = "Move:" + HomeAngle1[x] + "," + HomeAngle2[x] + "," + HomeAngle3[x] + ",0"
    run_test("MOVE Z back to 0", Target, "OK:MOVE", ser, timeout=15)
    Target = "Move:" + MotorAngle1[x] + "," + MotorAngle2[x] + "," + MotorAngle3 + ",0"
    run_test("MOVE Z back to 0", Target, "OK:MOVE", ser, timeout=15)







