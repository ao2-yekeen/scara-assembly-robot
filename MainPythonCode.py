import tkinter as tk
root = tk.Tk()

MovementMatrix = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                  [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                  [1, 1, 0, 0, 0, 0, 0, 0, 0, 0],

                  [0, 1, 1, 0, 0, 0, 0, 0, 0, 0],

                  [0, 0, 1, 1, 0, 0, 0, 0, 0, 0],

                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],

                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],

                  [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],

                  [0, 0, 0, 0, 1, 1, 0, 0, 0, 0]]

MovementInstructions = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

PlacementsX = []

PlacementsY = []

NonPriorityX = []

NonPriorityY = []

RedoGrid = True

ArmLength1 = 5

ArmLength2 = 7

ArmLength3 = 5

BaseOffset = 30


def MovementAddition():
    if MovementMatrix[row][col] == 0:
        MovementMatrix[row][col] = 1;
    else:
        MovementMatrix[row][col] = 0
def CloseGrid:

    for x in range(9):

        for y in range(9):

            if MovementMatrix[x][y] == 1 and MovementMatrix[x + 1][y] == 1 and MovementMatrix[x][y + 1] == 1 and \
                    MovementMatrix[x + 1][y + 1] == 1:

                print("There is a 2x2 grid");

                RedoGrid = True;

            else:

                RedoGrid = False;



while RedoGrid:
    for row in range(10):
        for col in range(10):
            tk.Button(
                root,
                text=f"Cell ({row}, {col})",
                width=10,
                height=5,
                command=lambda: MovementAddition(),
            ).grid(row=row, column=col)
    root.mainloop()





def CheckGrid():
    for y in range(1, 9):  # Defining Corners
        for x in range(1, 9):

            if MovementMatrix[x][y] == 1:

                if MovementMatrix[x + 1][y] == 1:

                    if MovementInstructions[x][y] == 0:
                        MovementInstructions[x][y] = 1  # 1 Represent horizontal gripper without priority

                    if MovementMatrix[x + 1][y - 1] == 1:
                        MovementInstructions[x + 1][y] = 3;

                    MovementInstructions[x + 1][y - 1] = 4;

for y in range(8):  # Defining Corners

    for x in range(8):

        if MovementMatrix[x][y] == 1:

            if MovementMatrix[x + 1][y] == 1:

                if MovementMatrix[x + 1][y + 1] == 1:
                    MovementInstructions[x + 1][y] = 3;  # 3 represents horizontal gripper with priority

                    MovementInstructions[x + 1][y + 1] = 4;  # 4 represents vertical gripper with priority

for x in range(9):

    for y in range(9):

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

for x in range(9):

    for y in range(9):

        TargetPoint = MovementInstructions[x][y];

        if TargetPoint == 1 or TargetPoint == 2:
            PlacementsX.append(x);

            PlacementsY.append(y);

