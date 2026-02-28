from robot_config import *


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
