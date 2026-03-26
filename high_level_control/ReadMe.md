# **README — SCARA Duplo Floor‑Plan Builder**

## **Overview**
This project controls a 2‑DOF SCARA robot to automatically build multi‑layer DUPLO floor‑plan layouts.  
The robot picks blocks from a supply position and places them onto a grid‑based workspace, following preset or custom house‑shaped patterns.

The system supports:

- Automatic inverse kinematics  
- Workspace validation  
- Multi‑layer construction  
- Dry‑run simulation (no hardware movement)  
- Real hardware execution via serial commands  

---

## **How It Works**
When the program starts, it:

1. Computes the reachable grid based on robot arm lengths and DUPLO block pitch.  
2. Displays a menu of preset house shapes (Studio, L‑shape, T‑shape, etc.).  
3. Lets the user toggle cells to customize the layout.  
4. Validates that every chosen grid cell is reachable.  
5. Homes the robot.  
6. Builds the structure layer by layer.

Each block placement consists of:

- Move to supply  
- Lower  
- Grip  
- Lift  
- Move to target  
- Lower  
- Release  
- Retract  

All robot motions are printed to the console.  
In **dry‑run mode**, no physical movement occurs — commands are only simulated.

---

## **Running the Program**

### **Dry‑run (simulation only)**
```
python3 main.py --port COM5 --dry-run
```

### **Real robot**
```
python3 main.py --port COM5
```

The `--port` argument specifies the serial port connected to the SCARA controller.

---

## **Grid System**
The workspace is a rectangular grid automatically sized from:

- `BLOCK_STUDS` — number of studs per block side  
- `STUD_PITCH` — stud‑to‑stud spacing (15.4 mm)  
- `L1`, `L2` — robot arm lengths  

Example from the run:

```
Grid 3×5 | Max 2 layers
```

Each grid cell corresponds to the **center of one DUPLO block**.

Coordinates are shown as `(row, col)` and converted to robot‑frame `(x, y)` positions.

---

## **Layering**
The robot supports multi‑layer construction.

- Layer 1 is placed at table height.  
- Before Layer 2, the robot re‑homes Z to the DUPLO brick height (19.2 mm).  
- Each subsequent layer is offset by the same height.

Example from the log:

```
[DRY] REHOME_Z:19.20 → OK
```

This ensures the gripper approaches the top of the previous layer safely.

---

## **Example Build: T‑Shaped House**
The user selected:

```
T-shaped house
```

The program displayed the grid, counted blocks, and confirmed reachability:

```
12 block(s) per layer | 24 total (×2 layers)
OK All positions reachable
```

Then the robot executed all pick‑and‑place motions for each block in both layers.

---

## **Serial Command Format**
Commands sent to the robot controller include:

- `MOVE:j1,j2,j3,z`  
- `GRIP`  
- `RELEASE`  
- `HOME`  
- `REHOME_Z:h`  

In dry‑run mode, each command is printed with a `[DRY]` prefix.

Example:

```
[DRY] MOVE:-34.05,117.67,-83.62,5.00 → OK:MOVE
```

---

## **Configuration**
All physical constants are in `config.py`.

Key parameters:

- `BLOCK_STUDS` — block size in studs (default: 2×2)  
- `STUD_PITCH` — 0.0154 m (15.4 mm)  
- `DUPLO_H_MM` — 19.2 mm  
- `L1`, `L2` — arm lengths  

Changing `BLOCK_STUDS` automatically rescales the grid and presets.

---

## **Presets**
Available floor‑plan shapes:

1. Studio  
2. Two rooms  
3. L‑shaped  
4. U‑shaped  
5. T‑shaped  
6. Bungalow  
7. Custom (blank grid)

All presets scale to the current grid size.

---

## **Dry‑Run Output**
The attached run shows:

- Grid generation  
- Shape selection  
- Full 2‑layer build  
- All robot motions simulated  
- Successful completion  

