# SCARA Autonomous Assembly Robot

_4-DOF SCARA-based robotic system for autonomous block assembly, developed at Heriot-Watt University._

---

## Demo

[![▶ Watch Demo on Google Drive](https://img.shields.io/badge/Watch%20Demo-%E2%96%B6%20Google%20Drive-4285F4?style=for-the-badge&logo=googledrive&logoColor=white)](https://drive.google.com/file/d/1bch_skkfbKCsSWpbhE6QLBJJI1j0O4o6/view?usp=sharing)

---

## Project Overview

This project implements a fully autonomous assembly robot built around a 4-degree-of-freedom SCARA (Selective Compliance Articulated Robot Arm) architecture. The system accepts user-defined grid layout files and autonomously executes the corresponding block placement sequence, handling all motion planning, sequencing logic, and microcontroller communication without manual intervention.

The project was developed as part of a robotics engineering programme at Heriot-Watt University and spans embedded firmware, high-level software control, electronics design, and kinematic simulation.

---

## System Architecture

```
+-------------------------------------------+
|           High-Level Control              |
|  (Python: grid parser, sequencer,         |
|   motion planner, serial interface)       |
+------------------+------------------------+
                   |  USB Serial (UART)
                   v
+-------------------------------------------+
|         Arduino Firmware (C++)            |
|  (motor drivers, joint control,           |
|   sensor feedback, command parser)        |
+------------------+------------------------+
                   |  PWM / Stepper signals
                   v
+-------------------------------------------+
|         SCARA Robot Hardware              |
|  (4 DOF arm, end-effector, sensors)       |
+-------------------------------------------+
```

The Python layer reads a user-supplied grid file, computes the required block placement sequence, resolves each target position via inverse kinematics, and streams motion commands to the Arduino over a serial connection. The firmware executes low-level joint actuation and reports state back to the host.

---

## Repository Structure

```
.
├── docs/                    # Project documentation, reports, and diagrams
├── electronics/             # Schematics, PCB layouts, and wiring diagrams
├── firmware/                # Arduino/C++ microcontroller code
│   └── main/                # Main firmware sketch and supporting modules
├── high_level_control/      # Python software framework (primary codebase)
│   ├── grid_parser.py       # Parses user-supplied grid layout files
│   ├── sequencer.py         # Converts grid data into ordered placement steps
│   ├── motion_planner.py    # Generates joint trajectories for each step
│   ├── serial_interface.py  # Handles serial communication with firmware
│   └── README.md            # Detailed usage guide for this module
├── simulations/             # Kinematic and motion simulations
├── .gitignore
└── README.md
```

---

## How to Run

### Prerequisites

- Python 3.8 or higher
- `pyserial` library
- Arduino IDE (for firmware upload)
- A connected SCARA robot (or simulation mode)

Install Python dependencies:

```bash
pip install pyserial
```

### Running the High-Level Controller

The main Python script **must be executed from within the `high_level_control/` directory**:

```bash
cd high_level_control
python main.py --grid <path_to_grid_file>
```

Refer to [`high_level_control/README.md`](high_level_control/README.md) for full usage options, grid file format specification, and configuration details.

### Uploading Firmware

Open `firmware/main/main.ino` in the Arduino IDE, select the correct board and port, and upload. Ensure the baud rate in the firmware matches the value configured in the Python serial interface.

---

## Tech Stack

| Layer | Technology |
|---|---|
| High-level control | Python 3 |
| Microcontroller firmware | C++ / Arduino |
| Serial communication | UART via `pyserial` |
| Kinematic simulation | Python (NumPy / Matplotlib) |
| Electronics design | KiCad / Fritzing |
| Version control | Git / GitHub |

---

## Developed At

**Heriot-Watt University** — Robotics Engineering Project
Academic Year 2025–2026
