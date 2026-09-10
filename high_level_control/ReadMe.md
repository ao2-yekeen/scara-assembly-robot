# SCARA floor-plan controller

Python control for a SCARA assembly system with two planar arm joints, wrist rotation and vertical motion.

## Offline quick start

Requires Python 3.10 or newer. From the repository root:

```bash
cd high_level_control
python3 main.py --floor example_floor_plan.txt --dry-run --layers 1
```

This rehearses two block placements using simulated serial acknowledgements. It does not connect to the robot or validate real motion.

## Floor-plan format

Use space-separated zeros and ones. Each line is a row; a one selects a block:

```text
1 0
0 1
```

Use the supplied example_floor_plan.txt for a nonempty example. The parser looks for individual tokens equal to 1, so write spaces between values. Grid dimensions depend on config.py; keep selected cells within the configured workspace.

## Command-line options

| Option | Behaviour |
| --- | --- |
| --floor PATH | Load a file instead of opening the editor |
| --dry-run | Use simulated serial acknowledgements |
| --layers 1 or --layers 2 | Select the number of layers; default is 2 |
| --port PORT | Hardware serial port; default COM5 |
| --baud RATE | Baud rate; default 115200 |
| --help | Print CLI help |

The CLI also accepts --zones, but main.py does not currently use that argument to select a different execution path.

Without --floor, the editor offers presets and accepts row,col cell toggles. Press Enter on an empty line to confirm.

## Execution flow

The controller validates selected target positions, opens the selected serial interface and homes the axes. It then runs the pickup/place sequence for each layer. For the second layer it sends REHOME_Z with the configured brick height.

Commands include HOME, MOVE:j1,j2,j3,z, GRIP, RELEASE and REHOME_Z:h. Dry-run responses are generated locally by serial_comms.py.

## Physical operation

Install pyserial, check the robot-specific values in config.py, and select the controller port:

```bash
python3 -m pip install pyserial
python3 main.py --floor example_floor_plan.txt --port COM5 --layers 1
```

This command causes real homing and motion. Confirm the work area and machine setup before running it. A successful dry run is not proof of physical clearance or successful grasping.

See the [project README](../README.md) for firmware build instructions, the demonstration and repository status.
