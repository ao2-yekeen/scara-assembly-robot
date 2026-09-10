# Testing and results

Source: Team ED5, *Design and Development of a SCARA Robot Prototype for Autonomous Block Placement*, B51RO, 15 April 2026, supplied project report. Reported measurements below are historical results, not new tests.

## Reported test stages

| Stage | Reported outcome |
| --- | --- |
| Individual motor tests | Independent stepping, direction reversal and speed calibration checked before integration |
| Electrical load tests | Four-motor supply voltage increased from 7.8 V to 11.7 V after splitting the supplies |
| Serial protocol tests | Expected command and error responses reported with the controller connected |
| Offline software tests | 59 unit tests reported across four modules |
| Integrated physical assembly | Full autonomous physical build not completed at the report stage |

## Reported software coverage

| Module | Coverage described in report | Test count |
| --- | --- | ---: |
| kinematics.py | IK, singularities and bounds | 18 |
| grid.py | File I/O, round trips and toggling | 17 |
| presets.py | Six floor-plan shapes | 6 |
| builder.py | Sequencing and Z offsets | 18 |
| Total | | 59 |

The test files are in [high_level_control/tests](../high_level_control/tests/). These counts are from the report; this documentation update does not claim a fresh run of all 59 tests.

## Reproducible offline example

From the repository root:

```bash
cd high_level_control
python3 main.py --floor example_floor_plan.txt --dry-run --layers 1
```

This example was checked during the portfolio documentation update and exited successfully with two block placements reported. It uses synthetic serial responses and does not validate physical placement.

## Physical accuracy and limitations

The report records approximately 2–5% variation in joint step calibration and approximately 5–8 mm end-effector positioning error, exceeding the ±1.5 mm project target. It also identifies PLA link flex as a source of vertical error at extension.

These values describe the report-stage prototype. No repeated-trial placement success rate or cycle-time dataset is supplied.

[Watch the project demonstration](https://drive.google.com/file/d/1bch_skkfbKCsSWpbhE6QLBJJI1j0O4o6/view?usp=sharing). The video is retained as a project demonstration without treating it as proof of a completed autonomous build.

## Engineering lessons

- Measure effective joint step conversion on the assembled mechanism.
- Separate gripper pulse generation from the stepper motion timer.
- Test power delivery under simultaneous motor load.
- Validate host control flow offline, then verify physical motion separately.

The current firmware differs from some report descriptions. See [system architecture](system_architecture.md) for rotational/Z sequencing, limit-switch monitoring and homing acknowledgement details.
