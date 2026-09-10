# Electronics and bill of materials

Source: Team ED5, *Design and Development of a SCARA Robot Prototype for Autonomous Block Placement*, B51RO, 15 April 2026, supplied project report. Reported measurements below are historical results, not new tests.

## Components

This is the report's component inventory, not a complete procurement list. Part variants, prices, mechanical hardware, cable lengths and connector quantities are not specified.

| Component | Quantity | Role |
| --- | ---: | --- |
| Arduino Mega 2560 | 1 | Embedded control |
| NEMA17 stepper motor | 4 | Three rotational joints and Z |
| DRV8825 driver | 4 | STEP/DIR motor control |
| Normally closed limit switch | 4 | Homing references |
| SG90 servo | 1 | Gripper |
| 12 V, 4 A power supply | 2 | Split motor supply |
| 100 µF capacitor | 4 | Per-driver decoupling |
| 1000 µF capacitor | 1 | Bulk decoupling |

## Signal assignments

The following mapping is taken from the current [firmware joint table](../firmware/src/main.cpp).

| Axis | STEP pin | DIR pin | Limit pin |
| --- | ---: | ---: | ---: |
| J1 | 2 | 3 | 21 |
| J2 | 4 | 5 | 18 |
| J3 | 6 | 7 | 19 |
| Z | 8 | 9 | 20 |

The gripper signal is on pin 10. Limit inputs use INPUT_PULLUP; the firmware treats HIGH as triggered. See the [architecture notes](../docs/system_architecture.md) for the current limits of motion monitoring.

## Power arrangement

The report describes one 12 V supply dedicated to J1 and a second supplying J2, J3 and Z, with a common ground for signal reference. It describes the SG90 powered from the Arduino 5 V pin and proposes a dedicated 5 V converter as future work.

| Load | Single supply | Dual supplies |
| --- | ---: | ---: |
| No motor load | 12.0 V | 12.0 V |
| One motor | 11.8 V | 11.9 V |
| Two motors | 11.6 V | 11.8 V |
| Four motors | 7.8 V | 11.7 V |

These are measurements reported in April 2026. The report links the supply change to resolving voltage collapse during multi-motor operation.

The report also describes per-driver and bulk decoupling to address resets. Raw oscilloscope captures and measurement conditions are not supplied here, so this page does not present a quantified noise-reduction benchmark.

## Schematic availability

The report includes a CircKit Designer schematic as Figure 2. An editable schematic source is not present in this repository. This inventory and pin table do not replace the full circuit schematic.
