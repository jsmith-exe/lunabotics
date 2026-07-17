# CAN simulator

`can_sim` emulates the SPARK MAX and linear-actuator traffic used by the `diffdrive_canbus` hardware plugin. It allows protocol and controller integration testing without energizing the rover.

## Start

After sourcing the repository environment and building:

```bash
qpl_can_sim
```

The launch file starts:

```bash
socat PTY,link=/dev/ttyUSB0,rawer PTY,link=/tmp/fake_can_rx,rawer
```

and the `can_sim` Python executable. The hardware plugin opens `/dev/ttyUSB0`; the simulator opens `/tmp/fake_can_rx`.

Do not run this launch while a real adapter is expected at `/dev/ttyUSB0`. Device paths are currently hard-coded.

## Simulated devices

The default application creates wheel motors on CAN IDs 1-4, linear actuators on IDs 5-6, and a drum motor on ID 7. The UI displays command and simulated state for these devices.

Default command-line options in `can_sim.main` include:

- `--port /tmp/fake_can_rx`
- serial baud matching the simulator configuration

## What it validates

- FRC extended CAN ID creation and matching
- Waveshare serial frame handling
- SPARK MAX duty and velocity commands
- Simulated status frames for velocity, position, applied output, and analog actuator feedback
- Integration with the real ros2_control hardware plugin

## What it does not validate

- Vehicle physics, traction, inertia, or load
- Motor current, thermal limits, or controller flash configuration
- Real serial timing, USB loss, CAN bus-off, noise, or contention
- Mechanical actuator limits, calibration, stalls, or injury risk
- A physical emergency stop

## Manual execution

The installed executable can be run directly after creating the PTYs:

```bash
ros2 run can_sim can_sim --port /tmp/fake_can_rx
```

`class_tests.py` is a manual developer exercise, not an automated pytest suite.

## Troubleshooting

- Confirm `socat` is installed using `qpl_packages`.
- Confirm neither PTY path already exists or belongs to hardware.
- Confirm the hardware Xacro and simulator agree on CAN IDs and baud settings.
- Inspect both terminal logs when frames are not being recognized.
