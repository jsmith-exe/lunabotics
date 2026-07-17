# diffdrive_canbus

Custom ROS 2 Humble `hardware_interface::SystemInterface` for the QPL rover. It connects ros2_control command/state interfaces to REV SPARK MAX devices through a serial-to-CAN adapter.

The authoritative physical-rover parameters are in `qpl_rover/description/ros2_control.xacro`. The description under this package is retained for standalone/demo bring-up and contains different CAN baud and timeout values.

## Devices

| CAN ID | Device | Command mode |
| --- | --- | --- |
| 1 | Front-left wheel | Native velocity |
| 2 | Front-right wheel | Native velocity |
| 3 | Rear-left wheel | Native velocity |
| 4 | Rear-right wheel | Native velocity |
| 5 | Left linear actuator | Duty cycle under analog position servo |
| 6 | Right linear actuator | Duty cycle under analog position servo |
| 7 | Drum | Duty cycle −1 to 1 |

Main rover communication settings:

- Serial device `/dev/ttyUSB0`
- Serial baud 2,000,000
- CAN baud 1,000,000
- Read timeout 5 ms
- Wheel gear ratio 100

## Exported interfaces

- Wheels: velocity command; position and velocity state
- Linear actuators: normalized position command 0-1; position state
- Drum: velocity command interpreted as duty; position and velocity state

## Safety behavior

The implementation:

- forces non-finite commands to zero;
- clamps duty commands to −1 through 1;
- sends periodic heartbeat and command frames;
- stops actuators when feedback is older than 500 ms;
- latches an actuator off after 1.5 seconds without adequate motion;
- monitors wheel telemetry for obvious runaway conditions;
- sends stop commands during normal deactivation.

Limitations:

- wheel velocity is not clamped in the hardware plugin;
- no command-age watchdog exists inside the hardware interface;
- normal write failures are logged but do not consistently force lifecycle error;
- actuator safety depends on valid analog calibration and has no documented physical limit switches;
- thresholds are hard-coded and lack automated tests.

See the repository `docs/safety.md` before hardware use.

## Build

The package uses CMake, C++17, pluginlib, ros2_control, and system `libserial`:

```bash
qpl_packages
qpl_build
```

The plugin is exported as:

```text
diffdrive_canbus/DiffDriveCanbusHardware
```

## Standalone diagnostic

`spark_max_test` is an interactive hardware diagnostic:

```bash
ros2 run diffdrive_canbus spark_max_test
```

Use it only with the rover restrained, conservative current limits, an exclusion zone, and an independent way to remove motor power. It is not registered as an automated test.

## CAN simulation

Run `qpl_can_sim` to create a virtual adapter and simulated devices, then launch the controller stack against `/dev/ttyUSB0`. The simulator validates protocol integration but not real hardware safety.

## Development notes

- Keep the main rover Xacro, simulator IDs, and hardware protocol in sync.
- New safety thresholds should be parameters with validated bounds and documented calibration.
- Add automated tests before changing heartbeat, stop, feedback, stall, or runaway logic.
