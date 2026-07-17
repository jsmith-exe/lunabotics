# Safety and failure behavior

## Status

The repository supports simulation and supervised development. It does not by itself establish that the rover is safe for unattended or competition operation.

## Confirmed software protections

- `twist_mux` times out missing teleop and navigation messages.
- Teleop commands have higher mux priority than navigation commands.
- Non-finite wheel commands are converted to zero.
- Drum and actuator duty values are clamped to −1 through 1 in the hardware layer.
- The actuator servo stops when analog feedback is stale for 500 ms.
- Actuator motion is latched off after 1.5 seconds without sufficient progress.
- Wheel telemetry has a conservative runaway detector and stop latch.
- Navigation goals time out, are cancelled, and are retried before the mission enters `HOLD`.
- Normal hardware deactivation sends stop commands before disconnecting the serial bus.

## Known hazards and limitations

### TCP loss can preserve motion

The base-station ROS publisher caches and republishes the last TCP command at 2 Hz. If TCP disconnects while a nonzero command is cached, ROS messages continue and the twist-mux timeout does not activate. Treat the network path as lacking a deadman/watchdog until this behavior is changed and tested.

### No verified independent E-stop

The drive-mux configuration contains a commented example stop topic, but no active software E-stop input. A teleop override is not an independent emergency stop because it shares the computers, ROS graph, controllers, serial adapter, CAN bus, and power electronics with normal control.

The repository does not document a physical contactor or other power-removal mechanism. Verify the physical rover separately.

### Limits are distributed

The diff-drive controller limits normal linear and angular velocity to 1.0. Wheel interfaces declare ±10 rad/s, but the custom hardware plugin does not enforce a wheel velocity clamp. Never rely on one upstream controller as the only motor-speed limit.

Acceleration limits are currently disabled.

### Open-loop blind modes

Blind excavation and construction drive at a fixed speed for a fixed time without localization or obstacle interlocks. Use only in a controlled test area with a clear path, an attentive operator, and an independent stop mechanism.

### Actuator assumptions

Linear actuators use analog position feedback and software stall detection. Code comments state that physical end-stop limit switches are not used. Incorrect calibration, wiring, or feedback polarity can therefore produce hazardous motion.

## Pre-operation checklist

- [ ] Physical E-stop/power isolation has been inspected and proof-tested.
- [ ] Rover is restrained or wheels are raised for initial command tests.
- [ ] Correct serial adapter identity has been verified, not only its device number.
- [ ] CAN IDs 1-7 match the connected devices.
- [ ] Wheel direction, encoder direction, and zero-command behavior are correct.
- [ ] Motor-controller current limits, brake/coast state, PID settings, and native watchdog are verified.
- [ ] Linear-actuator feedback moves in the expected direction and stays within calibrated voltage limits.
- [ ] Actuator mechanical travel and stall behavior have been tested at low risk.
- [ ] Teleop, navigation, and drum stale-command behavior has been demonstrated.
- [ ] Controller, localization, camera, and joint-state health is visible to the operator.
- [ ] The operating network is trusted and TCP port 5000 is not exposed unnecessarily.
- [ ] Test area is clear and a spotter has an unobstructed view.

## Required fault-injection tests

Before autonomous operation, record the observed result of:

1. Disconnecting the base-station TCP client during drive and drum motion.
2. Killing the teleop publisher, twist mux, drive controller, and controller manager separately.
3. Unplugging USB serial during motion.
4. Removing CAN communication or powering down one motor controller.
5. Freezing or disconnecting actuator feedback.
6. Stalling an actuator using a safe test fixture.
7. Sending NaN, infinity, out-of-range, and excessive velocity commands.
8. Losing Nav2, localization, camera, IMU, and `/joint_states` during each mission state.
9. Interrupting each launch and autonomy executable with SIGINT/SIGTERM.
10. Activating the physical E-stop at representative loads and speeds.

## Values requiring physical validation

- Wheel radius 0.15 m and controller separation 0.5836 m
- Gear ratio 100
- Wheel/interface limit ±10 rad/s
- Linear and angular controller limits 1.0
- Actuator feedback timeout 500 ms
- Actuator stall timeout 1.5 s and motion epsilon 0.02
- Actuator target tolerance 0.03; autonomy tolerance 0.06
- Runaway thresholds 8,000 RPM and 1,500 RPM error
- Drum full-duty commands ±1.0

Changes to these values require a documented calibration or safety rationale.
