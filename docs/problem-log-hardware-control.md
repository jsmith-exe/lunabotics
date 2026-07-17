# Problem log: unpredictable linear actuators and uncommanded wheel motion

## Status

- **State:** Open; physical hardware must be treated as unsafe to energize without restraint and independent power isolation.
- **Affected systems:** Linear actuators on CAN IDs 5-6, wheel motors on CAN IDs 1-4, base-station command forwarding, ros2_control hardware interface.
- **Reported symptoms:**
  - The linear actuators could not be controlled predictably.
  - Drive motors sometimes spun when the operator believed no motion command was active.
- **Analysis basis:** Static inspection of the `simulated-canbus` code. No physical rover, controller flash configuration, CAN capture, or historical runtime log was available during this analysis.

The reported behavior is consistent with multiple confirmed software defects. It should not initially be attributed solely to electrical noise or mechanical faults.

## Executive analysis

The most significant mechanisms are:

1. A released/default lift command of zero is converted into a position target of 0.5, causing movement toward half travel rather than stopping.
2. Both actuators initialize to the same 0.5 position target and can move as soon as feedback becomes valid, without an operator command.
3. Actuators are driven using full-power bang-bang control with assumed feedback calibration and polarity.
4. The actuator Status 3 matcher supports only one hard-coded CAN layout even though the general telemetry parser recognizes old and new SPARK MAX layouts.
5. The CAN hardware sends heartbeats while idle without first establishing zero wheel setpoints.
6. The base-station ROS publisher continually republishes its last command after controller or TCP loss.
7. The CAN hardware has no command-age watchdog and does not enforce a wheel-speed limit.
8. The nominal 100 Hz control loop contains enough deliberate sleeps to block for more than 100 ms during an active cycle.

Any one of several items can cause unsafe behavior. Their combination makes the system difficult to diagnose because a stale command, startup target, feedback interpretation error, and loop overrun can occur together.

---

## Linear-actuator findings

### LA-1: Stop and midpoint position are confused

**Classification:** Confirmed; Critical

The base controller sends a value of `0` when a button is released, explicitly describing it as a stop signal:

- `basestation/controls/controllers/base_controller.py`, `handle_button()`

The drum command interface converts a lift input from −1...1 into 0...1 using:

```python
return value / 2 + 0.5
```

The resulting interpretation is:

| Input | Converted command | Actual position-controller meaning |
| --- | --- | --- |
| `-1` | `0.0` | Move to fully retracted |
| `0` | `0.5` | Move to half travel |
| `+1` | `1.0` | Move to fully extended |

The hardware consumes this value through a ros2_control **position** interface. It calculates `target - measured` and drives until the measured position reaches that target. Therefore a released control commands both actuators to half travel; it does not stop them.

Relevant files:

- `qpl_ws/src/basestation/basestation/controls/controllers/base_controller.py`
- `qpl_ws/src/qpl_rover/qpl_rover/drum_command_interface.py`
- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`

The Xacro comments also call `0.5` “stop,” which is true only for an open-loop direction/duty convention. It is false after the value has become a position target:

- `qpl_ws/src/qpl_rover/description/ros2_control.xacro`

**Expected symptom:** Actuators move toward midpoint when a lift button is released, a joystick returns to zero, or a default zero input is published.

### LA-2: Startup commands both actuators toward midpoint

**Classification:** Confirmed; Critical

The initial command for each actuator is configured as `0.5`. Hardware activation resets each actuator command to that initial value. Once feedback is considered fresh, an actuator not already at position 0.5 is driven toward midpoint without a new operator command.

The hardware also substitutes target `0.5` for a non-finite actuator command. This converts an invalid/uninitialized value into physical motion instead of a stopped state.

Relevant files:

- `qpl_ws/src/qpl_rover/description/ros2_control.xacro`
- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `reset_actuator_state()` and `write_actuator_closed_loop()`

**Expected symptom:** Actuators move during controller startup or activation.

### LA-3: Full-power bang-bang control

**Classification:** Confirmed; High

The actuator servo has three outputs:

- `+1.0` full duty when measured position is below the target;
- `-1.0` full duty when measured position is above the target;
- `0.0` only inside the configured tolerance.

There is no proportional slowdown, acceleration limit, approach speed, current/load limit, or braking profile. The left and right actuators run as independent servos; there is no disagreement or synchronization check to prevent the mechanism racking.

Relevant file:

- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `write_actuator_closed_loop()`

**Expected symptoms:** Abrupt starts, overshoot, oscillation around a target, unequal movement, and mechanical twisting.

### LA-4: Feedback calibration and polarity are assumed

**Classification:** Confirmed configuration gap; High

The hardware defaults both actuator feedback ranges to:

- minimum: 0.279 V;
- maximum: 1.85 V.

The rover Xacro does not supply measured per-actuator calibration, so these defaults are used. The control law also assumes:

- voltage increases with extension;
- positive duty extends the actuator;
- both actuators use the same voltage range and polarity.

There are no feedback-inversion or motor-direction parameters for the actuators.

If either polarity assumption is wrong, the control loop becomes positive feedback: it drives full power away from the target until the stall watchdog trips.

Relevant files:

- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `parse_hardware_parameters()`
- `qpl_ws/src/qpl_rover/description/ros2_control.xacro`

**Unverified:** Actual actuator voltages, wiring polarity, travel endpoints, and motor direction.

### LA-5: Status 3 feedback may not match installed SPARK MAX firmware

**Classification:** Confirmed code inconsistency; hardware applicability unverified; High

The general SPARK MAX parser recognizes periodic-status API class 6 for firmware 24 and class 46 for firmware 25+. Actuator analog feedback, however, is accepted only when the CAN ID exactly matches a Status 3 base of `0x020518C0`, representing one layout.

If the installed controllers emit the newer Status 3 layout, actuator feedback does not refresh. The 500 ms stale-feedback protection then holds the actuator stopped. Mixed firmware or intermittent matching traffic could produce inconsistent start/stop behavior.

Relevant files:

- `qpl_ws/src/diffdrive_canbus/hardware/spark_max.cpp`
- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `is_actuator_status3_id()` and `observe_actuator_raw_frame()`

**Unverified:** Installed SPARK MAX firmware and actual Status 3 CAN IDs.

### LA-6: Implausible feedback is not rejected

**Classification:** Confirmed; Medium

Any matching Status 3 voltage is marked valid and refreshes the feedback timestamp. The normalized result is clamped to 0...1. There is no check for:

- voltage outside the calibrated physical range;
- impossible position jumps or rates;
- left/right actuator disagreement;
- intermittent or noisy feedback;
- sensor wiring faults that remain numerically finite.

A bad voltage can therefore be converted to exactly 0 or 1 and cause full-duty motion toward the opposite endpoint.

Relevant file:

- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `normalise_actuator_voltage()` and `observe_actuator_raw_frame()`

---

## Wheel-motor findings

### WM-1: Heartbeat can be sent before a zero setpoint

**Classification:** Confirmed unsafe sequence; Critical

Hardware configuration and activation explicitly send no zero setpoint. Every normal `write()` then calls `maybe_send_heartbeat()` before checking whether wheel commands are zero.

When the software believes the wheels are already idle, it returns without transmitting zero-duty frames. The startup sequence is therefore:

1. Connect and activate without sending zero.
2. Send a global heartbeat.
3. Observe zero ROS command.
4. Send no wheel stop because `motors_currently_commanded_` is already false.

Relevant file:

- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `on_configure()`, `on_activate()`, and `write()`

**Reasonable inference:** If a SPARK MAX retains its previous native velocity setpoint while heartbeat is absent, restarting heartbeat before sending zero can reactivate the retained setpoint. Controller retention behavior must be verified against the installed firmware.

**Expected symptom:** A wheel starts during hardware/controller startup despite a zero ROS command.

### WM-2: Base-station commands remain fresh indefinitely

**Classification:** Confirmed; Critical

`ControlsPublisher` stores the last command for every controlled topic and republishes it at 2 Hz forever. It does not associate a timestamp or maximum age with that state.

If the physical controller, UI, or TCP link disappears while the cached drive command is nonzero, new `/cmd_vel_teleop` messages continue to arrive. The twist mux therefore never observes its 0.15 second teleop timeout.

The physical-controller `stop()` method closes the DualSense connection but does not publish zero first.

Relevant files:

- `qpl_ws/src/basestation/basestation/nodes/controls_publisher.py`
- `qpl_ws/src/basestation/basestation/controls/controllers/physical_controller.py`
- `qpl_ws/src/qpl_rover/config/drive_mux.yaml`

**Expected symptom:** Rover continues moving after controller disconnect, UI failure, or TCP loss, appearing to move “without command.”

### WM-3: TCP disconnect handling does not clear command state

**Classification:** Confirmed; High

A graceful TCP close returns an empty byte string from `recv()`. The receiver does not check for empty data before passing it to the command parser. It therefore does not reliably leave the receive loop, accept a new client, notify `ControlsPublisher`, or clear cached commands.

Relevant file:

- `qpl_ws/src/basestation/basestation/forwarding/tcp_receiver.py`, `_listen_loop()`

### WM-4: No hardware command-age watchdog

**Classification:** Confirmed; High

The CAN hardware knows the current ros2_control command value but not when it was last updated. If a downstream controller or the controller-manager loop freezes while retaining a finite nonzero value, the plugin continues transmitting that command and heartbeat.

The twist mux protects only against missing upstream topic messages. It cannot protect against failures after the mux.

Relevant file:

- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `write()`

### WM-5: Wheel velocity is not clamped in the hardware plugin

**Classification:** Confirmed; High

The plugin converts non-finite wheel commands to zero but allows any finite magnitude. It explicitly reports that the velocity clamp was removed. Commands are multiplied by the gear ratio of 100 and converted into motor RPM before being sent as native velocity setpoints.

The Xacro declares `min_velocity` and `max_velocity`, but the custom hardware layer does not enforce them. A malformed but finite command can therefore become an excessive motor-RPM request.

Relevant files:

- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`
- `qpl_ws/src/qpl_rover/description/ros2_control.xacro`

### WM-6: The configured control rate is impossible for the blocking write path

**Classification:** Confirmed; High

Controller manager is configured for 100 Hz, allowing 10 ms per read-update-write cycle. The CAN implementation deliberately sleeps for 10 ms after outgoing frames.

An active write can spend approximately:

- 20 ms sending two actuator commands;
- 10 ms sending a drum command;
- 80 ms sending a heartbeat/gap and velocity/gap for four wheels.

This is at least approximately 110 ms of deliberate sleeping, excluding feedback reads, serial operations, controller updates, and scheduling. It exceeds the requested period by more than ten times.

Relevant files:

- `qpl_ws/src/qpl_rover/config/my_controllers.yaml`
- `qpl_ws/src/diffdrive_canbus/hardware/diffdrive_canbus_system.cpp`, `sleep_bus_gap()` and `write_one_motor_native_velocity()`

**Expected symptoms:** Delayed stop response, irregular update timing, stale feedback, command bursts, controller-manager overruns, and behavior that appears intermittent.

### WM-7: Required SPARK MAX configuration is not established by software

**Classification:** Confirmed repository gap; High

The code selects native PID slot 0 but does not establish or verify:

- PID gains;
- inversion;
- current limits;
- open-loop or closed-loop ramp rates;
- brake/coast state;
- native watchdog behavior.

Operation therefore depends on whatever configuration was last flashed to each controller. Different stored settings can make nominally identical motors respond differently.

**Unverified:** Actual configuration of each physical controller.

---

## Existing mitigations and their limits

The code contains several useful protections:

- non-finite wheel commands become zero;
- actuator and drum duty commands are clamped to −1...1;
- actuator motion stops when feedback is stale for 500 ms;
- actuator stall detection latches output off after 1.5 seconds without enough movement;
- a wheel runaway detector can send a stop burst;
- normal hardware deactivation sends stop commands;
- twist mux has teleop/navigation timeouts.

These do not resolve the root causes above:

- the twist-mux timeout is defeated by cached republishing;
- the actuator stale-feedback check cannot validate calibration or polarity;
- the stall watchdog still permits up to 1.5 seconds at full duty;
- normal deactivation does not protect startup or process crashes;
- the runaway detector depends on valid, sufficiently fresh telemetry;
- none of these protections is covered by automated tests.

## Information not yet verified

- Installed SPARK MAX firmware versions
- Retained-setpoint behavior after heartbeat loss and restoration
- PID, inversion, current limit, ramp, brake/coast, and watchdog settings
- Actual CAN traffic during an incident
- Actuator potentiometer voltages at both travel endpoints
- Feedback polarity and motor polarity for each actuator
- Mechanical binding, asymmetric load, and actuator synchronization
- USB serial adapter loss/recovery behavior
- Presence and function of physical limit switches or an independent E-stop

## Required diagnostic evidence

Do not begin with free-running hardware. Use restraints, conservative current limits, exclusion zones, and independent power isolation.

Capture the following for one device at a time:

1. ROS command topics and controller command values with timestamps.
2. Hardware logs showing raw command, cleaned command, target RPM/position, measured feedback, duty, and feedback age.
3. Raw CAN frames, especially heartbeat, duty/velocity setpoints, and Status 3 IDs/data.
4. SPARK MAX firmware and complete flashed configuration for every CAN ID.
5. Measured actuator voltage at retracted, midpoint, and extended positions.
6. Confirmed relationship between positive duty, physical direction, and feedback-voltage direction.
7. Controller-manager update-rate and missed-cycle measurements.

## Minimum resolution plan

The incident should remain open until all of the following are implemented and verified:

1. Define one unambiguous actuator command contract: position or velocity/direction, not both.
2. Make startup and invalid actuator commands hold the measured position or send zero duty; never substitute an arbitrary midpoint target.
3. Add per-actuator calibration, motor inversion, feedback inversion, velocity/duty limits, and plausibility checks.
4. Replace full-duty independent control with bounded motion and left/right synchronization protection.
5. Decode and test Status 3 for the exact installed firmware.
6. Send repeated zero setpoints before the first heartbeat and during every idle cycle until safe state is confirmed.
7. Stop and clear cached commands on controller/TCP loss; add timestamps, sequence numbers, and source-age limits.
8. Add a hardware-level command-age watchdog independent of ROS topic freshness.
9. Enforce wheel RPM/velocity and acceleration limits at the lowest practical layer.
10. Remove blocking sleeps from the control loop or lower the configured rate to a measured achievable rate.
11. Program or verify all safety-relevant SPARK MAX configuration at startup.
12. Add automated tests and restrained hardware fault-injection tests for every failure mode above.

## Closure criteria

This problem may be closed only when recorded test evidence demonstrates:

- no actuator movement on startup, control release, invalid command, or communication loss;
- each actuator moves in the requested direction and stops within calibrated bounds;
- left/right disagreement causes a safe stop;
- every wheel receives zero before heartbeat enable and remains stopped without a fresh command;
- controller and TCP disconnect stop drive and drum motion within a specified tested deadline;
- excessive, stale, NaN, and malformed commands are rejected at the hardware boundary;
- controller-manager timing remains within its configured period under worst-case traffic;
- USB loss, CAN loss, node death, and normal shutdown produce a verified safe state;
- an independent physical emergency stop has been proof-tested.

Until those criteria are met, the physical rover should not be operated autonomously or with wheels/actuators free to create hazardous motion.
