# Testing and simulation

## Current automated coverage

The vendored `serial` package contains C++ Unix serial and timer tests, built when `BUILD_TESTING` is enabled.

No automated tests currently cover the project-authored base-station, autonomy, rover nodes, launch files, CAN protocol, or ros2_control hardware plugin.

`spark_max_test`, `can_sim/class_tests.py`, `nav_sub`, controller utilities, and logging stress utilities are manual diagnostics, not automated regression tests.

## Run available checks

From the workspace:

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

For simulation smoke testing:

```bash
qpl_sim
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /odometry/filtered
```

Then command low-speed motion and verify wheel states, odometry, mux timeout, and stop behavior.

## CAN simulator

The CAN simulator models SPARK MAX command/status traffic through a pseudo-terminal pair. It is useful for protocol development without energizing motors. It is not a physics model and does not validate current limiting, real bus errors, actuator loads, mechanical stops, or motor-controller flash settings.

## Recommended test backlog

1. Unit-test TCP framing, disconnect, reconnect, and command freshness.
2. Unit-test drum source arbitration, normalization, timeout, and safe shutdown.
3. Unit-test CAN ID encoding/decoding and Waveshare serial frames.
4. Unit-test hardware command sanitization and limit enforcement.
5. Test actuator stale feedback, stall latch, target changes, and calibration bounds.
6. Test runaway detection and reset conditions.
7. Test navigation goal rejection, abort, timeout, cancellation, retry, and late results.
8. Test every autonomy FSM transition and shutdown state.
9. Add ROS launch tests for simulation and hardware descriptions.
10. Add a CI build and test workflow for ROS 2 Humble.

Hardware-in-the-loop tests must be performed with suitable restraints, current limits, exclusion zones, and independent power isolation.
