# Base station

The `basestation` package provides operator controls, TCP forwarding, a rover-side ROS command publisher, logging, RViz configuration, and arena overlays.

## Architecture

The operator application can run on Windows while `nav_pub` runs in the ROS 2/WSL environment:

```text
DualSense/desktop UI
  -> BaseStationState
  -> TCP transmitter
  -> TCP port 5000
  -> nav_pub
  -> ROS topics
```

Published topics:

- `/cmd_vel_teleop` (`geometry_msgs/Twist`)
- `/drum_spin_control/teleop` (`std_msgs/Float64`)
- `/drum_lift_control/teleop` (`std_msgs/Float64`)

The physical controller implementation currently targets a Sony DualSense through `pydualsense`; older references to an Xbox controller are obsolete.

## ROS-side setup

Build and source the workspace, then run:

```bash
ros2 run basestation nav_pub
```

The receiver binds `0.0.0.0:5000`. Restrict access to a trusted network. There is currently no authentication or encryption.

For a basic diagnostic subscriber:

```bash
ros2 run basestation nav_sub
```

This prints `/cmd_vel_teleop` messages and is not an automated test.

## Operator application

The package contains `basestation.main`, UI classes, desktop controls, and a DualSense controller implementation. Windows helper scripts are provided in the package and repository root. Ensure the TCP host points to the ROS environment reachable from Windows.

The application uses semicolon-delimited JSON records over TCP. The transport does not currently provide robust length framing, message timestamps, sequence numbers, authentication, or a heartbeat.

## Known command-loss limitation

`nav_pub` caches the most recent command and republishes it at 2 Hz. If TCP disconnects, the cached nonzero drive or drum command continues to be published. This prevents the rover twist mux from detecting a missing teleop publisher.

Do not treat network loss as an emergency stop. Maintain an independent physical stop mechanism and address this limitation before operational deployment.

## RViz

```bash
qpl_rviz        # simulation time
qpl_rviz_rover  # wall time
```

The launch starts RViz using the package configuration and publishes `/zone_overlay` markers for arena zones and the berm target.

## Logging

```bash
qpl_logs_rec
```

This subscribes `/rosout` and appends to `~/rosout_combined.log`. The path is currently fixed in source. `log_stress_test` is a manual utility for generating log traffic.

## Constants

Current code fixes the following values in `basestation/constants.py`:

- publish rate: 2 Hz
- reverse steering inversion: enabled
- motor throttle button factor: 0.8
- joystick dead zone: 0.1
- minimum analogue change: 0.05

Changes should be tested with both desktop and physical controllers.
