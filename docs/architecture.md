# System architecture

## Overview

The current implementation has two application computers and a hardware bus:

1. The rover computer runs ROS 2, localization, navigation, mission nodes, ros2_control, and camera drivers.
2. The base-station computer captures operator input and sends commands to a rover-side ROS publisher over TCP.
3. The ros2_control hardware plugin communicates with SPARK MAX devices through a serial-to-CAN adapter.

The source tree does not contain the microcontroller/UART firmware described by older diagrams. If an external microcontroller remains part of the physical rover, its firmware and interface are maintained elsewhere and are not verified here.

## Command flow

### Drive

```text
Controller or keyboard
  -> base-station state
  -> TCP port 5000
  -> basestation nav_pub
  -> /cmd_vel_teleop
                         \
Nav2 -> /cmd_vel_nav -----> twist_mux
                            -> /diff_cont/cmd_vel_unstamped
                            -> diff_drive_controller
                            -> wheel velocity interfaces
                            -> diffdrive_canbus plugin
                            -> serial-to-CAN adapter
                            -> SPARK MAX IDs 1-4
```

Teleop has priority 100 and a 0.15 second mux timeout. Navigation has priority 50 and a 0.25 second timeout. The TCP-to-ROS publisher currently republishes cached messages, so TCP loss alone does not trigger the mux timeout; see `safety.md`.

### Drum and lift

```text
Teleop or autonomy Float64 commands
  -> drum_command_interface
  -> JointGroup controllers
  -> ros2_control command interfaces
  -> diffdrive_canbus plugin
  -> linear actuators IDs 5-6 / drum ID 7
```

Teleop takes priority over autonomy for five seconds after its last message. Drum spin is stopped after 1.5 seconds without an accepted message. Lift is a normalized position target and remains latched.

## Perception and localization

- Front and rear depth camera descriptions are present in URDF and Gazebo.
- Hardware launch includes RealSense and Orbbec driver launch files.
- `apriltag_observer` consumes front and rear color images and camera calibration, publishing `/apriltag/pose`.
- Local and global `robot_localization` EKFs consume configured odometry, IMU, and AprilTag pose data.
- Nav2 uses `/odometry/filtered`, configured costmaps, and the `navigate_to_pose` action.

Exact hardware camera models, serial numbers, calibration validity, and runtime frame names must be verified on the rover.

## Mission autonomy

`qpl_autonomy` supplies finite-state machines for:

- navigation between excavation and construction waypoints;
- excavation and deposition stages;
- a combined full mission;
- open-loop blind excavation and construction tests.

Navigation goals have a default 120 second application timeout. Failed, rejected, aborted, or timed-out goals are retried up to three times before entering `HOLD`.

## Simulation

Gazebo uses the same rover description and controller names, replacing the real CAN hardware plugin with `gazebo_ros2_control/GazeboSystem`. The CAN simulator is separate: it creates paired pseudo-terminals and simulates CAN device traffic expected by the real hardware plugin.

Simulation is not evidence that real motor limits, actuator calibration, watchdogs, or emergency stopping are correct.
