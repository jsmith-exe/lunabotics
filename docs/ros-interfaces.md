# ROS 2 interfaces

This document lists project-authored interfaces and the principal standard interfaces configured by launch files. Standard parameter and lifecycle services created automatically by ROS nodes are omitted.

## Project nodes

| Node/executable | Subscribes | Publishes or calls | Parameters |
| --- | --- | --- | --- |
| `apriltag_observer` | Front/rear color image and camera-info topics | `/apriltag/pose` (`PoseWithCovarianceStamped`) | `use_sim_time` |
| `drum_command_interface` | `/drum_lift_control/{teleop,autonomy}`, `/drum_spin_control/{teleop,autonomy}` | `/drum_lift_cont/commands`, `/drum_cont/commands` | None; timing is currently hard-coded |
| `autonomy_node` | None | `navigate_to_pose` action | `config_path` |
| `excavation_node` | `/joint_states` | Drum/lift autonomy topics; `navigate_to_pose` | `config_path` |
| `deposition_node` | None | Drum autonomy topic; `navigate_to_pose` | `config_path` |
| `full_autonomy_node` | `/joint_states` | Drum/lift autonomy topics; `navigate_to_pose` | `config_path` |
| `blind_excavation_node` | `/joint_states` | `/cmd_vel_nav`, drum/lift autonomy topics | `forward_speed`, `drive_duration`, `use_sim_time` |
| `blind_construction_node` | None | `/cmd_vel_nav`, drum/lift autonomy topics | `forward_speed`, `drive_duration`, `dump_duration`, `use_sim_time` |
| `nav_pub` | TCP port 5000 | `/cmd_vel_teleop`, drum/lift teleop topics | None |
| `nav_sub` | `/cmd_vel_teleop` | Logs messages | None |
| `log_recorder` | `/rosout` | File output | None |
| `zone_overlay` | None | `/zone_overlay` (`MarkerArray`) | `use_sim_time` |
| `drum_lift_converter` | `cmd_drum_lift` | `/drum_lift_cont/commands` | None; legacy, not in current bring-up |

## Drive mux

| Input | Type | Timeout | Priority |
| --- | --- | --- | --- |
| `cmd_vel_teleop` | `geometry_msgs/Twist` | 0.15 s | 100 |
| `cmd_vel_nav` | `geometry_msgs/Twist` | 0.25 s | 50 |

Output is remapped to `/diff_cont/cmd_vel_unstamped`. The optional `cmd_vel_stop` input is present only as commented configuration and is not active.

## Controllers

| Controller | Type | Interface/topic |
| --- | --- | --- |
| `diff_cont` | `diff_drive_controller/DiffDriveController` | `/diff_cont/cmd_vel_unstamped`; four wheel velocity interfaces |
| `joint_broad` | `joint_state_broadcaster/JointStateBroadcaster` | `/joint_states` |
| `drum_lift_cont` | `position_controllers/JointGroupPositionController` | `/drum_lift_cont/commands`; two position interfaces |
| `drum_cont` | `velocity_controllers/JointGroupVelocityController` | `/drum_cont/commands`; drum velocity interface interpreted as duty cycle by real hardware |

## Hardware interfaces

| Joint | Command | State |
| --- | --- | --- |
| `front_left_wheel_joint` | velocity | position, velocity |
| `front_right_wheel_joint` | velocity | position, velocity |
| `rear_left_wheel_joint` | velocity | position, velocity |
| `rear_right_wheel_joint` | velocity | position, velocity |
| `left_linear_actuator_joint` | normalized position 0-1 | position |
| `right_linear_actuator_joint` | normalized position 0-1 | position |
| `drum_spin_joint` | velocity/duty −1 to 1 | position, velocity |

## CAN assignment

| ID | Device |
| --- | --- |
| 1 | Front-left wheel |
| 2 | Front-right wheel |
| 3 | Rear-left wheel |
| 4 | Rear-right wheel |
| 5 | Left linear actuator |
| 6 | Right linear actuator |
| 7 | Drum |

Main rover configuration uses `/dev/ttyUSB0`, 2,000,000 serial baud, and 1,000,000 CAN baud. A demo description under `diffdrive_canbus/description` contains different values and is not the authoritative rover configuration.

## External interfaces

Launch files also configure Nav2, robot state publisher, local/global EKFs, camera drivers, Gazebo, RViz, TF publishers, controller manager, and topic relays. Use `ros2 node list`, `ros2 topic list`, `ros2 service list`, and `ros2 action list` on a running target to capture the complete generated graph.
