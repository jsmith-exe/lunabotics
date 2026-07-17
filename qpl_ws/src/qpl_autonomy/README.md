# qpl_autonomy

Mission-level ROS 2 nodes for navigating, excavating, and depositing material with the QPL rover.

## Nodes

| Executable | Purpose |
| --- | --- |
| `autonomy_node` | Alternates navigation between excavation and construction waypoints |
| `excavation_node` | Navigates to excavation, lowers/spins the drum using joint feedback, then raises it |
| `deposition_node` | Navigates to construction and reverse-spins the drum to deposit material |
| `full_autonomy_node` | Combined navigation, excavation, transport, and deposition state machine |
| `blind_excavation_node` | Open-loop timed forward excavation without Nav2 |
| `blind_construction_node` | Open-loop timed forward/dump routine without Nav2 |

## Launch

After building and sourcing:

```bash
qpl_autonomy
qpl_excavate
qpl_construct
qpl_full_auto
```

Blind routines:

```bash
qpl_blind_excavate
qpl_blind_construct
```

Blind routines assume the rover is correctly positioned with a clear straight path. They have no obstacle interlock and must be treated as controlled test utilities.

## Navigation behavior

`NavigationManager` sends `nav2_msgs/action/NavigateToPose` goals on `navigate_to_pose` in the `map` frame. Its default goal timeout is 120 seconds. Rejected, aborted, failed, or timed-out goals are surfaced to the state machine. Mission nodes retry up to three times, then enter or remain in a hold condition instead of treating failure as arrival.

The manager waits for the Nav2 action server during initialization and logs every five seconds until it appears.

## Waypoints

`config/waypoints.yaml` is installed with the package and selected through the `config_path` parameter in launch files. Coordinates and yaw values are in the `map` frame. Keep them consistent with the arena world, masks, static AprilTag transform, and RViz zone overlay.

## Mechanism commands

- `/drum_spin_control/autonomy`: normalized duty request, normally +1 for excavation and −1 for deposition.
- `/drum_lift_control/autonomy`: −1 for lowered and +1 for raised; `qpl_rover` converts this to normalized actuator position.
- `/joint_states`: actuator position feedback used to detect lift completion.

Current tuned values are constants in the Python modules, including lift tolerance, nominal lift durations, drum duty, and maximum navigation retries. Blind speed and duration are ROS parameters.

## Shutdown and faults

Drive commands from blind nodes eventually expire through `twist_mux` when their publisher stops. Mission nodes do not currently have a uniform shutdown hook that explicitly publishes safe drive and mechanism commands. Always stop mission nodes before rover bring-up and confirm zero motion.

No automated autonomy tests currently exist. Test state transitions, action failure, missing joint feedback, clock loss, and shutdown before field operation.
