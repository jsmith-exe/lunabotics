# QPL ROS 2 workspace

This directory is the ROS 2 Humble workspace for the rover. All packages are under `src`; generated `build`, `install`, and `log` directories must not be edited or committed.

## Packages

| Package | Build type | Purpose |
| --- | --- | --- |
| `qpl_rover` | `ament_python` | Rover description, configuration, launch, localization, cameras, simulation, and command arbitration |
| `qpl_autonomy` | `ament_python` | Mission state machines and Nav2 action client |
| `basestation` | `ament_python` | Operator control, TCP/ROS bridge, RViz, and logging |
| `can_sim` | `ament_python` | CAN device simulator and UI |
| `diffdrive_canbus` | `ament_cmake` | ros2_control hardware plugin and SPARK MAX protocol |
| `serial` | `ament_cmake` | Vendored cross-platform serial library |

## Environment

The repository helper functions require:

```bash
export QPL_PROJECT="$HOME/lunabotics"
source "$QPL_PROJECT/process/startup.sh"
```

`startup.sh` sources ROS 2 Humble, sources this workspace if it has been built, loads helper functions, sets the Gazebo model path, and starts the ROS daemon.

## Install and build

```bash
qpl_packages
qpl_gazebo_packages  # simulation only
qpl_build
```

Equivalent manual build:

```bash
source /opt/ros/humble/setup.bash
cd "$QPL_PROJECT/qpl_ws"
colcon build --symlink-install
source install/setup.bash
```

Use `qpl_clean_build` when generated build state must be discarded.

## Launch

### Simulation

```bash
qpl_sim
qpl_rviz
qpl_kb       # optional testing teleop
```

### Physical rover

```bash
qpl_rover
qpl_rviz_rover
```

Complete the root `docs/safety.md` checklist before energizing hardware.

### Mission nodes

```bash
qpl_autonomy
qpl_excavate
qpl_construct
qpl_full_auto
```

The `qpl_blind_excavate` and `qpl_blind_construct` routines are open-loop tests and require a controlled area and independent stop mechanism.

### CAN simulator

```bash
qpl_can_sim
```

This requires `socat` and creates `/dev/ttyUSB0`; do not use it while a real adapter occupies that path.

## Controller flow

- Teleop publishes `cmd_vel_teleop`.
- Nav2 and blind mission nodes publish `cmd_vel_nav`.
- `twist_mux` prioritizes teleop and outputs to `/diff_cont/cmd_vel_unstamped`.
- `diff_cont` exposes four velocity command interfaces.
- Drum and lift commands pass through `drum_command_interface` to joint-group controllers.

The complete interface inventory is in the root `docs/ros-interfaces.md`.

## Navigation and localization

`components.launch.py` starts controllers, local odometry fusion, and global map localization. Nav2 configuration is in `qpl_rover/config/nav_params.yaml`. AprilTag global pose observations are published on `/apriltag/pose`.

Use the runtime graph to diagnose target-specific differences:

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 control list_controllers
```

## Tests

```bash
colcon test
colcon test-result --verbose
```

Only the vendored serial package currently has automated tests. See the root `docs/testing.md` for coverage and the recommended backlog.

## Adding dependencies

Add ROS dependencies to the package's `package.xml`, Python package dependencies to its packaging metadata where appropriate, and system installation requirements to `process/functions/install_packages.sh`. Rebuild from a clean environment to verify that the declaration is complete.

## Further documentation

Start at the repository root [README](../README.md) and the files under [docs](../docs).
