# Installation and operation

## Supported environment

- Ubuntu 22.04 or WSL 2
- ROS 2 Humble installed at `/opt/ros/humble`
- Bash shell
- A desktop/OpenGL environment for Gazebo and RViz

The helper scripts assume `apt`, `sudo`, and Linux device paths. Native Windows builds are not documented or supported.

## Install

```bash
git clone git@github.com:jsmith-exe/lunabotics.git
cd lunabotics
```

Add to `~/.bashrc`:

```bash
export QPL_PROJECT="$HOME/lunabotics"
source "$QPL_PROJECT/process/startup.sh"
```

Open a fresh shell, then install dependencies:

```bash
qpl_packages
qpl_gazebo_packages  # for simulation
```

The package script is the current dependency source of truth. Package manifests are not yet complete enough for a clean `rosdep`-only installation.

## Build

```bash
qpl_build
```

This runs:

```bash
cd "$QPL_PROJECT/qpl_ws"
colcon build --symlink-install
source install/setup.bash
```

After changing package metadata, C++, Xacro, launch, or installed data files, rebuild and start a fresh sourced shell. Python source changes are normally visible immediately because the build uses symlinks.

For a clean rebuild:

```bash
qpl_clean_build
```

This deletes only `qpl_ws/build`, `qpl_ws/install`, and `qpl_ws/log` before rebuilding.

## Simulation workflow

Terminal 1:

```bash
qpl_sim
```

Terminal 2:

```bash
qpl_rviz
```

Terminal 3, optional keyboard driving:

```bash
qpl_kb
```

Use `qpl_headless` for Gazebo without a GUI or `qpl_sim_minimal` when testing the world without rover components.

## CAN simulator workflow

```bash
qpl_can_sim
```

This runs `socat` to create `/dev/ttyUSB0` and `/tmp/fake_can_rx`, then starts the simulator UI. Ensure no real adapter is using `/dev/ttyUSB0`. See the package-specific CAN simulator README for details.

## Physical rover workflow

Before launch:

1. Complete the checks in `docs/safety.md`.
2. Verify the serial-to-CAN adapter is the intended `/dev/ttyUSB0` device.
3. Verify CAN IDs and motor directions with wheels clear of the ground.
4. Verify actuator analog feedback and mechanical clearance.
5. Ensure an operator can remove actuator power independently of ROS.

Then run:

```bash
qpl_rover
```

On the base station or another ROS host:

```bash
qpl_rviz_rover
```

Start mission nodes only after localization, controllers, sensor topics, and the stop mechanism have been verified.

## Diagnostics

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /odometry/filtered
```

Record logs with `qpl_logs_rec`. The current destination is `~/rosout_combined.log`.

## Networking

The startup scripts use ROS domain ID 42 and provide Cyclone DDS profiles under `dds/`. The base-station control bridge listens on TCP port 5000. It is unauthenticated and must be used only on a trusted, controlled network.

## Shutdown

Stop mission nodes first, confirm drive and drum commands are zero, then stop bring-up. The CAN plugin sends stop commands during normal ros2_control deactivation, but normal shutdown is not a substitute for a physical power-removal device.
