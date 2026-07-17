# QPL Lunabotics Rover

ROS 2 software for the QPL Lunabotics rover. The repository contains the rover model and bring-up, custom CAN hardware integration, mission autonomy, operator controls, and a CAN simulator.

The supported development target is **ROS 2 Humble on Ubuntu 22.04 or WSL 2**. Hardware operation additionally requires the configured cameras and a serial-to-CAN adapter at `/dev/ttyUSB0`.

## Repository layout

| Path | Purpose |
| --- | --- |
| `qpl_ws/src/qpl_rover` | Rover URDF, Gazebo worlds, ros2_control configuration, localization, cameras, Nav2 configuration, and launch files |
| `qpl_ws/src/diffdrive_canbus` | Custom ros2_control hardware plugin for SPARK MAX wheel, actuator, and drum controllers |
| `qpl_ws/src/qpl_autonomy` | Navigation, excavation, deposition, and full-mission state machines |
| `qpl_ws/src/basestation` | Operator UI, controller input, TCP forwarding, ROS command publisher, RViz, and logging |
| `qpl_ws/src/can_sim` | Virtual serial/CAN network and device simulator |
| `qpl_ws/src/serial` | Vendored C++ serial library |
| `process` | Environment setup, dependency installation, build helpers, launch aliases, networking, and camera utilities |
| `dds` | Cyclone DDS profiles |
| `docs` | Architecture, interfaces, operation, safety, and testing documentation |

## Quick start

Clone the repository and add the following to `~/.bashrc`, replacing the path if necessary:

```bash
export QPL_PROJECT="$HOME/lunabotics"
source "$QPL_PROJECT/process/startup.sh"
```

Start a new shell, then install dependencies and build:

```bash
qpl_packages
qpl_gazebo_packages   # required for simulation
qpl_build
```

Run the simulator and RViz in separate terminals:

```bash
qpl_sim
qpl_rviz
```

Drive the simulator using the testing keyboard teleop:

```bash
qpl_kb
```

For the physical rover:

```bash
qpl_rover
qpl_rviz_rover
```

Do not run physical hardware until the pre-operation safety checks in [docs/safety.md](docs/safety.md) have been completed.

## Common commands

| Command | Result |
| --- | --- |
| `qpl_build` | Build the workspace using `colcon build --symlink-install` |
| `qpl_clean_build` | Remove generated workspace output and rebuild |
| `qpl_packages` | Install core ROS and system dependencies |
| `qpl_gazebo_packages` | Install Gazebo dependencies |
| `qpl_sim` | Start Gazebo with rover components |
| `qpl_headless` | Start Gazebo without its GUI |
| `qpl_sim_minimal` | Start headless Gazebo without rover components |
| `qpl_rover` | Launch the physical rover stack |
| `qpl_components` | Launch controllers and localization components |
| `qpl_rviz` | Start RViz using simulation time |
| `qpl_rviz_rover` | Start RViz using wall time |
| `qpl_kb` | Keyboard teleop remapped to `cmd_vel_teleop` |
| `qpl_autonomy` | Navigate between excavation and construction zones |
| `qpl_excavate` / `qpl_construct` | Run one mission stage |
| `qpl_full_auto` | Run the combined mission state machine |
| `qpl_blind_excavate` / `qpl_blind_construct` | Run open-loop test routines; see safety documentation |
| `qpl_can_sim` | Start the virtual serial port and CAN simulator |
| `qpl_logs_rec` | Record `/rosout` to `~/rosout_combined.log` |

These commands are shell functions or aliases loaded by `process/startup.sh`; they are not standalone executables.

## Documentation

- [System architecture](docs/architecture.md)
- [ROS 2 nodes and interfaces](docs/ros-interfaces.md)
- [Installation and operation](docs/setup-and-operation.md)
- [Safety and failure behavior](docs/safety.md)
- [Testing and simulation](docs/testing.md)
- [ROS workspace guide](qpl_ws/README.md)
- [Base-station guide](qpl_ws/src/basestation/readme.md)
- [CAN hardware guide](qpl_ws/src/diffdrive_canbus/README.md)
- [CAN simulator guide](qpl_ws/src/can_sim/readme.md)

## Current limitations

- The repository does not contain a verified physical E-stop implementation or electrical wiring documentation.
- The base-station TCP command path currently republishes the last received command; loss-of-link handling must be treated as a known safety limitation.
- Safety-critical autonomy and CAN hardware behavior currently lacks automated test coverage.
- Camera, actuator calibration, motor-controller flash configuration, and mechanical limits must be verified on the physical rover.

See [docs/safety.md](docs/safety.md) for the full operational implications.

## License and ownership

Several packages still contain placeholder or upstream metadata. Confirm project ownership and licensing before redistribution.
