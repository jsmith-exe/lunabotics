# qpl_rover

Main rover bring-up package. It installs the robot description, Gazebo worlds, controller and localization configuration, launch files, calibration files, and two project nodes.

## Launch files

- `rover.launch.py`: physical rover description, components, RealSense/Orbbec cameras, and camera TF compatibility transforms.
- `sim.launch.py`: Gazebo world, rover spawn, simulated components, and camera topic relays.
- `components.launch.py`: controllers plus local and global localization.
- `spare.launch.py`: intentionally empty emergency development placeholder.
- `launch/components/controllers.launch.py`: controller manager, twist mux, drum interface, and controller spawners.
- `launch/components/odom_localisation.launch.py`: local EKF.
- `launch/components/map_localisation.launch.py`: AprilTag observer, static map/tag transform, and global EKF.
- `launch/components/navigation_launch.py`: Nav2 bring-up using package parameters.

Repository helpers provide `qpl_rover`, `qpl_sim`, `qpl_components`, and RViz commands. See the root README for the supported workflow.

## Project nodes

### `apriltag_observer`

Consumes front and rear color images plus camera information and publishes a map-localization observation on `/apriltag/pose`. It requires OpenCV, `cv_bridge`, and `pupil_apriltags`.

### `drum_command_interface`

Arbitrates teleop and autonomy lift/spin commands, giving teleop priority for five seconds. It converts lift input from −1…1 into normalized position 0…1 and publishes joint-group controller commands. Drum spin is set to zero after 1.5 seconds without accepted input; lift position remains latched.

## Configuration

- `my_controllers.yaml`: four-wheel diff drive, joint states, drum lift, and drum spin controllers.
- `drive_mux.yaml`: teleop/navigation priority and stale timeouts.
- `nav_params.yaml`: Nav2 servers, planners, controllers, costmaps, and behavior configuration.
- `ekf_*`: simulation and hardware local/global localization.
- `ros2_control.xacro`: real CAN hardware or Gazebo interfaces.
- camera calibration YAML files: rear-camera calibration snapshots; applicability must be verified against the connected camera and resolution.

## Real hardware

The real ros2_control description expects a serial-to-CAN adapter at `/dev/ttyUSB0` and loads `diffdrive_canbus/DiffDriveCanbusHardware`. CAN IDs, rates, actuator settings, and limitations are documented in the `diffdrive_canbus` README and root safety guide.

## Simulation

Gazebo replaces the real hardware plugin with `gazebo_ros2_control/GazeboSystem`. Available worlds include arena, AprilTag, obstacle, cone, heightmap, and real-world approximations. `arena_april.world` is currently selected in `sim.launch.py`.

Simulation camera relays normalize Gazebo topics into the names expected by the AprilTag observer. This does not prove that hardware camera frame names or calibration match.

## Safety notes

- The optional E-stop mux topic is not active.
- Acceleration limits are disabled.
- Controller spawners use fixed startup delays.
- Hardware readiness is not currently gated on camera, localization, or feedback health.
- Complete the root `docs/safety.md` checklist before physical operation.
