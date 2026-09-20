# Basestation
Functionality to be executed on the basestation should be kept in this package.

## Teleop
Keypresses, PlayStation DualSense controller inputs, and GUI slider changes can be parsed and published
to ROS nodes (Twist for navigation, Float64 for drum lift and spin).
Topics are listed in [constants.py](./basestation/constants.py).

- **Keyboard input is only detected when the teleop window is focused.**
- **DualSense controller is not supported on WSL.**

#### Setup
Ensure you've run qpl_packages. Run with `qpl_teleop`.
To use a DualSense controller:
```bash
sudo cp 70-ps5-controller.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### Code structure
The core source code is in basestation (qpl_ws/src/basestation/basestation). This is further split into the following modules:
- **controllers**: parses keyboard, physical controller, and GUI inputs and converts them into commands.
  - `tkinter_keyboard_controller.py`: keyboard input, captured via the teleop window's own Tkinter key
    bindings. Only sees keys while the window has focus; losing focus mid-keypress releases held keys
    immediately as a fail-safe.
  - `physical_controller.py`: PlayStation DualSense controller input.
- **nodes**: contains ROS nodes that the basestation uses, including `teleop_publisher.py`, the node that
  publishes teleop commands to ROS topics.
- **ui**: the teleop control window.
- **main.py**: wires the controllers, the window, and the ROS publisher node together, and spins the node.

Some highlights:
- Controls are configured in [control_maps.py](./basestation/control_maps.py).
- While teleop is enabled (toggleable via the UI), it will republish the previous controller state.
This is due to the receivers (the Twist mux and drum command interface) requiring constant input, else it will forward zeros.
This is a security mechanism to avoid loss of control in the event of network problems.

## RViz
We use RViz to visualise the robot state and view camera feeds.

Launch with `qpl_rviz`. We have two separate configurations, default and rover,
which can be configured in [the launch file](./launch/rviz.launch.py).

The [zone overlay node](./basestation/nodes/zone_overlay_node.py) publishes an overlay to be displayed on RViz.
