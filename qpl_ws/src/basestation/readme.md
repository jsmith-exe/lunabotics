# Basestation
Functionality for parsing data from the rover and sending commands should be kept here.

## Current functionality
Keypresses, PlayStation DualSense controller inputs, and GUI slider changes are parsed and published
directly as ROS messages (Twist for navigation, Float64 for drum control) - all within a single Linux
process, no forwarding between machines involved.
> Earlier versions of this gathered keyboard input at the native OS level (which could be Windows) and
> forwarded it over TCP to a separate ROS publisher process, since capturing keys used to require an
> OS-level hook. Keyboard input is now captured via the teleop window's own Tkinter key bindings instead,
> which works the same way under WSL (via WSLg) or native Ubuntu, so that split is no longer needed.

**Runs on Linux - tested on WSL (via WSLg for the GUI) and native Ubuntu. PlayStation controller support
has only been tested on Windows/WSL so far.**

## Code structure
The core source code is in basestation (qpl_ws/src/basestation/basestation). This is further split into the following modules:
- **controllers**: parses keyboard, physical controller, and GUI inputs and converts them into commands.
  - `tkinter_keyboard_controller.py`: keyboard input, captured via the teleop window's own Tkinter key
    bindings. Only sees keys while the window has focus; losing focus mid-keypress releases held keys
    immediately as a fail-safe.
  - `physical_controller.py`: PlayStation DualSense controller input.
- **nodes**: contains ROS nodes that the basestation uses, including `teleop_publisher.py`, the node that
  publishes teleop commands to ROS topics.
- **ui**: the Tkinter teleop control window.
- **main.py**: wires the controllers, the window, and the ROS publisher node together, and spins the node.

## Setup
A Python environment is needed, ideally Python 3.10, with the following installed:
```commandline
pip install pydualsense ttkbootstrap PyYAML
```

## Running the controller
```commandline
ros2 run basestation teleop
```
This opens the teleop window, starts listening for a DualSense controller if one's connected, and
publishes to ROS topics. Click the teleop window before using keyboard controls - keys are only captured
while it has focus.

The publisher can be tested by running nav_sub on a device running ROS on the local network:
```commandline
ros2 run basestation nav_sub
```

---
#### Docker documentation
I had planned to support MacOS via Docker, but ran into network issues.
For future reference, I'm documenting how I intended the Docker files to be used; these probably don't work.

Docker desktop should be installed.

The docker-compose.yaml file should contain the configuration needed to run the docker container, by running the command:
```commandline
docker compose up --build
```
This will automatically build the image from the Dockerfile and run the image.
