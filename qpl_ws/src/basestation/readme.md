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
- Can configure window scale via `QPL_TELEOP_WINDOW_SCALE` environment variable.

## RViz
We use RViz to visualise the robot state and view camera feeds.

Launch with `qpl_rviz`. We have two separate configurations, default and rover,
which can be configured in [the launch file](./launch/rviz.launch.py).

The [zone overlay node](./basestation/nodes/zone_overlay_node.py) publishes an overlay to be displayed on RViz.

## Teleop HUD (browser display)
A browser-based teleop display, an alternative to RViz for driving: one window with both cameras, the
arena map and the rover telemetry. **It only monitors** and never publishes a command, so you keep
driving with the controller exactly as before.

```bash
qpl_hud_rover   # against the rover: decodes the h264 (ffmpeg) camera feeds locally
qpl_hud         # against the sim: Gazebo already publishes JPEG
```
Both open http://localhost:8765. Press `H` on the page for the key list. The HUD keys avoid W/A/S/D
and the arrow keys so they never look like drive keys.

What's on screen:
- **Main camera** with drive guides: the predicted footprint sweep from the current command, projected
  through the real `camera_info` and TF, with 0.5/1/1.5/2 m rungs. Also a heading tape, ground speed,
  and a PIVOT cue when turning on the spot. Click the corner camera or press Space to swap; `V`
  switches to the rear camera automatically while reversing.
- **Tactical map**: arena zones from `qpl_rover/config/arena`, the global costmap, the Nav2 plan, the
  rover footprint and trail, AprilTag fixes, and the distance and bearing to the berm. Scroll to zoom,
  drag to pan, `F` to follow, `R` to rotate.
- **Header**: who has control (TELEOP / AUTONOMY / IDLE, from the drive mux inputs), three health
  verdicts (LINK, DRIVE, LOCALISE), the current zone, and a run timer (`T`).
- **Bottom strip**: commanded vs measured motion, wheel surface speeds with stall detection, pitch/roll,
  drum and lift, and a 30 s velocity trace.
- **Warnings**: telemetry loss, tilt, wall proximity / out of bounds, wheel stall, and a silent mux
  output. `L` opens per-topic rates and the `/rosout` log.

How it is built: `basestation/hud/hud_node.py` (node `teleop_hud`) subscribes to the topics and serves
the page in `hud/` with the Python standard library. Telemetry goes over Server-Sent Events and the
cameras over MJPEG, so it needs no rosbridge or extra pip packages. Thresholds (tilt, stall, staleness)
are at the top of `hud/js/util.js` and `hud/js/main.js`.

No rover? `qpl_hud_demo` publishes a fake rover (including camera frames) for trying the HUD out. Run it
on a private `ROS_DOMAIN_ID`; it refuses to start if anything else is on the command topics.

