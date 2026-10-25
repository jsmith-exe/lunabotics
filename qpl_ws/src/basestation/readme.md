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
qpl_hud_rover   # against the rover
qpl_hud         # against the sim (same, with use_sim_time)
```
Both open http://localhost:8765. Press `H` on the page for the key list. The HUD keys avoid W/A/S/D
and the arrow keys so they never look like drive keys.

What's on screen:
- **Main camera** with drive guides: the predicted footprint sweep from the current command, projected
  through the real `camera_info` and TF, with 0.5/1/1.5/2 m rungs. Also a heading tape, ground speed,
  and a PIVOT cue when turning on the spot. Click the corner camera or press Space to swap; `V`
  switches to the rear camera automatically while reversing. `1` / `2` (or the CAM 1 / CAM 2 chips)
  turn the front / rear stream off and on; off stops it crossing the link.
- **Tactical map**: arena zones from `qpl_rover/config/arena`, the global costmap, the Nav2 plan, the
  rover footprint and trail, AprilTag fixes, and the distance and bearing to the berm. Scroll to zoom,
  drag to pan, `F` to follow, `R` to rotate, `K` (COST) to drop the costmap and its subscription.
- **Header**: who has control (TELEOP / AUTONOMY / IDLE, from the drive mux inputs), three health
  verdicts (LINK, DRIVE, LOCALISE), the current zone, total link use (upload + download against the
  combined budget; amber above 85%, red over), and a run timer (`T`).
- **Bottom strip**: commanded vs measured motion, wheel surface speeds with stall detection, pitch/roll,
  drum and lift, and a 30 s velocity trace.
- **Warnings**: telemetry loss, tilt, wall proximity / out of bounds, wheel stall, a silent mux
  output, and the link over budget.
- **System drawer** (`L`, or click the health verdicts): link use, per-topic rates and sizes, and the
  `/rosout` log. Both
  scroll; drag the bar between them to resize (double-click resets). **TELEMETRY ON/OFF** stops the
  HUD subscribing to `/rosout`, so a chatty rover's logs stop using link bandwidth; it also stops the
  per-topic list. Health verdicts and warnings keep working. Start with it off using
  `telemetry:=false`.

#### Link bandwidth
The rover -> basestation budget is 3.6 Mbit/s, with 0.4 Mbit/s the other way: 4 Mbit/s in total. The
header's **LINK** box is the sum of both directions against that total, and the drawer's **LINK**
line splits it by direction. Both are the whole network interface (`link_interface:=`, default the
one with the default route), so they are what the limiter sees: video, every ROS topic, DDS discovery, and anything else on that interface. Per-topic KB/s is
the serialised payload of what the HUD subscribes to; DDS/UDP headers add ~80 B a message.

What the HUD pulls, roughly: each camera up to 1.3 Mbit/s (the encoder's `bit_rate`), the global
costmap ~0.6 Mbit/s while Nav2 runs, `/joint_states` ~0.4 Mbit/s, the two EKF odometry topics
~0.2-0.3 Mbit/s each. To stay inside the budget:
- The HUD only takes the h264 teleop streams, never `image_raw` or `image_raw/compressed`.
- Poses come from `/odometry/filtered` (odom) and `/odometry/global` (map), not `/tf`, which also
  carries every wheel/drum/lift joint at the joint_states rate. Only `/tf_static` is used, for the
  camera extrinsics.
- Cameras (`1`/`2`, `front_camera:=false`, `rear_camera:=false`), the costmap (`K`, `costmap:=false`)
  and telemetry can each be switched off, which drops the subscription.
- Don't run RViz alongside the HUD: with multicast off, each process gets its own copy of every stream.

Cameras are the same h264 teleop streams RViz shows, `/depth_camera_{front,rear}/color/teleop_stream/ffmpeg`
(see [rover_cameras](../rover_cameras/README.md)), on the rover and in sim. `teleop_hud` starts an
`image_transport` republisher per camera (`hud_front_decoder`, `hud_rear_decoder`, see
`basestation/hud/decoder.py`) that decodes them in software on the laptop to
`/hud/{front,rear}_cam/compressed`, which the HUD serves as MJPEG. Switching a camera off stops its
decoder, and the encoder only runs while something is watching, so the rover saves the encoding too.
Like RViz, the decoders set the h264 decoder to `h264`; NVIDIA's `h264_cuvid` lags and can turn the
picture green.

The drum lift bars span the actuator travel in
[can_config.yaml](../diffdrive_canbus/hardware/can_config.yaml) (`min_lift_mm`..`max_lift_mm`) and read in mm.
A lift command of 0 means "stop and hold", so the HUD shows it as `TGT HOLD`, not as a target.

How it is built: `basestation/hud/hud_node.py` (node `teleop_hud`) subscribes to the topics and serves
the page in `hud/` with the Python standard library. Telemetry goes over Server-Sent Events and the
cameras over MJPEG, so it needs no rosbridge or extra pip packages. Thresholds (tilt, stall, staleness)
are at the top of `hud/js/util.js` and `hud/js/main.js`.

No rover? `qpl_hud_demo` publishes a fake rover (including camera frames, straight onto
`/hud/{front,rear}_cam/compressed`) for trying the HUD out. Run it on a private `ROS_DOMAIN_ID` with
`ros2 launch basestation hud.launch.py`; it refuses to start if anything else is on the command topics. For the real h264 path without cameras, add
`ros2 launch rover_cameras camera_sim.launch.py` and launch the HUD normally.

#### Troubleshooting the HUD
For general ROS, network, camera and CAN problems, see [DnR.md](../../DnR.md). Below are only the
HUD-specific checks. Most HUD problems show up in the `qpl_hud*` terminal, so read that first.

| Symptom | Check |
|---|---|
| **HUD SERVER UNREACHABLE** | `teleop_hud` has stopped. Look at its terminal. `Address already in use` means another HUD (or something else) is on port 8765: close it, or use `port:=8766`. |
| **NO SIGNAL** on a camera | **CAMERA OFF**: press `1`/`2`. Otherwise the overlay names the topic it is waiting on; work back along the chain below. |
| **LINK OVER BUDGET** | Open the drawer (`L`): the LINK line against the per-topic sizes shows what's using it. Turn off a camera, the costmap or telemetry; close RViz or a second HUD. |
| Picture lags, or turns green | The decoder is not software `h264`: `ros2 param get /hud_front_decoder in.ffmpeg.decoders.h264` should say `h264`. The `hud_*_decoder` startup lines say which decoder opened. For blocky or smeared video, see rover_cameras' README. |
| Drive guides say **GUIDES ESTIMATED · NO CAMERA_INFO/TF** | The HUD has no `<camera>/color/camera_info`, or no `/tf_static` chain from `base_footprint` to that camera's frame. It logs `... camera model resolved from <frame>` once it has both. |
| **AWAITING TELEMETRY** / **NO ROVER DATA YET** | The page is fine, but nothing has arrived on `/odometry/filtered` or `/joint_states`. This is usually ROS discovery: see DnR.md's ROS networking section, and check `ROS_DOMAIN_ID` matches the rover. |
| CONTROL stays **IDLE** while driving | Nothing on `/cmd_vel_teleop`. Is teleop enabled in the teleop window? |
| **MUX OUTPUT SILENT** | Commands arrive but `/diff_cont/cmd_vel_unstamped` is quiet. Check twist_mux on the rover. |
| **NO MAP FIX** / LOCALISE not approved | Nothing live on `/odometry/global` (the global EKF's map → `base_footprint` pose), so the map falls back to odom. Localisation (AprilTag/EKF) isn't running, or hasn't seen a tag yet. |
| Map has no arena or zones | Look in the log for `Could not load arena config` (`qpl_rover/config/arena/selector.yaml`). |
| Lift bars use the wrong range | Look in the log for `Could not load lift limits`. The HUD reads `can_config.yaml` from the source tree, like `qpl_teleop`, so it needs a `--symlink-install` build. If the bars disagree with the real actuator, the feedback calibration is `RAW_MIN/MAX` and `DISTANCE_MIN/MAX_MM` in `diffdrive_canbus/hardware/diffdrive_interface/actuator.cpp`. |
| Event log is empty | Telemetry is off (drawer button), or the INFO+/WARN+/ERROR filter hides the lines. The HUD's own log lines are never shown. |

Camera chain, rover to page. The rover's stream encoder only encodes while something is watching, so
`ros2 topic hz` on a stream starts it up too:
```bash
ros2 topic hz /depth_camera_front/color/teleop_stream/ffmpeg   # rover -> laptop. Nothing: rover_cameras side
ros2 topic hz /hud/front_cam/compressed                        # laptop decoder. Nothing: see hud_front_decoder's output
curl -so /dev/null -w '%{http_code}\n' localhost:8765/cam/front.jpg   # 200: HUD has a frame; 204: none yet
```
In the browser, `window.__hud` in the devtools console holds the HUD's live state, and
`curl -N localhost:8765/events` shows the raw telemetry stream.

