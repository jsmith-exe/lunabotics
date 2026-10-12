# Offline documentation bank

Documentation for everything the rover depends on, for use without internet at the competition.
Download or update it with `./download.sh` (or `./download.sh <section>`; `./download.sh --list` shows
the sections). The default download is about 3.5 GB; `./download.sh web` adds JavaScript/HTML/CSS
references (~2.3 GB). The downloaded files are ignored by git, so run the script on every laptop that needs a
copy, or copy this folder to a USB stick.

## How to search

Each entry below has a **Keywords** line listing the terms you're likely to search for: error messages,
parameter names, file names from our repo. Search this index first, then search inside the entry's folder.

```bash
cd "$QPL_PROJECT/offline_docs"
grep -n -i "socketreceivebuffer" index.md     # which entry covers it?
grep -rli "SocketReceiveBufferSize" dds/       # which files mention it?
grep -rn --include='*.md' --include='*.rst' -i "max_vel_x" nav/ | less
xdg-open dds/cyclonedds-0.10.5-html/cyclonedds.io/docs/cyclonedds/0.10.5/index.html
```

`.md` and `.rst` files are plain text, so `grep` searches them well. `*-html` folders are websites: open
their `index.html` in a browser. The `libs/docsets` folder works best in [Zeal](https://zealdocs.org/)
(`sudo apt install zeal`), which also gives offline search across all the docsets.

## Already on your machine

Check these before the bank, since they always match the installed version:

| What | Command |
|---|---|
| Man pages | `man tc`, `man tc-htb`, `man ip-link`, `man nmcli`, `man udev`, `man systemd.service`, `man ffmpeg-codecs`; `man -k <word>` to search |
| ROS message, service and action definitions | `ros2 interface show nav_msgs/msg/Odometry`, `ros2 interface list` |
| Parameters of a running node | `ros2 param list /node`, `ros2 param describe /node <param>`, `ros2 param dump /node` |
| Launch file arguments | `ros2 launch <pkg> <file> --show-args` |
| ros2_control state | `ros2 control list_controllers`, `ros2 control list_hardware_interfaces` |
| ROS setup health check | `ros2 doctor --report` |
| TF tree | `ros2 run tf2_tools view_frames` (writes `frames_*.pdf`) |
| Every RTAB-Map parameter with its description | `rtabmap --params` (pipe to `grep -i`) |
| Installed packages' default configs and launch files | `/opt/ros/humble/share/<package>/` (e.g. `nav2_bringup/params/nav2_params.yaml`) |
| FFmpeg encoder options | `ffmpeg -h encoder=libx264` |
| libavcodec, OpenCV, yaml-cpp, SocketCAN C API | The headers' doc comments: `/usr/include/x86_64-linux-gnu/libavcodec/avcodec.h`, `/usr/include/opencv4/`, `/usr/include/linux/can.h` (`aarch64-linux-gnu` on the Jetson) |
| Python library docs | `python3 -m pydoc <module>`, or `python3 -m pydoc -b` for a browsable server |

---

## ROS 2 core (`ros/`)

### ROS 2 Humble documentation
- **Path:** `ros/ros2-humble-html/docs.ros.org/en/humble/index.html` (website); `ros/ros2_documentation/source/` (the same pages as `.rst`, for grep)
- **Covers:** concepts, tutorials and how-to guides for ROS 2 Humble: nodes, topics, QoS, launch files, parameters, composition, tf2, colcon, ament, RMW and DDS tuning, troubleshooting.
- **Start with:** `How-To-Guides/DDS-tuning`, `Concepts/Intermediate/About-Quality-of-Service-Settings`, `Tutorials/Intermediate/Composition`, `How-To-Guides/Launch-file-different-formats`, `Tutorials/Beginner-Client-Libraries/Colcon-Tutorial`.
- **Keywords:** ros2, humble, QoS, reliability, best_effort, durability, transient_local, history depth, incompatible QoS, discovery, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW_IMPLEMENTATION, daemon, launch, LaunchConfiguration, DeclareLaunchArgument, IncludeLaunchDescription, ComposableNodeContainer, component, use_sim_time, parameters, yaml, colcon build, symlink-install, ament_cmake, ament_python, setup.py, package.xml, tf2, static_transform_publisher, rosbag2, ros2 bag record, rqt, multicast

### rclpy and rclcpp
- **Path:** `ros/rclpy-api/` (rclpy API reference, website); `ros/rclpy/rclpy/docs/`; `ros/rclcpp/rclcpp/doc/` and the headers in `ros/rclcpp/rclcpp/include/` (for the C++ API, the doc comments in the headers are the reference)
- **Covers:** Python and C++ client library APIs: Node, publishers, subscriptions, timers, executors, callback groups, parameters, logging, clocks.
- **Keywords:** rclpy, rclcpp, Node, create_publisher, create_subscription, create_timer, spin, spin_once, MultiThreadedExecutor, SingleThreadedExecutor, callback group, ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup, declare_parameter, add_on_set_parameters_callback, get_logger, Clock, Duration, Time, QoSProfile, lifecycle, rclcpp_lifecycle, rclcpp_components

### image_transport and ffmpeg_image_transport
- **Path:** `ros/image_common/` (image_transport, camera_info_manager); `ros/ffmpeg_image_transport/README.md` (version 3.0.4, the one installed)
- **Used in:** `rover_cameras` (StreamEncoder publishes `FFMPEGPacket`), `basestation` HUD and RViz decoding, `rviz.launch.py` decoder parameter.
- **Keywords:** image_transport, compressed, compressedDepth, ffmpeg, FFMPEGPacket, ffmpeg_image_transport_msgs, h264, hevc, decoder, h264_cuvid, encoding, camera_info, CameraInfo, camera_info_manager, calibration yaml, republish, transport hints, green picture, decoders.h264

### xacro, robot_state_publisher
- **Path:** `ros/xacro/README.md`; `ros/robot_state_publisher/`
- **Used in:** `qpl_rover/description/*.xacro`, `rsp.launch.py`.
- **Keywords:** xacro, urdf, macro, xacro:property, xacro:include, xacro:arg, robot_description, robot_state_publisher, joint_states, fixed joint, continuous joint, link, inertial, collision, visual

### twist_mux
- **Path:** `ros/twist_mux/README.md`
- **Used in:** `qpl_rover/config/drive_mux.yaml` (choosing between teleop, autonomy and Nav2 velocity commands).
- **Keywords:** twist_mux, cmd_vel, cmd_vel_teleop, priority, timeout, locks, topics, mux, drive source, e-stop

## Navigation (`nav/`)

### Nav2 documentation
- **Path:** `nav/nav2-docs-html/jazzy/index.html` (website). **Caution:** Humble's docs are no longer hosted, so this is the Jazzy version, the closest one. Where a parameter name doesn't match, check the Humble source below.
- **Start with:** `jazzy/configuration_and_development/configuration_guide/` (every plugin's parameters), `.../tuning_guide/`, `.../migration_guides/` (what changed between Humble and Jazzy), `jazzy/getting_started/navigation_concepts/`, `jazzy/getting_started/nav2_behavior_trees/`.
- **Used in:** `qpl_rover/config/nav_params.yaml`, `launch/components/navigation_launch.py`, `qpl_autonomy` (sends Nav2 goals).
- **Keywords:** nav2, navigation2, bt_navigator, behavior tree, NavigateToPose, NavigateThroughPoses, controller_server, planner_server, DWB, dwb_core::DWBLocalPlanner, critics, max_vel_x, max_vel_theta, acc_lim, NavfnPlanner, global_costmap, local_costmap, VoxelLayer, InflationLayer, inflation_radius, footprint, robot_radius, obstacle_layer, SimpleGoalChecker, xy_goal_tolerance, yaw_goal_tolerance, SimpleProgressChecker, behaviors, Spin, BackUp, Wait, waypoint_follower, WaitAtWaypoint, velocity_smoother, lifecycle_manager, autostart, bond, transform tolerance, "Failed to make progress", "Robot is out of bounds", costmap clearing

### navigation2 source (Humble)
- **Path:** `nav/navigation2/` (branch `humble`; installed version 1.1.20)
- **Use for:** the true Humble parameter names and defaults (`declare_parameter` calls in each package's `src/`), the default `nav2_bringup/params/nav2_params.yaml`, the default behaviour tree XML files in `nav2_bt_navigator/behavior_trees/`, and each package's `README.md`.
- **Keywords:** nav2_params.yaml, default parameters, declare_parameter, navigate_to_pose_w_replanning_and_recovery.xml, nav2_simple_commander, BasicNavigator, nav2_msgs action, ComputePathToPose, FollowPath

### spatio_temporal_voxel_layer
- **Path:** `nav/spatio_temporal_voxel_layer/README.md`
- **Keywords:** STVL, spatio temporal voxel layer, voxel_decay, decay_model, observation sources, pointcloud costmap, depth camera costmap, clearing, marking

## Motor control (`control/`)

### ros2_control documentation
- **Path:** `control/control.ros.org-html/humble/index.html` (website, Humble version)
- **Start with:** `doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html`, `doc/ros2_controllers/diff_drive_controller/doc/userdoc.html`, `doc/ros2_control/ros2controlcli/doc/userdoc.html`, `doc/ros2_control/controller_manager/doc/userdoc.html`.
- **Used in:** `diffdrive_canbus` (the hardware plugin), `qpl_rover/config/my_controllers.yaml`, `gaz_ros2_ctl_use_sim.yaml`, `description/ros2_control.xacro`, `launch/components/controllers.launch.py`.
- **Keywords:** ros2_control, hardware_interface, SystemInterface, ActuatorInterface, on_init, on_configure, on_activate, read, write, return_type, CallbackReturn, state_interface, command_interface, velocity, position, pluginlib, PLUGINLIB_EXPORT_CLASS, controller_manager, spawner, update_rate, diff_drive_controller, wheel_separation, wheel_radius, left_wheel_names, odom, cmd_vel_unstamped, use_stamped_vel, publish_rate, joint_state_broadcaster, velocity_controllers, JointGroupVelocityController, forward_command_controller, "controller failed to activate", "Loaded hardware", resource manager, ros2 control list_controllers

### ros2_control and ros2_controllers source (Humble)
- **Path:** `control/ros2_control/`, `control/ros2_controllers/` (branch `humble`)
- **Use for:** exact parameter definitions (`*_parameters.yaml` in each controller's `src/`), error messages (grep the source for the text), and how the controller manager calls a hardware plugin's lifecycle.
- **Keywords:** diff_drive_controller_parameter.yaml, generate_parameter_library, joint_limits, transmission_interface, mock_components, GenericSystem

## Localisation and perception (`localisation/`)

### robot_localization
- **Path:** `localisation/robot_localization/doc/` (`.rst`: start with `state_estimation_nodes.rst`, `preparing_sensor_data.rst`, `integrating_gps.rst`) and `params/ekf.yaml` (every parameter, with comments)
- **Used in:** `qpl_rover/config/ekf_local_params.yaml`, `ekf_global_params.yaml`, `qpl_rover/ekf_config.py`, `launch/components/odom_localisation.launch.py`, `map_localisation.launch.py`.
- **Keywords:** robot_localization, ekf_node, ukf_node, ekf, two_d_mode, odom0, imu0, pose0, odom0_config, imu0_config, differential, relative, world_frame, odom_frame, map_frame, base_link_frame, process_noise_covariance, initial_estimate_covariance, frequency, sensor_timeout, publish_tf, covariance, drift, "Transform from base_link to map", set_pose

### RTAB-Map
- **Path:** `localisation/rtabmap.wiki/` (wiki pages as Markdown); `localisation/rtabmap_ros/` (ROS wrapper, branch `humble-devel`: `rtabmap_odom` and `rtabmap_launch` READMEs and launch files). For parameters, run `rtabmap --params`.
- **Used in:** visual odometry, `qpl_rover/config/vslam_odom_params*.yaml`, `launch/components/vslam_launch.py`.
- **Keywords:** rtabmap, rtabmap_odom, rgbd_odometry, stereo_odometry, visual odometry, vslam, Odom/Strategy, Vis/MinInliers, Vis/FeatureType, GFTT, ORB, frame-to-map, "Odometry lost", reset odom, approx_sync, queue_size, subscribe_rgbd, rgbd_sync, wait_imu_to_init, guess_frame_id, publish_tf

### imu_tools (Madgwick filter)
- **Path:** `localisation/imu_tools/` (README and `imu_filter_madgwick/`)
- **Used in:** `rover_cameras` RealSense launch (`imu_filter`), `imu_optical_to_standard.py`.
- **Keywords:** imu_filter_madgwick, madgwick, gain, zeta, use_mag, world_frame, enu, publish_tf, imu/data_raw, imu/data, orientation, gyro bias, rviz_imu_plugin

### AprilTag
- **Path:** `localisation/pupil-apriltags/README.rst` (the Python library we use); `localisation/apriltag/` (the C library: tag families, detector parameters, accuracy notes in its README)
- **Used in:** `qpl_rover/qpl_rover/apriltag_observer.py`, `config/localisation/apriltag.yaml`, `Media/tag36_11_00000_a2.png`, Gazebo `apriltag_model`.
- **Keywords:** apriltag, pupil_apriltags, Detector, tag36h11, tag_size, families, quad_decimate, quad_sigma, refine_edges, decode_sharpening, estimate_tag_pose, camera_params, fx fy cx cy, pose_R, pose_t, hamming, decision_margin, fiducial

## DDS networking (`dds/`)

### Eclipse Cyclone DDS 0.10.5
- **Path:** `dds/cyclonedds/docs/manual/options.md` (**the full XML configuration reference**: every element, default and unit); `dds/cyclonedds/docs/manual/config/` (guides, `.rst`); `dds/cyclonedds-0.10.5-html/cyclonedds.io/docs/cyclonedds/0.10.5/index.html` (website)
- **Used in:** `dds/cyclone_*.xml`, `dds/selector.py`, `qpl_dds_selector`, `qpl_net_socket_buffers` in `process/functions/networking.sh`.
- **Keywords:** cyclonedds, CycloneDDS, CYCLONEDDS_URI, cyclone xml, rmw_cyclonedds_cpp, Domain Id, General, Interfaces, NetworkInterface, autodetermine, AllowMulticast, multicast, unicast, Peers, Peer address, ParticipantIndex, MaxAutoParticipantIndex, SPDP, discovery, SocketReceiveBufferSize, SocketSendBufferSize, rmem_max, MaxMessageSize, FragmentSize, Tracing, Verbosity, OutputFile, "failed to find a free participant index", "ddsi_udp_conn_create", wifi, vpn, tailscale, wsl, large messages, fragments

### rmw_cyclonedds
- **Path:** `dds/rmw_cyclonedds/README.md` (branch `humble`)
- **Keywords:** rmw_cyclonedds_cpp, RMW_IMPLEMENTATION, ROS 2 Cyclone settings, ROS_DOMAIN_ID mapping, iceoryx, shared memory

## CAN bus and motor controllers (`can/`)

### REV SPARK MAX
- **Path:** `can/rev/rev-docs-full.md` (REV's entire docs site in one Markdown file, good for grep); `can/rev/pages/brushless/spark-max/` (SPARK MAX pages: specs, wiring, status LEDs, troubleshooting, parameters, control interfaces); `can/rev/pages/brushless/legacy/` (SPARK MAX Client, firmware recovery); `can/rev/pages/revlib/` (closed-loop control, PID tuning, configuration, changelog)
- **Used in:** `diffdrive_canbus` (`can_device.cpp`, duty cycle/velocity/position set API IDs), `sparkmax_params.txt` (P, I, D, F), `can_sim` (`sparkmax_helpers.py`, firmware 24.x periodic status frames).
- **Keywords:** spark max, sparkmax, SPARK MAX, REV, NEO, brushless, CAN ID, device id, status LED, blink codes, magenta, cyan, orange, "no CAN", firmware 24, firmware 25, periodic status frame, status 0 applied output, status 1 velocity, status 2 position, duty cycle, velocity control, position control, closed loop, PIDF, kP, kI, kD, kFF, feedforward, smart current limit, idle mode, brake, coast, encoder, hall sensor, gear ratio, conversion factor, recovery mode, REV Hardware Client, factory reset, burn flash

### FRC CAN addressing
- **Path:** `can/frc-can-addressing/can-addressing.html`
- **Covers:** how the 29-bit extended CAN ID is split into device type, manufacturer, API class, API index and device number, which SPARK MAX frames follow.
- **Used in:** `create_frc_id` in `can_sim/core/can_helpers.py`; `SPARKMAX_API_*` constants in `can_device.cpp`.
- **Keywords:** FRC CAN, extended id, 29-bit, arbitration id, device type, manufacturer code, api class, api index, device number, heartbeat, broadcast, WPILib

### Waveshare USB-CAN-A
- **Path:** `can/waveshare-usb-can-a/` (`USB-CAN-A.html`, the wiki page; `USB (Serial port) to CAN protocol defines.pdf`, the serial framing protocol; `USB-CAN-A-py.zip` and `USB-CAN-A-demo.zip`, example code; `USBCANV2.12_English-windows-tool.zip`, the Windows test tool; `CH341SER.zip`, the CH341 driver)
- **Used in:** `can_sim/core/waveshare_helpers.py`, the CH341 driver rebuilt in `jetson_kernel/`. The rover itself no longer uses this adapter: `diffdrive_canbus` talks SocketCAN on the Jetson's native `can0` (commit "Removed all things serial"). Only `can_sim` still speaks the Waveshare protocol.
- **Keywords:** waveshare, USB-CAN-A, usb can adapter, 0xAA 0x55 frame, serial to CAN, config packet, baud code, 1000 kbps, filter, ttyUSB, ttyACM, CH340, CH341, ch341ser, socat, slcan

### SocketCAN and can-utils
- **Path:** `can/linux-socketcan-can.rst` (the Linux kernel 5.15 SocketCAN documentation); `can/can-utils/README.md`
- **Used in:** `on_jetson_boot/can_setup.sh` (`ip link set can0 up type can bitrate 1000000 restart-ms 1000`), `diffdrive_canbus/hardware/can_comms/can_socket.cpp`.
- **Keywords:** socketcan, can0, ip link, bitrate, restart-ms, bus-off, error-passive, error-active, berr-counter, txqueuelen, "No buffer space available", ENOBUFS, CAN_RAW, sockaddr_can, can_frame, CAN_EFF_FLAG, filters, candump, cansend, cangen, canbusload, cansniffer, ip -details -statistics link show can0, termination, 120 ohm

## Jetson (`jetson/`)

### NVIDIA Jetson Linux Developer Guide (L4T R36.5, JetPack 6)
- **Path:** `jetson/jetson-linux-r36.5/docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/index.html`
- **Start with:** `HR/ControllerAreaNetworkCan.html` (CAN on Jetson: mttcan, pinmux), `SD/Kernel/KernelCustomization.html` (kernel build, as in `jetson_kernel/`), `SD/Bootloader/` and `extlinux.conf` (choosing which kernel boots), `SD/PlatformPowerAndPerformance/` (nvpmodel, jetson_clocks), `SD/Communications/` (Wi-Fi). Search the folder for anything else.
- **Used in:** `on_jetson_boot/can_setup.sh` (`busybox devmem` pinmux writes), `jetson_kernel/*.sh`, `docs/jetson-network-limiting.md`.
- **Keywords:** jetson, orin nano, L4T, R36.5, JetPack 6, tegra, 5.15.185-tegra, mttcan, can0 pinmux, devmem, kernel build, kernel config, defconfig, out-of-tree modules, OOT, nvidia-oot, extlinux.conf, boot menu, initrd, flash, recovery mode, nvpmodel, jetson_clocks, tegrastats, power mode, thermal, Wi-Fi, wlP1p1s0, usb, uart, device tree

## Cameras and controller (`cameras/`)

### librealsense 2.58.4 and realsense-ros 4.58.4
- **Path:** `cameras/librealsense/doc/` (SDK docs: installation, RSUSB vs V4L2 backend, IMU, troubleshooting, `udev`); `cameras/librealsense/scripts/`, `config/` (udev rules); `cameras/realsense-ros/README.md` (ROS wrapper: every launch parameter)
- **Used in:** `rover_cameras/launch/camera_realsense.launch.py`, `libuvc_installation.sh`, `Jetson RealSense setup.md`.
- **Keywords:** realsense, librealsense, librealsense2, realsense2_camera, RealSenseNodeFactory, D435i, D455, RSUSB, libuvc, V4L2 backend, FORCE_RSUSB_BACKEND, udev rules, 99-realsense-libusb.rules, enable_gyro, enable_accel, unite_imu_method, rgb_camera.color_profile, depth_module.depth_profile, align_depth, pointcloud, "No RealSense devices were found", "Frames didn't arrive", firmware update, rs-enumerate-devices, realsense-viewer, patchelf, rpath

### Orbbec SDK ROS 2
- **Path:** `cameras/OrbbecSDK_ROS2/` (README and `docs/`; launch files in `orbbec_camera/launch/`)
- **Used in:** `rover_cameras/launch/camera_orbbec.launch.py`, `process/functions/cameras.sh`.
- **Keywords:** orbbec, Astra Pro Plus, orbbec_camera, OBCameraNodeDriver, OrbbecSDK, udev, 99-obsensor-libusb.rules, install_udev_rules.sh, color_width, color_height, color_fps, depth_registration, uvc, serial_number, usb_port

### pydualsense (PS5 controller)
- **Path:** `cameras/pydualsense/README.md` and `docs/`
- **Used in:** `basestation/controllers/physical_controller.py`, `basestation/70-ps5-controller.rules`.
- **Keywords:** dualsense, PS5 controller, pydualsense, hidraw, hidapi, libhidapi, udev, 054c, 0ce6, bluetooth, triggers, rumble, lightbar, "No device detected"

## Network limiting (`network/`)

### wondershaper and Linux traffic control
- **Path:** `network/wondershaper/` (README and the script itself, which is short and readable); `network/lartc-howto/lartc.org/howto/index.html` (Linux Advanced Routing & Traffic Control HOWTO: qdiscs, HTB, SFQ, ingress, IFB, filters). See also `man tc`, `man tc-htb`, `man tc-sfq`, `man tc-mirred`.
- **Used in:** `qpl_net_limit_*` in `process/functions/networking.sh`, `docs/jetson-network-limiting.md`.
- **Keywords:** wondershaper, tc, traffic control, qdisc, htb, hierarchical token bucket, sfq, tbf, ingress, ifb, ifb0, mirred, redirect, u32 filter, egress, upload, download, bandwidth limit, kbps, rate, ceil, burst, tc -s qdisc show, nload, speedometer, 4 Mbit

## Simulation (`sim/`)

### Gazebo Classic 11 tutorials
- **Path:** `sim/gazebo_tutorials/` (the tutorial source as Markdown, one folder per tutorial). Gazebo Classic is end-of-life and its website may disappear.
- **Used in:** `qpl_rover/worlds/*.world`, `terrain_heightmap`, `qpl_sim`, `qpl_headless`.
- **Keywords:** gazebo, gazebo classic, gazebo 11, gzserver, gzclient, world file, model.config, model.sdf, GAZEBO_MODEL_PATH, heightmap, materials, ogre, plugins, sensors, camera sensor, depth camera, imu sensor, physics, real time factor, LIBGL_ALWAYS_SOFTWARE, rendering, headless

### SDFormat specification
- **Path:** `sim/sdformat/sdf/1.6/`, `sim/sdformat/sdf/1.7/` (one `.sdf` file per element, describing every attribute and default; SDFormat 9 is the version Gazebo 11 uses)
- **Keywords:** sdf, sdformat, world, model, link, joint, sensor, plugin, collision, surface, friction, mu, mu2, inertial, heightmap, include uri, pose

### gazebo_ros_pkgs and gazebo_ros2_control
- **Path:** `sim/gazebo_ros_pkgs/` (branch `ros2`: `gazebo_plugins/include/` headers document each plugin's SDF parameters); `sim/gazebo_ros2_control/` (branch `humble`)
- **Used in:** `qpl_rover/description/*.xacro` (camera, IMU plugins), `config/gaz_ros2_ctl_use_sim.yaml`, `sim.launch.py`.
- **Keywords:** gazebo_ros, gazebo_ros2_control, GazeboSystem, libgazebo_ros2_control.so, libgazebo_ros_camera, libgazebo_ros_imu_sensor, libgazebo_ros_ray_sensor, spawn_entity, robot_description, ros remapping in sdf, use_sim_time, /clock

## Languages and libraries (`libs/`)

### Python 3.10
- **Path:** `libs/python-3.10.22-docs-html/index.html` (the version Ubuntu 22.04 ships)
- **Keywords:** python, standard library, asyncio, threading, subprocess, struct, http.server, ThreadingHTTPServer, json, pathlib, dataclasses, enum, typing, tkinter, logging, argparse, match statement

### Dash docsets: C++, CMake, Bash, NumPy, OpenCV (and optionally JavaScript, HTML, CSS)
- **Path:** `libs/docsets/<Name>.docset/` (open in Zeal, or browse `Contents/Resources/Documents/` in a browser). JavaScript, HTML and CSS (for the HUD) add ~2.3 GB, so they're only downloaded by `./download.sh web`. **Caution:** the OpenCV docset is for the latest OpenCV; Ubuntu 22.04 has 4.5.4, so check `/usr/include/opencv4/` if a function doesn't match.
- **Keywords:** C++ standard library, std::thread, std::mutex, condition_variable, chrono, cppreference, CMake, find_package, target_link_libraries, ament, Bash, parameter expansion, test, arrays, JavaScript, ES modules, fetch, EventSource, canvas, WebSocket, DOM, HTML, CSS, NumPy, ndarray, OpenCV, cv::resize, cvtColor, INTER_AREA, VideoCapture, imencode

### FFmpeg
- **Path:** `libs/ffmpeg/ffmpeg-all.html` (every ffmpeg option, format and filter on one page); `libs/ffmpeg/ffmpeg-codecs.html` (codec options, including libx264). The libavcodec C API is documented in the installed headers (see "Already on your machine").
- **Used in:** `rover_cameras/src/stream_encoder/h264_encoder.cpp`.
- **Keywords:** ffmpeg, libavcodec, avcodec_send_frame, avcodec_receive_packet, AVCodecContext, AVFrame, AVPacket, av_dict_set, libx264, x264, preset, superfast, tune, zerolatency, keyframe, gop, intra-refresh, vbv, bitrate, crf, h264, yuv420p, I420, h264_cuvid

### urwid and ttkbootstrap
- **Path:** `libs/urwid/docs/` (terminal UI, used by `dds/selector.py`); `libs/ttkbootstrap/docs/` (Tk themes, used by the basestation teleop window)
- **Keywords:** urwid, MainLoop, ListBox, Button, palette, ExitMainLoop, ttkbootstrap, tkinter, ttk, Window, theme, bootstyle, Meter, Floodgauge

## Competition (`competition/`)

### Lunabotics rulebook
- **Path:** `competition/` (**download by hand**, see above)
- **Keywords:** Lunabotics, NASA, rules, guidebook, arena, starting zone, excavation zone, construction zone, berm, regolith, obstacles, rocks, craters, autonomy, telerobotic, communication, bandwidth limit, network, emergency stop, mass, dimensions, dust tolerance, scoring, run time
