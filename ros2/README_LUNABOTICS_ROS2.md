# 🛰️ Lunabotics ROS 2 & RViz – Project Structure and Usage Guide

This README explains **how the ROS 2 workspace in this repo is organised**, **where to run commands from**, and **how to build, run, visualise, and control the rover in RViz**.

It is written to be **step-by-step and foolproof**.
If you follow this exactly, it will work.

---

## 1️⃣ Repository Structure (What Lives Where)

```
lunabotics/
├── ros2/                     ← ROS 2 WORKSPACE ROOT (IMPORTANT)
│   ├── src/                  ← ALL ROS PACKAGES LIVE HERE
│   │   ├── lunabotics_description/
│   │   │   ├── urdf/          ← Robot model (URDF / Xacro)
│   │   │   ├── rviz/          ← Saved RViz configuration files (.rviz)
│   │   │   ├── launch/        ← Launch files (RViz + robot)
│   │   │   └── setup.py
│   │   │
│   │   ├── lunabotics_control/
│   │   │   └── diffdrive_sim  ← Differential drive simulation (cmd_vel → motion)
│   │   │
│   │   └── lunabotics_sensors/
│   │       └── lunabotics_sensors/
│   │           └── Publishers / subscribers (temperature, camera, markers)
│   │
│   ├── build/                ← Auto-generated (DO NOT TOUCH)
│   ├── install/              ← Auto-generated (DO NOT TOUCH)
│   └── log/                  ← Auto-generated (DO NOT TOUCH)
│
├── Jetson/               ← Non-ROS files
├── Media/
└── README.md
```

### 🔴 Golden Rules
- **All ROS packages must live inside `ros2/src/`**
- **All ROS commands are run from `ros2/`, NOT from `src/`**
- Never edit `build/`, `install/`, or `log/`

---

## 2️⃣ One-Time Setup (Per Machine)

```bash
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
```

Add to your `~/.bashrc`:
```bash
source ~/lunabotics/ros2/setup.bash
```

Reload:
```bash
source ~/.bashrc
```

---

## 3️⃣ Build the Workspace

```bash
cd ~/lunabotics/ros2
colcon build --symlink-install
source install/setup.bash
```

---

## 4️⃣ Launch the Rover in RViz

```bash
ros2 launch lunabotics_description view_rover.launch.py
```

This launches:
- robot_state_publisher
- diffdrive_sim
- RViz (preconfigured)

---

## 5️⃣ Drive the Rover (Keyboard)

Open a **new terminal** and run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Controls:
```
i  forward
k  stop
j  rotate left
l  rotate right
,  reverse
```

Click the terminal before pressing keys.

---

## 6️⃣ RViz Notes

- Fixed Frame: `map` (recommended)
- RViz config auto-loads from launch

---

## 7️⃣ Common Issues

- Package not found → forgot to source
- Robot doesn’t move → diffdrive_sim not running
- Nothing in RViz → wrong Fixed Frame

---

## 8️⃣ Typical Workflow

```bash
cd ~/lunabotics/ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch lunabotics_description view_rover.launch.py
```

Then drive:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

---

## 9️⃣ Mental Model

- URDF → geometry
- TF → pose
- /cmd_vel → motion
- RViz → visualisation only
