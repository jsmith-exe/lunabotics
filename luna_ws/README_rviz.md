# 🛰️ Lunabotics ROS 2 & RViz – Project Structure and Usage Guide

This README explains **how the ROS 2 workspace in this repo is organised**, **where to run commands from**, and **how to build, run, visualise, and drive the robot in RViz**.

It is written to be **step-by-step and foolproof**.  
If you follow this exactly, it will work.

---

## 1️⃣ Repository Structure (What Lives Where)

```
lunabotics/
├── luna_ws/                     ← ROS 2 WORKSPACE ROOT (IMPORTANT)
│   ├── src/                  ← ALL ROS PACKAGES LIVE HERE
│   │   ├── lunabotics_description/
│   │   │   ├── urdf/          ← Robot model (URDF / Xacro)
│   │   │   ├── rviz/          ← Saved RViz configuration files (.rviz)
│   │   │   ├── launch/        ← Launch files (RViz + robot)
│   │   │   └── setup.py
│   │   │
│   │   ├── lunabotics_control/
│   │   │   └── diffdrive_sim.py   ← Differential-drive simulator (cmd_vel → odom/TF/joints)
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
- **All ROS packages must live inside `luna_ws/src/`**
- **All ROS commands are run from `luna_ws/`, NOT from `src/`**
- Never edit `build/`, `install/`, or `log/`

---

## 2️⃣ Building the ROS 2 Workspace (VERY IMPORTANT)

### Always build from here:
```bash
cd ~/lunabotics/luna_ws
```

### Build:
```bash
colcon build --symlink-install
```

### Source:
```bash
source install/setup.bash
```

⚠️ If you forget to source, ROS will say **“package not found”**.

---

## 3️⃣ One-Time Setup (Per Machine)

### Install required ROS tools
```bash
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-robot-state-publisher
```

### 🔧 Automatically Loading the ROS 2 Environment (Recommended)

To avoid manually sourcing ROS every time a new terminal is opened, each team member should add one line to their local `.bashrc` file.

This will automatically load:
- ROS 2 Humble
- the Lunabotics ROS 2 workspace overlay

### ✅ How to set this up
```bash
nano ~/.bashrc
```

Add this line **at the very bottom**:
```bash
source ~/lunabotics/luna_ws/install/setup.bash
```

Save, exit, then reload:
```bash
source ~/.bashrc
```

### ✅ Verify setup
```bash
echo $ROS_DISTRO
ros2 pkg list | grep lunabotics
```

You should see `humble` and Lunabotics packages.

---

## 4️⃣ Launching the Rover in RViz

### What launch files do
Launch files:
- Start `robot_state_publisher`
- Start the differential-drive simulator
- Load the robot URDF
- Open RViz with a **pre-configured view**

### Launch the rover
```bash
ros2 launch lunabotics_description view_rover.launch.py
```

This will:
- Open RViz
- Spawn the rover model
- Enable TF, odometry, and wheel animation

⚠️ **Do not open RViz manually** unless debugging.

---

## 5️⃣ Driving the Rover (Keyboard Control)

Once the rover is launched, open **a new terminal** and run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### Keyboard controls (default)
- `i` → forward
- `k` → stop
- `j` / `l` → rotate left / right
- `,` → reverse
- `q` / `z` → increase / decrease speed

📌 **Important**
- Click inside the terminal before pressing keys
- Keep this terminal open while driving
- The rover will only move while the drive simulator is running

---

## 6️⃣ Running Individual Nodes (`ros2 run`)

### Command format
```bash
ros2 run <package_name> <executable_name>
```

### Examples
Camera publisher:
```bash
ros2 run lunabotics_sensors camera_pub
```

Temperature publisher:
```bash
ros2 run lunabotics_sensors temp_pub
```

---

## 7️⃣ RViz Configuration Notes

- RViz config files live in:
  ```
  lunabotics_description/rviz/
  ```
- They are automatically loaded by launch files

### Fixed Frame
Recommended:
```
Fixed Frame = map
```

If visuals disappear, check this first.

---

## 8️⃣ Common Problems & Fixes

### ❌ “Package not found”
```bash
source install/setup.bash
```

### ❌ RViz opens but shows nothing
- Fixed Frame is wrong (`map` or `base_link`)
- RViz opened manually instead of via launch
- Workspace not rebuilt after changes

### ❌ Keyboard does nothing
- `diffdrive_sim` is not running
- Teleop terminal not focused
- Wrong `/cmd_vel` topic

Check:
```bash
ros2 topic info /cmd_vel
```

---

## 9️⃣ Development Workflow (TL;DR)

Every time you change code:
```bash
cd ~/lunabotics/luna_ws
colcon build --symlink-install
source install/setup.bash
```

Then run:
```bash
ros2 launch lunabotics_description view_rover.launch.py
```

---

## 🔟 Mental Model (Remember This)

- **URDF** → what the robot looks like
- **TF (Transform)** → where the robot is
- **cmd_vel** → how the robot is commanded
- **diffdrive_sim** → converts commands into motion
- **RViz** → visualisation only (no physics)
- **Launch files** → one-command setup for the team

---
