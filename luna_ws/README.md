# 🛰️ Lunabotics ROS 2 & RViz – Project Structure and Usage Guide

This README explains **how the ROS 2 workspace in this repo is organised**, **where to run commands from**, and **how to build, run, visualise, and drive the robot in RViz**.

It is written to be **step-by-step and foolproof**.  
If you follow this exactly, it will work.

---

## 1. Repository Structure (What Lives Where)

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

### Golden Rules
- **All ROS packages must live inside `luna_ws/src/`**
- **All ROS commands are run from `luna_ws/`, NOT from `src/`**
- Never edit `build/`, `install/`, or `log/`

---

## 2. Building the ROS 2 Workspace (VERY IMPORTANT)

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

If you forget to source, ROS will say **“package not found”**.

---

## 3. One-Time Setup (Per Machine)

### Install required ROS tools
```bash
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-robot-state-publisher
```

### Automatically Loading the ROS 2 Environment (Recommended)

To avoid manually sourcing ROS every time a new terminal is opened, each team member should add one line to their local `.bashrc` file.

This will automatically load:
- ROS 2 Humble
- the Lunabotics ROS 2 workspace overlay

### How to set this up
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

### Verify setup
```bash
echo $ROS_DISTRO
ros2 pkg list | grep lunabotics
```

You should see `humble` and Lunabotics packages.

---

## 4. Launching the Rover Sim in Gazebo

### Launch the rover
```bash
ros2 launch luna_sim gazebo_rover.launch.py
```

This will:
- Open Gazebo
- Spawn the rover model
- Enable TF and all sensors on the rover

---

## 5. Launching RViz

```bash
ros2 launch luna_rviz rviz.launch.py
```

This will:
- Open RViz
- Spawn the rover model
- Allow the user to view all important topics about the rover

---

## 6. Driving the Rover (Telop Keyboard Control)

Once the rover is launched in Gazebo, open **a new terminal** and run:

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

## 7. ros2 Interaction

List all available topics,
```bash
ros2 topic list
```
Listen to a specific topic,
```bash
ros2 topic echo /topic_name
```

## 8. Development Workflow (TL;DR) / Rebuild Workspace (ws)

Every time you change code:
```bash
cd ~/lunabotics/luna_ws
colcon build --symlink-install
source install/setup.bash
```

## 9. How to Run Python Files

- If you use relative imports (from . / from ..) you must run with -m.
- Don't mix "run as a file path" with relative imports.

From your home directory,
```bash
cd ~/
python3 -m lunabotics
```

## 10. ros2 Multi Machine Communication Config

On Laptop:

```bash
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
source ~/.bashrc

echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

On Remote Jetson/Pi:
```bash
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
source ~/.bashrc

echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

