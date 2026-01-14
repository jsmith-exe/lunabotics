# 🛰️ Lunabotics ROS 2 & RViz – Project Structure and Usage Guide

This README explains **how the ROS 2 workspace in this repo is organised**, **where to run commands from**, and **how to build, run, and visualise the robot in RViz**.

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
│   │   └── lunabotics_sensors/
│   │       └── lunabotics_sensors/
│   │           └── Publishers / subscribers (temperature, camera, markers)
│   │   
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

### Install required ROS tools
```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui
```

### 🔧 Automatically Loading the ROS 2 Environment (Recommended)

To avoid manually sourcing ROS every time a new terminal is opened, each team member should add one line to their local `.bashrc` file.

This will automatically load:

- ROS 2 Humble

- the Lunabotics ROS 2 workspace overlay

✅ How to set this up (takes ~30 seconds)

1️⃣ Open your `.bashrc` file:
```bash
nano ~/.bashrc
```

2️⃣ Scroll to the very bottom of the file and add this line:
```bash
source ~/lunabotics/ros2/setup.bash
```
3️⃣ Save and exit:

4️⃣ Reload your terminal:
```bash
source ~/.bashrc
```

(or just open a new terminal)

✅ How to check it worked

Open a new terminal and run:
```bash
echo $ROS_DISTRO
```

You should see:
```nginx
humble
```

Then check:
```bash
ros2 pkg list | grep lunabotics
```

If packages appear, the environment is set up correctly.

---

## 3️⃣ Building the ROS 2 Workspace (VERY IMPORTANT)

### Always build from here:
```bash
cd ~/lunabotics/ros2
```

### Build the workspace:
```bash
colcon build --symlink-install
```

### Source the workspace:
```bash
source install/setup.bash
```

⚠️ If you forget to source, ROS will say **“package not found”**.

---

## 4️⃣ Running Nodes (`ros2 run`)

### Command format
```bash
ros2 run <package_name> <executable_name>
```

### Examples
Run temperature publisher:
```bash
ros2 run lunabotics_sensors temp_pub
```

Run camera publisher:
```bash
ros2 run lunabotics_sensors camera_pub
```

---

## 5️⃣ Launching RViz + Robot (`ros2 launch`)

### What launch files do
Launch files:
- Start `robot_state_publisher`
- Load the robot URDF
- Open RViz with a **pre-configured view**
- Ensure everyone sees the same setup

### Launch the rover visualisation
```bash
ros2 launch lunabotics_description view_rover.launch.py
```

⚠️ **Do not open RViz manually** unless debugging  
Always use launch files.

---

## 6️⃣ RViz Configuration

- RViz configuration files live in:
  ```
  lunabotics_description/rviz/
  ```
- These files are **installed and shared**
- Launch files load them automatically

### Fixed Frame
RViz uses:
```
Fixed Frame = map
```

If this is changed incorrectly, visuals will disappear.

---


## 8️⃣ Common Problems & Fixes

### ❌ “Package not found”
You forgot:
```bash
source install/setup.bash
```

### ❌ RViz opens but shows nothing
- Fixed Frame is wrong
- RViz was opened manually instead of via launch
- Workspace was not rebuilt after changes

---

## 9️⃣ Development Workflow (TL;DR)

Every time you change code:
```bash
cd ~/lunabotics/ros2
colcon build --symlink-install
source install/setup.bash
```

Then launch:
```bash
ros2 launch lunabotics_description view_rover.launch.py
```

---

## 10️⃣ Mental Model (Remember This)

- **URDF** → what the robot looks like
- **TF** → where the robot is
- **Markers** → debug & state visualisation
- **RViz** → visualisation only (no physics)
- **Launch files** → one-command setup for the team

---

If something breaks:
1. Check you are in `ros2/`
2. Check you built
3. Check you sourced
4. Check RViz Fixed Frame

That fixes 95% of issues.
