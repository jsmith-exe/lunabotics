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

## 2. Setup
Ensure your ~/.bashrc file contains the following, replacing the value of `LUNA_PROJECT`
with the path to your local copy of the repo:
```bash
export LUNA_PROJECT="/mnt/c/Users/caleb/Documents/Projects/lunabotics"
source /opt/ros/humble/setup.bash
source "$LUNA_PROJECT"/luna_ws/install/setup.bash
source "$LUNA_PROJECT"/process/startup.sh
```
> **Explanation:**
>
> The first line sets the environment variable `LUNA_PROJECT` for use in the rest of the commands and in helper commands.
> Second line sources ROS 2 Humble, third line installs any previously built version of lunabotics.
> The fourth line sets up shortcut scripts for building and running parts of the project.

### Verify setup
If you've built before:
```bash
ros2 run basestation nav_pub
```
- This should run a simple publisher; you should see some logs indicating that it's publishing messages. 
Ctrl+C to stop it. This means 1) ROS is sourced and 2) the workspace is sourced.

If you haven't built before, run ```luna_build```.

You'll now able to use a number of commands to build and run parts of the project, such as:
```bash
luna_packages # Run to ensure all packages are installed
luna_build # Build the ROS workspace
luna_sim_rviz # or luna_rviz_sim, order doesn't matter; this runs RViz (visualisation) with Gazebo (simulation)
luna_kb # See next section
```
>**Important!**
>- After any code changes in ROS packages, run ```luna_build```.
>- If you rely on any new package, add it to the **process/install_packages.sh**

## 3. Driving the rover via luna_kb
```luna_kb``` runs a keyboard teleoperation node; this was not made by us and is for testing only.
A more user-friendly node will be accessible soon.

**Default controls**
- `i` → forward
- `k` → stop
- `j` / `l` → rotate left / right
- `,` → reverse
- `q` / `z` → increase / decrease speed

📌 **Important**
- Click inside the terminal before pressing keys
- Keep this terminal open while driving

---

## 4. Helpful ROS commands

List all available topics,
```bash
ros2 topic list
```
Listen to a specific topic,
```bash
ros2 topic echo /topic_name
```

Test node communication (if you're having trouble with node communication)
```bash
# On some terminal, set up for receiving
ros2 multicast receive
# On another terminal, send multicast message
ros2 multicast send
```


## 5. How to Run Python Files

- If you use relative imports (from . / from ..) you must run with -m.
- Don't mix "run as a file path" with relative imports.

From your home directory,
```bash
cd $LUNA_PROJECT/../
python3 -m lunabotics
```

## 6. ros2 Multi Machine Communication Config

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

