#!/bin/bash
# The ROS daemon sometimes doesn't start on WSL.
ros2 daemon start

# Environment variables
# Disable hardware acceleration (causes issues with rviz on some machines)
## Disbale if you want to use GPU
#export LIBGL_ALWAYS_SOFTWARE=1
#export GALLIUM_DRIVER=llvmpipe

# Shortcuts
alias qpl_build='${QPL_PROJECT}/process/build.sh'
alias qpl_packages='${QPL_PROJECT}/process/install_packages.sh'
alias qpl_sim="ros2 launch qpl_rover launch_sim.launch.py"
alias qpl_rviz="ros2 launch qpl_rover rviz.launch.py"
alias qpl_kb="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"
alias qpl_slam="ros2 launch qpl_rover online_async_launch.py"
alias nav2="ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true"

# Make scripts executable
chmod +x "$QPL_PROJECT"/process/install_packages.sh
chmod +x "$QPL_PROJECT"/process/build.sh

source "$QPL_PROJECT"/qpl_ws/install/setup.bash
