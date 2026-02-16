#!/bin/bash
# The ROS daemon sometimes doesn't start on WSL.
ros2 daemon start

# Environment variables
# Disable hardware acceleration (causes issues with rviz on some machines)
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# Shortcuts
alias luna_build='${LUNA_PROJECT}/process/build.sh'
alias luna_packages='${LUNA_PROJECT}/process/install_packages.sh'
alias luna_sim_rviz="ros2 launch luna_sim gazebo_with_rviz.launch.py"
alias luna_rviz_sim="luna_sim_rviz"
alias luna_kb="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"

# Make scripts executable
chmod +x "$LUNA_PROJECT"/process/install_packages.sh
chmod +x "$LUNA_PROJECT"/process/build.sh
