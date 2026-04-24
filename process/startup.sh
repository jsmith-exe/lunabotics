#!/bin/bash
alias reload='source "$QPL_PROJECT/process/startup.sh"'

if [ -z "${QPL_PROJECT:-}" ]; then
  echo "ERROR: QPL_PROJECT is not set. Export it in bashrc."
  exit 1
fi

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source "$QPL_PROJECT/qpl_ws/install/setup.bash" > /dev/null 2>&1 # Hide output

# Source functions
source "$QPL_PROJECT/process/functions/build.sh"
source "$QPL_PROJECT/process/functions/executables.sh"
source "$QPL_PROJECT/process/functions/install_packages.sh"
source "$QPL_PROJECT/process/functions/networking.sh"
source "$QPL_PROJECT/process/functions/orbbec_sdk.sh"
source "$QPL_PROJECT/process/functions/teleop.sh"

# The ROS daemon sometimes doesn't start on WSL.
ros2 daemon start >/dev/null 2>&1 || true

# Environment vars
export GAZEBO_MODEL_PATH=$QPL_PROJECT/qpl_ws/src/qpl_rover/worlds:$GAZEBO_MODEL_PATH
