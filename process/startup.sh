#!/bin/bash

startup() {
  # Environment vars and helpful stuff
  export GAZEBO_MODEL_PATH=$QPL_PROJECT/qpl_ws/src/qpl_rover/worlds:$GAZEBO_MODEL_PATH
  alias reload='source "$QPL_PROJECT/process/startup.sh"'
  alias rover='ros2 launch qpl_rover'
  alias basestation='ros2 launch basestation'

  # Source ROS and workspace
  source /opt/ros/humble/setup.bash
  source "$QPL_PROJECT/qpl_ws/install/setup.bash" > /dev/null 2>&1 # Hide output

  # Source functions
  source "$QPL_PROJECT/process/functions/build.sh"
  source "$QPL_PROJECT/process/functions/executables.sh"
  source "$QPL_PROJECT/process/functions/install_packages.sh"
  source "$QPL_PROJECT/process/functions/networking.sh"
  source "$QPL_PROJECT/process/functions/cameras.sh"
  source "$QPL_PROJECT/process/functions/teleop.sh"

  # The ROS daemon sometimes doesn't start on WSL; run in background via &
  ros2 daemon start >/dev/null 2>&1 &
}

# Run whatever comes after it with no stdout or stderr
silent() {
    "$@" >/dev/null 2>/dev/null
}
alias s=silent

if [ -z "${QPL_PROJECT:-}" ]; then
  echo "ERROR: QPL_PROJECT is not set. Export it in bashrc."
else
  startup
fi
