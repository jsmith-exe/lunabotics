#!/bin/bash
set -e



cd "$QPL_PROJECT/qpl_ws" || {
  echo "Could not find qpl_ws. Check QPL_PROJECT is set correctly."
  exit 1
}

colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash