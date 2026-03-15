#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

cd "$QPL_PROJECT/qpl_ws" || {
  echo "Could not find qpl_ws. Check QPL_PROJECT is set correctly."
  exit 1
}

rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
