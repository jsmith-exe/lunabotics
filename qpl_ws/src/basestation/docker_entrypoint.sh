#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /lunabotics_ws/install/setup.bash

exec "$@" # Expands arguments; in this case, the command to run the ROS node