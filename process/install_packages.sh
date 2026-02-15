#!/bin/bash
# If you get an error to do with 'Unable to locate package', it may be due to Windows-style newlines
sudo apt -y install gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-topic-tools \
  python3-colcon-common-extensions \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control
