#!/bin/bash
# If you get an error to do with 'Unable to locate package', it may be due to Windows-style newlines
sudo apt update

sudo apt -y install gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-topic-tools \
  python3-colcon-common-extensions \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-twist-mux \
  ros-humble-diff-drive-controller \
  ros-humble-joint-trajectory-controller \
  ros-humble-effort-controllers \
  ros-humble-image-transport-plugins \
  ros-humble-robot-localization
  ros-humble-apriltag-ros
  ros-humble-image-pipeline

sudo apt -y install mesa-utils

sudo apt -y install libserial-dev
  
