#!/bin/bash
# If you get an error to do with 'Unable to locate package', it may be due to Windows-style newlines
sudo apt update

sudo apt -y install ros-humble-xacro \
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
  ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-camera-info-manager \
  ros-humble-v4l2-camera \
  v4l-utils \
  ros-humble-rtabmap-ros
  ros-humble-gazebo-ros-pkgs
#   gazebo \

sudo apt -y install mesa-utils

sudo apt -y install libserial-dev

# For camera
sudo apt install libgflags-dev nlohmann-json3-dev  \
  ros-$ROS_DISTRO-image-transport  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
  ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
  ros-$ROS_DISTRO-backward-ros libdw-dev
# Additional camera setup
cd ${QPL_PROJECT}/qpl_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
