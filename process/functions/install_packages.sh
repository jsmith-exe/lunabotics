# If you get an error to do with 'Unable to locate package', it may be due to Windows-style newlines
qpl_packages() {
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
    ros-humble-rtabmap-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-apriltag-ros \
    ros-humble-image-pipeline \
    gazebo \
    ros-humble-tf-transformations \
    mesa-utils \
    libserial-dev
}
