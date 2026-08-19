# If you get an error to do with 'Unable to locate package', it may be due to Windows-style newlines
#
# Both of these back exec_depends declared in qpl_rover/package.xml:
#   rtabmap-odom      - rgbd_odometry, run by vslam_launch.py. Already arrives transitively via
#                       rtabmap-ros, listed by name anyway: rtabmap-ros also drags in
#                       viz/demos/examples, so if that ever gets trimmed to save space on the
#                       Jetson, visual odometry would break silently.
#   realsense2-camera - the ROS wrapper, and NOT previously installed by anything here.
#                       libuvc_installation.sh builds the librealsense SDK from source, which is a
#                       different thing and does not provide this node.
qpl_packages() {
  sudo apt update
  sudo apt -y install ros-humble-xacro \
    ros-humble-topic-tools \
    python3-colcon-common-extensions \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
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
    ros-humble-realsense2-camera \
    ros-humble-v4l2-camera \
    v4l-utils \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-odom \
    ros-humble-image-pipeline \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    libopencv-dev \
    mesa-utils \
    libserial-dev \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ffmpeg-image-transport \
    ros-humble-nav2-common \
    ros-humble-spatio-temporal-voxel-layer \
    ros-humble-imu-tools \
    ros-humble-ament-cmake-vendor-package \
    socat
  python3 -m pip install --user \
    pupil-apriltags \
    imageio \
    urwid
}

qpl_gazebo_packages() {
  sudo apt update
  sudo apt -y install gazebo \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs
}
