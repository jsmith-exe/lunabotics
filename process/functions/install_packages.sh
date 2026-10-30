# If you get an error to do with 'Unable to locate package', it may be due to Windows-style newlines
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
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ffmpeg-image-transport \
    ros-humble-nav2-common \
    ros-humble-spatio-temporal-voxel-layer \
    ros-humble-imu-tools \
    ros-humble-ament-cmake-vendor-package \
    socat \
    python3-yaml \
    libyaml-cpp-dev \
    libhidapi-dev \
    python3-pip \
    ccache \
    libavcodec-dev \
    libavutil-dev \
    pkg-config \
    ros-humble-ffmpeg-image-transport-msgs \
    ros-humble-diagnostic-msgs \
    python3-numpy \
    python3-opencv \
    python3-scipy \
    nload
    # Don't add ros-humble-realsense2-camera, it should be built from source.
  python3 -m pip install --user \
    pupil-apriltags \
    imageio \
    urwid \
    pydualsense \
    ttkbootstrap
}

# Gazebo Harmonic, from the OSRF repo. Replaces Gazebo Classic (its gz command clashes
# with Harmonic's, so apt removes gazebo and the gazebo_ros packages).
# gz_ros2_control has no Humble+Harmonic binary; it is built from source into
# qpl_ws/src/external (see qpl_ws/sim.repos) and needs GZ_VERSION=harmonic, which
# startup.sh exports.
qpl_gazebo_packages() {
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt update
  sudo apt -y install gz-harmonic \
    ros-humble-ros-gzharmonic \
    python3-vcstool
  vcs import --skip-existing "$QPL_PROJECT/qpl_ws/src/external" < "$QPL_PROJECT/qpl_ws/sim.repos"
}
