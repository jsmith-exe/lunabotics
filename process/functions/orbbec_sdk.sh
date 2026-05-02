source "${QPL_PROJECT}/OrbbecSDK_ROS2/install/setup.bash" > /dev/null 2>&1

qpl_orbbecsdk_clone() {
  git clone --single-branch --branch main git@github.com:orbbec/OrbbecSDK_ROS2.git "${QPL_PROJECT}/OrbbecSDK_ROS2"
}

qpl_orbbecsdk_build() {
  # For camera
  sudo apt install libgflags-dev nlohmann-json3-dev  \
    ros-humble-image-transport  ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
    ros-humble-image-publisher ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater ros-humble-diagnostic-msgs ros-humble-statistics-msgs \
    ros-humble-backward-ros libdw-dev \
    ros-humble-orbbec-camera ros-humble-orbbec-description
  
  # Registration script
  sudo cp /opt/ros/$ROS_DISTRO/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger

  cd "${QPL_PROJECT}/OrbbecSDK_ROS2/orbbec_camera/scripts" || {
    echo "Failed to change directory to ${QPL_PROJECT}/OrbbecSDK_ROS2/orbbec_camera/scripts"
    return
  }
  sudo bash install_udev_rules.sh
  sudo udevadm control --reload-rules && sudo udevadm trigger

  local prev_path=$(pwd)
  cd "${QPL_PROJECT}/OrbbecSDK_ROS2" || {
    echo "Failed to change directory to ${QPL_PROJECT}/OrbbecSDK_ROS2"
    return
  }
  colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
  cd "$prev_path" || true
  source "${QPL_PROJECT}/OrbbecSDK_ROS2/install/setup.bash"
}

qpl_orbbecsdk_run() {
  ros2 launch orbbec_camera astra_pro_plus.launch.py
}
