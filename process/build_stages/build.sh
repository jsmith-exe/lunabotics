colcon_build() {
  colcon build --symlink-install
  source ${LUNA_WS}/install/setup.bash
}
colcon_build