cd ${LUNA_PROJECT}/luna_ws || echo 'Could not find luna_ws. Check LUNA_PROJECT is set correctly.' exit
colcon build --symlink-install
source ${LUNA_PROJECT}/luna_ws/install/setup.bash
