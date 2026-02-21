#!/bin/bash
cd "$QPL_PROJECT"/qpl_ws || echo 'Could not find qpl_ws. Check QPL_PROJECT is set correctly.' exit

# Delete old build output; otherwise errors can occur, and build takes longer
rm -rf build/ install/ log/

colcon build --symlink-install
source "$QPL_PROJECT"/qpl_ws/install/setup.bash
