#!/bin/bash
cd "$LUNA_PROJECT"/luna_ws || echo 'Could not find luna_ws. Check LUNA_PROJECT is set correctly.' exit

# Delete old build output; otherwise errors can occur, and build takes longer
rm -rf build/ install/ log/

colcon build --symlink-install
source "$LUNA_PROJECT"/luna_ws/install/setup.bash
