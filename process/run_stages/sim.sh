# Disable hardward acceleration
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
ros2 launch luna_sim gazebo_rover.launch.py