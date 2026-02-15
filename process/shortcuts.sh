# The ROS daemon sometimes doesn't start on WSL.
ros2 daemon start

source ${LUNA_PROJECT}/process/env.sh

alias luna_build="${LUNA_PROJECT}/process/build.sh"
alias luna_sim_rviz="ros2 launch luna_sim gazebo_with_rviz.launch.py"
alias luna_rviz_sim="luna_sim_rviz"
alias luna_kb="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"
