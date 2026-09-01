#!/usr/bin/env bash
# Synthetic tilt bench.
#
# Publishes a stationary IMU at a known attitude (gravity rotated into
# camera_imu_optical_frame exactly as the RealSense driver would) and checks that
# ekf_local recovers the same roll/pitch. Exercises the real imu_filter_madgwick
# parameters from camera_realsense.launch.py and the real, unmodified
# ekf_local_params_rover.yaml. No hardware, no Gazebo.
#
#   tools/tilt_bench.sh <roll_deg> <pitch_deg>
source /opt/ros/humble/setup.bash   # must precede `set -u`: it reads unset vars
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-78}
export ROS_LOCALHOST_ONLY=1

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG=$HERE/../qpl_ws/src/qpl_rover/config/ekf_local_params_rover.yaml
WORK="$(mktemp -d)"
PIDS=()
# setsid so each node gets its own process group. `ros2 run` spawns the node as a
# child, so killing the wrapper PID alone orphans it, and an orphaned ekf_node
# keeps publishing /odometry/filtered and silently corrupts every later run.
run() { setsid "$@" >>"$WORK/nodes.log" 2>&1 & PIDS+=($!); }
cleanup() { for p in "${PIDS[@]:-}"; do kill -TERM -"$p" 2>/dev/null; done; rm -rf "$WORK"; }
trap cleanup EXIT

# robot_description is too large for a -p override, so wrap it as a params file.
xacro "$HERE/../qpl_ws/src/qpl_rover/description/rover.urdf.xacro" \
      use_ros2_control:=true sim_mode:=false > "$WORK/rover.urdf"
python3 -c "
import yaml
urdf=open('$WORK/rover.urdf').read()
yaml.safe_dump({'robot_state_publisher':{'ros__parameters':
  {'robot_description':urdf,'publish_frequency':30.0}}}, open('$WORK/rsp.yaml','w'))"

run ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$WORK/rsp.yaml"
# The RealSense driver's own subtree, as tfs.cpp builds it.
run ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link_front camera_camera_link_front
run ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_camera_link_front camera_gyro_frame
run ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_gyro_frame camera_imu_frame
run ros2 run tf2_ros static_transform_publisher 0 0 0 -1.5707963 0 -1.5707963 camera_imu_frame camera_imu_optical_frame
# Parameters mirrored from camera_realsense.launch.py.
run ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -r __node:=imu_filter \
  -r /imu/data_raw:=/camera/camera/imu -p use_mag:=false -p publish_tf:=false -p world_frame:=enu \
  -p orientation_stddev:=0.05 -p gain:=0.05 -p zeta:=0.0 -p remove_gravity_vector:=false
run ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_local --params-file "$CFG"

timeout 25 bash -c 'until ros2 node list 2>/dev/null | grep -q ekf_local; do :; done'

fail() { echo "ABORT: $1"; echo "--- node log ---"; tail -25 "$WORK/nodes.log"; exit 1; }
# A stale ekf_node from an earlier run would interleave its own state into
# /odometry/filtered and make the result meaningless.
n=$(ros2 node list 2>/dev/null | grep -c '^/ekf_local$')
[ "$n" = 1 ] || fail "$n ekf_local nodes on ROS_DOMAIN_ID=$ROS_DOMAIN_ID (expected 1)"
# Without this transform robot_localization discards every IMU message in
# silence and the filter simply reports zeros, which looks like a real result.
timeout 15 ros2 run tf2_ros tf2_echo base_footprint camera_imu_optical_frame 2>&1 \
  | grep -q Translation || fail "TF base_footprint -> camera_imu_optical_frame does not resolve"

python3 "$HERE/tilt_bench.py" "$1" "$2"
