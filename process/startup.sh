#!/bin/bash
source /opt/ros/humble/setup.bash
# -------------------- ROS daemon --------------------
# The ROS daemon sometimes doesn't start on WSL.
ros2 daemon start >/dev/null 2>&1 || true

# -------------------- Project path --------------------
# Ensure QPL_PROJECT is set (you already rely on it below)
if [ -z "${QPL_PROJECT:-}" ]; then
  echo "ERROR: QPL_PROJECT is not set. Export it before sourcing this script."
  return 1 2>/dev/null || exit 1
fi

# -------------------- ROS2 network --------------------
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

use_server_sim() {
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://"${QPL_PROJECT}"/dds/cyclonedds.xml
  echo "Configured to use server sim. Type 'use_local_sim' to use local simulation."
}

use_local_sim() {
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI
  echo "Configured to use local sim. Type 'use_server_sim' to use server simulation."
}

if [ -n "$USE_SERVER_SIM" ]; then
  use_server_sim
else
  use_local_sim
fi

# -------------------- Render mode helpers --------------------
qpl_use_software_render() {
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
}

qpl_use_gpu_render() {
  if [ -n "${LIBGL_ALWAYS_SOFTWARE}" ]; then
    return
  fi
  # Unset software-render overrides so GL can use your GPU stack again
  unset LIBGL_ALWAYS_SOFTWARE
  unset GALLIUM_DRIVER
}

if [ -n "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
  # echo "Note: software rendering override is set."
  qpl_use_software_render
fi

qpl_render_status() {
  echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-<unset>}"
  echo "GALLIUM_DRIVER=${GALLIUM_DRIVER:-<unset>}"
  glxinfo -B 2>/dev/null | grep -i "OpenGL renderer" || true
}

qpl_print_renderer() {
  if command -v glxinfo >/dev/null 2>&1; then
    echo "[Renderer Info]"
    glxinfo -B | grep -E "OpenGL vendor|OpenGL renderer" || true
  else
    echo "[Renderer Info] glxinfo not installed (install mesa-utils)"
  fi
}

# -------------------- Shortcuts --------------------
alias qpl_build='${QPL_PROJECT}/process/build.sh'
alias qpl_clean_build='${QPL_PROJECT}/process/clean_build.sh'
alias qpl_packages='${QPL_PROJECT}/process/install_packages.sh'
alias qpl_kb='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop'
alias qpl_slam='ros2 launch qpl_rover online_async_launch.py'
alias qpl_cloud_to_scan='ros2 launch qpl_rover point_cloud_to_scan.launch.py'
alias qpl_nav='ros2 launch qpl_rover navigation_launch.py'
alias qpl_rover='ros2 launch qpl_rover launch_rover.launch.py'

# Use functions (not aliases) for anything that needs env switching
qpl_headless() {
  qpl_use_software_render

  echo "[qpl_headless] LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
  echo "[qpl_headless] GALLIUM_DRIVER=${GALLIUM_DRIVER}"

  qpl_print_renderer

  ros2 launch qpl_rover headless_sim.launch.py "$@"
}


qpl_sim() {
  qpl_use_gpu_render

  echo "[qpl_sim] LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-<unset>}"
  echo "[qpl_sim] GALLIUM_DRIVER=${GALLIUM_DRIVER:-<unset>}"

  qpl_print_renderer

  ros2 launch qpl_rover launch_sim.launch.py "$@"
}

qpl_rviz() {
  qpl_use_gpu_render

  echo "[qpl_rviz] LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-<unset>}"
  echo "[qpl_rviz] GALLIUM_DRIVER=${GALLIUM_DRIVER:-<unset>}"

  qpl_print_renderer

  ros2 launch basestation rviz.launch.py "$@"
}

# -------------------- Make scripts executable --------------------
chmod +x "$QPL_PROJECT/process/install_packages.sh" 2>/dev/null || true
chmod +x "$QPL_PROJECT/process/build.sh" 2>/dev/null || true
chmod +x "$QPL_PROJECT/process/clean_build.sh" 2>/dev/null || true

# -------------------- Source workspace --------------------
source "$QPL_PROJECT/qpl_ws/install/setup.bash"