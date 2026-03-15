#!/bin/bash

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
  unset RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI=file://"${QPL_PROJECT}"/dds/cyclonedds.xml
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

if [ "${LIBGL_ALWAYS_SOFTWARE}" == "true" ]; then
  echo "Note: LIBGL_ALWAYS_SOFTWARE set to true, only software rendering will be used."
  qpl_use_software_render
fi

# Optional: if you ever want to quickly check what mode you're in
qpl_render_status() {
  echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-<unset>}"
  echo "GALLIUM_DRIVER=${GALLIUM_DRIVER:-<unset>}"
}

# -------------------- Shortcuts --------------------
alias qpl_build='${QPL_PROJECT}/process/build.sh'
alias qpl_packages='${QPL_PROJECT}/process/install_packages.sh'
alias qpl_kb='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop'
alias qpl_slam='ros2 launch qpl_rover online_async_launch.py'
alias nav2='ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true'

# Use functions (not aliases) for anything that needs env switching
qpl_headless() {
  # Force CPU-only rendering for anything that uses OpenGL (RViz/Gazebo client)
  qpl_use_software_render

  # Extra: show what renderer this shell will use (helpful when debugging)
  echo "[qpl_headless] LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}  GALLIUM_DRIVER=${GALLIUM_DRIVER}"
  glxinfo -B 2>/dev/null | grep -i "OpenGL renderer" || true

  ros2 launch qpl_rover headless_sim.launch.py "$@"
}


qpl_sim() {
  qpl_use_gpu_render

  echo "[qpl_headless] LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}  GALLIUM_DRIVER=${GALLIUM_DRIVER}"
  glxinfo -B 2>/dev/null | grep -i "OpenGL renderer" || true

  ros2 launch qpl_rover launch_sim.launch.py "$@"
}

qpl_rviz() {
  qpl_use_gpu_render

  echo "[qpl_headless] LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}  GALLIUM_DRIVER=${GALLIUM_DRIVER}"
  glxinfo -B 2>/dev/null | grep -i "OpenGL renderer" || true

  ros2 launch basestation rviz.launch.py "$@"
}

# -------------------- Make scripts executable --------------------
chmod +x "$QPL_PROJECT/process/install_packages.sh" 2>/dev/null || true
chmod +x "$QPL_PROJECT/process/build.sh" 2>/dev/null || true

# -------------------- Source workspace --------------------
source "$QPL_PROJECT/qpl_ws/install/setup.bash"