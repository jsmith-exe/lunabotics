# Contains generic executables and aliases for running the rover, sim, and related tools

# Aliases
alias qpl_kb='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop'
alias qpl_nav='ros2 launch qpl_rover navigation_launch.py'
alias qpl_vslam='ros2 launch qpl_rover vslam_launch.py'
alias qpl_rover='ros2 launch qpl_rover rover.launch.py'
alias qpl_autonomy='ros2 launch qpl_autonomy autonomy.launch.py'
alias qpl_excavate='ros2 launch qpl_autonomy excavation.launch.py'
alias qpl_construct='ros2 launch qpl_autonomy construction.launch.py'
alias qpl_full_auto='ros2 launch qpl_autonomy full_autonomy.launch.py'
alias qpl_blind_excavate='ros2 launch qpl_autonomy blind_excavation.launch.py'
alias qpl_blind_construct='ros2 launch qpl_autonomy blind_construction.launch.py'
alias qpl_components='ros2 launch qpl_rover components.launch.py'
alias diffbot='ros2 launch diffdrive_canbus diffbot.launch.py'
alias qpl_restart_daemon="ros2 daemon stop && ros2 daemon start"
alias qpl_bro="qpl_build && qpl_rover"
alias qpl_spare="ros2 launch qpl_rover spare.launch.py"

alias qpl_logs_rec="ros2 run basestation log_recorder"
alias qpl_logs_cat="cat ~/rosout_combined.log"
alias qpl_logs_clear="rm ~/rosout_combined.log"


# -------------------- Simulation + RViz --------------------
# Add 'export LIBGL_ALWAYS_SOFTWARE=true' to bashrc if problems with rendering
qpl_sim() {
  if [ -z "${LIBGL_ALWAYS_SOFTWARE}" ]; then
    qpl_use_gpu_render
  fi
  qpl_print_renderer

  ros2 launch qpl_rover sim.launch.py "$@"
}

qpl_headless() { # Sim with no GUI
  qpl_use_software_render
  qpl_print_renderer

  ros2 launch qpl_rover sim.launch.py "$@" headless:=true
}

qpl_sim_minimal() {
  qpl_use_software_render
  qpl_print_renderer

  ros2 launch qpl_rover sim.launch.py "$@" headless:=true run_components:=false
}

qpl_rviz() {
  if [ -z "${LIBGL_ALWAYS_SOFTWARE}" ]; then
    qpl_use_gpu_render
  fi
  qpl_print_renderer

  ros2 launch basestation rviz.launch.py use_sim_time:=true"$@"
}

qpl_rviz_rover() {
  if [ -z "${LIBGL_ALWAYS_SOFTWARE}" ]; then
    qpl_use_gpu_render
  fi
  qpl_print_renderer

  ros2 launch basestation rviz.launch.py "$@"
}


# -------------------- Render mode helpers --------------------
qpl_use_software_render() {
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
}

qpl_use_gpu_render() {
  # Unset software-render overrides so GL can use your GPU stack again
  unset LIBGL_ALWAYS_SOFTWARE
  unset GALLIUM_DRIVER
}

if [ -n "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
  qpl_use_software_render
fi

qpl_print_renderer() {
  if command -v glxinfo >/dev/null 2>&1; then
    echo "[Renderer Info]"
    glxinfo -B | grep -E "OpenGL vendor|OpenGL renderer" || true
  else
    echo "[Renderer Info] glxinfo not installed (install mesa-utils)"
  fi
  echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-<unset>}"
  echo "GALLIUM_DRIVER=${GALLIUM_DRIVER:-<unset>}"
}


# -------------------- Other --------------------
qpl_list_processes() {
  local colors=(32 33 34 35 36)
  local n_colors=${#colors[@]}
  local i=0

  # Gets the processes containing 'ros' to PID and rest of line as args, prints them as different colours
  pgrep -af ros | while read -r pid args; do
      local node_name
      node_name=$(echo "$args" | grep -oP '(?<=__node:=)\S+')
      color=${colors[$((i % n_colors))]}
      [[ -z "$node_name" ]] && node_name="$args" # Fall back to using args directly
      echo -e "\033[${color}m${pid} ${node_name:0:100}\033[0m"
      (( i++ ))
    done
}

qpl_kill_processes() {
  pkill -f ros
  pkill -f gazebo
}
