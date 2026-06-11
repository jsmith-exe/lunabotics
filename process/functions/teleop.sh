alias qpl_controller_fwd='ros2 run basestation nav_pub'
alias qpl_teleop='qpl_wsl_run_controller'

# For WSL: a path that can be used in PowerShell to run controller scripts without worrying about WSL path translation
get_qpl_project_windows_path() {
  wslpath -w "$QPL_PROJECT"
}
qpl_wsl_run_controller() {
  powershell.exe -ExecutionPolicy Bypass -Command "
      cd $(get_qpl_project_windows_path);
      .\run_controller.ps1;
  "
}

qpl_wsl_setup_controller() {
  echo "Warning: this script may be broken. If it fails, run the steps in $(get_qpl_project_windows_path)\qpl_ws\src\basestation manually."
  powershell.exe -ExecutionPolicy Bypass -Command "
      cd $(get_qpl_project_windows_path)\qpl_ws\src\basestation;
      .\setup_controller.ps1;
  "
}