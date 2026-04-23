# Build tools

qpl_build() {
  mode=${1}

  # Involves changing directories, so save the current path to return to it later
  previous_path=$(pwd)
  cd "$QPL_PROJECT/qpl_ws" || {
    echo "Could not find qpl_ws. Check QPL_PROJECT is set correctly."
    exit 1
  }

  # Option for clean build
  if [ "$mode" = "clean" ]; then
    echo "Deleting previous build..."
    rm -rf build/ install/ log/
  fi

  colcon build --symlink-install
  source "$QPL_PROJECT/qpl_ws/install/setup.bash"

  cd "$previous_path" || {
    echo "Could not return to previous directory."
  }

}

alias qpl_clean_build="qpl_build clean"
