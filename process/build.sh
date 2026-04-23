#!/bin/bash

qpl_build() {
  mode=${1}

  previous_path=$(pwd)
  cd "$QPL_PROJECT/qpl_ws" || {
    echo "Could not find qpl_ws. Check QPL_PROJECT is set correctly."
    exit 1
  }

  if [ "$mode" = "clean" ]; then
    echo "Deleting previous build artifacts..."
    rm -rf build/ install/ log/
  fi

  colcon build --symlink-install

  cd "$previous_path" || {
    echo "Could not return to previous directory."
  }

  source "$QPL_PROJECT/qpl_ws/install/setup.bash"
}

alias qpl_clean_build="qpl_build clean"
