#!/bin/bash
# Builds diffdrive_canbus and runs its tests on the virtual interface vcan0.
# The tests never use can0, so this is safe to run with the motors powered.
#
# Usage (from anywhere): qpl_ws/src/diffdrive_canbus/test/run_can_tests.sh
set -e

WS_DIR="$(cd "$(dirname "$0")/../../.." && pwd)"

if ! ip link show vcan0 >/dev/null 2>&1; then
  echo "Creating vcan0 (needs sudo)..."
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan
  sudo ip link set vcan0 up
fi

source /opt/ros/humble/setup.bash
cd "$WS_DIR"
colcon build --symlink-install --packages-select diffdrive_canbus
colcon test --packages-select diffdrive_canbus --ctest-args -R test_can_fixes
colcon test-result --verbose --test-result-base build/diffdrive_canbus
