#!/bin/bash -xe

# Based on https://github.com/realsenseai/librealsense/blob/master/scripts/libuvc_installation.sh
# Builds and installs librealsense from source using RSUSB/libuvc backend.
# This hasn't been recently tested, but has been modified. Potentially broken.

# RS_VERSION must match the librealsense the ROS wrapper was built against. Check with:
#   apt-cache show ros-humble-realsense2-camera | grep ^Version
RS_VERSION=2.58.4

#Locally suppress stderr to avoid raising not relevant messages
exec 3>&2
exec 2> /dev/null
con_dev=$(ls /dev/video* | wc -l)
exec 2>&3

if [ $con_dev -ne 0 ];
then
        echo -e "\e[32m"
        read -p "Remove all RealSense cameras attached. Hit any key when ready"
        echo -e "\e[0m"
fi

lsb_release -a
echo "Kernel version $(uname -r)"
sudo apt-get update
cd ~/
sudo rm -rf ./librealsense_build
mkdir librealsense_build && cd librealsense_build

if [ $(sudo swapon --show | wc -l) -eq 0 ];
then
        echo "No swapon - setting up 1Gb swap file"
        sudo fallocate -l 2G /swapfile
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
        sudo swapon /swapfile
        sudo swapon --show
fi

echo Installing Librealsense-required dev packages
sudo apt-get install git cmake libssl-dev freeglut3-dev libusb-1.0-0-dev pkg-config libgtk-3-dev unzip patchelf -y
rm -f "./v${RS_VERSION}.zip"

wget "https://github.com/realsenseai/librealsense/archive/refs/tags/v${RS_VERSION}.zip"
unzip "./v${RS_VERSION}.zip" -d .
cd "./librealsense-${RS_VERSION}"

echo Install udev-rules
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo cp config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON
make -j2
sudo make install

# Older installs of this script left 2.57/2.58.1 copies behind in /usr/local/lib.
# Stale sonames there outrank nothing, but they make it impossible to tell at a
# glance which SDK the wrapper actually loaded, so clear anything but RS_VERSION.
echo Removing stale librealsense copies from /usr/local/lib
for stale in /usr/local/lib/librealsense2.so.2.* /usr/local/lib/librealsense2-gl.so.2.*; do
        case "$stale" in
                *".so.${RS_VERSION}") continue ;;
                *"*") continue ;;
        esac
        # Keep the two-component soname symlinks that point at RS_VERSION.
        if [ -L "$stale" ] && [ "$(readlink "$stale")" = "$(basename "${stale%.*.*}").${RS_VERSION}" ]; then
                continue
        fi
        echo "  rm $stale"
        sudo rm -f "$stale"
done
sudo ldconfig

echo -e "\e[92m\n\e[1mLibrealsense script completed.\n\e[0m"

# --- Point the ROS wrapper at this build -------------------------------------
# ros-humble-realsense2-camera links the librealsense2.so.2.NN soname, and
# /opt/ros/humble/lib/aarch64-linux-gnu is on LD_LIBRARY_PATH. If the ROS SDK
# package ever supplies a matching soname it silently wins over /usr/local/lib
# and swaps this RSUSB build for a V4L2 one, which streams video fine but has no
# IMU. DT_RPATH is searched before LD_LIBRARY_PATH, so it is what makes the
# choice stick. Every apt upgrade of the wrapper replaces the .so and drops the
# RPATH, hence the holds below.
WRAPPER=/opt/ros/humble/lib/librealsense2_camera.so
if [ -f "$WRAPPER" ]; then
        echo Re-pointing the ROS wrapper at /usr/local/lib
        sudo patchelf --set-rpath /usr/local/lib --force-rpath "$WRAPPER"
        ldd "$WRAPPER" | grep 'realsense2\.so' || true

        echo Holding the realsense packages so apt cannot drift them
        sudo apt-mark hold \
                ros-humble-librealsense2 \
                ros-humble-realsense2-camera \
                ros-humble-realsense2-camera-msgs
else
        echo -e "\e[33m"
        echo "ROS wrapper not installed yet. After 'sudo apt install ros-humble-realsense2-*', run:"
        echo "  sudo patchelf --set-rpath /usr/local/lib --force-rpath $WRAPPER"
        echo "  sudo apt-mark hold ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs"
        echo -e "\e[0m"
fi
