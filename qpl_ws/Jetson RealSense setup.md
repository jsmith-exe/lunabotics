For the IMU to work, you must use the RSUSB backend; to do this, you must build from source.
To build from source, you must tweak the libcurl dependency when the build initially fails.
- Run `./libuvc_installation.sh` (will fail, this is fine for now - will also take 20-30min)
- In `/home/caleb/librealsense_build/librealsense-master/CMake/external_libcurl.cmake`
  - Add -DCURL_USE_LIBIDN2=OFF to line 8
  - Add the following after line 61, with the same indentation of line 61.
```
find_library(LIBIDN2 idn2 REQUIRED)
target_link_libraries(curl INTERFACE ${LIBIDN2})
```
- Complete RealSense SDK setup.
```
cd librealsense_build/librealsense-master/build/
make -j2
sudo make install
```
- Configure ROS repositories: https://wiki.ros.org/Installation/Ubuntu/Sources
- Install ROS Humble realsense wrapper and patchelf.
```
sudo apt install ros-humble-realsense2-*
sudo apt install patchelf
```
- The ROS wrapper will be pointing to the wrong version of realsense, we want our custom build.
```bash
# This will show the ros version (/opt/ros/humble/...)
ldd /opt/ros/humble/lib/librealsense2_camera.so | grep realsense2.so

# This will force the ros wrapper to use the custom build of realsense
sudo patchelf --set-rpath /usr/local/lib --force-rpath \
  /opt/ros/humble/lib/librealsense2_camera.so

# Verify; it should indicate it's using /user/local/lib, which is where the custom build of realsense should be located.
ldd /opt/ros/humble/lib/librealsense2_camera.so | grep realsense2.so
```
