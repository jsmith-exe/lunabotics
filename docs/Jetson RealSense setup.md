# Jetson RealSense setup

The camera IMU only works with librealsense's RSUSB backend, and neither the apt
nor the ROS package is built with it. So we build the SDK from source and force
the ROS wrapper to load our build instead of the packaged one.

Two versions have to agree: wrapper `ros-humble-realsense2-camera` 4.NN.N pairs
with librealsense 2.NN.N. A mismatch shows up at node startup as:

```
** running with a different librealsense version **
```

`RS_VERSION` at the top of `libuvc_installation.sh` is what pins this. It is
currently 2.58.4, matching wrapper 4.58.4.

## Setup

1. Configure ROS repositories: https://wiki.ros.org/Installation/Ubuntu/Sources

2. Install the ROS wrapper **first**, so the build script can finish everything
   in one pass:

```
sudo apt install ros-humble-realsense2-*
```

3. Run the build script. 20-30 min on a Jetson.

```
cd qpl_ws
./libuvc_installation.sh
```

It builds librealsense `$RS_VERSION` with `FORCE_RSUSB_BACKEND=ON`, installs the
udev rules, clears stale copies out of `/usr/local/lib`, points the ROS wrapper
at our build, and holds the three realsense packages.

If you run the script before step 2, it does everything except the last part and
prints the two commands to run afterwards.

## Why the wrapper has to be patched

`ros-humble-realsense2-camera` links the soname `librealsense2.so.2.58`, and
`/opt/ros/humble/lib/aarch64-linux-gnu` is on `LD_LIBRARY_PATH`. The ROS SDK
package `ros-humble-librealsense2` ships that same soname built against V4L2,
and on the default search order it beats `/usr/local/lib`. The result streams
colour and depth perfectly and publishes no IMU, which silently costs both EKFs
their roll/pitch input. That is a bad failure to debug in the field, because
nothing errors.

`patchelf --set-rpath /usr/local/lib --force-rpath` writes a `DT_RPATH`, which
the loader searches *before* `LD_LIBRARY_PATH`. It is the only thing that makes
the choice stick.

Every apt upgrade of the wrapper replaces the `.so` and drops that RPATH, and
the wrapper can move independently of the SDK because its dependency on
`ros-humble-librealsense2` is unversioned. That is what the holds are for.
`ros-humble-realsense2-camera` is deliberately not in `qpl_packages`.

## Verify

```bash
# Must resolve to /usr/local/lib, not /opt/ros/...
ldd /opt/ros/humble/lib/librealsense2_camera.so | grep realsense2.so

# Must say RPATH, not RUNPATH. Only RPATH outranks LD_LIBRARY_PATH.
readelf -d /opt/ros/humble/lib/librealsense2_camera.so | grep -E 'RPATH|RUNPATH'

apt-mark showhold | grep realsense
```

With the camera plugged in, this is the test that actually matters. Matching
version numbers do not prove the RSUSB backend is live; IMU topics do:

```bash
ros2 topic hz /camera/camera/imu
```

Careful when debugging: `rs-enumerate-devices` resolves to `/opt/ros/humble/bin`,
which is the V4L2 build, and will report no IMU even when the node is fine. Use
`/usr/local/bin/rs-enumerate-devices` to exercise our build.

## Upgrading on purpose

```bash
sudo apt-mark unhold ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
sudo apt install ros-humble-realsense2-camera         # note the new 4.NN.N
```

Then set `RS_VERSION` in `libuvc_installation.sh` to the matching 2.NN.N and
re-run it. It re-pins, rebuilds, re-patches and re-holds.

## If the libcurl build fails

Not needed at 2.58.4 — it builds clean with `CHECK_FOR_UPDATES=ON` and an
unpatched tree (verified 2026-09-20). Kept in case a future bump brings it back.

The failure is libcurl's libidn2 dependency. In `CMake/external_libcurl.cmake`
of the extracted source tree:

- Add `-DCURL_USE_LIBIDN2=OFF` to the `set(CURL_FLAGS ...)` line.
- In the non-Windows branch, just after
  `target_link_libraries(curl INTERFACE OpenSSL::SSL OpenSSL::Crypto)`, add at
  the same indentation:

```cmake
find_library(LIBIDN2 idn2 REQUIRED)
target_link_libraries(curl INTERFACE ${LIBIDN2})
```

Then resume the build in place rather than re-running the script from the top:

```
cd ~/librealsense_build/librealsense-<RS_VERSION>/build/
make -j2
sudo make install
```
