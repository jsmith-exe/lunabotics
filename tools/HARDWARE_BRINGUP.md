# Hardware bring-up: RealSense + gyro yaw + visual odometry

Run these in order on the rover. Each step fails loudly on its own; do not skip ahead,
because the failure modes downstream are silent.

Everything below landed in merge `f0427e8` (`dead-reckoning` + `imu-enable`) and has been
verified in sim only. Nothing here has touched real hardware.

---

## 0. Dependencies

```bash
for p in rtabmap_odom robot_localization imu_filter_madgwick realsense2_camera tf2_ros; do
  printf '%-24s ' "$p"; ros2 pkg prefix $p >/dev/null 2>&1 && echo present || echo MISSING
done
```

All five are present on the dev laptop. `rtabmap_odom` is the one most likely missing on the
Jetson — `sudo apt install ros-humble-rtabmap-odom`.

## 1. Camera streams

```bash
ros2 launch qpl_rover camera_realsense.launch.py
ros2 topic hz /depth_camera_front/color/image_raw
ros2 topic hz /depth_camera_front/aligned_depth_to_color/image_raw
```

Expect ~30 Hz on both at 1280x800. These plus
`/depth_camera_front/color/camera_info` are exactly `ROVER_TOPICS` in
`launch/components/vslam_launch.py`.

If `aligned_depth_to_color` is absent, `align_depth.enable` did not take —
check `launch/hardware/camera_realsense.launch.py`.

## 2. TF — the highest-risk step

`rgbd_odometry` needs `base_footprint` -> the colour image's optical frame. If that lookup
misses it publishes **nothing and logs nothing useful**, which looks identical to a camera
that simply isn't running.

```bash
ros2 topic echo --once /depth_camera_front/color/image_raw --field header.frame_id
ros2 run tf2_ros tf2_echo base_footprint <that frame>
```

The second command must resolve. `front_camera_tf_transform` in `launch/rover.launch.py`
bridges `camera_link_front` -> `camera_camera_link_front` as a **guess** at the driver's
naming. If the lookup fails, fix the child frame *there*, not in the URDF — the URDF's
`camera_link_front` is also the mount point the physical extrinsics hang off.

## 3. IMU

```bash
ros2 topic hz /camera/camera/imu
ros2 node info /ekf_local | sed -n '/Subscribers/,/Publishers/p'
```

`ekf_local` must list `/camera/camera/imu`, `/diff_cont/odom` and `/vo/odom`. (Confirmed
locally against the hardware config.)

`/camera/camera/imu` is deliberately **not** in the remap list in
`camera_realsense.launch.py`. Remapping it silently removes the gyro and drops heading back
to wheel odometry — there is a comment there guarding this.

## 4. Visual odometry standalone

```bash
qpl_vslam          # alias for: ros2 launch qpl_rover vslam_launch.py
ros2 topic hz /vo/odom
```

Watch the `Odom: quality=` log lines:

| | sim measured | hardware target |
|---|---|---|
| inliers (median) | ~200 | comfortably above `Vis/MinInliers: 10` |
| `update time` | ~25 ms @848x480 | must stay under the frame period (~33 ms @30Hz) |
| zero-inlier frames | 6-8% | if much worse, the surface lacks texture |

**If `update time` exceeds ~33 ms**, the Jetson cannot hold full resolution. Set
`use_low_quality = "true"` in `launch/rover.launch.py` (line ~62) for 424x240x15 — no other
change needed. Prefer that over cutting `Vis/MaxFeatures`.

**If `/vo/odom` is silent**, go back to step 2. That is almost always TF.

## 5. Fused

```bash
ros2 launch qpl_rover rover.launch.py use_vslam:=true
```

Drive a slow out-and-back of a few metres and check `/odometry/filtered` returns near its
start. Then a square, and check heading closes.

Compare against `use_vslam:=false` on the same path — that difference is the whole point,
and it is the hardware equivalent of the sim A/B in `tools/dr_results/baseline/SUMMARY.txt`.

## 6. Calibration still outstanding

Independent of VO, and now the largest remaining error source:

- `wheel_separation` in `config/my_controllers.yaml` is **0.5836 and known wrong** — it is
  the chassis width, predating a re-measure, and ignores both the outboard wheels and
  skid-steer scrub. Expect >= 0.895.
- Sim runs showed the effective track is strongly **rate-dependent**: ~1.98 m measured at
  0.4 rad/s versus the 1.161 m calibrated at 0.8-1.0 rad/s. No single constant is correct.
  This is why the gyro owns heading and wheel yaw is only a fallback.
- The covariances in `my_controllers.yaml` are reasoned starting values, not measured. Refine
  from real bags on the competition surface.

## Reproducing the sim A/B

The harness that produced the numbers quoted above:

```bash
python3 tools/dr_drive.py --laps 1        # open-loop square, timed on the SIM clock
python3 tools/dr_eval.py --duration 70    # drift vs ground truth, relative to a latched start
```

`tools/dr_results/baseline/` holds the pre-merge runs (both friction conditions) and
`tools/dr_results/post_merge/` the regression re-run, each with a `SUMMARY.txt`. Note
`dr_drive.py` times segments on the sim clock deliberately — VO costs CPU and drags the
real-time factor down, so a wall-clock pattern would drive a shorter path in the VO arm and
the two conditions would not be comparable.
