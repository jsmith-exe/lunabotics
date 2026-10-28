# Full Autonomy Tuning: EKFs and Nav2

How to tune localisation (the two EKFs and their sensors) and navigation (Nav2) on the real rover, on regolith.

Almost every number in the current config was tuned in Gazebo. **Regolith hasn't been simulated**: the Gazebo wheels grip a rigid floor, so wheel slip, sinkage, skid-steer turning, dust, and how textured the ground looks to the cameras are all unknown. Treat every sim value as a starting point, not a known-good value. Expect wheel odometry in particular to behave very differently.

---

## 1. Ground rules

- **Work bottom-up.** Sensors first, then the local EKF, then AprilTags and the global EKF, then obstacle detection, then Nav2, then the autonomy waypoints. Nav2 can't be tuned on top of bad localisation; you'd only be tuning it to hide the localisation errors.
- **Change one thing at a time, and record it** (see [section 15](#15-tuning-log)). Most of these parameters interact.
- **Record a bag for every test** so results can be re-checked and EKF changes replayed offline (see [section 3](#3-tools-and-ground-truth)).
- **Ground truth comes from a tape measure, not the rover.** Mark start and end points on the floor. The AprilTag can't be used as ground truth while you're tuning the AprilTag.
- **Safety:** keep teleop open while testing autonomy. Teleop beats Nav2 in `drive_mux` (priority 100 vs 50), and `qpl_full_auto_stop` cancels the Nav2 goal. Keep the e-stop in reach.
- **Shared values:** `wheel_separation` (and other shared values) are hardcoded in several places on purpose. When you change one, grep for the old value and update every copy: `my_controllers.yaml`, basestation `constants.py` (`TANK_WHEEL_SEPARATION`), and the HUD (`hud_node.py`, `demo.py`, `main.js`).

---

## 2. Pre-flight checks

Do these before any tuning run.

| Check | How | Expect |
|---|---|---|
| **Nav2 uses wall-clock time** | `navigation_launch.py` defaults `use_sim_time` to `true`, and the `qpl_nav` alias doesn't override it. On the rover, run `qpl_nav use_sim_time:=false`. | Nav2 doesn't sit waiting for `/clock`, and TF lookups don't time out |
| TF tree complete | `ros2 run tf2_tools view_frames` | `map → odom → base_footprint → base_link → …`, with one publisher per edge |
| Only the EKFs publish moving TF | `ros2 run tf2_ros tf2_monitor` | `odom → base_footprint` from `ekf_local`, `map → odom` from `ekf_global`; nothing from `diff_cont` or VO |
| Sensor rates | `ros2 topic hz` on `/diff_cont/odom`, `/vo/odom`, `/camera/camera/imu_standard`, `/imu/data`, `/apriltag/pose` | Steady; VO around 10 Hz on the Jetson |
| Covariances are not zero | `ros2 topic echo --once` each sensor topic | No all-zero covariance matrices. `robot_localization` reads zero as "perfect" |
| Camera mounts match the URDF | Measure camera positions and pitch on the rover and compare with `depth_camera_front.xacro` / `depth_camera_rear.xacro` | Within ~1 cm and ~1°. Mount errors become false obstacles and AprilTag pose error |
| Arena tag position | Measure the real tag and update `config/arena/<arena>.yaml` (`apriltag.position`/`quaternion`); check `selector.yaml` | Done on the day, in the real arena |

---

## 3. Tools and ground truth

**Record:**
```bash
ros2 bag record -o tune_<test>_<date> \
  /tf /tf_static /diff_cont/odom /vo/odom /camera/camera/imu_standard /imu/data \
  /apriltag/pose /apriltag/pose_gated /odometry/filtered /odometry/global \
  /joint_states /cmd_vel_nav /cmd_vel_out /diagnostics \
  /navigate_to_pose/_action/status /plan /local_plan
```
For obstacle tuning, add the depth image topics and `/depth_camera_*/obstacles` and `/depth_camera_*/clearing`. They're large, so keep those bags short.

**Plot:** PlotJuggler (`ros2 run plotjuggler plotjuggler`) is the fastest way to overlay the same quantity from different sources, e.g. the wheel odom vx against the VO vx against the EKF vx.

**Replay EKF changes offline:** play a bag without the EKF outputs and run only the EKF with the new parameters:
```bash
ros2 bag play <bag> --clock --topics /diff_cont/odom /vo/odom /camera/camera/imu_standard /imu/data /apriltag/pose_gated /tf_static
# then launch the EKF with use_sim_time:=true
```
Running the EKFs against bags (which give it `/clock`) is how to compare covariance changes fairly: every run sees exactly the same data.

**Ground truth:**
- Straight lines: tape-measure start and end marks.
- Turns: mark the rover's heading against a straight line on the floor (a taped line or the arena wall), or use the gyro for short spins. Gyro yaw-rate integration is good for tens of seconds.
- Absolute pose (for the AprilTag): measured points on a grid from the arena corner, which is the `map` origin.

---

## 4. Wheel odometry (`diff_cont`)

**Why regolith differs:** the sim wheels don't slip. On regolith expect:
- **Longitudinal slip**: the wheels turn further than the rover moves, so `/diff_cont/odom` overestimates distance. Slip is worse uphill, when pushing into soil, and when the bucket is dragging.
- **Sinkage**: changes the effective wheel radius.
- **Skid-steer turning**: wheels must scrub sideways to turn. The rover turns slower than `(v_r - v_l) / wheel_separation`, i.e. the *effective* wheel separation is larger than the geometric one. The sim measured a turn rate of about 0.8× the commanded rate; regolith will be different.

**File:** `qpl_ws/src/qpl_rover/config/my_controllers.yaml` (`diff_cont`).

### 4.1 Straight-line slip test
1. Teleop straight for 3 m at 0.1 m/s and at 0.2 m/s (the Nav2 max). Do 3 runs each: on flat regolith, uphill, and downhill.
2. Compare the tape-measured distance `d_true` with the odom distance `d_odom`.
3. Slip ratio `s = 1 - d_true / d_odom`.

| Result | Action |
|---|---|
| Slip consistent (spread under ~5%) | Correct `wheel_radius` by `r × d_true / d_odom`. This bakes the average slip into the radius, which is fine as long as it's consistent. |
| Slip varies a lot (by speed or slope) | Leave the radius at its measured physical value and loosen the vx entry in `twist_covariance_diagonal` so the EKF leans on VO and the IMU. |

**Setting the vx covariance:** use the variance of the measured velocity error, `(std of vx error)²`. For example, a ±15% error at 0.2 m/s gives std 0.03 m/s and variance ≈ 1e-3. The current value is 0.01 (std 0.1 m/s). What matters is the weight *relative* to VO, so compare it with the `/vo/odom` twist covariance.

### 4.2 Spin-in-place test (effective `wheel_separation`)
1. Spin in place at several rates for ~10 s each, in both directions.
2. Get `v_l` and `v_r` from the `/joint_states` wheel velocities × `wheel_radius`, and the yaw rate `ω` from the gyro (`/camera/camera/imu_standard` `angular_velocity.z`).
3. Effective separation = `(v_r - v_l) / ω`. The TODO in the file says to expect ≥ 0.895; the current value is 1.161 (from the sim).
4. Update every copy (see [section 1](#1-ground-rules)).

The turn ratio also sets several Nav2 timeouts (`movement_time_allowance`, Spin `time_allowance`); see [section 10](#10-nav2-controller-dwb).

Wheel odom vyaw is fused only as a loose fallback behind the gyro. If the effective separation varies a lot with the surface, loosen the vyaw covariance (currently 0.05) further rather than trying to make it accurate.

### 4.3 Keep these as they are
- `vz` covariance stays tight (0.01). It encodes "the rover can't move along its own z axis", which stops z from running away in both EKFs (see the comment in the file).
- `pose_covariance_diagonal` doesn't matter: neither EKF fuses wheel odom pose.
- `publish_tf: false` / `enable_odom_tf: false`: the EKF owns `odom → base_footprint`.

---

## 5. IMU (RealSense gyro and Madgwick)

On the rover:
- `imu0` = `/camera/camera/imu_standard`, yaw rate only.
- `imu1` = `/imu/data` (`imu_filter_madgwick`), roll/pitch only.

The Madgwick yaw is unreferenced (no magnetometer) and is never fused.

**File:** `qpl_ws/src/rover_cameras/launch/camera_realsense.launch.py` (Madgwick params).

### 5.1 Stationary test (5 min)
Record with the rover still, first with the motors idle, then with the **drum spinning**. Vibration is the most likely difference from the sim.
- **Gyro bias:** the mean of `angular_velocity.z` while still. `robot_localization` doesn't estimate gyro bias, so any bias turns straight into yaw drift in `odom` (bias in rad/s × time). Over 0.005 rad/s (≈ 17°/min) is worth dealing with; the global EKF can only fix it when it sees a tag.
- **Gyro noise:** the std of `angular_velocity.z` should match the IMU message's `angular_velocity_covariance`. Check it isn't zero.
- **Roll/pitch noise:** the std of roll/pitch from `/imu/data` sets `orientation_stddev` (currently 0.05 rad, TODO: measure).

### 5.2 Tilt test
Park on a known slope (measure it with a phone inclinometer) and compare it with roll/pitch from `/imu/data`. If the rover shows phantom tilt while accelerating or vibrating, lower the Madgwick gain (the comment in the launch file says it has already been halved from the 0.1 default).

Roll/pitch matter more than they look. `depth_obstacle_filter` measures obstacle height in the IMU-levelled `odom` frame, so a 2° tilt error moves a floor point 4 m away by 0.14 m: that's a fake obstacle or a missed one (see [section 9](#9-obstacle-detection-depth_obstacle_filter)).

---

## 6. Visual odometry (`rgbd_odometry`)

**Why regolith differs:** regolith is grey, low-texture and repetitive. Dust, the bucket in view, and the arena lighting all reduce visual features. VO may produce fewer inliers, get lost more often, or be confidently wrong.

**Files:**
- `qpl_ws/src/qpl_rover/config/vslam_odom_params_rover.yaml` (the comments record the Jetson profiling)
- `odom1_*` in both EKF configs

### 6.1 Tests
- Repeat the straight-line and spin tests from [section 4](#4-wheel-odometry-diff_cont). Compare the `/vo/odom` twist with the ground truth and with wheel odom.
- Watch the rate (`ros2 topic hz /vo/odom`) and the inlier count. `Odom/FillInfoData` is on, so find the info topic with `ros2 topic list | grep -i info`.
- Note when VO goes silent: `publish_null_when_lost: false`, so the EKF just coasts on wheels and the gyro.

### 6.2 What to change

| Problem | Try |
|---|---|
| Inliers often below `Vis/MinInliers` (10), VO lost a lot | Raise `Vis/MaxFeatures` (costs CPU, so re-check the rate); lower `Odom/ImageDecimation` to 1 if the Jetson can keep up; check camera exposure |
| Rate drops while driving | The fallback noted in the file: `Odom/Strategy "1"` + `Vis/CorType "1"` (frame-to-frame optical flow) |
| VO consistently wrong (scale or direction) | Check camera calibration and the camera TF first; if it's still wrong, run with `use_vslam:=false` and depend on wheels + IMU + tags |
| VO jumps occasionally | `odom1_twist_rejection_threshold` (3.0, a Mahalanobis distance) rejects measurements far from the EKF's current estimate; lower it to reject more |

VO is fused only as body-frame vx/vy. If wheel slip turns out to be large, VO becomes the main source of speed, so getting it reliable is worth the effort.

---

## 7. Local EKF (`ekf_local`, `odom → base_footprint`)

**File:** `qpl_ws/src/qpl_rover/config/ekf_local_params.yaml`. The `common` block is merged with the `rover` block; rover values override.

The local EKF should be **smooth and continuous**. It will drift, and that's expected; the global EKF corrects the drift. Nav2's local costmap, DWB, the velocity smoother and the recovery behaviours all work in `odom`, so jumps here are worse than drift.

The covariances the EKF uses mostly come from the **sensor messages** ([sections 4–6](#4-wheel-odometry-diff_cont)). Fix those first; then tune `process_noise_covariance` here.

### 7.1 Tests
- **Rest drift:** park for 5 minutes and check how much `/odometry/filtered` moves. Yaw drift here is the gyro bias; any x/y drift means a sensor is reporting motion while still.
- **Loop closure:** drive a 2 × 2 m square (or out and back) and return to the start mark. The difference between the start and end pose in `odom` is the accumulated drift. Record it for every config change. This is the main number to compare configs by.
- **Smoothness:** plot `/odometry/filtered` x/y/yaw. There should be no steps.
- **z:** stays near 0 on flat ground and follows real slopes. It mustn't run away (see the process-noise comment in the file).

### 7.2 What to change

| Symptom | Likely cause and change |
|---|---|
| Estimate lags behind real motion | Process noise too low, or sensor covariances too high → raise the matching `process_noise_covariance` diagonal entries (vx, vyaw) |
| Estimate jittery or noisy | The opposite → lower the process noise, or raise the noisy sensor's covariance |
| Distance too long on regolith | Wheel slip is winning → loosen the wheel odom vx covariance ([section 4.1](#41-straight-line-slip-test)) |
| Yaw drifts steadily | Gyro bias ([section 5.1](#51-stationary-test-5-min)); also check the wheel vyaw isn't fighting the gyro |
| `sensor_timeout` warnings in `/diagnostics` | A sensor is slower than 0.1 s on the Jetson → check its rate before raising the timeout |

Keep `two_d_mode: false`, the 1e-6 z/vz/az process noise, and the fused vz = 0. Together they keep the real roll/pitch and stop z from running away.

---

## 8. AprilTag and global EKF (`ekf_global`, `map → odom`)

**Files:**
- `qpl_ws/src/qpl_rover/config/ekf_global_params.yaml`
- `qpl_ws/src/qpl_rover/qpl_rover/apriltag_observer.py` (the covariance is hardcoded in `publish_pose`)
- `qpl_ws/src/qpl_rover/qpl_rover/apriltag_gate.py` (`min_period_active`)
- `qpl_ws/src/qpl_rover/config/localisation/apriltag.yaml`
- `qpl_ws/src/qpl_rover/config/arena/*.yaml`

### 8.1 AprilTag accuracy grid
1. Place the rover at tape-measured map positions: 1, 2, 3, 4 and 5 m from the tag, straight on and at ±30–45°, with each camera.
2. At each point, record 10 s of `/apriltag/pose` (the raw output, before the gate).
3. Work out the mean error (bias) and std (noise) for x, y and yaw against the measured pose.

**What the results mean:**
- **Bias** that is consistent across all points usually means the tag position or orientation in the arena yaml, or the camera TF, is wrong. Fix the measurement, not a covariance.
- **Noise** sets the covariance. `publish_pose` currently uses a diagonal of `[0.001, 0.001, 0.005, 0.1, 0.1, 0.01]`, i.e. 3 cm std in x/y and ~6° in yaw, the same at every distance. AprilTag error grows quickly with distance and viewing angle. If it does here, scale the covariance with distance (roughly ∝ range²) instead of using one value.
- If far or steep detections are bad, raise `min_decision_margin` (35) or add a maximum detection range.

Dust on the tag and arena lighting affect detection; test in conditions as close to the competition as possible.

### 8.2 Global EKF behaviour

| Parameter | Effect / when to change |
|---|---|
| `pose0_rejection_threshold` (3.0) | Mahalanobis gate on tag fixes. If the tag covariance is too tight, or the rover has drifted far since the last fix, *valid* fixes get rejected and the rover never re-localises. Symptom: the tag is visible, `/apriltag/pose_gated` is publishing, but `map → odom` doesn't move. Raise it, or fix the covariance. |
| `initial_estimate_covariance` (x/y 100) | Large on purpose, so the first tag fix is accepted wherever the rover starts. Leave as is. |
| `min_period_active` (gate, 5 s) | One fix per 5 s while a Nav2 goal is active. If `map → odom` jumps disturb Nav2 (path snaps, DWB wobbles), raise it. If drift between fixes is too large, lower it. |
| `smooth_lagged_data` / `history_length` (1.0 s) | Lets late tag fixes be applied at the right time. Raise `history_length` if the detector's delay on the Jetson is above ~1 s. |

### 8.3 Test
Drive a loop that loses sight of the tag for a while and then sees it again. Plot the `/odometry/global` correction, i.e. the size of the `map → odom` jump when the tag comes back. It should be accepted, of a size that matches the local drift you measured, and smooth enough that Nav2 doesn't react badly.

---

## 9. Obstacle detection (`depth_obstacle_filter`)

**File:** `qpl_ws/src/qpl_rover/config/depth_obstacle_filter.yaml`. The header comment explains the maths.

The defaults were sized for the sim terrain world (±7 cm hills, ~2° IMU tilt error) and the rulebook obstacles (boulders 30–40 cm wide, 12–20 cm high; craters). Real depth data on regolith will differ: noise on dark or low-texture ground, dust, and IR interference.

### 9.1 Tests (watch both costmaps in RViz)
1. **Flat empty regolith**, parked and driving: the `/depth_camera_*/obstacles` clouds and the costmaps should stay **empty**. Any marks are false positives.
2. **Known obstacles:** a boulder-sized rock and a dug crater at 1–3.5 m. They should be marked, stay marked as the rover gets close (memory), and clear once removed and seen again as floor.
3. **Walls:** marked from up to ~4 m away and kept in the global costmap.

### 9.2 What to change

| Symptom | Change |
|---|---|
| False marks on flat ground, near the rover | Raise `min_bump_height` / `min_dip_depth`, or `min_cell_points` (lone noisy returns) |
| False marks far away | Raise `height_slope_per_m` / `local_slope_per_m` (they allow more error with range); check IMU tilt ([section 5.2](#52-tilt-test)) and camera pitch in the URDF |
| False marks in the rover's own bucket, wheels or body | `self_filter_x` / `self_filter_y` |
| Boulders missed | Lower `min_bump_height` (0.07) carefully; raise `local_max_range` if they're missed at range |
| Craters missed | `min_dip_depth`, the `shadow_*` parameters |
| Marks never clear | `memory_duration`, `memory_floor_band` |
| Marks drift, or walls appear doubled | Localisation, not this node: `map → odom` jumping, or VO/odom drift |

Keep the front and rear blocks the same unless the two cameras actually behave differently (they're different models: RealSense at the front, Orbbec at the rear).

---

## 10. Nav2 controller (DWB)

**File:** `qpl_ws/src/qpl_rover/config/nav_params.yaml` (`controller_server`, `velocity_smoother`, `behavior_server`).

Do this only once localisation and obstacle detection pass their tests.

### 10.1 Velocity tracking first
Send fixed commands (teleop, or `ros2 topic pub` on `/cmd_vel_nav` with teleop idle) at 0.05, 0.1 and 0.2 m/s, and at 0.2 and 0.4 rad/s. Compare them with `/odometry/filtered` twist.

| Finding | Change |
|---|---|
| Rover stalls at low speeds (motors can't overcome regolith resistance) | Set `deadband_velocity` in `velocity_smoother`, or raise the minimum useful speed; DWB samples down to 0 |
| Real acceleration is much lower than commanded (slip) | Lower `acc_lim_x` / `decel_lim_x` (0.4) and `acc_lim_theta` (0.8). DWB assumes the rover reaches the commanded speed, so limits it can't meet make its predicted paths wrong |
| Turning in place digs the wheels in or stalls | Lower `max_vel_theta` (0.4) or make arcs more attractive; check the turn ratio |
| Turn ratio differs from 0.8× | Re-check `progress_checker.movement_time_allowance` (30 s) and the BT `Spin time_allowance` (15 s), which are both based on it |

The velocity smoother limits must stay ≥ the DWB limits, and `min_velocity[0]` must allow DWB's reverse (`min_vel_x: -0.10`).

### 10.2 Path following
Run short goals (2–3 m) in open space, then near walls, then through a gap.

| Symptom | Change |
|---|---|
| Wobbles or oscillates along the path | Lower `PathAlign.scale` / `PathDist.scale`, or check whether `map → odom` jumps are moving the path (gate, [section 8.2](#82-global-ekf-behaviour)) |
| Cuts corners or leaves the path | Raise `PathDist.scale` relative to `GoalDist` |
| Stops short near walls | `ObstacleFootprint.scale` (0.05; see the comment about 25 in the file) and the inflation `cost_scaling_factor` (8.0) |
| Shuffles at the goal trying to line up | Loosen `yaw_goal_tolerance` / `xy_goal_tolerance`, and keep them in step with `nav_autonomy.yaml` ([section 12](#12-nav-autonomy-waypoints-and-sequence)) |
| Can't find a way around obstacles, dithers | `sim_time` (3.0 s), `vx_samples` / `vtheta_samples` (check the Jetson CPU) |
| Aborts with "failed to make progress" | `required_movement_radius` / `movement_time_allowance`; `failure_tolerance` |

`inflation_radius` must stay above the footprint's inscribed radius (0.53). The footprint itself comes from measured wheel and camera positions; only change it if the hardware changes.

---

## 11. Planner, behaviour tree and recoveries

**Files:**
- `nav_params.yaml` (`planner_server`, `global_costmap`)
- `qpl_ws/src/qpl_rover/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

- The global costmap is fixed (10 × 9 m from `map` origin −0.5, −0.5) and starts empty; walls appear only once seen. `allow_unknown: true` lets the planner route through unseen space. If early plans run into walls the rover hasn't seen yet, drive a short look-around at the start so the walls get mapped.
- The BT never clears the global costmap, on purpose (see its header comment). Keep it that way.
- **Recoveries:**
  - `BackUp` is 0.60 m at 0.10 m/s. It needs to move the rover's centre past the 0.53 m inscribed radius. On regolith, check that the rover actually reverses that far (slip) within `time_allowance` (10 s).
  - `Spin` is 1.57 rad within 15 s.
  - If recoveries routinely time out, the turn ratio or slip assumptions are wrong.
- `behavior_server` must stay in `odom` (see its comment: with `map` it fails with "off grid").

---

## 12. Nav autonomy waypoints and sequence

**File:** `qpl_ws/src/qpl_autonomy/config/nav/nav_autonomy.yaml`. See the `qpl_autonomy` README for the flow.

- **Waypoints** are map-frame `[x, y, yaw]`:
  - Check them against the measured arena zones in `config/arena/<arena>.yaml`.
  - The current values are for the US arena. A UK set needs adding if the UK arena is used.
  - After a run, compare where the rover actually parked (tape) with the waypoint. Any difference is the error from localisation plus the goal tolerance.
- **Arrival:** `arrival_radius_m` (0.25) and `arrival_yaw_tolerance_rad` (0.3) must stay in line with Nav2's `xy_goal_tolerance` (0.20) and `yaw_goal_tolerance` (0.3).
- **Dig/dump timing:**
  - `dig_duration_s` and `dump_duration_s` are timed because nothing measures how full the bucket is. Tune them on real regolith by weighing what the bucket holds after different durations.
  - Note the comments disagree with the values: the README says 30 s, the yaml has 10 s and 15 s. Check what you're actually running.
- **Bucket positions:** `travel_lift_position_m` (0.22) has to keep the bucket out of the regolith on uneven ground. `excavation.lowered_position_m` (0.03) sets the dig depth. Both must stay in the actuator range (25–225 mm, `can_config.yaml`), or the sequence waits forever.
- **Retries:** `goal_timeout_s` (180) and `max_retries` (2). Size the timeout from the measured time for the longest leg plus a margin.
- **Full-cycle test:** run `qpl_full_auto` for several cycles. Record the time per cycle, the parking error at each waypoint, the regolith moved, and every recovery and retry.

---

## 13. Symptom → where to look

| Symptom | First place to look |
|---|---|
| Rover thinks it travelled further than it did | Wheel slip ([section 4.1](#41-straight-line-slip-test)), VO not contributing ([section 6](#6-visual-odometry-rgbd_odometry)) |
| Heading wanders while parked | Gyro bias ([section 5.1](#51-stationary-test-5-min)) |
| Turns overshoot or undershoot in `odom` | Effective `wheel_separation` ([section 4.2](#42-spin-in-place-test-effective-wheel_separation)), gyro covariance |
| Rover jumps in RViz (`map` fixed frame) | AprilTag fix after drift: expected but should be small; check the gate and tag accuracy ([section 8](#8-apriltag-and-global-ekf-ekf_global-map--odom)) |
| Tag visible but pose never corrects | `pose0_rejection_threshold` / the tag covariance is too tight ([section 8.2](#82-global-ekf-behaviour)) |
| Phantom obstacles on flat ground | Depth filter thresholds, IMU tilt, camera pitch in the URDF ([sections 9](#9-obstacle-detection-depth_obstacle_filter), [5.2](#52-tilt-test)) |
| Walls doubled or smeared in the global costmap | Localisation error or jumps, not the costmap |
| Rover stalls or creeps | Velocity tracking, deadband, acceleration limits ([section 10.1](#101-velocity-tracking-first)) |
| Recoveries time out | Turn ratio / slip vs the BT time allowances ([section 11](#11-planner-behaviour-tree-and-recoveries)) |
| Nav2 never starts, or TF errors | `use_sim_time` ([section 2](#2-pre-flight-checks)) |
| Arrives but autonomy doesn't start digging | Arrival tolerances vs Nav2 goal tolerances ([section 12](#12-nav-autonomy-waypoints-and-sequence)) |

---

## 14. Where every knob lives

| Area | File |
|---|---|
| Wheel odom (radius, separation, covariances) | `qpl_ws/src/qpl_rover/config/my_controllers.yaml` (+ the copies of the separation, [section 1](#1-ground-rules)) |
| Madgwick / IMU | `qpl_ws/src/rover_cameras/launch/camera_realsense.launch.py` |
| Visual odometry | `qpl_ws/src/qpl_rover/config/vslam_odom_params_rover.yaml` |
| Local EKF | `qpl_ws/src/qpl_rover/config/ekf_local_params.yaml` |
| Global EKF | `qpl_ws/src/qpl_rover/config/ekf_global_params.yaml` |
| EKF config merging | `qpl_ws/src/qpl_rover/qpl_rover/ekf_config.py` |
| AprilTag detection and covariance | `qpl_ws/src/qpl_rover/qpl_rover/apriltag_observer.py`, `config/localisation/apriltag.yaml` |
| AprilTag gate | `qpl_ws/src/qpl_rover/qpl_rover/apriltag_gate.py` |
| Arena and tag position | `qpl_ws/src/qpl_rover/config/arena/{selector,us,uk}.yaml` |
| Obstacle detection | `qpl_ws/src/qpl_rover/config/depth_obstacle_filter.yaml` |
| Nav2 (costmaps, DWB, smoother, recoveries) | `qpl_ws/src/qpl_rover/config/nav_params.yaml` |
| Behaviour tree | `qpl_ws/src/qpl_rover/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` |
| Nav2 launch | `qpl_ws/src/qpl_rover/launch/components/navigation_launch.py` |
| Command mux | `qpl_ws/src/qpl_rover/config/drive_mux.yaml` |
| Nav autonomy | `qpl_ws/src/qpl_autonomy/config/nav/nav_autonomy.yaml` |

---

## 15. Tuning log

Add one row per change. Keep the bag names so results can be re-checked.

| Date | Stage | Parameter (file) | Old → new | Test | Result (number) | Bag |
|---|---|---|---|---|---|---|
| | | | | | | |
