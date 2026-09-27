# rover_cameras: camera package and stream encoder

`qpl_ws/src/rover_cameras` owns everything to do with the cameras:

- the launch files for both drivers
- per-camera stream settings and calibration
- `camera_sim` and `imu_optical_to_standard`
- `rover_cameras::StreamEncoder`, a component that encodes each camera's colour stream to
  H.264 for the basestation over the 4 Mbit/s link

```
ros2 launch rover_cameras camera_realsense.launch.py   # front; qpl_realsense_run
ros2 launch rover_cameras camera_orbbec.launch.py      # rear;  qpl_orbbecsdk_run
ros2 launch rover_cameras camera_sim.launch.py         # no hardware
```

**On the basestation:** view `/depth_camera_{front,rear}/color/teleop_stream/ffmpeg` with
Reliable reliability (`basestation/rviz/default.rviz` is already set up this way, for the
rover and the Gazebo sim alike). With Best Effort, frames lost
over Wi-Fi showed up as smeared, garbled blocks during motion. The stock
`ffmpeg_image_transport` plugin decodes it, including with `h264_cuvid`.

## Why a separate encoder

The old path encoded inside each driver, through the `ffmpeg_image_transport` plugin.
Measured on the Orin Nano:

- **Slow colour conversion.** The plugin's RGB→YUV conversion cost ~36 ms per frame at
  1280x800 and ~67 ms at 1080p. OpenCV does it in 2–4 ms.
- **A slow encode slowed the camera.** The encode blocked the driver's publish call, so
  while the stream was viewed the whole colour topic, including the AprilTag input, dropped
  to ~11–21 fps and lagged behind a backlog of queued frames.
- **Missing or wrong encoder settings.** libx264 was left single-threaded and assumed
  100 fps, and the plugin gave no access to VBV, keyframe control, resizing or frame-rate
  limits.

## How it works

- **In the Gazebo sim** (`qpl_rover` `sim.launch.py`), an encoder per camera runs as its own
  process, reading Gazebo's `<camera>/image_raw` with sim time, and publishing the same topics.
- **Same process as the driver.** Each launch file runs the driver and a `StreamEncoder` in
  one component container, with intra-process communication, so frames are handed over
  without copying.
  - The container respawns on a crash and reloads both components about 12 s later. That
    delay is how long the dead container's DDS entry takes to expire.
  - The Orbbec driver keeps its own launch file's parameter defaults (read from
    `astra_pro_plus.launch.py`).
- **Never stalls the driver.** The subscription is Best Effort, keep-last 1. The callback
  only stores the newest frame; a worker thread encodes, and frames it can't get to are
  dropped and counted.
- **Encodes only while watched.** The encoder subscribes to the camera only while
  `stream/ffmpeg` has a subscriber, and sends a keyframe whenever a new viewer joins.
- **Encode pipeline:**
  1. OpenCV resize (`INTER_AREA` for whole-number ratios, otherwise `INTER_LINEAR`).
  2. Conversion to I420.
  3. libx264 through libavcodec: `superfast`, `zerolatency`, slice threads, no B-frames.
     Packets come out in the same call, with no frame of delay.
- **Rate control:**
  - A keyframe (IDR) is forced every `keyframe_interval` seconds (0.5 s). There is no
    intra-refresh, because the laptop's `h264_cuvid` decoder needs real IDR frames.
  - A VBV buffer (`vbv_buffer_ms`) caps how far any single frame can overshoot.
  - The real frame rate is measured from the timestamps and the rate given to libx264 is
    rescaled to match, so `bit_rate` holds at any frame rate. (x264's variable-frame-rate
    mode would do the same, but adds a frame of delay.)
- **Output** is `ffmpeg_image_transport_msgs/FFMPEGPacket` with encoding
  `h264;yuv420p;bgr8;<rgb8|bgr8|mono8>`, on `<camera>/color/teleop_stream/ffmpeg`, plus
  `camera_info` rescaled to the output size.
- **Drivers now publish raw colour only.** With intra-process comms both drivers use a
  plain ROS publisher (default QoS: reliable, depth 10) instead of image_transport, so there
  is no driver-side `/compressed` or `/ffmpeg`, and `color_qos` is ignored.
- **Stats** are logged every 5 s and published on `/diagnostics`:
  - input and output fps, overwritten and throttled frames
  - kbit/s and largest frame
  - conversion and encode time
  - latency from frame stamp to publish

## Settings

Per camera, in `config/front_stream.yaml` and `config/rear_stream.yaml`. Low-quality mode
(`use_low_quality:=true`) keeps the driver's reduced size and sets 400 kbit/s at 15 fps
(`LOW_QUALITY_ENCODER` in `rover_cameras/launch_utils.py`).

| Parameter | Default (front / rear) | Live? |
|---|---|---|
| `width`, `height` | 0 = native 1280x800 / `width: 1280` (1080p → 720p) | reopens |
| `max_fps` | 30 | yes |
| `bit_rate` | 1300000 | yes |
| `vbv_buffer_ms` | 200 | yes |
| `keyframe_interval` | 1.0 s | yes |
| `preset`, `tune`, `threads`, `codec` | superfast, zerolatency, 4, libx264 | reopens |
| `av_options` | `key=value,key=value`, passed to `avcodec_open2` | reopens |

Example: `ros2 param set /depth_camera_rear/stream_encoder bit_rate 2000000`

## Status

**Done:**
- Unit tests pass (`build/rover_cameras/test_stream_core`):
  - conversion, including padded rows and resizing
  - option parsing
  - the bit rate holds at 30 and 10 fps
  - forced keyframes
  - live bit-rate changes
  - packet pts round-trip
- Checked with `camera_sim` feeding a standalone encoder:
  - the stream decodes from a keyframe
  - keyframes arrive about once a second
  - ~7 ms conversion and ~12 ms encode for 1080p → 720p
  - ~35 ms from frame stamp to publish
- Container respawn and reload tested.

**To do:**
- First run on the real cameras.
- Measure against these acceptance criteria, viewing on the laptop, then tune:

| Criterion | Target |
|---|---|
| Raw colour topics while viewing | Stay at 30 fps |
| Front stream | ≥ 25 fps |
| Rear stream | 720p at 30 fps |
| Bit rate | Within ±15% of target |
| Largest frame | ≤ 3× the average frame |
| Glass-to-glass latency (median) | < 250 ms |
| CPU | ≤ 1.5 cores per encoder |

**Known leftovers:**
- `enable_decimation_filter` and `decimation_filter_scale` in `camera_orbbec.launch.py`
  have never reached the Orbbec driver, because its launch file doesn't declare them. This
  is unchanged; the launch prints them as ignored.
- Low-quality mode on the Orbbec points at `rear_calib_640_cam_info.yaml`, which doesn't
  exist.
