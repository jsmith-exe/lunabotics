# rover_cameras

Launch files, configuration and calibration for the rover's two cameras, plus
`StreamEncoder`, which turns each camera's colour image into a low-latency H.264 stream
the basestation can watch over the 4 Mbit/s link.

| Camera | Driver | Namespace | Colour |
|---|---|---|---|
| Front | Intel RealSense (`realsense2_camera`) | `/depth_camera_front` | 1280x800 @ 30 |
| Rear | Orbbec Astra Pro Plus (`orbbec_camera`) | `/depth_camera_rear` | 1920x1080 @ 30 |

## Running the cameras

```bash
ros2 launch rover_cameras camera_realsense.launch.py   # or: qpl_realsense_run
ros2 launch rover_cameras camera_orbbec.launch.py      # or: qpl_orbbecsdk_run
```

`rover.launch.py` in `qpl_rover` includes both launch files; it starts the Orbbec 20 s
after the RealSense.

Both take `use_low_quality:=true`, which shrinks the driver's colour and depth profiles
(front 424x240 @ 15, rear 640x480) and drops the stream to 400 kbit/s at 15 fps.

Each launch file starts one component container, `/<namespace>/camera_container`, which
holds the camera driver and its `stream_encoder`. The RealSense launch also starts the IMU
nodes (`imu_optical_to_standard`, `imu_filter`) and a static transform.

If the container crashes it restarts, and both components are loaded again about 12 s
later. That delay is how long the dead container's DDS entry takes to expire. Until then,
a load request would go to the dead process.

## Viewing on the basestation

Show `/depth_camera_front/color/stream/ffmpeg` and `/depth_camera_rear/color/stream/ffmpeg`
in an rviz Image display. `basestation/rviz/rover.rviz` is already set up this way.

- Set **Reliability** to **Best Effort**. The stream is published Best Effort, and a
  Reliable subscriber won't connect.
- The viewer needs the `ffmpeg_image_transport` plugin (`ros-humble-ffmpeg-image-transport`).
  It decodes the stream, including with the NVIDIA `h264_cuvid` decoder.
- A new viewer gets a picture straight away: the encoder sends a keyframe whenever someone
  subscribes.
- The encoder only runs while someone is watching, so an unviewed stream costs nothing.

Other colour topics are still published for use on the rover:

- `.../color/image_raw`: raw, used by AprilTag detection and visual odometry.
- `.../color/image_raw/compressed`: JPEG, for debugging. At 1080p this is too big for the
  link.

The drivers no longer publish `.../image_raw/ffmpeg`.

## Tuning the stream

Settings live in `config/front_stream.yaml` and `config/rear_stream.yaml`. Low-quality mode
overrides them from `LOW_QUALITY_ENCODER` in `rover_cameras/launch_utils.py`.

| Parameter | Default | Meaning | Changeable live? |
|---|---|---|---|
| `bit_rate` | `1300000` | Target bits/s. Two cameras at 1.3 Mbit/s leaves room for telemetry in the 3.6 Mbit/s uplink. | yes |
| `max_fps` | `30.0` | Frame-rate cap (frames are thinned evenly). `0` = camera rate. | yes |
| `width`, `height` | front `0`/`0`, rear `1280`/`0` | Output size. `0` keeps the input size, or keeps the aspect ratio if the other is set. Rounded down to even. | reopens encoder |
| `keyframe_interval` | `1.0` | Seconds between keyframes: the longest a stream takes to recover from a lost packet. | yes |
| `vbv_buffer_ms` | `200` | Caps how far one frame (mostly keyframes) can overshoot the bit rate, which bounds latency spikes. `0` = off. | yes |
| `preset` | `superfast` | x264 speed/quality trade-off. Slower presets compress better and cost more CPU. | reopens encoder |
| `tune` | `zerolatency` | Keep this: other tunes add frames of delay. | reopens encoder |
| `threads` | `4` | Encoder slice threads. These add no latency. | reopens encoder |
| `codec` | `libx264` | Any libavcodec H.264/H.265 encoder. The Orin Nano has no hardware encoder. | reopens encoder |
| `av_options` | `""` | Extra encoder options as `key=value,key=value`, e.g. `profile=main,x264-params=aq-mode=2`. | reopens encoder |
| `input_topic`, `camera_info_topic`, `output_topic`, `output_reliability` | see YAML | Wiring. | no (set at startup) |

Change a setting on a running camera:

```bash
ros2 param set /depth_camera_rear/stream_encoder bit_rate 2000000
ros2 param set /depth_camera_front/stream_encoder max_fps 15.0
```

"Reopens encoder" means the change applies straight away but restarts the encoder, which
sends a fresh keyframe. To keep a change, put it in the YAML file.

Rough guidance:

- **Blocky picture:** lower the resolution or `max_fps` before raising `bit_rate`.
  Halving the frame rate roughly doubles the bits each frame gets.
- **Stuttering or lag on the laptop:** the link is probably saturated. Lower `bit_rate`, or
  check what else is being pulled off the rover.
- **Garbled picture after a Wi-Fi drop:** it clears at the next keyframe. Shorten
  `keyframe_interval` for faster recovery, at some cost in quality.
- **Don't enable x264 intra-refresh.** The laptop's `h264_cuvid` decoder can't start from
  an intra-refresh stream, and the feed never appears.

## Monitoring

Every 5 s while being watched, each encoder logs:

```
in 30.0 fps, out 30.0 fps (0 overwritten, 0 throttled), 1290 kbit/s, max frame 24.1 KB,
5 keyframes, convert 6.9 ms, encode 11.6 ms, latency 36 ms
```

| Field | Meaning |
|---|---|
| `overwritten` | Frames the encoder couldn't keep up with and skipped. Above zero means it's CPU-bound: lower the size, fps or preset. The camera itself is never slowed. |
| `throttled` | Frames skipped deliberately by `max_fps`. |
| `latency` | Camera timestamp to publish, on the rover only. Excludes the network and decoding. |

The same numbers are published on `/diagnostics`.

## Testing without cameras

```bash
ros2 launch rover_cameras camera_sim.launch.py                       # both cameras
ros2 launch rover_cameras camera_sim.launch.py camera:=rear pattern:=texture noise_fraction:=0.3
ros2 launch rover_cameras camera_sim.launch.py use_low_quality:=true
```

`camera_sim` publishes synthetic colour frames on the real topic names, with the real
resolutions and calibration. Next to each camera it runs a `stream_encoder` with the same
settings, as a separate process.

- **Patterns:** `noise` (worst case), `texture` (realistic) or `gradient` (best case).
  `noise_fraction` blends between them, and `scene_cut_period` forces scene changes.
- **Other arguments:** `enable_stream:=false` skips the encoder; `enable_compressed:=false`
  skips JPEG.
- Being Python, it tops out around 10–15 fps at 1080p. When it can't keep up it says so, so
  read throughput figures from the encoder's log.

## Building and unit tests

```bash
cd qpl_ws
colcon build --packages-select rover_cameras --symlink-install
./build/rover_cameras/test_stream_core
```

The tests cover colour conversion, resizing, option parsing, and the encoder's bit rate at
different frame rates. They also cover forced keyframes, live bit-rate changes, and packet
timestamps.

Build requirements: libavcodec/libavutil (`libavcodec-dev`), OpenCV and
`ffmpeg_image_transport_msgs`. The Orbbec driver comes from the separate `OrbbecSDK_ROS2`
workspace, so source it before launching.

## Package layout

```
launch/
  camera_realsense.launch.py    front driver + encoder in one container; IMU nodes
  camera_orbbec.launch.py       rear driver + encoder in one container
  camera_sim.launch.py          synthetic cameras + encoders, no hardware
config/
  front_stream.yaml             encoder settings per camera
  rear_stream.yaml
calibration/                    rear colour calibration (1280 and 1920 wide)
rover_cameras/launch_utils.py   shared launch helpers (container with reload, encoder setup)
src/stream_encoder.cpp          the StreamEncoder component
src/h264_encoder.cpp            libavcodec wrapper (no ROS dependency)
src/convert.cpp                 OpenCV resize + I420 conversion (no ROS dependency)
scripts/camera_sim              synthetic camera node
scripts/imu_optical_to_standard RealSense IMU axis conversion for the EKFs
test/test_stream_core.cpp       unit tests
```

## Notes for maintainers

- **The Orbbec runs in our own container, not through `astra_pro_plus.launch.py`.** That
  way the encoder can share its process. `orbbec_parameters()` reads that file's declared
  arguments and defaults, so the driver gets exactly the parameters it did before.
  `enable_decimation_filter` and `decimation_filter_scale` have never been declared there,
  so they don't reach the driver; the launch prints them as ignored.
- **Low-quality mode on the Orbbec** points `color_info_url` at
  `rear_calib_640_cam_info.yaml`, which doesn't exist yet.
- **Why not `ffmpeg_image_transport` on the driver?** It encoded inside the driver's publish
  call, and its RGB→YUV conversion cost 36–67 ms per frame on the Orin Nano. So while the
  stream was viewed, the whole colour topic, including AprilTag detection, slowed to
  11–21 fps and lagged. It also left x264 single-threaded and assumed 100 fps for its bit
  rate. The full write-up is in `docs/camera-encoder-node.md`.
- **The root `.gitignore` ignores every directory named `test`.** Add `test/` with
  `git add -f`.
