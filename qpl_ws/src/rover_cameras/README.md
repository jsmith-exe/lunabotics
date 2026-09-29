# rover_cameras

Launch files, configuration and calibration for the rover's two cameras, plus
`StreamEncoder`, which turns each camera's colour image into a low-latency H.264 stream
the basestation can watch over the 4 Mbit/s link.

**The compression node is `StreamEncoder`, in
[`src/stream_encoder/stream_encoder_node.cpp`](src/stream_encoder/stream_encoder_node.cpp).**
It's built as a component (`rover_cameras::StreamEncoder`) and as a standalone executable
(`stream_encoder_node`). See [How it works](#how-it-works) for the other parts.

| Camera | Driver | Namespace | Colour |
|---|---|---|---|
| Front | Intel RealSense (`realsense2_camera`) | `/depth_camera_front` | 1280x800 @ 30 |
| Rear | Orbbec Astra Pro Plus (`orbbec_camera`) | `/depth_camera_rear` | 1920x1080 @ 30 |

## Running the cameras

```bash
ros2 launch rover_cameras camera_realsense.launch.py   # or: qpl_realsense_run
ros2 launch rover_cameras camera_orbbec.launch.py      # or: qpl_orbbecsdk_run
```

Each launch file starts one component container, `/<namespace>/camera_container`, which
holds the camera driver and its `stream_encoder`. The RealSense launch also starts the IMU
nodes (`imu_optical_to_standard`, `imu_filter`) and a static transform.

If the container crashes, it restarts, and both components are loaded again about 12 s
later. That delay is how long the dead container's DDS entry takes to expire. Until then,
a load request would go to the dead process.

## Tuning the stream

Settings live in `config/front_stream.yaml` and `config/rear_stream.yaml`. Low-quality mode
overrides them from `LOW_QUALITY_ENCODER` in `launch/launch_utils.py`.

| Parameter | Default | Meaning | Changeable live? |
|---|---|---|---|
| `bit_rate` | `1300000` | Target bits/s. Two cameras at 1.3 Mbit/s leaves room for telemetry in the 3.6 Mbit/s uplink. | yes |
| `max_fps` | `15.0` | Frame-rate cap (frames are thinned evenly). `0` = camera rate. | yes |
| `width`, `height` | `640`/`0` | Output size. `0` keeps the input size, or keeps the aspect ratio if the other is set, so the front is 640x400 and the rear 640x360. Rounded down to even. | reopens encoder |
| `keyframe_interval` | `0.5` | Seconds between keyframes: the longest a stream takes to recover from a lost frame. | yes |
| `vbv_buffer_ms` | `200` | Caps how far one frame (mostly keyframes) can overshoot the bit rate, which bounds latency spikes. `0` = off. | yes |
| `preset` | `superfast` | x264 speed/quality trade-off. Slower presets compress better and cost more CPU. | reopens encoder |
| `tune` | `zerolatency` | Keep this: other tunes add frames of delay. | reopens encoder |
| `threads` | `4` | Encoder slice threads. These add no latency. | reopens encoder |
| `codec` | `libx264` | Any libavcodec H.264/H.265 encoder. The Orin Nano has no hardware encoder. Unknown names are rejected. | reopens encoder |
| `av_options` | `""` | Extra encoder options as `key=value,key=value`, e.g. `profile=main,x264-params=aq-mode=2`. | reopens encoder |
| `output_reliability` | `reliable` | `reliable` or `best_effort`. Keep `reliable`: see [Viewing on the basestation](#viewing-on-the-basestation). | no (set at startup) |
| `input_topic`, `camera_info_topic`, `output_topic` | see YAML | Wiring. | no (set at startup) |

Change a setting on a running camera:

```bash
ros2 param set /depth_camera_rear/stream_encoder bit_rate 2000000
ros2 param set /depth_camera_front/stream_encoder max_fps 10.0
```

"Reopens encoder" means the change applies straight away but restarts the encoder, which
sends a fresh keyframe. To keep a change, put it in the YAML file.

Rough guidance:

- **Blocky picture:** lower the resolution or `max_fps` before raising `bit_rate`.
  Halving the frame rate roughly doubles the bits each frame gets.
- **Stuttering or lag on the laptop:** the link is probably saturated. Lower `bit_rate`, or
  check what else is being pulled off the rover.
- **Garbled or smeared blocks that drag with motion:** frames are being lost. Check that
  the viewer is set to Reliable. Anything still lost clears at the next keyframe; a shorter
  `keyframe_interval` recovers faster, at some cost in quality.
- **Soft or blocky picture during motion (but not garbled):** try `preset: faster`. At
  640x480 @ 15 it measured noticeably better than `superfast` in motion, for ~9 ms per
  frame instead of ~4.
- **Don't enable x264 intra-refresh.** The laptop's `h264_cuvid` decoder can't start from
  an intra-refresh stream, and the feed never appears.

## Monitoring

The encoder logs what it's doing: when it starts encoding (with its settings), when
viewers connect or leave, and any setting changes or errors. Every 5 s while being
watched, it also logs:

```
Teleop stream /depth_camera_front/color/teleop_stream/ffmpeg: camera 27.2 fps -> sent 14.8 fps
(0 dropped as encoder busy, 63 skipped by max_fps), 1121 kbit/s, largest frame 28.7 KB,
10 keyframes; per frame convert 1.0 ms + encode 4.1 ms; camera-to-publish latency 14 ms
```

- **dropped as encoder busy:** frames the encoder couldn't keep up with. Above zero means
  it's CPU-bound: lower the size, fps or preset. The camera itself is never slowed.
- **skipped by max_fps:** frames dropped deliberately to hold the frame-rate cap.
- **camera-to-publish latency:** measured on the rover (or the sim machine, in sim time).
  It excludes the network and decoding.

The same numbers are published on `/diagnostics`.

## How it works

- **`StreamEncoder`** (`src/stream_encoder/stream_encoder_node.cpp`), the ROS node:
  - **Only runs when watched.** It subscribes to the camera only while its output has a
    viewer, and forces a keyframe whenever a viewer joins.
  - **Never holds up the camera.** The subscription callback just stores the newest frame.
    A worker thread takes it, resizes it, converts it and encodes it. Frames that arrive
    while it's busy are replaced, not queued, and counted in the stats.
  - **Output** is `ffmpeg_image_transport_msgs/FFMPEGPacket` (encoding
    `h264;yuv420p;bgr8;<rgb8|bgr8|mono8>`) on `<camera>/color/teleop_stream/ffmpeg`, plus
    `camera_info` rescaled to the output size.
  - It also handles live parameter changes, the stats log and `/diagnostics`.
- **`H264Encoder`** (`src/stream_encoder/h264_encoder.cpp`): a thin libavcodec wrapper
  around libx264, with no ROS dependency.
  - **Low latency:** `zerolatency` tune, slice threads and no B-frames, so each packet
    comes out in the same call as its frame.
  - **Keyframes:** the node forces one every `keyframe_interval`.
  - **Rate control:** a VBV buffer caps how far any single frame can overshoot. The real
    frame rate is measured from the timestamps and the rate given to x264 is rescaled to
    match, so `bit_rate` holds at any frame rate.
- **`toI420`** (`src/stream_encoder/convert.cpp`): the OpenCV resize and RGB/BGR → I420
  conversion that x264 needs. It uses `INTER_AREA` for whole-number downscales and
  `INTER_LINEAR` otherwise.
- **Launch helpers** (`launch/launch_utils.py`):
  - load the encoder beside a driver (`stream_encoder`), or as its own process for
    camera_sim and Gazebo (`stream_encoder_process`, `gazebo_stream_encoders`)
  - `respawning_container`, which reloads both components after a crash
- **Python nodes** (`src/`):
  - `camera_sim.py`: synthetic cameras for testing without hardware.
  - `imu_optical_to_standard.py`: re-publishes the RealSense IMU in ROS axes for the EKFs.

## Testing without cameras via camera_sim

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

After building, run tests via `./build/rover_cameras/test_stream_core`.

The tests cover colour conversion, resizing, input validation and option parsing. They also
cover the encoder's bit rate at different frame rates, forced keyframes, live bit-rate
changes and packet timestamps.

Build requirements: libavcodec/libavutil (`libavcodec-dev`), OpenCV and
`ffmpeg_image_transport_msgs`. The Orbbec driver comes from the separate `OrbbecSDK_ROS2`
workspace, so source it before launching.

The Python nodes in `src/` are installed as copies, renamed without `.py`, so after editing
them rebuild the package, even with `--symlink-install`.

## Notes for maintainers

- **Why not `ffmpeg_image_transport` on the driver?** That was the original setup.
  Measured on the Orin Nano:
  - It encoded inside the driver's publish call, and its RGB→YUV conversion (swscale) cost
    ~36 ms per frame at 1280x800 and ~67 ms at 1080p. OpenCV does the same in 2–4 ms.
  - So while the stream was viewed, the whole colour topic, including AprilTag detection,
    slowed to 11–21 fps and lagged behind queued frames.
  - It also left x264 single-threaded, assumed 100 fps for its bit rate, and gave no access
    to VBV, keyframe control, resizing or frame-rate limits.
- **Why not x264's variable-frame-rate mode?** It would make `bit_rate` hold at any frame
  rate by itself, but it delays every packet by a frame. Hence the rate rescaling in
  `H264Encoder`.
- **The Orbbec runs in our own container, not through `astra_pro_plus.launch.py`.** That
  way the encoder can share its process. `orbbec_parameters()` reads that file's declared
  arguments and defaults, so the driver gets exactly the parameters it did before.
  `enable_decimation_filter` and `decimation_filter_scale` have never been declared there,
  so they don't reach the driver; the launch logs them as ignored.
- **Low-quality mode on the Orbbec** points `color_info_url` at
  `rear_calib_640_cam_info.yaml`, which doesn't exist yet.
- **Targets when retuning,** viewing on the laptop:
