#!/usr/bin/env python3
"""Synthetic colour camera source, for network testing without the cameras attached.

Stands in place of realsense2_camera_node / the Orbbec node: publishes
<camera_name>/color/image_raw + camera_info on the same topic names, so the
basestation, apriltag and rviz subscribe unchanged.

/ffmpeg is NOT produced here. The real drivers never encode anything themselves -
they create an image_transport publisher and the transport plugins do the work.
We keep that property by publishing raw and letting `image_transport republish`
(see camera_sim.launch.py) run the same ffmpeg plugin with the same parameters.
/compressed is the one exception: the compressed plugin is cv2.imencode under the
hood, so doing it here directly is equivalent and saves a second raw hop.
"""
import array
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

# Noise is taken as random crops from one pre-generated pool rather than freshly
# sampled each frame: RNG at 1920x1080 costs ~12 ms/frame on the Jetson, a crop
# ~1.2 ms. The pool must be larger than the frame for the crop offset to vary.
NOISE_POOL_MARGIN = 64


class CameraSim(Node):
    def __init__(self):
        super().__init__('camera_sim')

        self.camera_name = self.declare_parameter('camera_name', 'depth_camera_front').value
        self.width = int(self.declare_parameter('width', 1280).value)
        self.height = int(self.declare_parameter('height', 800).value)
        self.fps = float(self.declare_parameter('fps', 30.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'camera_link_front').value

        # noise      - uniform per-pixel RGB. Incompressible upper bound; the real
        #              worst case for JPEG, which has no rate control.
        # texture    - high spatial detail panned across frame. Defeats motion
        #              estimation while staying representative of real footage.
        # gradient   - low-entropy floor, for a best-case baseline.
        self.pattern = self.declare_parameter('pattern', 'noise').value
        # Blends noise over the chosen pattern, so bandwidth can be swept between
        # the realistic and worst cases rather than only sampled at the extreme.
        self.noise_fraction = float(self.declare_parameter('noise_fraction', 1.0).value)
        self.pan_speed = float(self.declare_parameter('pan_speed', 3.0).value)
        # Forces a scene change: the bitrate spike an encoder cannot predict away.
        self.scene_cut_period = float(self.declare_parameter('scene_cut_period', 0.0).value)

        self.publish_compressed = bool(self.declare_parameter('publish_compressed', True).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 10).value)

        # Frame number written into the pixels as high-contrast blocks. Survives
        # jpeg_quality 10 and qmax 40, so a receiver can recover frame loss and
        # ordering downstream of the codec, where the ROS header may still be intact
        # but the content is not.
        self.burn_frame_id = bool(self.declare_parameter('burn_frame_id', True).value)
        self.burn_block_size = int(self.declare_parameter('burn_block_size', 32).value)

        camera_info_url = self.declare_parameter('camera_info_url', '').value
        # The Orbbec driver leaves image_qos at "default" (rmw_qos_profile_default,
        # i.e. RELIABLE depth 10) and camera_orbbec.launch.py has color_qos commented
        # out, so the real cameras publish reliably. Matched here because QoS changes
        # network behaviour more than image content does - reliable turns loss into
        # retransmission. It is also what republish's subscriber requires.
        reliability = self.declare_parameter('qos_reliability', 'reliable').value
        qos_depth = int(self.declare_parameter('qos_depth', 10).value)
        self.stats_period = float(self.declare_parameter('stats_period', 5.0).value)

        # libx264 refuses odd dimensions ("width not divisible by 2"), so /ffmpeg
        # would die in the republish process while raw and /compressed carry on -
        # a dead topic with the error buried in another node's log.
        if self.width % 2 or self.height % 2:
            self.get_logger().warn(
                f'{self.width}x{self.height} has an odd dimension. libx264 will refuse it, so '
                '/ffmpeg will publish nothing; raw and /compressed are unaffected.')

        if self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is true. Frame stamps will come from /clock, which makes '
                'any latency measured against them meaningless. Run this node on wall time.')

        qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT if reliability == 'best_effort'
                         else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
        )
        if reliability == 'best_effort':
            self.get_logger().warn(
                'qos_reliability=best_effort: image_transport republish subscribes RELIABLE, '
                'so /ffmpeg will receive nothing. Use this only with ffmpeg disabled.')

        base = f'/{self.camera_name}/color/image_raw'
        self.pub_image = self.create_publisher(Image, base, qos)
        self.pub_info = self.create_publisher(CameraInfo, f'/{self.camera_name}/color/camera_info', qos)
        self.pub_compressed = (self.create_publisher(CompressedImage, f'{base}/compressed', qos)
                               if self.publish_compressed else None)

        self.camera_info = self._load_camera_info(camera_info_url)
        self._build_sources()

        # JPEG on 1080p noise costs ~24 ms/frame on this Jetson - most of a 30 Hz
        # budget. Encoding on the timer thread drags the raw publish down with it
        # (measured 19-24 fps), so it runs on a worker and drops frames under load:
        # better to under-sample /compressed and report it than to skew every rate.
        self._enc_lock = threading.Lock()
        self._enc_pending = None
        self._enc_wake = threading.Condition(self._enc_lock)
        self._enc_running = True
        self._n_comp = 0
        self._comp_bytes = 0
        self._comp_dropped = 0
        self._enc_thread = None
        if self.pub_compressed is not None:
            self._enc_thread = threading.Thread(target=self._encode_worker, daemon=True)
            self._enc_thread.start()

        self.frame_no = 0
        self._pan = np.array([0.0, 0.0])
        self._pan_dir = np.array([1.0, 0.37])
        self._last_cut = time.monotonic()
        self._stats_reset(time.monotonic())

        self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(
            f'camera_sim: {self.camera_name} {self.width}x{self.height}@{self.fps:g} '
            f'pattern={self.pattern} noise_fraction={self.noise_fraction:g} '
            f'compressed={"on" if self.publish_compressed else "off"}')

    # --- sources -----------------------------------------------------------

    def _build_sources(self):
        rng = np.random.default_rng()
        h, w = self.height, self.width

        if self.pattern not in ('noise', 'texture', 'gradient'):
            self.get_logger().warn(f"unknown pattern '{self.pattern}', using 'noise'")
            self.pattern = 'noise'
        self.noise_fraction = float(np.clip(self.noise_fraction, 0.0, 1.0))

        self.noise_pool = None
        if self.noise_fraction > 0.001:
            self.noise_pool = rng.integers(
                0, 256, (h + NOISE_POOL_MARGIN, w + NOISE_POOL_MARGIN, 3), dtype=np.uint8)

        # pattern picks the base content; noise_fraction overlays noise on top of it.
        # At full noise the base is never sampled, so skip building it.
        if self.noise_fraction >= 0.999:
            self.base = None
            return

        base_pattern = self.pattern
        if base_pattern == 'noise':
            # Blending noise into noise is a no-op, so a fractional noise_fraction
            # here can only sensibly mean noise over structured content.
            self.get_logger().warn(
                f'pattern=noise with noise_fraction={self.noise_fraction:g} would blend noise '
                'into noise; using the texture base instead. Set pattern explicitly to silence this.')
            base_pattern = 'texture'

        # Panning needs margin around the frame; the base is built oversized once.
        self.pan_margin = max(64, int(self.pan_speed * self.fps * 2))
        bh, bw = h + self.pan_margin, w + self.pan_margin

        if base_pattern == 'gradient':
            xs = np.linspace(0, 255, bw, dtype=np.uint8)
            self.base = np.repeat(xs[None, :, None], bh, axis=0).repeat(3, axis=2)
        else:  # 'texture'
            # Multi-scale blurred noise, sharpened: gives the broad spatial frequency
            # content of real terrain rather than the flat spectrum of pure noise.
            acc = np.zeros((bh, bw), dtype=np.float32)
            for scale, weight in ((3, 0.5), (9, 0.3), (31, 0.2)):
                n = rng.integers(0, 256, (bh, bw), dtype=np.uint8).astype(np.float32)
                acc += weight * cv2.GaussianBlur(n, (scale, scale), 0)
            acc = cv2.normalize(acc, None, 0, 255, cv2.NORM_MINMAX)
            gray = cv2.addWeighted(acc, 1.6, cv2.GaussianBlur(acc, (0, 0), 3), -0.6, 0)
            gray = np.clip(gray, 0, 255).astype(np.uint8)
            self.base = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def _noise_crop(self):
        i = self.frame_no
        y = (i * 13) % NOISE_POOL_MARGIN
        x = (i * 29) % NOISE_POOL_MARGIN
        return self.noise_pool[y:y + self.height, x:x + self.width]

    def _base_crop(self):
        now = time.monotonic()
        if self.scene_cut_period > 0.0 and (now - self._last_cut) >= self.scene_cut_period:
            self._pan = np.array([np.random.uniform(0, self.pan_margin),
                                  np.random.uniform(0, self.pan_margin)])
            self._last_cut = now
        else:
            self._pan += self._pan_dir * self.pan_speed
            # Bounce rather than wrap: wrapping would tear the image and read as a cut.
            for a in (0, 1):
                if self._pan[a] < 0 or self._pan[a] > self.pan_margin:
                    self._pan_dir[a] *= -1
                    self._pan[a] = float(np.clip(self._pan[a], 0, self.pan_margin))
        x, y = int(self._pan[0]), int(self._pan[1])
        return self.base[y:y + self.height, x:x + self.width]

    def _make_frame(self):
        if self.noise_fraction >= 0.999 or self.base is None:
            frame = self._noise_crop()
        elif self.noise_fraction <= 0.001:
            frame = self._base_crop()
        else:
            a = self.noise_fraction
            frame = cv2.addWeighted(self._noise_crop(), a, self._base_crop(), 1.0 - a, 0.0)
        # Always copy: the crops are views into the pool/base, and the frame-id
        # blocks are written in place.
        frame = frame.copy()
        if self.burn_frame_id:
            self._burn_id(frame, self.frame_no)
        return frame

    def _burn_id(self, frame, fid):
        bs = self.burn_block_size
        # Two sync blocks then 16 bits LSB-first, so a receiver can locate the
        # field before decoding it.
        cells = [255, 0] + [255 if (fid >> b) & 1 else 0 for b in range(16)]
        for i, value in enumerate(cells):
            x0 = i * bs
            if x0 + bs > self.width:
                break
            frame[0:bs, x0:x0 + bs] = value

    # --- publishing --------------------------------------------------------

    def _tick(self):
        t0 = time.monotonic()
        frame = self._make_frame()
        gen_ms = (time.monotonic() - t0) * 1000.0

        stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        # 1307 ms/frame at 1080p if this is assigned as bytes: rclpy validates
        # uint8[] element by element. array.array('B', ...) takes the fast path (0.9 ms).
        msg.data = array.array('B', frame.tobytes())
        self.pub_image.publish(msg)

        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.pub_info.publish(self.camera_info)

        if self.pub_compressed is not None:
            with self._enc_wake:
                if self._enc_pending is not None:
                    self._comp_dropped += 1
                self._enc_pending = (frame, stamp)
                self._enc_wake.notify()

        self.frame_no += 1
        self._stats(len(msg.data), gen_ms)

    def _encode_worker(self):
        while self._enc_running:
            with self._enc_wake:
                while self._enc_pending is None and self._enc_running:
                    self._enc_wake.wait(timeout=0.5)
                if not self._enc_running:
                    return
                frame, stamp = self._enc_pending
                self._enc_pending = None

            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                continue
            cmsg = CompressedImage()
            cmsg.header.stamp = stamp
            cmsg.header.frame_id = self.frame_id
            cmsg.format = 'jpeg'
            cmsg.data = array.array('B', buf.tobytes())
            self.pub_compressed.publish(cmsg)
            with self._enc_lock:
                self._n_comp += 1
                self._comp_bytes += buf.size

    def stop(self):
        with self._enc_wake:
            self._enc_running = False
            self._enc_wake.notify_all()
        if self._enc_thread is not None:
            self._enc_thread.join(timeout=1.0)

    # --- telemetry ---------------------------------------------------------

    def _stats_reset(self, now):
        self._t_stats = now
        self._n_stats = 0
        self._raw_bytes = 0
        self._gen_ms = 0.0
        with self._enc_lock:
            self._n_comp = 0
            self._comp_bytes = 0
            self._comp_dropped = 0

    def _stats(self, raw_bytes, gen_ms):
        self._n_stats += 1
        self._raw_bytes += raw_bytes
        self._gen_ms += gen_ms

        now = time.monotonic()
        dt = now - self._t_stats
        if dt < self.stats_period:
            return

        achieved = self._n_stats / dt
        line = (f'{achieved:5.1f}/{self.fps:g} fps  gen {self._gen_ms / self._n_stats:4.1f} ms  '
                f'raw {self._raw_bytes / dt / 1e6:6.1f} MB/s')
        if self.pub_compressed is not None:
            with self._enc_lock:
                n_comp, comp_bytes, dropped = self._n_comp, self._comp_bytes, self._comp_dropped
            line += f'  compressed {n_comp / dt:4.1f} fps {comp_bytes / dt / 1e6:5.2f} MB/s'
            if dropped:
                # Expected on worst-case noise; the raw rate above is still honest.
                line += f' ({dropped} dropped, encoder CPU-bound)'

        # The guard that matters: if the node cannot hold the requested rate, the
        # bandwidth figures below it are wrong for reasons unrelated to the network.
        if achieved < self.fps * 0.9:
            self.get_logger().warn(f'{line}  <- NOT KEEPING UP, bandwidth figures unreliable')
        else:
            self.get_logger().info(line)
        self._stats_reset(now)

    # --- calibration -------------------------------------------------------

    def _load_camera_info(self, url):
        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'

        path = url[7:] if url.startswith('file://') else url
        if not path or not os.path.isfile(path):
            if path:
                self.get_logger().warn(f'camera_info_url not found: {path}. Using a synthetic model.')
            f = float(self.width)
            info.k = [f, 0.0, self.width / 2.0, 0.0, f, self.height / 2.0, 0.0, 0.0, 1.0]
            info.p = [f, 0.0, self.width / 2.0, 0.0, 0.0, f, self.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.d = [0.0] * 5
            return info

        with open(path) as fh:
            y = yaml.safe_load(fh)

        k = list(map(float, y['camera_matrix']['data']))
        p = list(map(float, y['projection_matrix']['data']))
        info.r = list(map(float, y['rectification_matrix']['data']))
        info.d = list(map(float, y['distortion_coefficients']['data']))
        info.distortion_model = y.get('distortion_model', 'plumb_bob')

        # Rescale rather than publish a calibration that silently disagrees with
        # the resolution being generated.
        cal_w, cal_h = int(y['image_width']), int(y['image_height'])
        if (cal_w, cal_h) != (self.width, self.height):
            sx, sy = self.width / cal_w, self.height / cal_h
            self.get_logger().warn(
                f'calibration is {cal_w}x{cal_h} but generating {self.width}x{self.height}; '
                f'scaling intrinsics by ({sx:.3f}, {sy:.3f})')
            for idx, s in ((0, sx), (2, sx), (4, sy), (5, sy)):
                k[idx] *= s
            for idx, s in ((0, sx), (2, sx), (5, sy), (6, sy)):
                p[idx] *= s

        info.k = k
        info.p = p
        return info


def main(args=None):
    rclpy.init(args=args)
    node = CameraSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
