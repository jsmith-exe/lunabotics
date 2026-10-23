"""Laptop-side h264 decoder for one camera, started and stopped by teleop_hud.

Each camera's <camera>/color/teleop_stream/ffmpeg is decoded to JPEG by an
image_transport republisher, the same software decode RViz uses. Turning a
camera off in the HUD stops its republisher, so the subscription goes away and
the rover stops sending that stream over the link. The republisher is not a
component in Humble, so it runs as a child process.
"""

import ctypes
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix

PR_SET_PDEATHSIG = 1
RESTART_S = 5.0  # wait between restarts of a decoder that keeps dying


def _die_with_parent():
    # A HUD that is killed outright must not leave decoders subscribed.
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)


class Decoder:
    def __init__(self, name, stream_topic, out_topic, use_sim_time, log):
        """stream_topic: base of the ffmpeg stream (without /ffmpeg).
        out_topic: base of the decoded output (without /compressed)."""
        self.name = name
        self.log = log
        exe = f"{get_package_prefix('image_transport')}/lib/image_transport/republish"
        self.cmd = [
            exe, "ffmpeg", "compressed", "--ros-args",
            "-r", f"__node:={name}",
            "-r", f"in/ffmpeg:={stream_topic}/ffmpeg",
            "-r", f"out/compressed:={out_topic}/compressed",
            "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            # Software h264 decoding, as in rviz.launch.py: the default h264_cuvid
            # holds several frames before showing one and has turned the picture
            # green. The parameter takes the un-remapped base topic name, "in".
            "-p", "in.ffmpeg.decoders.h264:=h264",
        ]
        self.proc = None
        self.started = 0.0

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def ensure(self, on):
        """Start or stop to match `on`; restarts a decoder that has died."""
        if on and not self.running:
            if self.proc is not None:
                if time.monotonic() - self.started < RESTART_S:
                    return
                self.log.warn(f"{self.name} exited (code {self.proc.returncode}); restarting")
            self.start()
        elif not on and self.proc is not None:
            self.stop()

    def start(self):
        self.started = time.monotonic()
        try:
            self.proc = subprocess.Popen(self.cmd, preexec_fn=_die_with_parent)
            self.log.info(f"Started {self.name}")
        except OSError as e:
            self.proc = None
            self.log.error(f"Could not start {self.name}: {e}")

    def stop(self, quiet=False):
        """quiet: at shutdown, when the node can no longer log."""
        proc, self.proc = self.proc, None
        if proc is None or proc.poll() is not None:
            return
        proc.send_signal(signal.SIGINT)
        if not quiet:
            self.log.info(f"Stopped {self.name}")

        def reap():
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

        threading.Thread(target=reap, daemon=True).start()
