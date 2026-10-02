"""Standard-library HTTP server for the teleop HUD.

Serves three things, so the browser needs nothing but a URL:
  /            the static web app (index.html, css, js)
  /events      a Server-Sent Events stream of JSON telemetry channels
  /cam/<name>  camera feeds as MJPEG (.mjpg) or the latest still (.jpg)

Kept to the standard library on purpose: the basestation laptop has no
rosbridge, aiohttp or websockets installed, and SSE + MJPEG cover a read-only
monitoring display without them. Everything here runs on HTTP server threads;
the ROS side only ever calls Hub.publish() and CameraFeed.push().
"""

import json
import mimetypes
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Hub:
    """Latest value per named channel, plus a version counter to wait on.

    Clients only ever want the newest state, so a slow client skips stale
    versions instead of queueing them.
    """

    def __init__(self):
        self._cond = threading.Condition()
        self._version = 0
        self._channels = {}  # name -> (version, encoded bytes)

    def publish(self, name, obj):
        data = json.dumps(obj, separators=(",", ":"), allow_nan=False).encode()
        with self._cond:
            self._version += 1
            self._channels[name] = (self._version, data)
            self._cond.notify_all()

    def wait_newer(self, seen_version, timeout):
        """Block until something newer than seen_version exists.

        Returns (current version, [(name, data), ...] newer than seen_version).
        """
        with self._cond:
            self._cond.wait_for(lambda: self._version > seen_version, timeout)
            changed = [(name, data) for name, (ver, data) in self._channels.items()
                       if ver > seen_version]
            return self._version, changed


class CameraFeed:
    """Holds the newest JPEG for one camera and wakes the MJPEG streams."""

    def __init__(self, name):
        self.name = name
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0

    def push(self, jpeg):
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def latest(self):
        with self._cond:
            return self._seq, self._jpeg

    def wait_newer(self, seq, timeout):
        with self._cond:
            self._cond.wait_for(lambda: self._seq > seq, timeout)
            return self._seq, self._jpeg


# Cap per-client MJPEG rate; the cameras run at 15-30 Hz and the browser gains
# nothing from more than this.
MAX_STREAM_FPS = 30.0
SSE_KEEPALIVE_S = 1.0


def make_handler(web_root, hub, cameras, log):
    web_root = os.path.abspath(web_root)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "QPL-HUD/1.0"

        def log_message(self, fmt, *args):  # quiet the per-request stderr spam
            pass

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            try:
                if path == "/events":
                    self._events()
                elif path.startswith("/cam/"):
                    self._camera(path[len("/cam/"):])
                else:
                    self._static(path)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass  # browser went away; nothing to clean up

        def _send_headers(self, code, ctype, length=None, extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            if length is not None:
                self.send_header("Content-Length", str(length))
            for key, value in (extra or {}).items():
                self.send_header(key, value)
            self.end_headers()

        def _static(self, path):
            if path in ("", "/"):
                path = "/index.html"
            # Normalise without resolving symlinks (colcon --symlink-install points
            # the installed files into build/), then refuse anything with ../.
            rel = os.path.normpath(path.lstrip("/"))
            full = os.path.join(web_root, rel)
            if rel.startswith("..") or os.path.isabs(rel) or not os.path.isfile(full):
                body = b"not found"
                self._send_headers(404, "text/plain", len(body))
                self.wfile.write(body)
                return
            with open(full, "rb") as f:
                body = f.read()
            ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
            if ctype.startswith("text/") or ctype.endswith("javascript"):
                ctype += "; charset=utf-8"
            self._send_headers(200, ctype, len(body))
            self.wfile.write(body)

        def _events(self):
            self.close_connection = True
            self._send_headers(200, "text/event-stream", extra={"Connection": "close"})
            self.wfile.write(b"retry: 1000\n\n")
            version = 0
            while True:
                version, changed = hub.wait_newer(version, SSE_KEEPALIVE_S)
                if not changed:
                    # Comment line: keeps proxies happy and surfaces a dead
                    # socket as BrokenPipe so this thread exits.
                    self.wfile.write(b": ka\n\n")
                    self.wfile.flush()
                    continue
                chunks = []
                for name, data in changed:
                    chunks.append(b"event: " + name.encode() + b"\ndata: " + data + b"\n\n")
                self.wfile.write(b"".join(chunks))
                self.wfile.flush()

        def _camera(self, rest):
            name, _, ext = rest.partition(".")
            feed = cameras.get(name)
            if feed is None or ext not in ("mjpg", "jpg"):
                body = b"no such camera"
                self._send_headers(404, "text/plain", len(body))
                self.wfile.write(body)
                return

            if ext == "jpg":
                _, jpeg = feed.latest()
                if jpeg is None:
                    self._send_headers(204, "image/jpeg", 0)
                    return
                self._send_headers(200, "image/jpeg", len(jpeg))
                self.wfile.write(jpeg)
                return

            self.close_connection = True
            self._send_headers(200, "multipart/x-mixed-replace; boundary=hudframe",
                               extra={"Connection": "close"})
            seq = 0
            min_dt = 1.0 / MAX_STREAM_FPS
            last = 0.0
            while True:
                new_seq, jpeg = feed.wait_newer(seq, 1.0)
                if new_seq == seq or jpeg is None:
                    continue  # no frame yet; the HUD shows NO SIGNAL from telemetry
                seq = new_seq
                now = time.monotonic()
                if now - last < min_dt:
                    continue
                last = now
                self.wfile.write(
                    b"--hudframe\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()

    return Handler


class HudServer:
    def __init__(self, host, port, web_root, hub, cameras, log):
        handler = make_handler(web_root, hub, cameras, log)
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.httpd.daemon_threads = True
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
