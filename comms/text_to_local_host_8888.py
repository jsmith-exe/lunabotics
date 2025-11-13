#!/usr/bin/env python3
"""
Simple local text forwarder (no flask_socketio).

- Runs a Flask server on port 8888
- Anything you type in this terminal is streamed to a web page
- Open http://localhost:8888 to view the live text
"""

from flask import Flask, Response, render_template_string
from threading import Thread, Lock
import sys
import time

app = Flask(__name__)

messages = []
messages_lock = Lock()

HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>Text Forward</title>
  <style>
    body { font-family: monospace; background: #111; color: #0f0; padding: 20px; }
    #log  { white-space: pre-wrap; }
  </style>
</head>
<body>
  <h2>Live Text Stream</h2>
  <div id="log"></div>
  <script>
    var log = document.getElementById('log');
    var es  = new EventSource('/stream');
    es.onmessage = function(e) {
      log.textContent += e.data + "\\n";
      window.scrollTo(0, document.body.scrollHeight);
    };
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

def event_stream():
    """
    Server-Sent Event stream of all messages.
    Each client keeps its own index into the global messages list.
    """
    idx = 0
    while True:
        # Copy out any new messages
        with messages_lock:
            if idx < len(messages):
                data = messages[idx]
                idx += 1
            else:
                data = None

        if data is not None:
            # SSE format: "data: <text>\\n\\n"
            yield f"data: {data}\n\n"
        else:
            # No new data; avoid busy-wait
            time.sleep(0.2)

@app.route('/stream')
def stream():
    return Response(event_stream(), mimetype='text/event-stream')

def stdin_reader():
    """
    Read lines from stdin and append to messages list.
    """
    print("[TextForward] Type here. Your text will appear at http://localhost:8888\n")
    for line in sys.stdin:
        line = line.rstrip('\n')
        if not line:
            continue
        with messages_lock:
            messages.append(line)

if __name__ == "__main__":
    t = Thread(target=stdin_reader, daemon=True)
    t.start()
    # Similar to your MJPEG Flask app pattern
    app.run(host="0.0.0.0", port=8888, threaded=True, use_reloader=False)
