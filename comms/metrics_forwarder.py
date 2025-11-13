#!/usr/bin/env python3
"""
Rock Pi Metrics (Proof of Concept)
---------------------------------
Flask + SSE dashboard for CPU, Memory, Temp, and Network stats.

Routes:
  /rockpi_metrics          → UI page (renders templates/rockpi_metrics.html)
  /rockpi_metrics/stream   → Server-Sent Events JSON stream
"""

from flask import Flask, Response, request, render_template, redirect
import psutil, json, os, time, argparse

app = Flask(__name__)

_prev_net = {"ts": None, "bytes_recv": None, "bytes_sent": None}

@app.route("/")
def root_redirect():
    return redirect("/rockpi_metrics", code=302)

@app.route("/rockpi_metrics")
def rockpi_metrics():
    return render_template("rockpi_metrics.html")

def read_temps():
    temps = {}
    try:
        t = psutil.sensors_temperatures(fahrenheit=False)
        for name, entries in t.items():
            temps[name] = [{"label": e.label or name, "current": e.current} for e in entries]
    except Exception:
        base = "/sys/class/thermal"
        zones = []
        if os.path.isdir(base):
            for entry in os.listdir(base):
                if entry.startswith("thermal_zone"):
                    path = os.path.join(base, entry, "temp")
                    try:
                        with open(path) as fh:
                            milli = int(fh.read().strip())
                            zones.append({"zone": entry, "current": milli / 1000.0})
                    except Exception:
                        pass
        if zones:
            temps["thermal_zones"] = zones
    return temps

def snapshot():
    try:
        load1, load5, load15 = os.getloadavg()
    except Exception:
        load1 = load5 = load15 = None

    vm = psutil.virtual_memory()._asdict()

    # Network totals + rate
    global _prev_net
    now = time.time()
    nio = psutil.net_io_counters(pernic=False)
    rate_rx = rate_tx = None
    if nio:
        if _prev_net["ts"] is not None:
            dt = max(1e-6, now - _prev_net["ts"])
            d_rx = max(0, nio.bytes_recv - (_prev_net["bytes_recv"] or 0))
            d_tx = max(0, nio.bytes_sent - (_prev_net["bytes_sent"] or 0))
            rate_rx = d_rx / dt / 1024.0  # kB/s
            rate_tx = d_tx / dt / 1024.0
        _prev_net.update(ts=now, bytes_recv=nio.bytes_recv, bytes_sent=nio.bytes_sent)

    return {
        "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hostname": os.uname().nodename if hasattr(os, "uname") else os.getenv("COMPUTERNAME", ""),
        "cpu": {
            "percent_total": psutil.cpu_percent(interval=None),
            "loadavg": {"1m": load1, "5m": load5, "15m": load15}
        },
        "memory": {"virtual": vm},
        "temps": read_temps(),
        "net": {
            "io": nio._asdict() if nio else {},
            "rate_rx_kBps": rate_rx,
            "rate_tx_kBps": rate_tx
        }
    }

@app.route("/rockpi_metrics/stream")
def stream():
    try:
        interval_ms = int(request.args.get("interval", "1000"))
    except ValueError:
        interval_ms = 1000

    def gen():
        psutil.cpu_percent(interval=None)  # Prime CPU reading
        last_heartbeat = time.time()
        while True:
            data = snapshot()
            txt = json.dumps(data, separators=(',', ':'))
            yield "event: metrics\n"
            yield f"data: {txt}\n\n"

            if time.time() - last_heartbeat > 15:
                yield ":\n\n"
                last_heartbeat = time.time()
            time.sleep(max(0.0, interval_ms / 1000.0))

    headers = {
        "Cache-Control": "no-cache, no-store, must-revalidate",
        "Pragma": "no-cache",
        "Expires": "0",
        "X-Accel-Buffering": "no",
    }
    return Response(gen(), mimetype="text/event-stream", headers=headers)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()
    app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)

if __name__ == "__main__":
    main()
