"""Watch the car's logs in a browser, one tag at a time.

    py tools\\telemetry_viewer.py [--serial auto|COM5|none] [--run-broker]
                                  [--broker HOST] [--port N] [--id CLIENT_ID]
                                  [--save DIR] [--http-port 8765]

Opens http://127.0.0.1:8765 and streams three sources into one page:

  mqtt    the car's records, decoded by telemetry_decode.decode() (same lines
          as tools/telemetry_decode.py prints)
  serial  the Pico's USB console.  `auto` finds the Pico (USB VID 2E8A) and
          reopens it after every reset.  Needs pyserial (pip install pyserial);
          without it the page still shows MQTT.
  broker  with --run-broker, the amqtt broker is started from
          tools/run_broker.py and its log is shown too.

Every line is tagged ([imu], [motion], [comms] S, E mission, GAP, ...), and the
page hides or shows each tag with one click.  Saved logs (one of the
Attempt N.md files, or anything in the `[HH:MM:SS.mmm] text` format) can be
dropped onto the page and filtered the same way.

--save DIR writes serial.log, decoder.log and broker.log in that same format,
ready to paste into the Embedded_Docs log notes.

Only one program can hold a COM port: close the serial monitor first, or run
with --serial none.
"""

import argparse
import collections
import datetime
import json
import os
import queue
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telemetry_decode  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "telemetry_viewer.html")
PICO_VID = 0x2E8A
HISTORY = 20000          # lines kept for a page (re)load


def stamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


class Hub:
    """Fan every line out to the open pages, and keep the recent ones."""

    def __init__(self, save_dir):
        self.lock = threading.Lock()
        self.history = collections.deque(maxlen=HISTORY)
        self.clients = []
        self.status = {"mqtt": "starting", "serial": "off", "broker": "off"}
        self.files = {}
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            for src, name in (("serial", "serial.log"), ("mqtt", "decoder.log"),
                              ("broker", "broker.log")):
                self.files[src] = open(os.path.join(save_dir, name), "a",
                                       encoding="utf-8", buffering=1)

    def _send(self, event):
        data = json.dumps(event)
        for q in self.clients:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass        # a stalled page loses lines, never the others

    def line(self, src, text):
        event = {"kind": "line", "src": src, "time": stamp(), "text": text}
        with self.lock:
            self.history.append(event)
            self._send(event)
            f = self.files.get(src)
            if f:
                f.write(f"[{event['time']}] {text}\n")

    def set_status(self, src, state):
        with self.lock:
            if self.status.get(src) == state:
                return
            self.status[src] = state
            self._send({"kind": "status", "status": dict(self.status)})

    def subscribe(self):
        q = queue.Queue(maxsize=50000)
        with self.lock:
            q.put_nowait(json.dumps({"kind": "status", "status": dict(self.status)}))
            for event in self.history:
                q.put_nowait(json.dumps(event))
            self.clients.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)


# ---------------------------------------------------------------- sources

def run_mqtt(hub, args):
    import paho.mqtt.client as mqtt

    topic = f"robocar/{args.id}/#"
    decoder = telemetry_decode.Decoder()

    # Reuse the decoder's own line format: capture what Decoder.record() logs.
    telemetry_decode.log = lambda text: hub.line("mqtt", text)

    def on_connect(client, userdata, flags, reason_code, properties=None):
        hub.set_status("mqtt", f"connected {args.broker}:{args.port}")
        client.subscribe(topic, qos=0)

    def on_disconnect(client, userdata, flags, reason_code, properties=None):
        hub.set_status("mqtt", f"waiting for {args.broker}:{args.port}")

    def on_message(client, userdata, msg):
        decoder.record(msg.topic, msg.payload)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id="robocar-viewer")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)
    hub.set_status("mqtt", f"waiting for {args.broker}:{args.port}")
    client.connect_async(args.broker, args.port, 60)
    client.loop_start()


def find_pico(list_ports):
    for p in list_ports.comports():
        if p.vid == PICO_VID:
            return p.device
    return None


def run_serial(hub, port_arg):
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        hub.set_status("serial", "pyserial missing: py -m pip install pyserial")
        return

    def loop():
        while True:
            port = find_pico(list_ports) if port_arg == "auto" else port_arg
            if not port:
                hub.set_status("serial", "no Pico found (waiting)")
                time.sleep(1)
                continue
            try:
                ser = serial.Serial(port, 115200, timeout=0.5)
            except (serial.SerialException, OSError) as why:
                busy = "denied" in str(why).lower()
                hub.set_status("serial", f"{port} busy (close the serial monitor)"
                               if busy else f"{port} unavailable")
                time.sleep(1)
                continue
            hub.set_status("serial", f"open {port}")
            hub.line("viewer", f"serial {port} opened")
            buf = b""
            try:
                while True:
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        text = raw.decode("utf-8", "replace").rstrip("\r")
                        if text.strip():
                            hub.line("serial", text)
            except (serial.SerialException, OSError):
                hub.line("viewer", f"serial {port} closed (Pico reset or unplugged)")
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            time.sleep(0.5)

    threading.Thread(target=loop, daemon=True).start()


def run_broker(hub):
    script = os.path.join(HERE, "run_broker.py")
    proc = subprocess.Popen([sys.executable, "-u", script],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace")
    hub.set_status("broker", "running")

    def pump():
        for text in proc.stdout:
            text = text.rstrip()
            if text:
                hub.line("broker", text)
        hub.set_status("broker", f"exited ({proc.wait()})")

    threading.Thread(target=pump, daemon=True).start()
    return proc


# ---------------------------------------------------------------- web page

def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(PAGE, "rb") as f:
                    body = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                q = hub.subscribe()
                try:
                    while True:
                        try:
                            data = q.get(timeout=15)
                            self.wfile.write(f"data: {data}\n\n".encode())
                        except queue.Empty:
                            self.wfile.write(b": keep-alive\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    hub.unsubscribe(q)
            else:
                self.send_error(404)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--id", default="pico-racer-01",
                    help="the car's UTK_MQTT_CLIENT_ID (config/broker_config.h)")
    ap.add_argument("--serial", default="auto",
                    help="auto (find the Pico), a port such as COM5, or none")
    ap.add_argument("--run-broker", action="store_true",
                    help="also start the amqtt broker and show its log")
    ap.add_argument("--save", metavar="DIR",
                    help="append serial.log, decoder.log and broker.log here")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    hub = Hub(args.save)
    broker = run_broker(hub) if args.run_broker else None
    run_mqtt(hub, args)
    if args.serial.lower() != "none":
        run_serial(hub, args.serial)

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(hub))
    server.daemon_threads = True
    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"telemetry viewer on {url}  (Ctrl+C to stop)", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if broker:
            broker.terminate()


if __name__ == "__main__":
    main()
