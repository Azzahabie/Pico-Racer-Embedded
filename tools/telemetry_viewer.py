"""Watch the car's logs in a browser, one tag at a time.

    py tools\\telemetry_viewer.py            (or TelemetryViewer.exe)
    py tools\\telemetry_viewer.py [--broker HOST] [--port N] [--id CLIENT_ID]
                                  [--serial auto|COM5|none] [--run-broker]
                                  [--save DIR] [--http-port 8765] [--no-browser]

Opens http://127.0.0.1:8765 and streams three sources into one page:

  mqtt    the car's records, decoded by tools/telemetry_decode.py
  serial  the Pico's USB console.  `auto` finds the Pico (USB VID 2E8A) and
          reopens it after every reset.  Needs pyserial.
  broker  optional: an amqtt broker run inside the viewer, with its log.

Nothing about the network is built in.  Broker address and port, the car's
client ID, the serial port, the built-in broker and the save folder are set
from the page's Settings dialog and kept in
%APPDATA%\\TelemetryViewer\\settings.json.  Command-line flags override the
saved values for one run.

Every line is tagged ([imu], [motion], [comms] S, E mission, GAP, ...), and the
page hides or shows each tag with one click.  Saved logs (Attempt N.md, or
anything in the `[HH:MM:SS.mmm] text` format) can be dropped onto the page.

The save folder gets serial.log, decoder.log and broker.log in that format.

Only one program can hold a COM port: close the serial monitor first, or set
the serial port to none.  Build the exe with tools/build_viewer.bat.
"""

import argparse
import asyncio
import collections
import datetime
import json
import logging
import os
import queue
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import telemetry_decode  # noqa: E402

# Inside a PyInstaller exe the page is unpacked to sys._MEIPASS.
PAGE = os.path.join(getattr(sys, "_MEIPASS", HERE), "telemetry_viewer.html")
SETTINGS_FILE = (os.environ.get("TELEMETRY_VIEWER_SETTINGS")
                 or os.path.join(os.environ.get("APPDATA") or os.path.expanduser("~"),
                                 "TelemetryViewer", "settings.json"))
PICO_VID = 0x2E8A
HISTORY = 20000          # lines kept for a page (re)load

# First-run values only.  The broker defaults to this PC; the car's own
# broker address lives in its firmware config, not here.
DEFAULTS = {
    "broker_host": "127.0.0.1",
    "broker_port": 1883,
    "client_id": "pico-racer-01",
    "serial": "auto",
    "run_broker": False,
    "save_dir": "",
    "http_port": 8765,
}


def stamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def load_settings():
    s = dict(DEFAULTS)
    try:
        with open(SETTINGS_FILE, encoding="utf-8") as f:
            s.update({k: v for k, v in json.load(f).items() if k in DEFAULTS})
    except (OSError, ValueError):
        pass
    return s


def save_settings(s):
    os.makedirs(os.path.dirname(SETTINGS_FILE), exist_ok=True)
    with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
        json.dump(s, f, indent=2)


def clean_settings(raw, base):
    """Accept only known keys with sane values; keep `base` for the rest."""
    s = dict(base)
    for key, val in raw.items():
        if key not in DEFAULTS:
            continue
        if key in ("broker_port", "http_port"):
            val = int(val)
            if not 1 <= val <= 65535:
                raise ValueError(f"{key} must be 1-65535")
        elif key == "run_broker":
            val = bool(val)
        else:
            val = str(val).strip()
        s[key] = val
    if not s["broker_host"]:
        raise ValueError("broker host is empty")
    if not s["client_id"]:
        raise ValueError("client id is empty")
    if not s["serial"]:
        s["serial"] = "none"
    return s


class Hub:
    """Fan every line out to the open pages, and keep the recent ones."""

    def __init__(self):
        self.lock = threading.Lock()
        self.history = collections.deque(maxlen=HISTORY)
        self.clients = []
        self.status = {"mqtt": "off", "serial": "off", "broker": "off"}
        self.files = {}
        self.save_dir = ""

    def set_save(self, save_dir):
        with self.lock:
            if save_dir == self.save_dir:
                return
            for f in self.files.values():
                f.close()
            self.files = {}
            self.save_dir = save_dir
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

class MqttSource:
    """Subscribe to robocar/<id>/# and decode each record."""

    def __init__(self, hub):
        self.hub = hub
        self.client = None
        self.decoder = telemetry_decode.Decoder()
        # Reuse the decoder's own line format: capture what record() logs.
        telemetry_decode.log = lambda text: hub.line("mqtt", text)

    def restart(self, host, port, client_id):
        import paho.mqtt.client as mqtt

        self.stop()
        self.decoder = telemetry_decode.Decoder()
        topic = f"robocar/{client_id}/#"
        where = f"{host}:{port}"

        def on_connect(client, userdata, flags, reason_code, properties=None):
            self.hub.set_status("mqtt", f"connected {where}")
            self.hub.line("viewer", f"mqtt connected to {where}, subscribed to {topic}")
            client.subscribe(topic, qos=0)

        def on_disconnect(client, userdata, flags, reason_code, properties=None):
            self.hub.set_status("mqtt", f"waiting for {where}")

        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id=f"robocar-viewer-{os.getpid()}")
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = lambda c, u, msg: self.decoder.record(msg.topic, msg.payload)
        client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.hub.set_status("mqtt", f"waiting for {where}")
        client.connect_async(host, port, 60)
        client.loop_start()
        self.client = client

    def stop(self):
        if self.client:
            try:
                self.client.disconnect()
                self.client.loop_stop()
            except Exception:
                pass
            self.client = None


def list_serial_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [{"device": p.device, "desc": p.description,
             "pico": p.vid == PICO_VID} for p in list_ports.comports()]


class SerialSource:
    """Read the Pico's console; reopen the port whenever it comes back."""

    def __init__(self, hub):
        self.hub = hub
        self.gen = 0

    def restart(self, port_arg):
        self.gen += 1
        gen = self.gen
        if port_arg.lower() == "none":
            self.hub.set_status("serial", "off")
            return
        try:
            import serial
            from serial.tools import list_ports
        except ImportError:
            self.hub.set_status("serial", "pyserial missing: py -m pip install pyserial")
            return

        def find():
            if port_arg.lower() != "auto":
                return port_arg
            for p in list_ports.comports():
                if p.vid == PICO_VID:
                    return p.device
            return None

        def loop():
            while gen == self.gen:
                port = find()
                if not port:
                    self.hub.set_status("serial", "no Pico found (waiting)")
                    time.sleep(1)
                    continue
                try:
                    ser = serial.Serial(port, 115200, timeout=0.3)
                except (serial.SerialException, OSError) as why:
                    busy = "denied" in str(why).lower()
                    self.hub.set_status("serial", f"{port} busy (close the serial monitor)"
                                        if busy else f"{port} unavailable")
                    time.sleep(1)
                    continue
                self.hub.set_status("serial", f"open {port}")
                self.hub.line("viewer", f"serial {port} opened")
                buf = b""
                try:
                    while gen == self.gen:
                        chunk = ser.read(ser.in_waiting or 1)
                        if not chunk:
                            continue
                        buf += chunk
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            text = raw.decode("utf-8", "replace").rstrip("\r")
                            if text.strip():
                                self.hub.line("serial", text)
                except (serial.SerialException, OSError):
                    self.hub.line("viewer", f"serial {port} closed (Pico reset or unplugged)")
                finally:
                    try:
                        ser.close()
                    except Exception:
                        pass
                time.sleep(0.5)

        threading.Thread(target=loop, daemon=True).start()


class BrokerLogHandler(logging.Handler):
    def __init__(self, hub):
        super().__init__(logging.INFO)
        self.hub = hub
        self.setFormatter(logging.Formatter("%(levelname)s - %(message)s"))

    def emit(self, record):
        try:
            self.hub.line("broker", self.format(record))
        except Exception:
            pass


class BrokerSource:
    """An amqtt broker on its own event loop, inside this process, so the exe
    needs no Python install.  Listens on every interface (the car must reach
    it over the LAN) on the configured port."""

    def __init__(self, hub):
        self.hub = hub
        self.loop = None
        self.broker = None
        self.port = None
        log = logging.getLogger("amqtt")
        log.setLevel(logging.INFO)
        log.addHandler(BrokerLogHandler(hub))
        log.propagate = False

    def apply(self, run, port):
        if run and self.loop and port == self.port:
            return
        self.stop()
        if run:
            self.start(port)

    def start(self, port):
        try:
            from amqtt.broker import Broker
        except ImportError:
            self.hub.set_status("broker", "amqtt missing: py -m pip install amqtt")
            return
        config = {
            "listeners": {"default": {"type": "tcp", "bind": f"0.0.0.0:{port}"}},
            "plugins": {
                "amqtt.plugins.logging_amqtt.EventLoggerPlugin": {},
                "amqtt.plugins.authentication.AnonymousAuthPlugin": {"allow_anonymous": True},
            },
        }
        loop = asyncio.new_event_loop()
        started = threading.Event()

        def run():
            asyncio.set_event_loop(loop)
            try:
                self.broker = Broker(config, loop=loop)
                loop.run_until_complete(self.broker.start())
            except Exception as why:
                self.hub.set_status("broker", f"failed: {why}")
                self.hub.line("broker", f"ERROR - broker did not start on port {port}: {why}")
                self.loop = None
                started.set()
                return
            self.hub.set_status("broker", f"running 0.0.0.0:{port}")
            started.set()
            loop.run_forever()

        self.loop, self.port = loop, port
        threading.Thread(target=run, daemon=True).start()
        started.wait(10)

    def stop(self):
        loop, broker = self.loop, self.broker
        self.loop = self.broker = None
        if loop and broker:
            try:
                asyncio.run_coroutine_threadsafe(broker.shutdown(), loop).result(5)
            except Exception:
                pass
            loop.call_soon_threadsafe(loop.stop)
        self.hub.set_status("broker", "off")


class Viewer:
    def __init__(self, settings):
        self.hub = Hub()
        self.settings = settings
        self.broker = BrokerSource(self.hub)
        self.mqtt = MqttSource(self.hub)
        self.serial = SerialSource(self.hub)
        self.lock = threading.Lock()

    def apply(self, new, old=None):
        """Restart only what the change touches."""
        with self.lock:
            s = self.settings = new
            changed = (lambda *keys: old is None or any(old[k] != s[k] for k in keys))
            self.hub.set_save(s["save_dir"])
            if changed("run_broker", "broker_port"):
                self.broker.apply(s["run_broker"], s["broker_port"])
            if changed("broker_host", "broker_port", "client_id"):
                self.mqtt.restart(s["broker_host"], s["broker_port"], s["client_id"])
            if changed("serial"):
                self.serial.restart(s["serial"])


# ---------------------------------------------------------------- web page

def make_handler(viewer):
    hub = viewer.hub

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send_json(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(PAGE, "rb") as f:
                    body = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/config":
                self.send_json(200, {"settings": viewer.settings,
                                     "ports": list_serial_ports(),
                                     "settings_file": SETTINGS_FILE,
                                     "first_run": not os.path.exists(SETTINGS_FILE)})
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

        def do_POST(self):
            if self.path != "/config":
                self.send_error(404)
                return
            # The page is served from 127.0.0.1 only; refuse other origins.
            origin = self.headers.get("Origin", "")
            if origin and not origin.startswith(("http://127.0.0.1", "http://localhost")):
                self.send_error(403)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                raw = json.loads(self.rfile.read(length) or b"{}")
                old = dict(viewer.settings)
                new = clean_settings(raw, old)
                save_settings(new)
                viewer.apply(new, old)
                hub.line("viewer", "settings saved to " + SETTINGS_FILE)
                self.send_json(200, {"ok": True, "settings": new})
            except (ValueError, TypeError, OSError) as why:
                self.send_json(400, {"ok": False, "error": str(why)})

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--broker", dest="broker_host", help="broker address to subscribe to")
    ap.add_argument("--port", dest="broker_port", type=int, help="broker port")
    ap.add_argument("--id", dest="client_id",
                    help="the car's UTK_MQTT_CLIENT_ID (config/broker_config.h)")
    ap.add_argument("--serial", help="auto (find the Pico), a port such as COM5, or none")
    ap.add_argument("--run-broker", dest="run_broker", action="store_true", default=None,
                    help="also run a broker inside the viewer and show its log")
    ap.add_argument("--save", dest="save_dir", metavar="DIR",
                    help="append serial.log, decoder.log and broker.log here")
    ap.add_argument("--http-port", dest="http_port", type=int)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    settings = load_settings()
    overrides = {k: v for k, v in vars(args).items() if k in DEFAULTS and v is not None}
    settings = clean_settings(overrides, settings)

    viewer = Viewer(settings)
    viewer.apply(settings)

    try:
        server = ThreadingHTTPServer(("127.0.0.1", settings["http_port"]), make_handler(viewer))
    except OSError:
        url = f"http://127.0.0.1:{settings['http_port']}/"
        print(f"port {settings['http_port']} is in use - is the viewer already open? {url}")
        webbrowser.open(url)
        return
    server.daemon_threads = True
    url = f"http://127.0.0.1:{settings['http_port']}/"
    print(f"Telemetry viewer on {url}\nSettings: {SETTINGS_FILE}\n"
          f"Close this window (or Ctrl+C) to stop.", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.broker.stop()
        viewer.mqtt.stop()


if __name__ == "__main__":
    main()
