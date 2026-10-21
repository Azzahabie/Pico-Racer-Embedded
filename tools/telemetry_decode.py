"""Decode the car's packed MQTT records (docs/comms/MQTT_SCHEMA.md, schema v2).

    python tools/telemetry_decode.py [--broker HOST] [--port N] [--id CLIENT_ID]

Subscribes to robocar/<id>/# and prints one line per record.  Checks what the
M7 gates check: the fixed length per record type, the schema version, and that
s goes up by exactly 1 (a gap is flagged).  Decodes the header of every record;
the bodies of S, E and D are not decoded yet (only H exists so far).

Needs paho-mqtt 2.x (pip install paho-mqtt).
"""

import argparse
import datetime
import time

import paho.mqtt.client as mqtt

SCHEMA_VERSION = 2
# t -> (name, fixed length in bytes), MQTT_SCHEMA.md section 3 rule 1.
RECORDS = {0: ("S", 15), 1: ("E", 9), 2: ("H", 7), 3: ("D", 21)}
HEADER = [("t", 2), ("v", 3), ("s", 16), ("up", 32)]  # 53 bits, section 3.6


def log(text):
    """Print with a wall-clock prefix in the serial console's format, so the
    two logs can be lined up side by side."""
    stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{stamp}] {text}", flush=True)


def unpack(payload, fields):
    """LSB-first: read the payload as one little-endian integer."""
    word = int.from_bytes(payload, "little")
    out, pos = {}, 0
    for name, bits in fields:
        out[name] = (word >> pos) & ((1 << bits) - 1)
        pos += bits
    return out


class Decoder:
    def __init__(self):
        self.last_s = None
        self.last_arrival = None

    def record(self, topic, payload):
        now = time.monotonic()
        gap = "" if self.last_arrival is None else f" +{now - self.last_arrival:.3f}s"
        self.last_arrival = now
        raw = payload.hex(" ")

        if not payload:
            log(f"REJECT {topic}: empty payload")
            return
        h = unpack(payload, HEADER)
        name, length = RECORDS[h["t"]]
        if len(payload) != length:
            log(f"REJECT {topic}: {name} must be {length} B, got {len(payload)} B  [{raw}]")
            return
        if h["v"] != SCHEMA_VERSION:
            log(f"REJECT {topic}: unknown schema v={h['v']}  [{raw}]")
            return

        note = ""
        if self.last_s is not None:
            expected = (self.last_s + 1) & 0xFFFF
            if h["s"] != expected:
                note = f"  GAP: expected s={expected}"
        self.last_s = h["s"]
        log(f"{name} s={h['s']} up={h['up']}ms  {len(payload)}B [{raw}]{gap}{note}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--id", default="pico-racer-01",
                    help="the car's UTK_MQTT_CLIENT_ID (config/broker_config.h)")
    args = ap.parse_args()

    topic = f"robocar/{args.id}/#"
    decoder = Decoder()

    def on_connect(client, userdata, flags, reason_code, properties=None):
        log(f"connected to {args.broker}:{args.port} (rc={reason_code}), "
            f"subscribing to {topic}")
        client.subscribe(topic, qos=0)

    def on_disconnect(client, userdata, flags, reason_code, properties=None):
        log(f"disconnected from {args.broker}:{args.port} (rc={reason_code}); "
            f"retrying")

    def on_message(client, userdata, msg):
        decoder.record(msg.topic, msg.payload)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id="robocar-decoder")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    # Retry until the broker is up, instead of dying if it is started second.
    client.reconnect_delay_set(min_delay=1, max_delay=5)
    log(f"waiting for broker {args.broker}:{args.port}")
    client.connect_async(args.broker, args.port, 60)
    client.loop_forever(retry_first_connection=True)


if __name__ == "__main__":
    main()
