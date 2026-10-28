"""Decode the car's packed MQTT records (docs/comms/MQTT_SCHEMA.md, schema v5).

    python tools/telemetry_decode.py [--broker HOST] [--port N] [--id CLIENT_ID]

Subscribes to robocar/<id>/# and prints one line per record.  Dispatches on the
record type `t`, and for E on the event kind `k`; checks what the M7 gates
check: the schema version, the exact length for the type (and for E, the kind),
and that s goes up by exactly 1 (a gap is flagged).  Decodes every field.

Needs paho-mqtt 2.x (pip install paho-mqtt) to connect; decode() alone does not.
The field tables mirror docs/comms/Tribunal/packed_size.py and
app_program/common/messages.h; tests/host/test_decoder.py checks all three agree.
"""

import argparse
import datetime
import time

SCHEMA_VERSION = 5
HEADER = [("t", 2), ("v", 3), ("s", 16), ("up", 32)]  # 53 bits, section 3.6
HEADER_BITS = 53

S_BODY = [("faults", 6), ("mot", 4), ("dst", 8), ("lin", 2), ("dev", 8), ("bar", 5),
          ("act", 2), ("hum", 4), ("hgt", 8), ("obs", 8), ("byp", 2), ("mis", 3),
          ("mpv", 4), ("drp", 4), ("tlt", 6), ("trn", 12)]
D_EXTRA = [("rssi", 7), ("ip", 32), ("qd", 3), ("mqv", 2)]

# Event bodies after the 3-bit kind (section 5.1).  `k`: (name, fields).
E_BODIES = {
    0: ("motion", [("state", 4), ("speed_l", 8), ("speed_r", 8), ("dist_mm", 15),
                   ("turn_rate", 10), ("fault", 2), ("ovf", 1)]),
    1: ("line", [("state", 2), ("deviation", 8)]),
    2: ("barcode", [("letter", 5), ("action", 2)]),
    3: ("imu", [("class", 3), ("hump_count", 4), ("hump_height_mm", 8)]),
    4: ("obstacle", [("width_cm", 8), ("bypass", 2)]),
    5: ("mission", [("state", 3)]),
}
# The shared fault bitmask, bit 0 first (section 4.1, messages.h FAULT_*).
FAULT_NAMES = ["motor_stall", "encoder", "imu", "scan_sensor_lost", "line_sensor", "wifi_loss"]
# Offset-encoded fields: stored = value + offset (section 3 rule 5).
OFFSETS = {"dev": 128, "deviation": 128, "speed_l": 128, "speed_r": 128, "turn_rate": 512,
           "trn": 2048}
# "Not available" codes (section 4): FR-29 reports tilt and turn rate when available.
NOT_AVAILABLE = {"tlt": 63, "trn": 0}


def nbytes(bits):
    return (bits + 7) // 8


def e_length(kind):
    return nbytes(HEADER_BITS + 3 + sum(b for _, b in E_BODIES[kind][1]))


LENGTHS = {0: nbytes(HEADER_BITS + sum(b for _, b in S_BODY)),
           2: nbytes(HEADER_BITS),
           3: nbytes(HEADER_BITS + sum(b for _, b in S_BODY + D_EXTRA))}


class Reject(Exception):
    pass


def unpack(payload, fields, pos=0):
    """LSB-first: read the payload as one little-endian integer."""
    word = int.from_bytes(payload, "little")
    out = {}
    for name, bits in fields:
        out[name] = (word >> pos) & ((1 << bits) - 1)
        pos += bits
    return out, pos


def decode(payload):
    """Return (record name, stored field values).  Raises Reject on a record the
    contract does not allow: unknown v, reserved kind, or the wrong length."""
    if not payload:
        raise Reject("empty payload")
    h, pos = unpack(payload, HEADER)
    if h["v"] != SCHEMA_VERSION:
        raise Reject(f"unknown schema v={h['v']}")
    t = h["t"]
    if t == 1:
        k = (int.from_bytes(payload, "little") >> HEADER_BITS) & 0x7
        if k not in E_BODIES:
            raise Reject(f"E with reserved kind k={k}")
        name, fields = E_BODIES[k]
        want, rec = e_length(k), f"E {name}"
        fields = [("k", 3)] + fields
    else:
        name = {0: "S", 2: "H", 3: "D"}[t]
        want, rec = LENGTHS[t], name
        fields = {0: S_BODY, 2: [], 3: S_BODY + D_EXTRA}[t]
    if len(payload) != want:
        raise Reject(f"{rec} must be {want} B, got {len(payload)} B")
    body, _ = unpack(payload, fields, pos)
    return rec, dict(h, **body)


def describe(values):
    """The fields after the header, with offsets removed and letters shown."""
    parts = []
    for key, val in values.items():
        if key in ("t", "v", "s", "up", "k"):
            continue
        if NOT_AVAILABLE.get(key) == val:
            val = "n/a"
        elif key in OFFSETS:
            val = val - OFFSETS[key]
        elif key in ("bar", "letter"):
            val = "none" if val == 31 else chr(65 + val) if val <= 25 else f"?{val}"
        elif key == "faults":
            val = "|".join(n for i, n in enumerate(FAULT_NAMES) if val >> i & 1) or "0"
        elif key == "ip":
            val = ".".join(str((val >> (8 * i)) & 0xFF) for i in range(4))
        parts.append(f"{key}={val}")
    return " ".join(parts)


def log(text):
    """Print with a wall-clock prefix in the serial console's format, so the
    two logs can be lined up side by side."""
    stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{stamp}] {text}", flush=True)


class Decoder:
    def __init__(self):
        self.last_s = None
        self.last_arrival = None

    def record(self, topic, payload):
        now = time.monotonic()
        gap = "" if self.last_arrival is None else f" +{now - self.last_arrival:.3f}s"
        self.last_arrival = now
        raw = payload.hex(" ")

        try:
            name, values = decode(payload)
        except Reject as why:
            log(f"REJECT {topic}: {why}  [{raw}]")
            return

        note = ""
        if self.last_s is not None:
            expected = (self.last_s + 1) & 0xFFFF
            if values["s"] != expected:
                note = f"  GAP: expected s={expected}"
        self.last_s = values["s"]
        fields = describe(values)
        fields = f" {fields}" if fields else ""
        log(f"{name} s={values['s']} up={values['up']}ms{fields}  "
            f"{len(payload)}B [{raw}]{gap}{note}")


def main():
    import paho.mqtt.client as mqtt

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
