"""Decode the car's packed MQTT records (docs/comms/MQTT_SCHEMA.md, schema v5).

A module, not a program: tools/telemetry_viewer.py (TelemetryViewer.exe) feeds
it every record it receives and shows the lines.  Dispatches on the record type
`t`, and for E on the event kind `k`; checks what the M7 gates check: the schema
version, the exact length for the type (and for E, the kind), and that s goes
up by exactly 1 (a gap is flagged).  Decodes every field.  Needs no MQTT
library.

The field tables mirror docs/comms/Tribunal/packed_size.py and
app_program/common/messages.h; tests/host/test_decoder.py checks all three agree.
"""

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


# Inbound commands (section 8): the laptop packs them, the car checks them.
# cmd is one byte, op in bits 0-2; ops 3-7 go to nav_ctrl_task (design decision 24).
CMD_OPS = {"snap": 1, "ping": 2, "start": 3, "pause": 4, "resume": 5, "reset": 6, "finish": 7}
# map: 4-bit mv, then a 2-bit action per letter, A first (section 8.3).
MAP_LETTERS = 26
ACTIONS = "FLRU"  # 0 forward, 1 left, 2 right, 3 U-turn


def nbytes(bits):
    return (bits + 7) // 8


def pack_cmd(name):
    """The 1-byte cmd payload for a command name in CMD_OPS."""
    return bytes([CMD_OPS[name]])


def pack_map(mv, actions):
    """The 7-byte map payload.  actions: 26 letters from ACTIONS, or 26 ints 0-3."""
    if len(actions) != MAP_LETTERS:
        raise ValueError(f"a map needs {MAP_LETTERS} actions, got {len(actions)}")
    word = mv & 0xF
    for i, a in enumerate(actions):
        code = ACTIONS.index(a.upper()) if isinstance(a, str) else a
        if not 0 <= code <= 3:
            raise ValueError(f"action for {chr(65 + i)} must be 0-3, got {code}")
        word |= code << (4 + 2 * i)
    return word.to_bytes(nbytes(4 + 2 * MAP_LETTERS), "little")


def describe_command(topic, payload):
    """A sent cmd or map, for the log; None when the topic is neither."""
    if topic.endswith("/cmd"):
        names = {v: k for k, v in CMD_OPS.items()}
        op = payload[0] if len(payload) == 1 else None
        return "cmd " + names.get(op, "invalid [" + payload.hex(" ") + "]")
    if topic.endswith("/map"):
        if len(payload) != nbytes(4 + 2 * MAP_LETTERS):
            return f"map invalid ({len(payload)} B)"
        word = int.from_bytes(payload, "little")
        table = "".join(ACTIONS[(word >> (4 + 2 * i)) & 3] for i in range(MAP_LETTERS))
        return f"map mv={word & 0xF} {table}"
    tail = topic.rsplit("/", 1)[-1]
    if tail not in ("telemetry", "heartbeat", "state"):
        return f"{tail} (not a car topic) [{payload.hex(' ')}]"
    return None


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

        # The laptop's own commands come back on the same wildcard subscription.
        sent = describe_command(topic, payload)
        if sent is not None:
            log(f"SENT {sent}  {len(payload)}B [{raw}]")
            return

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
