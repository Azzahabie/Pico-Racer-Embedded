"""Minimum-bit layout for the MQTT records (Change Requirement 1), schema v5.

Week 13 evidence, Test 1.  Run it and the printed tables are the data
representation table, the per-kind event table with its headroom, the message
size comparison and the MQTT_SCHEMA.md worked examples:

    python docs/comms/Tribunal/packed_size.py

Calculated, not measured on hardware.  Ranges are proposed from the units in
docs/comms/MQTT_SCHEMA.md and the Week 6 freeze, so a total moves if a range
moves.  Kept in the repo so the figures can be regenerated rather than trusted.

`pack()` is the reference bit order that the C helpers in
app_program/common/messages.h and the laptop decoder (tools/telemetry_decode.py)
must both follow (MQTT_SCHEMA.md section 3).  tests/host/gen_vectors.py builds
the shared test vectors from these tables.

History: 2026-10-19 (Tribunal 1) schema v2: 53-bit header, H, D, cmd, map.
2026-10-22 schema v3: one body layout per event kind (E is variable length),
`mot` in S widened from 2 to 4 bits (S stays 15 bytes), IMU event class added.
2026-10-23 schema v4: fault bitmask 5 -> 6 bits as agreed with Buddy 5 (bit 4
line sensor, bit 5 Wi-Fi loss), so S is 16 bytes; D stays 21.
2026-10-28 schema v5: tilt and turn rate in S for FR-29 (S 18 bytes, D 23),
the outbound record limit 21 -> 24 bytes, IMU motion class 0..6.
"""

SCHEMA_VERSION = 5

# field, meaning, proposed range, bits, note
HEADER = [
    ("t",     "record type",              "0 S, 1 E, 2 H, 3 D",    2, "same position in every record"),
    ("v",     "schema version",           "0..7",                  3, "5 = this layout (4 = no tilt/turn, 3 = 5 fault bits, 2 = Tribunal 1, JSON was 1)"),
    ("s",     "sequence number",          "0..65535 over a run",  16, "a 5 min run is under 300 lines"),
    ("up",    "ms since boot",            "0..2^32-1",            32, "boot semantics kept; run-relative would be 19"),
]

S_BODY = [
    ("faults","shared fault bitmask",     "6 named faults",        6, "one bit per fault; v4: was 5"),
    ("mot",   "motion state",             "0..15",                 4, "v3: was 2 bits, as motion E state"),
    ("dst",   "distance since last change","0..255 cm",            8, "clamped; comms_task derives it from dist_mm"),
    ("lin",   "line state",               "0..2",                  2, ""),
    ("dev",   "line deviation",           "-128..127 tenths",      8, "offset-encoded, stored + 128"),
    ("bar",   "barcode letter",           "0..25 = A..Z, 31 none", 5, "index, the decoder adds 65"),
    ("act",   "action code",              "0..3",                  2, "Buddy 3's 2-bit action"),
    ("hum",   "humps detected",           "0..15",                 4, ""),
    ("hgt",   "last hump height",         "0..255 mm",             8, "clamped"),
    ("obs",   "last obstacle width",      "0..255 cm",             8, "clamped"),
    ("byp",   "bypass taken",             "0..2",                  2, ""),
    ("mis",   "mission state",            "7 states",              3, "matches the requirement's own example"),
    ("mpv",   "barcode table version",    "0..15",                 4, "wraps"),
    ("drp",   "dropped count",            "0..15",                 4, "saturates"),
    ("tlt",   "tilt from level",          "0..62 deg, 63 n/a",     6, "FR-29; from imu_task's tilt-state word"),
    ("trn",   "turn rate",                "-2047..2047 deg/s, 0 n/a", 12, "FR-29; offset-encoded, stored + 2048"),
]

# Event bodies, one per kind: kind in bits 0-2, then the kind's fields.
# The same body is the tele_mbf intake message and, after the header, record E.
KIND = ("k", "event kind", "0..5 (6, 7 reserved)", 3, "")
E_BODIES = {
    0: ("motion", "Buddy 2", [
        KIND,
        ("state",     "motion state",        "0..15",                 4, ""),
        ("speed_l",   "left wheel speed",    "-128..127 cm/s",        8, "PLACEHOLDER; offset-encoded, stored + 128"),
        ("speed_r",   "right wheel speed",   "-128..127 cm/s",        8, "PLACEHOLDER; offset-encoded, stored + 128"),
        ("dist_mm",   "distance travelled",  "0..32767 mm, wraps",   15, "wrapping counter; take differences"),
        ("turn_rate", "turn rate",           "-512..511 deg/s",      10, "offset-encoded, stored + 512"),
        ("fault",     "fault at the moment", "bit 0 stall, bit 1 encoder", 2, "the S bitmask is the current state"),
        ("ovf",       "a field was clamped", "0..1",                  1, "PLACEHOLDER"),
    ]),
    1: ("line", "Buddy 3", [
        KIND,
        ("state",     "line state",          "0 on, 1 lost, 2 crossing", 2, ""),
        ("deviation", "line deviation",      "-128..127 tenths",      8, "offset-encoded, stored + 128"),
    ]),
    2: ("barcode", "Buddy 3", [
        KIND,
        ("letter",    "letter index",        "0..25 = A..Z, 31 unreadable", 5, ""),
        ("action",    "action code",         "0..3",                  2, ""),
    ]),
    3: ("imu", "Buddy 4", [
        KIND,
        ("class",     "motion event class",  "0..6 (FR-27 and more)", 3, "7 reserved"),
        ("hump_count","humps detected",      "0..15",                 4, ""),
        ("hump_height_mm", "hump height",    "0..255 mm",             8, "clamped"),
    ]),
    4: ("obstacle", "Buddy 5", [
        KIND,
        ("width_cm",  "obstacle width",      "0..255 cm",             8, "clamped"),
        ("bypass",    "bypass taken",        "0 none, 1 left, 2 right", 2, ""),
    ]),
    5: ("mission", "Vehicle Controller", [
        KIND,
        ("state",     "mission state",       "0..6",                  3, ""),
    ]),
}

# CYW43_MQTT_RECORD_MAX: the largest record the radio's outbound queue takes.
E_SIZE_LIMIT = 24

D_EXTRA = [
    ("rssi", "signal strength",   "0..127, stores -dBm", 7, ""),
    ("ip",   "IPv4 address",      "32 bits",            32, "first octet in the low byte"),
    ("qd",   "publish queue depth","0..7",               3, ""),
    ("mqv",  "MQTT verdict",      "0..2",                2, "0 disconnected, 1 connecting, 2 connected"),
]

S_FIELDS = HEADER + S_BODY
H_FIELDS = HEADER
D_FIELDS = HEADER + S_BODY + D_EXTRA

CMD_FIELDS = [
    ("op",  "command opcode", "1 snap, 2 ping", 3, "0 and 3..7 rejected"),
    ("rsv", "reserved",       "must be 0",      5, "non-zero is rejected"),
]
MAP_FIELDS = [("mv", "table version", "0..15", 4, "wraps")] + [
    (f"act_{chr(65 + i)}", f"action for {chr(65 + i)}", "0..3", 2, "") for i in range(26)
]


def body_fields(kind):
    return E_BODIES[kind][2]


def e_fields(kind):
    return HEADER + body_fields(kind)


def total(fields):
    return sum(b for _, _, _, b, _ in fields)


def nbytes(fields):
    return (total(fields) + 7) // 8


def pack(fields, values):
    """LSB-first: field 0 starts at bit 0 of byte 0; bytes are little-endian."""
    word, pos = 0, 0
    for key, _, _, bits, _ in fields:
        v = values.get(key, 0)
        assert 0 <= v < (1 << bits), f"{key}={v} does not fit {bits} bits"
        word |= v << pos
        pos += bits
    return word.to_bytes(nbytes(fields), "little")


def headroom():
    """(kind, name, owner, body bits, intake B, spare intake bits, wire B, spare wire bits)."""
    rows = []
    for kind, (name, owner, fields) in E_BODIES.items():
        body = total(fields)
        intake = (body + 7) // 8
        wire_bits = total(HEADER) + body
        wire = (wire_bits + 7) // 8
        rows.append((kind, name, owner, body, intake, intake * 8 - body, wire, wire * 8 - wire_bits))
    return rows


def show(name, fields):
    print(f"### {name}\n")
    print("| Field | Meaning | Proposed range | Bits | Note |")
    print("|---|---|---|---|---|")
    for key, meaning, rng, bits, note in fields:
        print(f"| `{key}` | {meaning} | {rng} | {bits} | {note} |")
    bits = total(fields)
    print(f"\nTotal {bits} bits, which pads to {nbytes(fields)} bytes.\n")


# Worked examples (MQTT_SCHEMA.md section 10), as stored values.
EXAMPLES = {
    "S": (S_FIELDS, dict(t=0, v=SCHEMA_VERSION, s=137, up=41230, faults=0, mot=2, dst=20,
                         lin=1, dev=3 + 128, bar=4, act=2, hum=3, hgt=12, obs=0, byp=0,
                         mis=1, mpv=3, drp=0, tlt=4, trn=-30 + 2048)),
    "E motion": (e_fields(0), dict(t=1, v=SCHEMA_VERSION, s=138, up=41310, k=0, state=2,
                                   speed_l=20 + 128, speed_r=24 + 128, dist_mm=1500,
                                   turn_rate=-30 + 512, fault=0, ovf=0)),
    "E line": (e_fields(1), dict(t=1, v=SCHEMA_VERSION, s=139, up=41320, k=1, state=1,
                                 deviation=-12 + 128)),
    "E barcode": (e_fields(2), dict(t=1, v=SCHEMA_VERSION, s=140, up=41330, k=2, letter=4,
                                    action=2)),
    "E imu": (e_fields(3), dict(t=1, v=SCHEMA_VERSION, s=141, up=41340, k=3, **{"class": 2},
                                hump_count=1, hump_height_mm=12)),
    "E obstacle": (e_fields(4), dict(t=1, v=SCHEMA_VERSION, s=142, up=41350, k=4, width_cm=18,
                                     bypass=1)),
    "E mission": (e_fields(5), dict(t=1, v=SCHEMA_VERSION, s=143, up=41360, k=5, state=1)),
    "H": (H_FIELDS, dict(t=2, v=SCHEMA_VERSION, s=144, up=42000)),
}


if __name__ == "__main__":
    show("Record S, the periodic state line", S_FIELDS)
    for kind, (name, owner, fields) in E_BODIES.items():
        show(f"Event body k={kind}, {name} ({owner})", fields)
    print(f"Record H: {total(H_FIELDS)} bits, {nbytes(H_FIELDS)} bytes.  "
          f"Record D: {total(D_FIELDS)} bits, {nbytes(D_FIELDS)} bytes.  "
          f"cmd: {nbytes(CMD_FIELDS)} byte.  map: {total(MAP_FIELDS)} bits, {nbytes(MAP_FIELDS)} bytes.\n")

    hdr = total(HEADER)
    max_body = E_SIZE_LIMIT * 8 - hdr
    print("### Event sizes and headroom\n")
    print("| `k` | Kind | Owner | Body bits | Intake bytes | Spare intake bits | Wire E bytes | Spare wire bits |")
    print("|---|---|---|---|---|---|---|---|")
    for kind, name, owner, body, intake, spare_i, wire, spare_w in headroom():
        print(f"| {kind} | {name} | {owner} | {body} | {intake} | {spare_i} | {wire} | {spare_w} |")
    print(f"\nA wire E is capped at {E_SIZE_LIMIT} bytes (CYW43_MQTT_RECORD_MAX), "
          f"so a body may grow to {max_body} bits.\n")

    s_bytes = nbytes(S_FIELDS)
    JSON_MEASURED = 141
    KV_MEASURED = 101

    print("### Comparison\n")
    print(f"- JSON state line, measured: {JSON_MEASURED} bytes")
    print(f"- key=value state line, measured: {KV_MEASURED} bytes")
    print(f"- packed record S, calculated: {s_bytes} bytes")
    print(f"- reduction against JSON: {JSON_MEASURED / s_bytes:.1f}x, saving {JSON_MEASURED - s_bytes} bytes")
    print(f"- map payload: 36 bytes as key=value text, {nbytes(MAP_FIELDS)} bytes packed")

    s_bits = total(S_FIELDS)
    top = sorted(S_FIELDS, key=lambda f: -f[3])[:2]
    print(f"- the two biggest fields, {top[0][0]} at {top[0][3]} bits and {top[1][0]} at {top[1][3]} bits, are "
          f"{(top[0][3] + top[1][3]) * 100 / s_bits:.0f}% of the record")
    run_relative = s_bits - (32 - 19)
    print(f"- if `up` is rescoped to ms since run start (19 bits), S is {run_relative} bits "
          f"= {(run_relative + 7) // 8} bytes\n")

    print(f"### Worked examples (MQTT_SCHEMA.md), schema v{SCHEMA_VERSION}\n")
    for name, (fields, vals) in EXAMPLES.items():
        print(f"- {name}: {pack(fields, vals).hex(' ')}")
    print(f"- cmd snap: {pack(CMD_FIELDS, dict(op=1)).hex(' ')}")
    print(f"- map: {pack(MAP_FIELDS, dict(mv=3, act_E=3, act_L=1)).hex(' ')}")
