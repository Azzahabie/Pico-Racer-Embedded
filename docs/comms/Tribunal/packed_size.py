"""Proposed minimum-bit layout for the MQTT records (Change Requirement 1).

Week 13 evidence, Test 1.  Run it and the printed tables are the data
representation table plus the message size comparison:

    python docs/comms/Tribunal/packed_size.py

Calculated, not measured on hardware.  Ranges are proposed from the units in
docs/comms/MQTT_SCHEMA.md and the Week 6 freeze, so the total moves if a range
moves.  Kept in the repo so the figure can be regenerated rather than trusted.

Updated from Tribunal 1 (2026-10-19): `act` is 2 bits (Buddy 3's 2-bit action),
every record starts with the same 53-bit header (`t` is 2 bits everywhere, so a
decoder can dispatch on the first byte), the H and D records and the two inbound
payloads are added, and `pack()` is the reference bit order that the C helpers
in messages.h and the laptop decoder must both follow (MQTT_SCHEMA.md section 3).
"""

# field, meaning, proposed range, bits, note
HEADER = [
    ("t",     "record type",              "0 S, 1 E, 2 H, 3 D",    2, "same position in every record"),
    ("v",     "schema version",           "0..7",                  3, "2 = this binary layout (JSON was 1)"),
    ("s",     "sequence number",          "0..65535 over a run",  16, "a 5 min run is under 300 lines"),
    ("up",    "ms since boot",            "0..2^32-1",            32, "boot semantics kept; run-relative would be 19"),
]

S_BODY = [
    ("faults","shared fault bitmask",     "5 named faults",        5, "one bit per fault"),
    ("mot",   "motion state",             "0..3",                  2, ""),
    ("dst",   "distance since last change","0..255 cm",            8, "clamped"),
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
]

E_BODY = [
    ("k",  "event kind",         "6 kinds (0..5)", 3, ""),
    ("a",  "payload value 1",    "widest kind",    8, ""),
    ("b",  "payload value 2",    "widest kind",    8, ""),
]

D_EXTRA = [
    ("rssi", "signal strength",   "0..127, stores -dBm", 7, ""),
    ("ip",   "IPv4 address",      "32 bits",            32, "first octet in the low byte"),
    ("qd",   "publish queue depth","0..7",               3, ""),
    ("mqv",  "MQTT verdict",      "0..2",                2, "0 disconnected, 1 connecting, 2 connected"),
]

S_FIELDS = HEADER + S_BODY
E_FIELDS = HEADER + E_BODY
H_FIELDS = HEADER
D_FIELDS = HEADER + S_BODY + D_EXTRA

CMD_FIELDS = [
    ("op",  "command opcode", "1 snap, 2 ping", 3, "0 and 3..7 rejected"),
    ("rsv", "reserved",       "must be 0",      5, "non-zero is rejected"),
]
MAP_FIELDS = [("mv", "table version", "0..15", 4, "wraps")] + [
    (f"act_{chr(65 + i)}", f"action for {chr(65 + i)}", "0..3", 2, "") for i in range(26)
]


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


def show(name, fields):
    print(f"### {name}\n")
    print("| Field | Meaning | Proposed range | Bits | Note |")
    print("|---|---|---|---|---|")
    for key, meaning, rng, bits, note in fields:
        print(f"| `{key}` | {meaning} | {rng} | {bits} | {note} |")
    bits = total(fields)
    print(f"\nTotal {bits} bits, which pads to {nbytes(fields)} bytes.\n")


if __name__ == "__main__":
    show("Record S, the periodic state line", S_FIELDS)
    show("Record E, the edge event", E_FIELDS)
    print(f"Record H: {total(H_FIELDS)} bits, {nbytes(H_FIELDS)} bytes.  "
          f"Record D: {total(D_FIELDS)} bits, {nbytes(D_FIELDS)} bytes.  "
          f"cmd: {nbytes(CMD_FIELDS)} byte.  map: {total(MAP_FIELDS)} bits, {nbytes(MAP_FIELDS)} bytes.\n")

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

    print("### Worked examples (MQTT_SCHEMA.md)\n")
    s_ex = dict(t=0, v=2, s=137, up=41230, faults=0, mot=2, dst=20, lin=1, dev=3 + 128,
                bar=4, act=2, hum=3, hgt=12, obs=0, byp=0, mis=1, mpv=3, drp=0)
    e_ex = dict(t=1, v=2, s=138, up=41310, k=2, a=4, b=2)
    cmd_snap = dict(op=1)
    map_ex = dict(mv=3, act_E=3, act_L=1)
    for name, fields, vals in [("S", S_FIELDS, s_ex), ("E", E_FIELDS, e_ex),
                               ("cmd snap", CMD_FIELDS, cmd_snap), ("map", MAP_FIELDS, map_ex)]:
        print(f"- {name}: {pack(fields, vals).hex(' ')}")
