"""Generate the shared test vectors for the C packer and the Python decoder.

    python tests/host/gen_vectors.py

Writes tests/host/vectors.h (checked by test_messages.c) and
tests/host/vectors.json (checked by test_decoder.py).  The bytes come from
packed_size.py's reference pack(), and the clamping and offset-encoding of the
inputs is done here in Python, independently of the C helpers, so the C test
compares two separate implementations.  Re-run after any layout change.
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "docs" / "comms" / "Tribunal"))
import packed_size as ps  # noqa: E402


def clamp(v, bits):
    top = (1 << bits) - 1
    return (min(max(v, 0), top), v > top or v < 0)


def offset(v, bits):
    half = 1 << (bits - 1)
    c = min(max(v, -half), half - 1)
    return (c + half, c != v)


def encode(kind, args):
    """Producer inputs -> stored field values, mirroring each comm_w_pack_* helper."""
    if kind == 0:
        state, sl, sr, dist, turn, fault, ovf = args
        st, c1 = clamp(state, 4)
        l, c2 = offset(sl, 8)
        r, c3 = offset(sr, 8)
        t, c4 = offset(turn, 10)
        return dict(k=0, state=st, speed_l=l, speed_r=r, dist_mm=dist & 0x7FFF,
                    turn_rate=t, fault=fault & 3, ovf=int(bool(ovf) or c1 or c2 or c3 or c4))
    if kind == 1:
        state, dev = args
        return dict(k=1, state=clamp(state, 2)[0], deviation=offset(dev, 8)[0])
    if kind == 2:
        letter, action = args
        return dict(k=2, letter=letter if 0 <= letter <= 25 else 31, action=clamp(action, 2)[0])
    if kind == 3:
        cls, count, height = args
        return {"k": 3, "class": clamp(cls, 3)[0], "hump_count": clamp(count, 4)[0],
                "hump_height_mm": clamp(height, 8)[0]}
    if kind == 4:
        width, bypass = args
        return dict(k=4, width_cm=clamp(width, 8)[0], bypass=clamp(bypass, 2)[0])
    (state,) = args
    return dict(k=5, state=clamp(state, 3)[0])


# name, kind, producer inputs (in comm_w_pack_* argument order)
CASES = [
    ("motion min",          0, (0, -128, -128, 0, -512, 0, 0)),
    ("motion max",          0, (15, 127, 127, 32767, 511, 3, 1)),
    ("motion mid",          0, (7, 0, 0, 16384, 0, 1, 0)),
    ("motion clamp",        0, (20, 300, -300, 100, 1000, 0, 0)),
    ("motion wrap 32768",   0, (1, 10, 10, 32768, 0, 0, 0)),
    ("motion wrap 32772",   0, (1, 10, 10, 32772, 0, 0, 0)),
    ("line min",            1, (0, -128)),
    ("line max",            1, (3, 127)),
    ("line mid",            1, (1, 0)),
    ("line dev -200",       1, (0, -200)),
    ("line dev +200",       1, (0, 200)),
    ("line dev -12",        1, (2, -12)),
    ("barcode min",         2, (0, 0)),
    ("barcode max",         2, (25, 3)),
    ("barcode mid",         2, (12, 1)),
    ("barcode unreadable",  2, (31, 0)),
    ("barcode letter 40",   2, (40, 2)),
    ("imu min",             3, (0, 0, 0)),
    ("imu max",             3, (7, 15, 255)),
    ("imu mid",             3, (2, 7, 128)),
    ("imu clamp",           3, (4, 20, 300)),
    ("obstacle min",        4, (0, 0)),
    ("obstacle max",        4, (255, 3)),
    ("obstacle mid",        4, (128, 1)),
    ("obstacle clamp",      4, (400, 2)),
    ("mission min",         5, (0,)),
    ("mission max",         5, (7,)),
    ("mission mid",         5, (3,)),
    ("mission clamp",       5, (9,)),
]

# Records with no producer helper yet, built from the field lists.
RECORDS = [
    ("S example", ps.EXAMPLES["S"]),
    ("S all faults", (ps.S_FIELDS, dict(ps.EXAMPLES["S"][1], faults=0x3F))),
    ("S Wi-Fi loss", (ps.S_FIELDS, dict(ps.EXAMPLES["S"][1], faults=0x20))),
    ("S tilt/turn n/a", (ps.S_FIELDS, dict(ps.EXAMPLES["S"][1], tlt=63, trn=0))),
    ("H example", ps.EXAMPLES["H"]),
    ("D example", (ps.D_FIELDS, dict(ps.EXAMPLES["S"][1], t=3, rssi=58, ip=0x0732A8C0, qd=2, mqv=2))),
]

# Inbound commands (schema section 8): built by the laptop, checked by the car.
D_DIAG = dict(rssi=58, ip=0x0732A8C0, qd=2, mqv=2)
MAP_EXAMPLE = dict(mv=11, **{f"act_{chr(65 + i)}": (i * 7 + 1) % 4 for i in range(26)})
COMMANDS = [
    ("cmd snap", "cmd", (ps.CMD_FIELDS, dict(op=1))),
    ("cmd ping", "cmd", (ps.CMD_FIELDS, dict(op=2))),
    ("cmd start", "cmd", (ps.CMD_FIELDS, dict(op=3))),
    ("cmd finish", "cmd", (ps.CMD_FIELDS, dict(op=7))),
    ("map example", "map", (ps.MAP_FIELDS, MAP_EXAMPLE)),
]


def c_bytes(b):
    return "{" + ", ".join(f"0x{x:02x}" for x in b) + "}"


def main():
    out_c, out_j = [], {"events": [], "records": []}
    for i, (name, kind, args) in enumerate(CASES):
        stored = encode(kind, args)
        s, up = 1000 + i, 50000 + 10 * i
        body = ps.pack(ps.body_fields(kind), stored)
        wire = ps.pack(ps.e_fields(kind), dict(stored, t=1, v=ps.SCHEMA_VERSION, s=s, up=up))
        vals = [stored[f[0]] for f in ps.body_fields(kind)[1:]]
        ins = list(args) + [0] * (8 - len(args))
        out_c.append(f'    {{"{name}", {kind}, {{{", ".join(str(a) for a in ins)}}}, '
                     f'{{{", ".join(str(v) for v in vals + [0] * (8 - len(vals)))}}}, '
                     f"{s}, {up}u, {len(body)}, {c_bytes(body)}, {len(wire)}, {c_bytes(wire)}}},")
        out_j["events"].append(dict(name=name, kind=kind, stored=stored, s=s, up=up,
                                    body=body.hex(), wire=wire.hex()))
    for name, (fields, vals) in RECORDS:
        out_j["records"].append(dict(name=name, values=vals, wire=ps.pack(fields, vals).hex()))
    out_j["commands"] = [dict(name=name, topic=topic, values=vals, payload=ps.pack(fields, vals).hex())
                         for name, topic, (fields, vals) in COMMANDS]
    s_fields, s_vals = ps.EXAMPLES["S"]
    s_bytes = ps.pack(s_fields, s_vals)
    d_bytes = ps.pack(ps.D_FIELDS, dict(s_vals, t=3, **D_DIAG))
    map_bytes = ps.pack(ps.MAP_FIELDS, MAP_EXAMPLE)

    header = [
        "/* Generated by tests/host/gen_vectors.py from docs/comms/Tribunal/packed_size.py.",
        "   Do not edit; re-run the generator after a layout change. */",
        "",
        "#ifndef TEST_VECTORS_H",
        "#define TEST_VECTORS_H",
        "",
        "#include <stdint.h>",
        "",
        "typedef struct {",
        "    const char *name;",
        "    uint8_t  kind;",
        "    int32_t  in[8];        /* comm_w_pack_* arguments, in order       */",
        "    uint32_t stored[8];    /* expected stored fields after `kind`     */",
        "    uint16_t s;",
        "    uint32_t up;",
        "    uint8_t  body_len;",
        "    uint8_t  body[8];",
        "    uint8_t  e_len;",
        "    uint8_t  e[24];",
        "} T_VECTOR;",
        "",
        "static const T_VECTOR vectors[] = {",
    ]
    footer = [
        "};",
        "",
        f"/* MQTT_SCHEMA.md section 10 S example, schema v{ps.SCHEMA_VERSION}, in S field order. */",
        "static const uint32_t s_example_values[] = {" + ", ".join(
            str(s_vals.get(f[0], 0)) for f in s_fields) + "};",
        f"static const uint8_t s_example_bytes[{len(s_bytes)}] = {c_bytes(s_bytes)};",
        "",
        "/* Record D: the S example's header (t = 3) and fields, then these diagnostics. */",
        f"static const uint32_t d_example_diag[4] = {{{D_DIAG['rssi']}, 0x{D_DIAG['ip']:08x}u, "
        f"{D_DIAG['qd']}, {D_DIAG['mqv']}}};   /* rssi, ip, qd, mqv */",
        f"static const uint8_t d_example_bytes[{len(d_bytes)}] = {c_bytes(d_bytes)};",
        "",
        "/* A map payload (schema section 8.3) and what it unpacks to. */",
        f"static const uint8_t map_example_mv = {MAP_EXAMPLE['mv']};",
        "static const uint8_t map_example_act[26] = {" + ", ".join(
            str(MAP_EXAMPLE[f"act_{chr(65 + i)}"]) for i in range(26)) + "};",
        f"static const uint8_t map_example_bytes[{len(map_bytes)}] = {c_bytes(map_bytes)};",
        "",
        "#endif /* TEST_VECTORS_H */",
        "",
    ]
    (HERE / "vectors.h").write_text("\n".join(header + out_c + footer), encoding="utf-8")
    (HERE / "vectors.json").write_text(json.dumps(out_j, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {len(CASES)} event vectors and {len(RECORDS)} record vectors")


if __name__ == "__main__":
    main()
