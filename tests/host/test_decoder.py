"""Check the laptop decoder against the shared test vectors.

    python tests/host/test_decoder.py

The vectors (vectors.json, from gen_vectors.py) are the same bytes the C test
(test_messages.c) checks the firmware packer against, so passing both means the
car and the laptop agree byte for byte.  Needs no MQTT library.
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "tools"))
import telemetry_decode as td  # noqa: E402

failures = 0
passes = 0


def ok(cond, what):
    global failures, passes
    if cond:
        passes += 1
    else:
        failures += 1
        print(f"FAIL {what}")


def rejects(payload, what):
    try:
        td.decode(payload)
    except td.Reject:
        ok(True, what)
        return
    ok(False, f"{what}: accepted")


def with_v(payload, v):
    """The same record with its 3-bit version field replaced."""
    word = int.from_bytes(payload, "little")
    word = (word & ~(0x7 << 2)) | (v << 2)
    return word.to_bytes(len(payload), "little")


def main():
    vectors = json.loads((HERE / "vectors.json").read_text(encoding="utf-8"))

    for ev in vectors["events"]:
        wire = bytes.fromhex(ev["wire"])
        name, values = td.decode(wire)
        kind_name = td.E_BODIES[ev["kind"]][0]
        ok(name == f"E {kind_name}", f"{ev['name']}: dispatched as {name}")
        want = dict(ev["stored"], t=1, v=td.SCHEMA_VERSION, s=ev["s"], up=ev["up"])
        ok(values == want, f"{ev['name']}: fields {values} != {want}")
        rejects(wire + b"\x00", f"{ev['name']}: one byte long")
        rejects(wire[:-1], f"{ev['name']}: one byte short")
        rejects(with_v(wire, 2), f"{ev['name']}: v=2")
        rejects(with_v(wire, 4), f"{ev['name']}: v=4")

    for rec in vectors["records"]:
        wire = bytes.fromhex(rec["wire"])
        name, values = td.decode(wire)
        ok(values == rec["values"], f"{rec['name']}: fields {values} != {rec['values']}")
        rejects(wire + b"\x00", f"{rec['name']}: one byte long")
        rejects(with_v(wire, 2), f"{rec['name']}: v=2")

    # Reserved kinds 6 and 7, at every length a real E can have.
    for k in (6, 7):
        word = 1 | (td.SCHEMA_VERSION << 2) | (k << td.HEADER_BITS)
        for length in (8, 9, 13):
            rejects(word.to_bytes(length, "little"), f"kind {k}, {length} B")
    rejects(b"", "empty payload")

    print(f"{passes} checks passed, {failures} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
