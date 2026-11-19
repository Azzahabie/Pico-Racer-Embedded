"""Check tools/barcode_map.py, the day-of map helper, without a broker.

    python3 tests/host/test_barcode_map.py

The payload must be what the car decodes: the same bytes as
telemetry_decode.pack_map (checked against the firmware's vectors in
test_decoder.py) and as the hand-worked example in tests/line
test_barcode_action.c (A left, B right, C straight, D U-turn, mv 1 =
91 0c 00 00 00 00 00).  Needs no MQTT library.
"""

import contextlib
import io
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = HERE.parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import barcode_map as bm  # noqa: E402
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


def raises(fn, *args):
    try:
        fn(*args)
    except ValueError:
        return True
    return False


def run(argv):
    """main() with its output and exit code, as a user would see them."""
    out, err = io.StringIO(), io.StringIO()
    code = 0
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            code = bm.main(argv)
        except SystemExit as e:
            code = e.code
    return code, out.getvalue(), err.getvalue()


def main():
    example = "LRFU" + "F" * 22

    # Pairs, words, case, and the rest filled with F (or --rest).
    ok(bm.parse_table(["A=L", "B=R", "C=F", "D=U"]) == example, "pairs -> table")
    ok(bm.parse_table(["a=left", "b=Right", "c=straight", "d=u-turn"]) == example, "words")
    ok(bm.parse_table(["D=UTURN", "A=L", "B=R"]) == example, "any order, C not named is F")
    ok(bm.parse_table(["C=forward", "A=L", "B=R", "D=U_TURN"]) == example, "FORWARD, U_TURN")
    ok(bm.parse_table([example]) == example, "a 26-letter table as is")
    ok(bm.parse_table([example.lower()]) == example, "a table in lower case")
    ok(bm.parse_table(["Z=R"], "U") == "U" * 25 + "R", "--rest U")
    ok(bm.parse_table(["A=L", "A=left"]) == "L" + "F" * 25, "the same letter twice, same action")

    # Every action for every letter lands in its own 2 bits.
    for i, letter in enumerate(bm.LETTERS):
        for code in "FLRU":
            table = bm.parse_table([f"{letter}={code}"], "F")
            payload = td.pack_map(1, table)
            word = int.from_bytes(payload, "little")
            ok(((word >> (4 + 2 * i)) & 3) == "FLRU".index(code), f"{letter}={code} in its bits")
            ok((word >> 4) & ~(3 << (2 * i)) == 0, f"{letter}={code}: no other letter set")

    # Mistakes are refused with a message, never sent.
    for bad in (["A=X"], ["1=L"], ["AB=L"], ["A"], ["A=L", "A=R"], ["LRFU"],
                ["LRFUFFFFFFFFFFFFFFFFFFFFFFF"], ["LRFXFFFFFFFFFFFFFFFFFFFFFF"], ["=L"]):
        ok(raises(bm.parse_table, bad), f"refused: {bad}")
    ok(raises(bm.parse_table, ["A=L"], "X"), "refused: --rest X")

    # The payload: the decoder's packer and the hand-worked bytes agree.
    text, topic, payload = bm.report(example, 1, "pico-racer-01", "10.0.0.2")
    ok(payload == bytes.fromhex("910c0000000000"), "writeup example, mv 1 = 91 0c 00 00 00 00 00")
    ok(payload == td.pack_map(1, example), "same as telemetry_decode.pack_map")
    ok(topic == "robocar/pico-racer-01/map", "topic")
    ok(td.describe_command(topic, payload) == f"map mv=1 {example}", "decodes back")
    ok(f"[comms] map mv=1 {example} -> barcode_task" in text, "comms console line")
    ok(f"[barcode] map v1 applied: {example}" in text, "barcode console line")
    ok(f"Send map, table {example}, mv 1" in text, "viewer instructions")
    ok(f'BARCODE_DEFAULT_TABLE "{example}"' in text and f"TABLE={example}" in text,
       "compiled-in fallback")
    _, _, p15 = bm.report(example, 15, "car-7", "h")
    ok(p15[0] & 0x0F == 15, "mv 15 in bits 0-3")

    # The mosquitto_pub line writes exactly the payload (run through sh).
    line = next(l for l in text.splitlines() if "mosquitto_pub" in l)
    ok("-t robocar/pico-racer-01/map -r -q 1 -s" in line and "-h 10.0.0.2" in line,
       "mosquitto_pub: topic, retained, QoS 1, stdin")
    fmt = line.split("printf '", 1)[1].split("'", 1)[0]
    for shell in ("sh", "bash"):
        got = subprocess.run([shell, "-c", f"printf '{fmt}'"], capture_output=True).stdout
        ok(got == payload, f"{shell} printf gives the 7 bytes")

    # main(): a dry run sends nothing; bad input exits 2 with a message.
    code, out, _ = run(["A=L", "B=R", "C=F", "D=U", "--mv", "2"])
    ok(code == 0 and "dry run: nothing sent" in out and "mv       2" in out, "dry run")
    code, out, _ = run(["A=L", "--mv", "0"])
    ok(code == 0 and "mv 0 looks the same as no map" in out, "mv 0 warning")
    code, _, err = run(["A=Q"])
    ok(code == 2 and "action must be" in err, "bad action exits 2")
    code, _, err = run(["A=L", "--mv", "16"])
    ok(code == 2 and "--mv must be 0-15" in err, "mv 16 exits 2")
    code, _, err = run(["A=L", "--publish"])
    ok(code == 2 and "--publish needs --broker" in err, "publish without a broker")

    print(f"barcode_map tool: {passes} checks passed, {failures} failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
