"""Turn the barcode commands given on the day into the car's map.

    python3 tools/barcode_map.py A=L B=R C=F D=U                 (dry run)
    python3 tools/barcode_map.py A=left B=right C=straight D=uturn --mv 2
    python3 tools/barcode_map.py LRFUFFFFFFFFFFFFFFFFFFFFFF --mv 2
    python3 tools/barcode_map.py A=L B=R --mv 2 --publish --broker 192.168.1.10

Takes letter=action pairs (or the 26-letter table itself, A first) and
prints, without sending anything:

  - the 26-letter F/L/R/U table, for the viewer's "Send map" box and for a
    compiled-in table (line_config.h BARCODE_DEFAULT_TABLE, or the build
    kit's TABLE=);
  - the 7-byte map payload (MQTT_SCHEMA.md section 8.3), packed by
    tools/telemetry_decode.py, the packer tests/host/test_decoder.py checks
    against the firmware's vectors;
  - a mosquitto_pub line that publishes it (retained, QoS 1, on
    robocar/<id>/map), and the console lines the car prints once applied.

--publish sends it with paho-mqtt (the viewer's venv has it), retained and
QoS 1, exactly as the viewer's "Send map" does.

Actions: F (also FORWARD, STRAIGHT, S), L (LEFT), R (RIGHT), U (UTURN,
U-TURN, U_TURN).  Letters not named get --rest (default F).  The map has
no "no command" value: once it is loaded every letter A-Z gives a
command, so F is the safe filler (straight across the junction).
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import telemetry_decode  # noqa: E402

LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
ACTION_WORDS = {
    "F": "F", "FORWARD": "F", "STRAIGHT": "F", "S": "F",
    "L": "L", "LEFT": "L",
    "R": "R", "RIGHT": "R",
    "U": "U", "UTURN": "U", "U-TURN": "U", "U_TURN": "U",
}
ACTION_NAMES = {"F": "FORWARD", "L": "LEFT", "R": "RIGHT", "U": "U_TURN"}


def parse_table(items, rest="F"):
    """26-letter F/L/R/U table, A first, from letter=action pairs or from
    one 26-letter table.  Raises ValueError with a message for the user."""
    rest = ACTION_WORDS.get(str(rest).upper())
    if rest is None:
        raise ValueError("--rest must be F, L, R or U")
    if len(items) == 1 and "=" not in items[0]:
        table = items[0].strip().upper()
        if len(table) != 26 or any(c not in "FLRU" for c in table):
            raise ValueError(f"a table needs 26 of F, L, R, U (A first), got {items[0]!r}")
        return table
    table = [None] * 26
    for item in items:
        letter, sep, action = item.partition("=")
        letter = letter.strip().upper()
        if not sep or len(letter) != 1 or letter not in LETTERS:
            raise ValueError(f"{item!r}: write LETTER=ACTION, e.g. A=L")
        code = ACTION_WORDS.get(action.strip().upper())
        if code is None:
            raise ValueError(f"{item!r}: the action must be F, L, R or U "
                             "(or forward/straight, left, right, uturn)")
        i = LETTERS.index(letter)
        if table[i] is not None and table[i] != code:
            raise ValueError(f"{letter} is given twice: {table[i]} and {code}")
        table[i] = code
    if all(c is None for c in table):
        raise ValueError("no letter=action pairs given")
    return "".join(rest if c is None else c for c in table)


def octal_printf(payload):
    """A printf argument that writes exactly these bytes in sh, bash or zsh."""
    return "".join(f"\\{b:03o}" for b in payload)


def report(table, mv, client_id, broker):
    """The dry-run text: what to type where, and what the car prints."""
    payload = telemetry_decode.pack_map(mv, table)
    topic = f"robocar/{client_id}/map"
    named = ", ".join(f"{LETTERS[i]} {ACTION_NAMES[c]}" for i, c in enumerate(table) if c != "F")
    lines = [
        f"table    {table}",
        f"         (A first; not F: {named or 'none'})",
        f"mv       {mv}   (the car echoes it as S.mpv; bump it for every new map)",
        f"payload  {len(payload)} bytes: {payload.hex(' ')}   "
        f"({telemetry_decode.describe_command(topic, payload)})",
        f"topic    {topic}   (QoS 1, retained)",
        "",
        "Send it one of these ways, before pressing start:",
        f"  viewer:  Send map, table {table}, mv {mv}",
        f"  CLI:     printf '{octal_printf(payload)}' | "
        f"mosquitto_pub -h {broker} -p 1883 -t {topic} -r -q 1 -s",
        f"  script:  python3 tools/barcode_map.py {table} --mv {mv} --publish --broker {broker}",
        "Fallback, a compiled-in table (needs a rebuild and a flash):",
        "  app_program/common/line_config.h: BARCODE_DEFAULT_TABLE_SET (1),",
        f"  BARCODE_DEFAULT_TABLE \"{table}\"   (or the build kit's TABLE={table})",
        "",
        "The car's console once it has the map (between barcodes):",
        f"  [comms] map mv={mv} {table} -> barcode_task",
        f"  [barcode] map v{mv} applied: {table}   (the table is printed from PR #23 on)",
        "and every S/D record after it shows mpv=%d." % mv,
    ]
    return "\n".join(lines), topic, payload


def publish(broker, port, topic, payload):
    """Publish retained at QoS 1 and wait for the broker's PUBACK."""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        raise SystemExit("--publish needs paho-mqtt 2.x (pip install paho-mqtt), "
                         "or use the mosquitto_pub line above")
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=f"robocar-map-{os.getpid()}")
    client.connect(broker, port, 30)
    client.loop_start()
    try:
        info = client.publish(topic, payload, qos=1, retain=True)
        info.wait_for_publish(timeout=10)
        if not info.is_published():
            raise SystemExit(f"no PUBACK from {broker}:{port} within 10 s")
    finally:
        client.loop_stop()
        client.disconnect()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("items", nargs="+", metavar="A=L | TABLE",
                    help="letter=action pairs, or the 26-letter F/L/R/U table, A first")
    ap.add_argument("--mv", type=int, default=1, help="map version 0-15 (default 1)")
    ap.add_argument("--rest", default="F", help="action for letters not named (default F)")
    ap.add_argument("--id", dest="client_id", default="pico-racer-01",
                    help="the car's UTK_MQTT_CLIENT_ID (config/broker_config.h)")
    ap.add_argument("--broker", default="<broker-ip>", help="broker address")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--publish", action="store_true",
                    help="send it (paho-mqtt); without this nothing is sent")
    args = ap.parse_args(argv)

    if not 0 <= args.mv <= 15:
        ap.error("--mv must be 0-15")
    try:
        table = parse_table(args.items, args.rest)
    except ValueError as e:
        ap.error(str(e))
    text, topic, payload = report(table, args.mv, args.client_id, args.broker)
    print(text)
    if args.mv == 0:
        print("note: mv 0 looks the same as no map in S.mpv; use 1-15")
    if args.publish:
        if args.broker.startswith("<"):
            ap.error("--publish needs --broker")
        publish(args.broker, args.port, topic, payload)
        print(f"published to {args.broker}:{args.port} (retained); "
              f"check that S.mpv becomes {args.mv}")
    else:
        print("(dry run: nothing sent; add --publish --broker HOST to send)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
