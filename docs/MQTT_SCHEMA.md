# MQTT_SCHEMA — the wire contract for `robocar/<id>/*`

The published interface of the WiFi + Telemetry subsystem: the topics, the JSON
records, the event catalogue, and the exact command strings the car accepts. This
is the document to hand to a teammate — or an instructor — who must write a
subscriber **without reading any C**.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Status:** contract, 2026-09-28. Derived from
  [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) §4, §7, §8, §9, §12 —
  that document is the design and holds the reasoning and the `file:line`
  evidence; this one is the flat statement of *what goes on the wire*. Items
  marked **proposed** are not yet frozen and are listed again in §12.
- **Scope:** the message contract only. It does not cover build flags, the
  broker recipe, or the RTOS internals — see the companions.
- **Companions:** [`WIFI_SUBSYSTEM_DESIGN.md`](WIFI_SUBSYSTEM_DESIGN.md) for the
  design and evidence; [`WIFI_SUBSYSTEM_SUMMARY.md`](WIFI_SUBSYSTEM_SUMMARY.md)
  for the same design in software-engineering terms;
  [`NETWORKING.md`](NETWORKING.md) §4 for the broker environment and §6 for the
  M6 protocol evidence; [`APPLICATION.md`](APPLICATION.md) §9 for why this
  document blocks the other four buddies.

---

## 1. What this document is, and what it is not

It is the **contract**: if you can read this and a JSON library, you can consume
the car's telemetry and drive its command channel. It is written so that the
reader is a *client author*, not the firmware author — which is exactly why the
outbound format is JSON and not a bespoke line format (design §8.1: on demo day
the reader is the instructor's client, not a script we wrote).

It is **not** the design. Where this document says "the car publishes a state
line every 1 s", the *why* (the hybrid periodic + edge publish model) and the
`file:line` evidence live in the design document. If the two ever disagree, the
design document wins and this one is the bug.

> **What it means:** think of this as the API reference for a service. The
> service is the car; the endpoint is an MQTT topic; the request/response bodies
> are the JSON objects below. Nothing here requires a custom parser on the
> client side.

---

## 2. Topics

`<id>` is the car's client identifier — the value of `UTK_MQTT_CLIENT_ID` in
`config/broker_config.h` (`config/broker_config.example.h:46` ships
`"pico-racer-01"`). Using it in the topic means two cars on one broker never
collide, and a subscriber can watch one car with `robocar/pico-racer-01/#` or all
cars with `robocar/+/telemetry`.

| Topic | Direction | QoS | Retain | Carries |
|---|---|---|---|---|
| `robocar/<id>/telemetry` | car → broker | 0 | no | record **S** (periodic) and record **E** (edge) — §4, §5 |
| `robocar/<id>/heartbeat` | car → broker | 0 | no | liveness, so silence is a signal — §6 |
| `robocar/<id>/state` | car → broker | 0 | **yes** | the on-request snapshot reply to `snap` — §7 |
| `robocar/<id>/cmd` | broker → car | 1 (sub) | no | the commands in §8 |
| `robocar/<id>/map` | broker → car | 1 (sub) | **yes** | the barcode → action table — §8.3 |

QoS and retain are per design §9.2 and decision 11. The short version of two
non-obvious choices:

- **Telemetry is QoS 0.** It is a stream; the next line is 1 s away, so an ack
  buys nothing. (lwIP never retransmits a QoS 1 publish anyway — the request
  expires with `ERR_TIMEOUT`, design §16 decision 11.)
- **`map` is retained.** A subscriber that arrives late — including the car after
  a reboot — is handed the current table immediately, with nothing re-published
  by hand. This, not broker session state, is what makes the remap survive a
  restart (lwIP forces a clean session on every connect; design §9.2).

> **Proposed, not yet in place:** the topic prefix (`robocar`) and the two
> endpoint names are **not configurable today** — they would have to become
> values in `config/broker_config.h` alongside the broker address (design §12,
> "topic prefix and map topic"). Until that change lands, treat the strings above
> as fixed. A reader of this document is not affected; a deployer is.

---

## 3. Rules that apply to every record

1. **One JSON object per MQTT message.** No arrays, no batching, no trailing
   newline required.
2. **Integers only.** The RP2040 has no FPU (design §8 constraint 1), so every
   numeric value is an integer with its unit fixed by this document. There is no
   field whose value is a fraction. `ip` (§7) is the one string-valued field.
3. **Fixed key order, but do not depend on it.** The car emits keys in the order
   shown; a reader must look keys up by name, never by position. A new key may be
   appended at any time (§11).
4. **`t` names the record type** and is always present. This document defines
   `"S"`, `"E"`, `"H"` and `"D"`.
5. **`s` is the sequence number** — monotonic over the whole run, shared by all
   record types on `telemetry` so that state and events interleave into one
   causal order. A gap in `s` means lost messages; `drp` (below) says whether the
   car knows about it.
6. **`v` is the schema version.** Bump-on-breaking-change only (§11).

---

## 4. Record `S` — the periodic state line

Published on `robocar/<id>/telemetry` once per second (design §7; the cadence is
proposed at 1 s). It is a **fold of the latest value per source**, not a delta —
so a late subscriber waits at most one second for a complete picture, and the
line doubles as the liveness signal.

```json
{"t":"S","v":1,"s":137,"up":41230,"mot":2,"dst":20,"lin":1,"dev":3,"bar":69,"act":4,"hum":3,"hgt":12,"obs":0,"byp":0,"mis":1,"mpv":3,"drp":0}
```

| Key | Type | Unit / range | Meaning |
|---|---|---|---|
| `t` | string | `"S"` | record type |
| `v` | int | — | schema version (currently `1`) |
| `s` | int | — | sequence number for the run |
| `up` | int | ms | milliseconds since boot (`tk_get_otm()`, design §4.1) |
| `mot` | int | 0–3 | motion state: 0 stop, 1 straight, 2 left, 3 right (Buddy 2) |
| `dst` | int | cm | distance travelled since the last motion change (Buddy 2) |
| `lin` | int | 0–2 | line state: 0 on-line, 1 lost, 2 crossing (Buddy 3) |
| `dev` | int | tenths | line deviation from centre, in tenths of a sensor unit (Buddy 3) |
| `bar` | int | ASCII | **last decoded barcode letter as its ASCII code** — 69 is `'E'` (Buddy 3) |
| `act` | int | — | action code the barcode mapped to (Buddy 3); `0` until the first read |
| `hum` | int | count | humps detected (Buddy 4) |
| `hgt` | int | mm | height of the last hump (Buddy 4); `0` until the first |
| `obs` | int | cm | width of the last obstacle seen (Buddy 5); `0` = none |
| `byp` | int | 1–2 | bypass taken: 1 left, 2 right (Buddy 5); `0` = none |
| `mis` | int | — | mission state (Vehicle Controller) |
| `mpv` | int | — | the barcode-table version the car is **actually running** (§9.3) |
| `drp` | int | count | events dropped at the intake queue in the last window — see below |

> **The `0` sentinel.** For `bar`, `act`, `hgt`, `obs` and `byp`, the value `0`
> means *"this source has not reported yet"*, not *"zero"*. A reader should show
> such a field as blank rather than as `0`. (The example above is consistent:
> `obs` and `byp` are `0` because no obstacle has been seen.)

> **Why `drp` is on the wire.** Failures in this stack are silent — no exception,
> no log (design §5, §14). `drp` is a cumulative "messages the car dropped"
> counter, so a hole in `s` is attributable: a gap with `drp` unchanged means the
> *network* lost it, a gap with `drp` rising means the *car* did. Treat every
> counter here as instrumentation added because you cannot attach a debugger to a
> robot mid-demo.

---

## 5. Record `E` — the edge event

Published on the **same** topic, the instant something happens (design §7). It is
the audit trail; unlike `S`, a lost `E` is a real gap in the run map.

```json
{"t":"E","v":1,"s":138,"up":41310,"k":"B","a":69,"b":4}
```

| Key | Type | Meaning |
|---|---|---|
| `t` | string | `"E"` |
| `v` | int | schema version |
| `s` | int | sequence number, shared with `S` |
| `up` | int | milliseconds since boot at the moment of the event |
| `k` | string | **event kind letter** — one of the letters in §5.1 |
| `a`, `b` | int | the two payload values; their meaning depends on `k` (§5.1) |

There are exactly **two** value fields. That is deliberate: it keeps the record
fixed-size and keeps one `tm_sprintf` per record (design §8 constraint 2). If a
source genuinely needs three values, it emits **two events** rather than this
record growing.

### 5.1 The event catalogue — the contract with the other four buddies

This is the one table to agree on before anyone writes code. It is design §4.4
with the kind letters added. Two letters are pinned by the design's own examples
— `B` and `I`, in the record example (§8.2) and the M7 gate (§14). `M`, `L` and
`O` follow the same first-letter convention, and `N` is chosen for mission to
avoid colliding with the `M` already used by motion — so **those four letters are
proposed** (§12 item 1).

| `k` | Event | Producer | `a` | `b` |
|---|---|---|---|---|
| `M` | motion | Buddy 2 | state: 0 stop / 1 straight / 2 left / 3 right | distance since last event (cm) |
| `L` | line | Buddy 3 | state: 0 on-line / 1 lost / 2 crossing | deviation (tenths of a sensor unit) |
| `B` | barcode | Buddy 3 | decoded letter (**ASCII code**, e.g. 69 = `'E'`) | action code |
| `I` | imu | Buddy 4 | hump count | hump height (mm) |
| `O` | obstacle | Buddy 5 | obstacle width (cm) | bypass: 1 left / 2 right |
| `N` | mission | Vehicle Controller | mission state | `0` |

> **Two different uses of letters — do not conflate them.** `k` is a *kind
> discriminator*: a fixed vocabulary of six letters, one per source. The barcode's
> *decoded character* is a different thing, carried as its **ASCII code in `a`**
> (and as `bar` in record `S`). So `"k":"B"` means "this is a barcode event",
> while `"a":69` means "the barcode read was the letter `E`". A subscriber that
> compares `k` to `a` will be confused; they are unrelated alphabets.

---

## 6. Record `H` — heartbeat

Published on `robocar/<id>/heartbeat` (its own topic by design §16 decision 6, so
that "the car is alive" is a subscription rather than a predicate over the
telemetry stream). Sent on the same cadence as `S`; also sent once in reply to
the `ping` command (§9.2).

```json
{"t":"H","v":1,"s":7,"up":41230}
```

| Key | Type | Meaning |
|---|---|---|
| `t` | string | `"H"` |
| `v` | int | schema version |
| `s` | int | sequence number |
| `up` | int | milliseconds since boot |

> **Proposed:** the heartbeat record shape. The design fixes the topic and the
> requirement, not the body; this four-field envelope is the minimum that lets a
> subscriber distinguish "alive" from "stale" using `up`.

---

## 7. Record `D` — the on-request snapshot on `robocar/<id>/state`

Published only in reply to `snap` (§9.1), and **retained**, so a subscriber that
connects after the reply still sees the most recent one. It is record `S` plus
diagnostics: this is the one place where *"why did it fail?"* can be answered
from the laptop without a debugger (design §9.3).

```json
{"t":"D","v":1,"s":140,"up":41500,"mot":2,"dst":20,"lin":1,"dev":3,"bar":69,"act":4,"hum":3,"hgt":12,"obs":0,"byp":0,"mis":1,"mpv":3,"drp":0,"rssi":-58,"ip":"192.168.50.7","qd":0,"mqv":"connected"}
```

It carries **every key of record `S`** (same names, same units, same `0`
sentinel), plus:

| Key | Type | Meaning |
|---|---|---|
| `rssi` | int | Wi-Fi signal strength, dBm (negative; closer to 0 is stronger) |
| `ip` | string | the leased IPv4 address, dotted-quad — the one string field on the wire |
| `qd` | int | publish-queue depth at the moment of the reply (0 = caught up) |
| `mqv` | string | MQTT client verdict: `"connected"`, `"connecting"`, `"disconnected"` |

> **Proposed:** the diagnostics key set (`rssi`, `ip`, `qd`, `mqv`). The design
> requires the snapshot to carry "uptime, RSSI, IP, queue depth, drop counters,
> MQTT verdict" (design §9.3) but does not name the keys. These four names plus
> the reused `up`/`drp` cover that list; agree them before the M8 test.

---

## 8. Inbound — the commands the car accepts

**Format: `key=value`, not JSON.** This is deliberate and is the one place the
two directions differ (design §8.1, §16 decision 17): we author the publisher, so
the reader is ours, and a `strtok` parse on the device is far cheaper to defend
line by line than a JSON scanner. A subscriber only ever *reads* JSON; a
publisher *writes* `key=value`.

Publish to `robocar/<id>/cmd` (QoS 1, no retain) unless the table says otherwise.
Anything unrecognised is counted and printed on the car's console, and changes
nothing.

### 8.1 `snap`

```
snap
```

Replies with one record `D` on `robocar/<id>/state` within one telemetry tick.

### 8.2 `ping`

```
ping
```

Replies with one record `H` on `robocar/<id>/heartbeat`, so an operator can check
liveness without waiting for the 1 s cadence.

### 8.3 `map` — the live barcode remap (publish to `robocar/<id>/map`, retained)

```
map mv=3 t=01234567890123456789012345
```

| Field | Meaning |
|---|---|
| `mv` | the table **version** — a counter the publisher increments. Deliberately *not* named `v`, which is the schema version in the JSON records |
| `t` | exactly **26 single-digit action codes**, index 0 = letter `A` … index 25 = letter `Z` |

In the example, `A`→0, `B`→1, … and `L`→1 again. The car applies the table,
starts using it immediately, and from then on reports `mpv=<mv>` in every record
`S`/`D`. **The publisher verifies convergence by comparing the `mv` it sent with
the `mpv` the car echoes** — do not assume delivery, observe it (design §9.2).

Two properties worth stating because a client may rely on them:

- **Idempotent.** Applying the same table twice changes nothing, so a retained
  replay or a duplicate is harmless.
- **Bounded.** The payload is 36 bytes; the car's single-fragment inbound limit
  is ~95 bytes (design §16 decision 16), so a longer payload is *rejected as
  malformed*, never silently truncated. More than 26 codes, or a non-digit, is a
  rejected command.

### 8.4 A malformed command

Produces exactly one visible line on the car's console and no state change. This
is a graded requirement (design §14, M8) — the failure is *visible*, not silent.

---

## 9. Reading it with a generic client — the demo-day path

No custom parser is required, which is the point of the JSON choice
(design §8.1). Three ways, cheapest first:

```sh
# Watch every record from one car, raw:
mosquitto_sub -h <broker-ip> -t 'robocar/pico-racer-01/telemetry' -v

# One state line's motion state, with jq:
mosquitto_sub -h <broker-ip> -t 'robocar/pico-racer-01/telemetry' \
  | jq -c 'select(.t=="S") | {s, up, mot, dst}'

# Send a command:
mosquitto_pub -h <broker-ip> -t 'robocar/pico-racer-01/cmd' -m 'snap'
```

GUI equivalent: point **MQTT Explorer** at the broker and subscribe to
`robocar/#`; JSON payloads render as an expandable tree with no configuration.
That is the test a reader who has never seen this repo should be able to run
(design §14, M10 gate).

---

## 10. A worked run — what a subscriber actually sees

In order, from one car. (`…` elides the state fields that did not change; `s`
increases by 1 across *both* record types, because they share the topic.)

```
robocar/pico-racer-01/telemetry  {"t":"S","v":1,"s":136,"up":40310,"mot":1,"dst":0,...,"mpv":3,"drp":0}
robocar/pico-racer-01/telemetry  {"t":"S","v":1,"s":137,"up":41230,"mot":2,"dst":20,...}
robocar/pico-racer-01/telemetry  {"t":"E","v":1,"s":138,"up":41310,"k":"B","a":69,"b":4}
robocar/pico-racer-01/telemetry  {"t":"S","v":1,"s":139,"up":42230,"mot":2,"dst":20,"bar":69,"act":4,...}
```

Read it as: the car was going straight (`mot=1`), then turned left (`mot=2`,
`dst=20` cm travelled since the change), then read barcode `E` (ASCII 69) mapping
to action `4`, and the next state line reflects the barcode (`bar`, `act` now
nonzero) — with `s` contiguous, so nothing was lost.

---

## 11. Versioning and compatibility

- **Additive changes are free.** A new key appended to a record does not bump
  `v`; a reader must ignore keys it does not know. This is why JSON was chosen
  over a positional format (design §8.5): position is meaning there, so a
  version skew and corruption look identical.
- **Breaking changes bump `v`.** A renamed key, a changed unit, or a removed
  key sets `v` to `2`. A reader that checks `v` fails loudly instead of
  misreading.
- **`s` resets to 0 on reboot.** It is a run counter, not a wall clock; use `up`
  for time-within-a-run and `s` for gaps-within-a-run.
- **`up` wraps** after a little under 50 days (`tk_get_otm()` low word). Far
  beyond a demo run; noted so nobody treats it as absolute.

---

## 12. Open items — not frozen yet

Everything here is marked **proposed** in place above; collected so it can be
closed deliberately rather than by accident.

| # | Item | Where | Current proposal |
|---|---|---|---|
| 1 | The `N` letter for `EVT_MISSION` | §5.1 | `N` (avoiding the `M` already used by motion) |
| 2 | The heartbeat record shape | §6 | the four-field `H` envelope |
| 3 | The snapshot diagnostics key names | §7 | `rssi`, `ip`, `qd`, `mqv` |
| 4 | The publish cadence | §4 | 1 s |
| 5 | Topic prefix / endpoint names as config | §2 | add to `config/broker_config.h` (design §12) |
| 6 | Inbound stays `key=value` vs. flat JSON | §8 | `key=value` (design §16 decision 17; publisher unconfirmed) |

---

## 13. Don't redo

- **Don't write a new event catalogue.** §5.1 is design §4.4; change it in one
  place, not two.
- **Don't add a third value field to record `E`.** Emit a second event (§5).
- **Don't parse the outbound JSON on the device.** The car only *produces* it.
  The device's parser is for the `key=value` inbound commands (§8).
- **Don't rely on key order.** Look keys up by name (§3).
- **Don't treat `k` and the barcode's ASCII letter as the same alphabet** (§5.1).
- **Don't expect QoS 1 to survive a disconnect.** lwIP forces a clean session;
  retained `map` plus the `mpv` echo is what actually survives (design §9.2).
- **Don't hardcode the broker or the topic prefix** — they are the instructor's
  on demo day (design §12).

---

## 14. Glossary

| Term | Plain meaning |
|---|---|
| **Topic** | The channel name a message is published to, e.g. `robocar/<id>/telemetry`. |
| **Payload** | The message body — in our case the JSON object. |
| **QoS 0 / QoS 1** | MQTT delivery guarantees: "at most once, fire and forget" / "at least once, acknowledged". |
| **Retain** | A broker flag: it keeps the last message on a topic and hands it to any later subscriber. |
| **Edge event** | A record published the moment something happens, rather than at the next tick. |
| **Snapshot** | The current state at one instant, as opposed to the history of events. |
| **Sequence number (`s`)** | A per-run counter that makes gaps observable. |
| **Sentinel** | A reserved value meaning "no value yet" — here, `0` in the state fields. |
| **Idempotent** | Doing it twice has the same effect as doing it once — why a replayed `map` is safe. |
| **ASCII code** | The integer for a character: `'A'`=65, `'E'`=69, `'Z'`=90. |
| **Clean session** | An MQTT flag meaning "forget my previous session"; lwIP always sets it. |
| **FPU** | Floating-point unit. The RP2040 has none, so every value on the wire is an integer. |