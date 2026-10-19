# Tribunal 1 — what it means for Buddy 1

The one page for Tribunal 1 (Change Requirement 1, minimum-bit data
representation) from the WiFi + Telemetry side.

- **Owner:** Buddy 1 (WiFi + Telemetry)
- **Date:** 2026-10-19
- **Baseline:** [`completed_in_trib.md`](completed_in_trib.md) is the team's
  **submitted** Version B. It is frozen and is what the finished project gets
  checked against. Do not edit it.
- **Also in this folder:** `Change Requirement 1.docx` (the source text) and
  [`packed_size.py`](packed_size.py) (regenerates the Test 1 size table).
- **Replaces:** `readme.md`, `in_tribunal_content.md`, `tribunal_matters.md`,
  `pre_tribunal.local.md` and `in_tribunal.local.md`. Everything from them that
  still matters is below.
- **Numbering rule:** every figure is tagged **measured** or **estimated**.
- **Docs updated to match (2026-10-19):** `MQTT_SCHEMA.md` (rewritten as binary
  schema v2), `WIFI_SUBSYSTEM_DESIGN.md`, `APPLICATION.md`, `RTOS_INTERFACES.md`,
  `GLOSSARY.md`, `STATECHART1_DIAGRAM_SPEC.md`, `README.md`,
  `resolve_feedback/Actions.md` and `study_guide.html`. Each change is labelled
  "Updated from Tribunal 1".

---

## 1. The new requirement, in plain words

Four rules, word-for-word source in the `.docx`:

1. **Minimum bits.** Every application data field uses only the bits its valid
   range needs (1 to 5 → 3 bits; up to 8 states → 3 bits), using bit-fields or
   masks and shifts. It applies everywhere: motion, line/barcode, obstacle, hump,
   states, shared data, **inter-task queue messages and MQTT payloads**.
2. **One shared fault bitmask.** Motor stall, encoder fault, IMU fault, no echo
   and **Wi-Fi loss** become individual bits in one shared word, not separate
   variables.
3. **MQTT is compact binary**, not values sent as text. JSON is out.
4. **No races on packed data.** If a packed word is shared between tasks, or an
   ISR and a task, read-modify-write must be atomic or synchronised.

**Week 13 evidence it demands:** a data-representation table (field, range,
bits, how), a RAM comparison before and after, a message-size comparison before
and after, and proof that nothing broke and timing did not degrade.

---

## 2. How it affects my subsystem

Of the five buddies I am hit hardest, because rule 3 names MQTT directly and I
own both the wire format and `messages.h`.

| Area | What changes | What does not change |
|---|---|---|
| **Wire format** | JSON → packed binary records. The status line `S` goes from **141 B (measured)** to **15 B (estimated, Test 1)**, about 9× smaller. Reverses our 2026-09-28 JSON decision (design decision 9). A laptop decoder script becomes mandatory, and the "generic MQTT client can read it" demo gate (M10) has to be replaced. | Topic set, the S/E record split, one S line per second (NFR-10). |
| **Fault bitmask** | I must **set and clear the Wi-Fi-loss bit** (core 0) and **publish the whole mask** in the `S` record. The other bits are set on core 1, so the word crosses cores. | The team agreed the mask is a µT-Kernel **event flag owned by Buddy 5** (`nav_ctrl_task`), not by me. `tk_set_flg` / `tk_clr_flg` are atomic, so I need no lock. |
| **Telemetry intake (`tele_mbf`)** | Producers send packed records. Buddy 2 already proposed one: ~54 bits → 7 B, which the kernel stores as 8 B, with spare bits for a sequence number, a clamp-plus-overflow bit, sent at 1 Hz and on every state change. | Senders use `TMO_POL`; a full buffer drops and counts in `drp`. |
| **Inbound commands** | The `map` payload packs too. Buddy 3 decided the barcode action is **2 bits** (not the 4 I assumed), so 26 actions are 52 bits ≈ **7 B (estimated)**, not 13 B. | Allow-list of commands (FR-24); remote commands never drive motors. |
| **RAM** | Only the buffers and the wire shrink. The kernel stores a message buffer entry as a 4-byte header plus the payload rounded up to 4 bytes, so small records barely save RAM. | Stacks (`comms_task` 4 KB, radio 6 KB), the ~2.3 KB status snapshot. |
| **Timing / tasks** | Packing is a few shifts and masks, done in the producers. Device CPU is equal or better because the `tm_sprintf` per record goes away. | `comms_task` priority 16 on core 0; `cyw43_utk_task` priority 4 on core 0 is still the only lwIP caller. |
| **Link** | The team asked whether binary improves Wi-Fi. At 1 Hz × 141 B ≈ 1.1 kbit/s there is almost no airtime to save. It stays an untested assumption, and nobody may rely on it. | NFR-09 (15 s recovery) is not expected to move. |

### The traps (unchanged from my prep, still true)

- **Never use C bit-fields for anything that leaves the chip.** Their bit order
  and padding are compiler-defined. Use masks and shifts with a fixed byte order
  (little-endian), in plain C functions that can be unit-tested on a PC.
  Bit-fields are fine for task-local state.
- **Turning interrupts off protects only one core**, and the M0+ has no
  LDREX/STREX, so C11 atomics are not lock-free here. Cross-core bits go through
  the event flag.
- **The spinlock fallback** the team wrote in must not use SIO spinlocks 0–2;
  they are kernel-reserved (`docs/PORT_RP2040.md` safety rule 1).
- **Two fields in one word is still one shared word.** One writer per packed
  word, or the race comes back.
- **Change the wire schema and `messages.h` in the same pull request** (NFR-17).

---

## 3. Did the submitted report reflect it?

Compared against my prepared draft. ✅ = in the submission, ⚠️ = partly or with an
error, ❌ = missing.

| Item | In the submission? | Where / what is missing |
|---|---|---|
| JSON → packed binary + decoder | ✅ | Change log row 1, Major redesign. **Evidence cell is blank**; 141 → 15 B is not stated. |
| Fault bitmask in a kernel object | ✅ | My contribution row and change-log row 3. **Owner cell blank** on that row; elsewhere the team set the owner to Buddy 5. |
| lwIP: radio task is the only caller | ✅ | Change log, No change. |
| Binary improves Wi-Fi? | ✅ | Resource review: kept as an untested assumption. |
| My challenge to Buddy 2 | ✅ | Answered in Buddy 2's row: 54 bits / 8 B record, 1 Hz plus state changes, `TMO_POL`, clamp and overflow bit. That answer is now an input to my design. |
| Inbound map/snap test with Buddy 3 | ⚠️ | Test plan row exists; status left blank. |
| Challenge received from Buddy 5 | ⚠️ | Recorded, but says NFR-11 where the 20 ms brake is NFR-18. The measured 85 µs worst-case kernel lock wait (about 0.4% of 20 ms) is not quoted. |
| **Curveball row for Buddy 1** | ❌ | **Empty.** Every other buddy has one. |
| **Test 2, the two-core race test (measured)** | ❌ | Not mentioned anywhere, though it is the only hardware evidence about packed-data races the team has. |
| No-change: one `comms_task`, 1 s S line, topics | ❌ | Not in the log (the team still reached 3 changes + 3 no-changes overall). |
| Barcode letter as 5-bit index, map compaction | ❌ | Not recorded. And Buddy 3's 2-bit action now contradicts my 4-bit `act` field. |
| My own interface risk (`messages.h` change breaks builds) | ⚠️ | Not in the risk register, but readiness step 1 ("freeze one shared `messages.h`") covers the work. |

**Bottom line:** the two big decisions that land on me — binary payloads and the
fault bitmask — are in the submission. What is missing is evidence and detail,
not direction. Because the report is frozen, close the gaps in the Week 13
write-up: quote Test 1 and Test 2, state the 85 µs figure, and record the
`act` width and the fault-flag ownership as follow-ups that came out of the
Zoom.

### Commitments the submission now holds me to

1. A **packed binary MQTT record plus a laptop decoder** (change log).
2. The **lwIP single-caller rule** stays (change log).
3. A **shared `messages.h`** with every packed message, the fault-bitmask bit
   layout and the binary MQTT layout, frozen before Week 10 (readiness step 1;
   I own `messages.h`).
4. An **unpacked baseline first**, with its RAM and message sizes recorded, for
   the before-and-after comparison (readiness step 2).
5. **Wi-Fi join re-measured with the timer fix** (A1, readiness step 4).
6. **Week 10 target: binary telemetry reaching the laptop.**
7. Wi-Fi load supplied for the IMU-stop and brake timing tests; joint owner of
   the map/snap test with Buddy 3; participant in the fault-bitmask stress test.

---

## 4. Roadmap from here

This merges the existing F1 Wi-Fi reliability plan with the tribunal
commitments. Order matters: each step unblocks the next.

| # | Step | Why now | Done when |
|---|---|---|---|
| 1 | **Finish F1.** Decide ACD in [`../resolve_feedback/ACD_DECISION.md`](../resolve_feedback/ACD_DECISION.md), then F1.5 (T1, T3, T4, T5) in [`Actions.md`](../resolve_feedback/Actions.md). Re-measure 20 boots for A1. | Already in flight; A1 is a submitted commitment. | Results recorded in `results.md`. |
| 2 | **Agree the field ranges** with each buddy and update `packed_size.py` (already updated 2026-10-19 with `act` at 2 bits, a common 53-bit header, H/D/cmd/map): Buddy 2's 54-bit motion record, the fault-bit positions (with Buddy 5, who owns the flag), and whether `up` stays 32 bits. | Every later step uses these widths. | `packed_size.py` prints the agreed table. This is the first draft of the Week 13 data-representation table. |
| 3 | **Write `messages.h`**: the `tele_mbf` record(s), fault-bit names, and pack/unpack functions using masks and shifts, plus PC unit tests for round trips and out-of-range clamping. | Readiness step 1. Everyone builds against it. | Compiles in Docker; the PC tests pass; one PR with the schema (NFR-17). |
| 4 | **Baseline build: M7 with the JSON encoder.** Bring up `comms_task`, the `tele_mbf` intake and the S line in JSON first, then record `arm-none-eabi-size`, the linker map and the message sizes. Tag the commit. | Week 13 needs a real "before". JSON is the Week 6 design, so this is the honest baseline. | Tagged commit; sizes recorded in `results.md`. |
| 5 | **Switch to binary.** Swap the encoder to the packed S/E records, add `s` (sequence) and `drp`, and write the laptop decoder (Python, reads the same layout). Update `MQTT_SCHEMA.md` from JSON to binary in the same PR. | The submitted change-log decision. | The decoder prints 10 consecutive S lines, `s` increments by 1, `drp` is 0 (the M7 gates). |
| 6 | **Wi-Fi-loss fault bit.** `comms_task` sets and clears it on the event flag from the connection state changes that F1.4 already logs, and copies the whole mask into each S record. Agree with Buddy 5 who may clear each bit. | Requirement rule 2 names Wi-Fi loss. | Turn the AP off: the bit appears in S within 1 s of the drop and clears on reconnect. |
| 7 | **Week 10 demo:** binary telemetry on the laptop while the car drives. Give Wi-Fi load to the brake and IMU-stop timing tests. | The submitted Week 10 target. | Decoder log captured alongside the team demo. |
| 8 | **M8 inbound, binary.** Packed `map` (2-bit actions) and `snap`; run the map/snap test with Buddy 3. | Submitted test-plan row. | Remap applies with no recompile; a bad table keeps the old one. |
| 9 | **Week 13 evidence.** Representation table (step 2), RAM before/after (step 4 vs 5), message size before/after (141 B measured vs packed measured), correctness (M7 gates) and timing (NFR-10 cadence, NFR-09 recovery) on the binary build. Offer `app_program/race_test/` as the harness for the team's fault-bitmask stress test. | The requirement's four deliverables. | All four in the repo. |

**Watch item:** the team plans to switch the kernel tick to 1 ms before Week 10.
After that change, re-run the M6 MQTT publish and the F1 rejoin test, because the
radio's 2 ms poll and the retry timers sit on the tick.

---

## 5. Evidence already in hand

### Test 1 — packed record size (calculated, not measured)

From [`packed_size.py`](packed_size.py), updated 2026-10-19 to match
[`../MQTT_SCHEMA.md`](../MQTT_SCHEMA.md) schema v2. Ranges are proposed and may
still move in roadmap step 2.

| Field | Meaning | Range | Bits |
|---|---|---|---|
| `t` | record type | S, E, H, D | 2 |
| `v` | schema version | 0–7 (2 = binary) | 3 |
| `s` | sequence number | 0–65535 | 16 |
| `up` | ms since boot | 0–2³²−1 | 32 |
| `faults` | shared fault bitmask | 5 faults | 5 |
| `mot` | motion state | 0–3 | 2 |
| `dst` | distance since last change | 0–255 cm | 8 |
| `lin` | line state | 0–2 | 2 |
| `dev` | line deviation | −128–127 tenths, offset-encoded | 8 |
| `bar` | barcode letter index | 0–25 = A–Z, 31 none | 5 |
| `act` | action code | 0–3 (Buddy 3's 2-bit action) | 2 |
| `hum` | humps detected | 0–15 | 4 |
| `hgt` | last hump height | 0–255 mm | 8 |
| `obs` | last obstacle width | 0–255 cm | 8 |
| `byp` | bypass taken | 0–2 | 2 |
| `mis` | mission state | 7 states | 3 |
| `mpv` | barcode table version | 0–15 | 4 |
| `drp` | dropped count, saturates | 0–15 | 4 |

**S = 118 bits → 15 B** (E = 72 bits → 9 B, H = 7 B, D = 21 B, `cmd` = 1 B, `map` = 7 B). Against the measured 141 B
JSON line that is 9.4× smaller. `up` and `s` are 41% of the record; making `up`
"ms since run start" would cut it to 19 bits (S → 14 B), but that changes behaviour, so it
is a decision rather than a free win.

### Test 2 — two-core packed-word race (measured on hardware, two boots)

`app_program/race_test/race_test.c`, enabled by `TM_RACE_TEST` in `race_test.h`,
built with `make SMP=1 CONSOLE=usb_cdc -j8` (text 64466 / data 364 / bss 19916).
One task per core bumps its own 16-bit half of one 32-bit word, 60000 times each.

```text
boot 1  phase 1 no lock : high=42217 low=17783  loss=60000
        phase 2 mutex   : high=60000 low=60000  loss=0
boot 2  phase 1 no lock : high=60000 low=0      loss=60000
        phase 2 mutex   : high=60000 low=60000  loss=0
```

With no lock, **50% of updates were lost** (60000 of 120000) on both boots, and
which half survived was random. With the kernel mutex, none were lost. This is
the requirement's rule 4 shown on the board, and why the fault mask lives in a
kernel object. `volatile` does not prevent it (PORT_RP2040 safety rule 3).

> `race_test/` and the 9-line guarded hook in `app_main.c` are not committed yet.
> The test is off while `TM_RACE_TEST` is 0, so the demo is unchanged.

### Figures to quote

| Figure | Value | Status |
|---|---|---|
| JSON S line | 141 B | measured |
| `key=value` S line | 101 B | measured |
| Packed S / E record | 15 B / 9 B | estimated (Test 1) |
| Packed `map` | ~7 B (26 × 2-bit) | estimated |
| Race test, no lock / mutex | 50% lost / 0 lost | measured (Test 2) |
| Kernel lock hold / wait, max | 85 µs / 36 µs | measured (PORT_RP2040) |
| Radio wake and dispatch | 16 µs avg / 43 µs max | measured (PORT_RP2040) |
| Image, TCP profile | text 340505 / data 4640 / bss 52188 | measured |
| Wi-Fi join success | ~70% | measured before fix 993ebc7, out of date |

---

## 6. Do not redo

- No packed fault word in a plain global, and no `volatile` as cross-core sync.
- No mutex taken from an ISR, and no lock added to the 20 ms stop path or the
  1 ms sampling path.
- No C bit-fields in anything sent over MQTT.
- Don't let the status snapshot grow to carry telemetry.
- Don't change `MQTT_SCHEMA.md` without changing `messages.h` in the same PR.
