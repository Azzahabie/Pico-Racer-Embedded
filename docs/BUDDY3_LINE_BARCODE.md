# Buddy 3: line tracking, junctions, barcode capture and decoding

## Wiring

| Sensor | Grove | Pin | Signal |
|---|---|---|---|
| Left deviation | 5 | GP26 / ADC0 | A0 analogue (GP6 not used) |
| Right deviation | 6 | GP27 / ADC1 | A0 analogue |
| Barcode | 7 | GP7 | DO digital (GP28 / ADC2 not used; reserved for battery sense) |

Pins, thresholds, polarity and every tunable below are in
`app_program/common/line_config.h`. Values marked PROVISIONAL there are
unmeasured. Both tasks are built in with `TM_LINE_TASK` 1
(`app_program/line_task/line_task.h`).

## Tasks and interfaces

| Task | Core / priority | Does |
|---|---|---|
| `ir_sense_task` (`line_task.c`) | core 1 / 10 | 1 ms cyclic: reads GP26, GP27, GP7; publishes `line_state`; junction events; barcode widths; calibration |
| `barcode_task` (`barcode_task.c`) | core 0 / 14 [PROPOSED] | decodes reads, keeps the table, applies map updates, holds the barcode command, prints, telemetry |

| From -> to | Object | Content |
|---|---|---|
| ir_sense -> motion | `line_state_word()` | one aligned 32-bit word every 1 ms (`line_state.h`) |
| ir_sense -> nav | `NAV_FLG_JUNCTION` | exactly one per junction |
| nav -> ir_sense | `ir_flg`, `IR_FLG_CAL_REQ` | calibration request; answer `NAV_FLG_CAL_OK_IR` / `NAV_FLG_CAL_FAIL` |
| ir_sense -> barcode | `bar_mbf`, 4 bytes | `{WIDTH|END|ABORT, read_id, value16}`, sent with `TMO_POL` |
| comms -> barcode | `barcode_map_submit()` -> `map_mbf` | the 7-byte map of MQTT_SCHEMA.md 8.3 |
| barcode -> nav | `barcode_cmd_word()` | pending `{letter, action, seq}` (`barcode_action.h`) |
| nav -> motion | `nav_cmd_mbf` | `{NAV_CMD_JUNCTION, action}` (`nav_ctrl.h`) |
| ir_sense, barcode -> comms | `comms_tele_send()` | E line on state edges; E barcode on every read (letter 31 = rejected) |
| ir_sense -> comms (`S.faults`) | `fault_set()` / `fault_clear()` (`nav_ctrl.h`) | `FAULT_LINE_SENSOR`, bit 4 of the shared fault flag, this task's bit only |

Neither task drives the motors.

## Line tracking

- **Calibration** (on CAL_REQ, car still, `IR_CAL_SAMPLES` ms, the 1 ms loop
  keeps running). Per sensor:
  - If it sees black and white (span at least `IR_CAL_MIN_CONTRAST`), the
    thresholds are the midpoint ± `IR_CAL_HYST_PCT`.
  - If it sees white only (the normal start pose), the tested thresholds are
    kept, and white must sit `IR_CAL_WHITE_MARGIN` below the falling one.
  - A reading at an ADC rail, or too little contrast, gives `CAL_FAIL`.
- **Every sample:**
  - normalised readings (0 white .. 1000 black);
  - hysteresis classification and the 6/8-sample 0bRL debounce;
  - the error `(right − left)/10`, −100..+100. **Positive = correct left**
    (counter-clockwise, the same sign as `S.trn`);
  - lost after `LINE_LOST_MS` without a confirmed black. This task only
    reports it; motion decides what to do (LINE_SEARCH, then LINE_LOST
    pauses the mission). `LINE_STOP_ON_LOST` (`robot_config.h`) is unused:
    no code reads it, so 0 and 1 build the same car. It is kept so a build
    that passes `-DLINE_STOP_ON_LOST` still compiles;
  - a sensor fault after `IR_RAIL_FAULT_SAMPLES` at a rail, or for a sample
    whose ADC read failed. The fault sets `line_state`'s fault bit, and
    `ir_sense_task` sets `FAULT_LINE_SENSOR` (`S.faults` bit 4,
    MQTT_SCHEMA.md 4.1) on the edge where it starts and clears it on the
    edge where it ends, before the console, so a barcode read never holds
    it back. A failed ADC start-up sets it for good (the task stops).
    This glue in `line_task.c` is kernel code and is not host-tested: the
    host test covers only `line_state`'s fault bit, which it follows.
    The bit is a report: nav does not read the fault flag
    (`FAULT_STOP_MASK` has no user), so a line fault still stops the car
    the old way: motion treats it as the line missing, LINE_SEARCH, then
    LINE_LOST pauses the mission.
- **Junction:** GP26 and GP27 are closer together than the 18 mm line, so
  both read black (rl 11) whenever the car is centred. That is normal
  following, never a junction, and those two sensors cannot see a
  crossing. The junction comes from the barcode sensor: GP7 black for
  `LINE_JUNCTION_SAMPLES` (J_n = 280, provisional) samples running gives one
  event. J_n must be longer than any barcode bar and shorter than the
  crossbar at the speed GP7 passes them, so **J_n is tied to motion's
  line-following base `MOTION_LF_BASE`** (`motion_cfg.h`): change both
  together. With the course bars (see "The course barcodes" below):

  | Base | Speed (C) | 9 mm wide bar (C) | 18 mm crossbar (C) | With J_n 280 |
  |---|---|---|---|---|
  | 4 | 31.7 mm/s | 284 ms | 567 ms | wide bars fire junctions (host test) |
  | **6 (now)** | **47.6 mm/s** | **189 ms** | **378 ms** | at a constant GP7 speed: no bar fires, the crossbar does, for bars and crossbar ±2 mm off print (host test, sim) |
  | 8 | 63.5 mm/s | 142 ms | 284 ms | no bar fires, but a crossbar seen 1 mm or more narrow is missed (host test) |

  Speed: base × 100 counts/s × π × 64 mm / 2535 counts (C);
  INTEGRATION.md S4 gives about 4.7 cm/s at base 6 from lap times.
  - **The margins hold only at a constant GP7 speed near the base speed.**
    With J_n 280, a 9 mm bar fires a junction if GP7 is slower than about
    32 mm/s, and an 18 mm crossbar is missed if GP7 is faster than about
    64 mm/s (C). Steering moves GP7's speed. On 4 Nov the car turned at
    41 deg/s during line following
    (`tests/scan_bench/logs/trials_batch1_2026-11-04.txt`, line 2554; the
    49 deg/s at line 2561 came after the line was already lost). If GP7 sits
    45 mm to the side of the centreline (assumed; the real mount is not
    measured), that moves GP7's speed by about ±32 mm/s (C), past both
    limits. So a
    weaving car can miss a crossbar or fire on a wide bar. The host test
    and the sim both feed a constant speed.
  - **The gate for J_n 280** is the car session C1 count of junction lines
    per sheet and per crossbar, with the car weaving as it really does.
  - **J_n must move with `MOTION_LF_BASE`.** With J_n 280 the host test
    passes at base 5 to 7 and fails at base 4 or below and 8 or above.
    Nothing in the firmware checks it: an image built with
    `-DMOTION_LF_BASE=4` or `=8` quietly keeps 280.
  - The old J_n 80 (sized for about 12 cm/s) was 3.8 mm of black at base 6,
    so every 9 mm bar fired a junction: 6 per sheet (sim). motion accepts a
    JUNCTION in LINE_FOLLOW, so a TEST image, or a command still pending
    from a missed junction, turned the car in the middle of a sheet.
  - On 4 Nov a 124 ms GP7 run on a plain line fired a false junction during
    line following (`tests/scan_bench/logs/trials_batch1_2026-11-04.txt`,
    lines 2555-2559, `0: B124`). J_n 280 would not fire on it.
  - At base 6 J_n 280 fires with GP7 13.3 mm into the crossbar (3.8 mm with
    80) (C), about 200 ms later; `MOTION_TURN_FWD_MM` (Buddy 2) counts from
    there.
  - `-DLINE_JUNCTION_SAMPLES=80` builds the old value for an A/B run.

  It rearms only after GP7 has read white for `LINE_JUNCTION_REARM_SAMPLES`
  samples running. A barcode read doesn't block it, because the crossbar
  starts a capture of its own.
- **Barcode hold (`LINE_BARCODE_HOLD`, default 1, today's behaviour):**
  while a barcode is being read, and `BARCODE_JUNCTION_HOLDOFF_MS` after, the
  word's rl/steer/error are held at 0 and its barcode bit is set, so motion
  holds its heading. With 0 nothing is held. The hold was written for bars
  that cross the line sensors; on the demo course the sheets sit beside the
  line (the prof's course photos, 19 Nov), so GP26/GP27 never see a bar.
  Which setting is better there is **not decided yet**:
  - Holding keeps the heading, so GP7 passes the bars at a constant speed,
    which the decoder needs. With GP7's speed varying ±20 %, today's decoder
    decodes 79-91 of 208 reads; with ±10 %, all 208 (sim).
  - Not holding avoids blind straight driving: up to about 3.4 s (about
    158 mm) per sheet, and about 2.2 s per crossbar, because the crossbar
    under GP7 opens a read too (sim).
  - Avani and Hanis choose after the car session C1 A/B. Build the other
    side with `-DLINE_BARCODE_HOLD=0`.

  Held or not, the dark time and the lost bit follow the real line sensors.
  Before, a hold also kept the dark time at 0, so with both sensors on white
  motion still read "line seen" (rl 00 but dark under `MOTION_LF_SEEN_MS`) and
  the line was never lost: a turn that landed off the arm went on to
  LINE_FOLLOW and drove blind for 0.38-2.87 s instead of LINE_SEARCH (sim).
  Now a held car on white stops being "line seen" after `MOTION_LF_SEEN_MS`
  and is lost after `LINE_LOST_MS` (host test).
- **Mission change:** the junction is disarmed and a partial barcode
  aborted.

## The course barcodes

From the prof's Barcode_Line_Specification.pdf and Barcode_Sample.pdf, and
the course photos (19 Nov):

- **Code 39**, start/stop `*`, one letter per sheet (`*A*`, `*Z*` in the
  sample).
- A printed sheet **about 141 mm long**, placed **beside the line**, just
  before a junction, bars running across the direction of travel. One photo
  has a sheet on each side of the line.
- The bars start **20 mm from the line edge** (29 mm from the centre of the
  18 mm line), so GP26 and GP27 never see them: only GP7 reads them, and it
  must sit over the bars on the sheet's side.
- **Narrow 3 mm, wide 9 mm** (C: `*X*` is 47 narrow modules, 141 / 47 =
  3.0 mm, wide = 3 × narrow).
- At base 6 GP7 sees a narrow bar for 63 ms and a wide bar for 189 ms (C).
  That sets J_n (above).
- The 3 mm sheet has never been read on the car. The "KY-033 cannot resolve
  2 mm bars" result (INTEGRATION.md S3) does not cover it.

## Barcode

- **Capture** (`bar_capture.c`): widths are counted in 1 ms ticks, so there
  are no timestamps and no wraparound.
  - `END` after the quiet zone Q = max(`BARCODE_Q_TICKS`,
    4 × the narrowest width).
  - `ABORT` on more than `BARCODE_W_MAX` widths, a width over
    `BARCODE_W_LONG_TICKS`, a read over `BARCODE_READ_MAX_TICKS`, a full
    `bar_mbf`, or a mission change.
  - After an abort, the capture waits for a quiet zone, so the rest of the
    same barcode never starts a read. Each read has a new `read_id`, so
    nothing from it can join the next.
- **Decode** (`barcode_rx.c`, `barcode_decode.c`): Code 39 with `*`, read
  forwards or backwards.
  - Narrow and wide are split per symbol by the relative ratio R
    (`BARCODE_WIDE_NARROW_MIN_RATIO_X100`).
  - The element count, start/stop, the gaps and the pattern are all checked.
- **Rejected reads** (malformed, more than one letter, an unknown letter,
  aborted, widths lost) issue no command, keep any stored one, print the
  reason (with the widths for malformed reads) and send letter 31.
- **Action codes:** 0 FORWARD (straight), 1 LEFT, 2 RIGHT, 3 U_TURN.
- **Table:**
  - **The letters are only given on the day**, so no table is compiled in
    (`BARCODE_DEFAULT_TABLE_SET` 0). Until a 7-byte map arrives, every
    letter is rejected as unknown. How to load one: "Commands on the day"
    below.
  - Maps are applied only between reads.
  - A map of the wrong length is rejected and the working table kept.
- **The stored command:** it survives a pause and is cleared on a new
  mission, Finished or Fault. nav takes it once, at the next junction while
  Running. With none stored, nav sends nothing and motion follows straight
  on.

## Commands on the day

The letter for each command is only given on the day (expected: left,
right, straight and U-turn; diagonal is out of scope). Nothing needs a
reflash: the table goes to the car over MQTT before the run.

**The chain, and what each step prints** (host tests in brackets):

| Step | Console | Test |
|---|---|---|
| 1. The map arrives (comms checks the length, `barcode_task` applies it between reads) | `[comms] map mv=2 LRFUFF... -> barcode_task`, then `[barcode] map v2 applied: LRFUFF...` | `test_barcode_task`, `tests/host` |
| 2. A sheet is read; the letter is looked up; the command is stored for the next junction | `[barcode] 'B' -> RIGHT (dir=fwd), stored for the next junction` | `test_barcode_task`, `test_barcode_action` |
| 3. GP7 sees the crossbar | `[line] junction (GP7 black 280 ms) -> nav` | `test_line_track` |
| 4. nav sends the stored action to motion | `[nav] -> motion: JUNCTION(RIGHT)` | `tests/nav` (PR #24) |
| 5. motion turns: F drives `MOTION_TURN_CROSS_MM` across; L and R drive `MOTION_TURN_FWD_MM`, then pivot 90 deg; U pivots 180 deg counter-clockwise | `[motion] nav_cmd JUNCTION (raw 0x2002): accepted, TURNING`, `[motion] state LINE_FOLLOW -> TURNING` | `tests/motion_task_sim` (PR #22) |
| 6. The turn ends on an arm (or not) | `[motion] state TURNING -> LINE_FOLLOW, sent MOVE_DONE` (or `-> LINE_SEARCH, sent MOVE_DONE`), `[nav] motion: MOVE_DONE` | `tests/motion_task_sim` |

The raw value in step 5 is `0x20` followed by the action: 00 F, 01 L,
02 R, 03 U. The crossbar also opens a barcode read of its own, so each
junction adds `[barcode] REJECTED: malformed (bad_length)` and a width
dump; the stored command is not touched by it.

**Loading the table (the normal way, no reflash).** Needs the MQTT car
image (the USB-only image has no `comms_task`).

1. Write down the letter for each command, e.g. A left, B right, C
   straight, D U-turn.
2. `python3 tools/barcode_map.py A=L B=R C=F D=U --mv 2` (on
   `proposal/comms-fixes`, PR #26). It prints the 26-letter table
   (`LRFUFFFFFFFFFFFFFFFFFFFFFF`, A first; letters not named are F), the
   7 bytes, and a `mosquitto_pub` line. It sends nothing unless given
   `--publish --broker HOST`.
3. Send it, **before pressing start**, one of three ways: the viewer's
   "Send map" box (the 26 letters and mv); the printed `mosquitto_pub`
   line; or the script with `--publish`. All three publish retained at
   QoS 1 on `robocar/<UTK_MQTT_CLIENT_ID>/map`, so a car that reboots gets
   it again when it subscribes.
4. Check the two console lines of step 1 and that `S.mpv` shows the mv.
   Use mv 1-15 and a new one for every new map: `S.mpv` is 0 with no map,
   and the car has no version gate, so an older mv is applied too.

Rejected maps change nothing: `[comms] map rejected #n: len=6 bytes=...`
(wrong length), `[comms] radio refused n inbound message(s): ...` (empty,
over 7 bytes or a wrong topic), `[comms] map mv=3 ... not passed on
(er=-50): table kept, mpv stays 2` (two maps already waiting behind a
read). A map stays until the car is reset; a new mission keeps it.

**Fallback: a compiled-in table (rebuild and flash).** Edit the two lines
in `line_config.h`: `BARCODE_DEFAULT_TABLE_SET (1)` and
`BARCODE_DEFAULT_TABLE "LRFUFFFFFFFFFFFFFFFFFFFFFF"` (or build with the
session kit's `TABLE=`). Build from a clean tree or through the kit: an
in-place rebuild does not recompile on a changed `-D`. The boot line then
reads `[barcode] decoder ready, table: compiled-in default LRFUFF...`. A
character other than F, L, R, U gives `... table: none until a map arrives
(compiled-in table is not 26 of F/L/R/U)`, and a wrong length stops the
build. `S.mpv` stays 0, and a later map replaces the table.

**No command.** With no map and no compiled-in table, every read prints
`[barcode] REJECTED: 'A' unknown (no barcode map loaded)` and sends letter
31; at the junction nav prints `[nav] junction, no barcode command:
straight on`, sends nothing, and line following carries on across. Once a
table is loaded every letter A-Z has an action (the map has no empty
value), and anything that is not A-Z is outside the decoder's alphabet
(`REJECTED: malformed`). A barcode F is a command: motion drives 40 mm
across without steering, which is not the same as line following across.

**A map sent during a run** is applied only when no barcode is being
read: the read in progress uses the table it started with, and a command
already stored for the next junction is not changed (host test). The rule
stays: publish before start (MQTT_SCHEMA.md 8.3).

**Limits that no table fixes** (car session checks):

- GP7 reads only a sheet on its own side of the line. Which side GP7 is
  on is not measured; a sheet on the other side is never read.
- A T met from the side, with the stem on the side away from GP7: GP7
  crosses no line, so there is no junction event (sim). The stored
  command waits for the next junction, the wrong place.
- A T met from the stem: GP26/GP27 (125 mm ahead of the axle, measured)
  leave the end of the stem about 1.48 s before GP7 (50 mm ahead,
  `MOTION_TURN_FWD_MM`, proposed) fires the junction at base 6 (sim,
  straight approach). The line is lost after `LINE_LOST_MS` 1500 ms, a
  20 ms margin, and in that time line following steers hard toward the
  last side it saw the line on (register S1), which the sim does not
  model. If motion has left Line Following, it refuses the JUNCTION.
- A dead end has no crossbar, so no junction: a U-turn command there never
  runs, and the car goes Line Search, Line Lost, and pauses (C, from the
  code).
- A command with no arm to go to (straight at a T met from the stem, or a
  turn to the side a T has no arm on) ends `TURNING -> LINE_SEARCH`, then
  `LINE_SEARCH -> LINE_LOST, sent LINE_LOST` unless the 45 deg sweep finds
  a line (host test; the sweep against real lines is not modelled).

## Console

```
[line] raw ADC GP26=312 GP27=298 norm 41/38 err=0 | barcode DO GP7=0 (white) | missed 0 ticks (0 since boot)
[line] calibration OK: GP26 ok (white 300, rise 2500) GP27 ok (...)
[barcode] decoder ready, table: none until a map arrives
[barcode] map v2 applied: LRFUFFFFFFFFFFFFFFFFFFFFFF
[barcode] 'C' -> RIGHT (dir=rev), stored for the next junction
[barcode] REJECTED: malformed (bad_length)
[line] junction (GP7 black 280 ms) -> nav
```

`LINE_COMMISSIONING_MODE=1` prints the heartbeat every 100 ms instead of
2 s. Nothing prints while a barcode is under the car.

**Missed ticks (NFR-11, line sampling every 1 ms ± 0.5 ms).** The heartbeat
ends with the 1 ms ticks `ir_sense_task` did not serve on time: since the
previous heartbeat line, and since boot. A tick is missed when the task
wakes a whole tick or more late (`wait_tick`, the wake-ups still queued).
That tick gets no line sample of its own; the barcode capture still counts
it as time. 0 on every line means no sample came a whole tick late. It
cannot show jitter under 1 ms. Take NFR-11 evidence from the plain car
image: print-heavy images (`LINE_GP7_DIAG`, commissioning mode) add console
work inside the 1 ms loop, and the netlog image measured worse 10 ms
windows for the IMU (docs/imu/terrain_analysis_report.md, NFR-11 rows).

## Tests

```sh
docker compose run --rm build bash -lc "make -C tests/line"
```

| Suite | What it covers |
|---|---|
| `test_ir_cal` | calibration pass and fail (rail, contrast), hysteresis, normalising, both polarities |
| `test_line_track` | error direction, J_n debounce, one event, rearm, J_n against the course bars and crossbar at `MOTION_LF_BASE` (bars ±2 mm), noise, barcode hold (the dark time and the loss still follow the sensors under it), noisy loss, stuck sensors and a failed ADC read (`line_state`'s fault bit starts and ends, also under a barcode hold; the S.faults bit 4 glue in `line_task.c` is not host-tested) |
| `test_barcode_pipeline` | every letter, both directions, 3 speeds, from 1 ms samples through capture, messages, decode and table; malformed, unknown, multi-letter, END/ABORT, W_long, W_max, timeout, cancel, stale `read_id`, lost widths |
| `test_barcode_action` | the table, map apply and reject, the command word; every action for every letter from a map; the writeup example as the laptop's 7 bytes; the comms vector; built again with a compiled-in table (`test_barcode_action_table`) |
| `test_barcode_task` | `barcode_task.c`'s own loop on a fake kernel: the boot table line, a map from comms, each action stored from it, a map sent while a sheet is read (waits for the read), a map between a read and its junction (the stored command stays), a full map queue, a wrong length, a new mission; built three times (no table, a compiled-in table, one with a bad character) |
| `test_line_state` | the word, and the motion statechart contract |

These are software tests only. Thresholds, the barcode sensor at speed, the
sensor spacing against the artwork, and calibration on the real start pose
are not verified on hardware.

## Open items for the team

- **Buddy 2:** nav_cmd_mbf, reading `line_state`, the drive and the turns.
- **Buddy 1:** done: the `map` command calls `barcode_map_submit()` (M8b)
  and `S.mpv` is `barcode_map_version()`.
- **Team:** name who sends the map on the day, from which laptop, to which
  broker, and check a retained map reaches the car on that broker
  (register J1, C1). The action codes are fixed: 0 F, 1 L, 2 R, 3 U.
- **Car session:** the limits under "Commands on the day" (GP7's side, a T
  met from the stem, a T met from the side).
