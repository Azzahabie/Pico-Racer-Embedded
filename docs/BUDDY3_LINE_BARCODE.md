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
- **Junction, barcode-armed (`LINE_JUNCTION_RULE` 1, default; the team's
  rule, 19 Nov).** The course has a barcode sheet before or at each
  junction, in sizes not known in advance, so no single GP7 black length
  can tell every bar from the crossbar. Instead (`junction_arm.c`):
  1. The capture ENDs a read as soon as its 29 widths (start, letter,
     stop) decode as a valid `*X*`, either way round
     (`BARCODE_END_ON_STOP`), so a crossbar close behind the sheet can no
     longer join the read.
  2. `barcode_task` judges the read, stores the command, and only then
     stores its judged word (`barcode_judged_word()`).
  3. `ir_sense_task` sees the read judged with a new command stored: the
     junction is **armed** (`[line] junction armed: ...`).
  4. The next black run GP7 sees is the junction, once it has lasted
     `LINE_ARMED_MIN_SAMPLES` (5 ms, noise) and `LINE_ARMED_CONFIRM_MM`
     (10 mm, provisional) of travel from `distance_state`. 10 mm is longer
     than the first bar of any barcode in the size range (every Code 39
     barcode starts with a narrow bar, at most 6 mm), so a second sheet
     before the same junction does not fire, and shorter than the 18 mm
     crossbar. Where it fires, which `MOTION_TURN_FWD_MM` counts from:
     about 10 mm into the crossbar at base 6 and 8 (J_n 280 is 13.3 and
     17.8 mm there, C). **At base 4 J_n 280 is 8.9 mm (C), so the length
     rule fires first** and the line still reads `[line] junction (armed
     by barcode ..., GP7 black 8 mm)` or `9 mm` (host test: 8.9-10.3 mm at
     a steady GP7 speed). With GP7's speed ±0-20 % off the car's and blur
     ±1 mm, by either rule: 7.6-10.3 mm at base 4, 7.8-15.5 at base 6,
     7.4-19.1 at base 8 (sim). A speck or dropout before or in the
     crossbar (below) can move it later, up to the crossbar's end.
  5. nav reads the stored command at the flag, so the command is always
     there first. A junction that falls due while a complete read waits
     for its verdict waits up to `LINE_ARMED_VERDICT_WAIT_MS` (50 ms).

  While the bars of a read arrive (past its first black run and short of
  29 widths), a black run is a bar, not a junction, unless it is too long
  to be a bar of that read ("Not a bar" under Capture below). Without
  that, a crossbar that started inside such a read never fired, armed or
  not (H1, checker, 19 Nov): after a dropout in the crossbar, a speck or
  sheet edge just before it, or an unreadable sheet close before it. When
  a junction fires inside an open read, the read ENDs there: `barcode_task`
  prints the widths before the crossbar as a malformed reject (for
  example `2 widths`, `0: B63 w4` for a dropout 3 mm into the crossbar at
  base 6), and the rest of the crossbar opens no read. A rejected read
  (malformed, unknown letter, no table) does not arm. The arming ends at
  the junction, after `LINE_ARMED_HOLD_MM` (150 mm, a guess: D_hold,
  week6_design_review.md assumption 2) of travel with no junction (the
  stored command stays for nav), and on any mission change. With nothing
  armed, the length rule below is the fallback. When the junction fires,
  the crossbar's own read is dropped (`aborted (junction)`, printed by
  nobody), so it neither holds `line_state` for its quiet zone nor sends a
  reject. Host test `test_junction_arm` (the envelope below);
  `-DLINE_JUNCTION_RULE=0` builds the length rule alone for an A/B.

  **Defaults that changed without car data** (each a guess until the car
  session; each switch's `-D` builds the behaviour before it):

  | Switch | Default | Before |
  |---|---|---|
  | `LINE_JUNCTION_RULE` (Hanis's decision) | 1, with 10 mm `LINE_ARMED_CONFIRM_MM`, 150 mm `LINE_ARMED_HOLD_MM`, 50 ms `LINE_ARMED_VERDICT_WAIT_MS`, all three guesses, none measured | `-DLINE_JUNCTION_RULE=0` |
  | `BARCODE_END_ON_STOP` | 1 | `-DBARCODE_END_ON_STOP=0` (only with rule 0) |
  | `BARCODE_DECODE_SPLIT` | 1 (no wrong letter in 21060 + 42120 sim reads) | `-DBARCODE_DECODE_SPLIT=0` |
  | `BARCODE_Q_WIDTH_FACTOR` | 6 (trade-off under Capture) | `-DBARCODE_Q_WIDTH_FACTOR=4` |
  | `BARCODE_GLITCH_TICKS` | 3 | `-DBARCODE_GLITCH_TICKS=0` |
  | `BARCODE_NOT_BAR_RATIO_X10` | 25 | `-DBARCODE_NOT_BAR_RATIO_X10=0` |

  All of them together are `OLD_RULE` in `tests/line/Makefile`. A `-D`
  car image needs a clean tree and `make GCC="arm-none-eabi-gcc -D..."`:
  `CFLAGS` is set with `:=` (`build_make/pico_rp2040.mk:257`).
- **Junction, length rule (the fallback; alone with `LINE_JUNCTION_RULE`
  0):** GP26 and GP27 are closer together than the 18 mm line, so
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
  samples running. A read's first black run is not blocked, because the
  crossbar starts a capture of its own; with `LINE_JUNCTION_RULE` 1 a run
  while the bars of a read arrive is, unless it is too long to be a bar.
  With rule 1 the junction side counts GP7's runs after the capture's
  glitch filter.
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
- **On the day the sizes differ** (team, 19 Nov), and sheets sit before or
  AT the junction. Host sim over narrow 2-6 mm, wide 2-3 × narrow, base
  4/6/8, GP7's speed ±0/10/20 % (steering), bars read −1..+1 mm off
  (sensor blur), all 26 letters, crossbar 40 mm after (63180 reads):

  | | before (branch tip e4a904e) | now |
  |---|---|---|
  | reads decoded | 25696 | 49217 |
  | junction events inside a barcode | 88913 | 0 |
  | decoded reads with the command at the crossbar junction | 22268 | 49217 (all) |
  | crossbar missed | 9516 (base 8) | 2374 (base 8, undecoded reads only) |

  The H1 fix (glitch filter, "not a bar") leaves every number in the "now"
  column the same (sim, re-run 19 Nov), and so does reading half of 42120
  reads backwards: 0 junctions inside a barcode, 0 wrong letters.

  Gap from the last bar to the crossbar that works for all 26 letters:
  before, about 4 × narrow (10 mm for 2 mm bars, 12 for 3, 20 for 4, 25
  for 6); now 0 (a sheet printed right up to the crossbar), except 2 mm
  bars read 1 mm thin, which never decode.
  What still fails: 2 mm bars read 1 mm off, 2.5-3 mm bars read 1 mm thin
  with steering, and wide 2 × narrow with GP7's speed ±20 % (6-15 % of
  reads decode). The hold (`LINE_BARCODE_HOLD` 1) keeps GP7's speed
  steady over the bars.
- The 3 mm sheet has never been read on the car. The "KY-033 cannot resolve
  2 mm bars" result (INTEGRATION.md S3) does not cover it, and no code can
  fix a sensor that does not resolve a bar: a 2 mm sheet may not be
  readable at all with the KY-033.

## Barcode

- **Capture** (`bar_capture.c`): widths are counted in 1 ms ticks, so there
  are no timestamps and no wraparound.
  - `END` as soon as the first 29 widths decode as a valid `*X*`
    (`BARCODE_END_ON_STOP` 1), on the sample after the stop character's
    last bar. If that bar runs on into a crossbar printed against it, the
    read is tried with the bar taken as narrow (the 21st width) once the
    run is twice that.
  - **Glitch filter** (`BARCODE_GLITCH_TICKS` 3): a GP7 run of black or
    white 3 samples or shorter counts as the level either side of it, for
    the capture and for the junction side, so a 1-3 ms dropout inside the
    crossbar neither opens nor splits a read. GP7 has done it: `B63 w4
    B24` (`tests/scan_bench/logs/trials_batch2_2026-11-04.txt:196`). The
    narrowest real bar is 31 ms (2 mm at base 8), 13 ms read 1 mm thin
    with GP7 20 % fast (C). Edges are seen 3 ms late; widths are unchanged.
  - **Not a bar** (`BARCODE_NOT_BAR_RATIO_X10` 25): inside an open read, a
    black run longer than 2.5 × the widest width before the space in front
    of it is not a bar; the junction rules apply to it (armed: 10 mm;
    otherwise J_n). That space is left out because it may be the gap to
    the crossbar. 2.5 holds from the read's 11th width on: by then a wide
    element has gone by (from any bar of `*X*`, at most 8 narrow elements
    come before a wide one, C), and no bar long enough for either rule is
    over 2.1 × a wide element (C). Before that the ratio is 7
    (`BARCODE_NOT_BAR_EARLY_X10`): a reversed start character's 3rd width
    is a wide bar, up to 6.3 × the narrow bar before it (C). So a speck in
    front of the crossbar must be under 2.6 mm, and the junction fires 7 ×
    the speck into the crossbar (14 mm for 2 mm, host test). After a
    dropout (a first space under 1/5 of the first bar, which no barcode
    space is, C) the ratio is 2.5 again: armed, a 4 ms dropout up to 5 mm
    into the crossbar still fires (sim, base 4/6/8). Sim: the envelope
    above and 42120 reads with half read backwards decode the same and
    fire 0 junctions inside barcodes, with and without it.
  - Otherwise `END` after the quiet zone Q = max(`BARCODE_Q_TICKS`,
    `BARCODE_Q_WIDTH_FACTOR` 6 × the narrowest width). 6 (was 4): with
    the bars 0.5 mm wide and GP7's speed 10 % off, the course's 9 mm wide
    space is 4.2 × its narrowest width (C), so 4 × ended reads inside the
    barcode. **The cost:** a GP7 black that does not decode (a brush of
    the line, a seam, a failed sheet, the crossbar in a rule 0 image)
    keeps its read open 6 × its width instead of 4 ×, and with
    `LINE_BARCODE_HOLD` 1 the steering hold lasts as long (width + Q +
    300 ms): the 124 ms blip of 4 Nov holds 1168 ms instead of 920 ms,
    about 250 ms or 12 mm more blind at base 6 (C). Re-run with the H1
    fix (sim, 63180 reads): factor 4 decodes 33679 and fires 1132
    junctions inside barcodes, 6 decodes 49217 with none, so 6 stays.
  - `ABORT` on more than `BARCODE_W_MAX` widths, a width over
    `BARCODE_W_LONG_TICKS`, a read over `BARCODE_READ_MAX_TICKS`, a full
    `bar_mbf`, or a mission change.
  - After an abort, the capture waits for a quiet zone, so the rest of the
    same barcode never starts a read. Each read has a new `read_id`, so
    nothing from it can join the next.
- **Decode** (`barcode_rx.c`, `barcode_decode.c`): Code 39 with `*`, read
  forwards or backwards.
  - Narrow and wide are split per symbol by the relative ratio R
    (`BARCODE_WIDE_NARROW_MIN_RATIO_X100`). If that fails, bars and spaces
    are split apart (`BARCODE_DECODE_SPLIT` 1): the two widest bars and the
    widest space are wide. Sensor blur reads every bar wider and every
    space narrower by the same amount, which the one split cannot take
    past about a quarter of the narrow width.
  - The element count, start/stop, the gaps and the pattern are all checked.
- **Rejected reads** (malformed, more than one letter, an unknown letter,
  aborted, widths lost) issue no command, keep any stored one, print the
  reason (with the widths for malformed reads) and send letter 31. A read
  ENDed by a junction is one of these: the widths before the crossbar.
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
| 3. The read ends at its stop character and arms the junction; GP7 reaches the crossbar | `[line] junction armed: barcode 'B' -> RIGHT, the next black under GP7 is the junction`, then `[line] junction (armed by barcode 'B' RIGHT, GP7 black 10 mm) -> nav` | `test_junction_arm`, `test_line_track` |
| 4. nav sends the stored action to motion | `[nav] -> motion: JUNCTION(RIGHT)` | `tests/nav` (PR #24) |
| 5. motion turns: F drives `MOTION_TURN_CROSS_MM` across; L and R drive `MOTION_TURN_FWD_MM`, then pivot 90 deg; U pivots 180 deg counter-clockwise | `[motion] nav_cmd JUNCTION (raw 0x2002): accepted, TURNING`, `[motion] state LINE_FOLLOW -> TURNING` | `tests/motion_task_sim` (PR #22) |
| 6. The turn ends on an arm (or not) | `[motion] state TURNING -> LINE_FOLLOW, sent MOVE_DONE` (or `-> LINE_SEARCH, sent MOVE_DONE`), `[nav] motion: MOVE_DONE` | `tests/motion_task_sim` |

The raw value in step 5 is `0x20` followed by the action: 00 F, 01 L,
02 R, 03 U. The crossbar also opens a barcode read of its own; with the
barcode-armed junction that read is dropped silently at the junction.
With `LINE_JUNCTION_RULE` 0 (or a junction by the length rule with no
barcode) the line reads `[line] junction (GP7 black 280 ms, length rule)
-> nav` (`... 280 ms) -> nav` with rule 0), and rule 0 adds `[barcode]
REJECTED: malformed (bad_length)` and a width dump for the crossbar; the
stored command is not touched by it.

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
[line] junction armed: barcode 'C' -> RIGHT, the next black under GP7 is the junction
[line] junction (armed by barcode 'C' RIGHT, GP7 black 10 mm) -> nav
[line] junction (GP7 black 280 ms, length rule) -> nav
[line] barcode 'C' read, no command stored: junction by length only
[line] barcode 'C' read, no verdict from barcode_task in 50 ms: junction by length only
[line] junction disarmed: no junction within 150 mm of barcode 'C' (command kept for nav)
[line] junction disarmed: mission PAUSED
[barcode] REJECTED: malformed (bad_length)
```

The `[line] junction armed` and `disarmed` lines print even while a
barcode is read (as the junction line does).

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
| `test_junction_arm` | the barcode-armed junction end to end, 1 ms samples through capture, messages, `barcode_task`'s store (modelled, with a latency), `junction_arm` and the gate: every size (narrow 2-6 mm, wide 2-3 ×, base 4/6/8, blur ±0.5 mm) decodes, fires exactly one junction at the crossbar with the command and none inside the barcode, 8.9-10.3 mm into the crossbar; the course sheet with GP7's speed ±10-20 % and blur ±1 mm; the crossbar 0-80 mm after the sheet; the command stored before the flag with `barcode_task` up to 40 ms late, and a dead `barcode_task`; noise, a malformed read and an unknown letter do not arm; D_hold and a mission change disarm; two sheets before one junction; the length-rule fallback, and its crossbar read dropped. H1, at base 4/6/8: a 4 ms dropout 3 mm into the crossbar after an armed sheet (and the next crossroad finds no command), a 2 mm speck 5 and 10 mm before the crossbar, a 1 ms white 0.5 mm in and 2-3 ms at 12 mm in, armed and not, and an unreadable sheet of 17/21/25 widths 5-10 mm before it, each exactly one junction at the crossbar; a second sheet either way round before the junction (288 runs), none inside it. `make -C tests/line junction_old` builds it on the old rule: 11 of 13 checks fail; `make -C tests/line junction_h1_off` without the H1 fix: 34 of 150 fail |
| `test_barcode_decode` (`tests/`) | every letter and 3 strings both ways with ±10-20 % edge bias; A-Z with bars 25-33 % of narrow wider and spaces narrower at wide 2-3 × (needs the bar/space split) |
| `test_line_track` | error direction, J_n debounce, one event, rearm, J_n against the course bars and crossbar at `MOTION_LF_BASE` (bars ±2 mm), the barcode side's gate (bars arriving block, armed fires, a due junction waits for the verdict), noise, barcode hold (the dark time and the loss still follow the sensors under it), noisy loss, stuck sensors and a failed ADC read (`line_state`'s fault bit starts and ends, also under a barcode hold; the S.faults bit 4 glue in `line_task.c` is not host-tested) |
| `test_barcode_pipeline` | every letter, both directions, 3 speeds, from 1 ms samples through capture, messages, decode and table; malformed, unknown, multi-letter, END/ABORT, W_long, W_max, timeout, cancel, stale `read_id`, lost widths; END at the stop character, a crossbar 2 ticks past the glitch filter, a sheet printed against the crossbar, 29 invalid widths; blur with steering against the quiet factor; the glitch filter (1-3 tick glitches in every bar and space of a sheet merged, 4 ticks kept, a short speck opens no read); not a bar (where a dropout, a speck, a reversed start, 8 narrow widths and an unreadable sheet stop being bars; no bar of any letter, either way, 6 blur and ratio cases, ever is not one); a read ENDed or dropped by a junction |
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
