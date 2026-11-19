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
  - lost after `LINE_LOST_MS` without a confirmed black;
  - a sensor fault after `IR_RAIL_FAULT_SAMPLES` at a rail.
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
  | 4 | 31.7 mm/s | 284 ms | 567 ms | wide bars fire junctions |
  | **6 (now)** | **47.6 mm/s** | **189 ms** | **378 ms** | no bar fires, the crossbar does, for bars and crossbar ±2 mm off print (host test, sim) |
  | 8 | 63.5 mm/s | 142 ms | 284 ms | marginal: a crossbar seen 1 mm narrow is missed (host test) |

  Speed: base × 100 counts/s × π × 64 mm / 2535 counts (C);
  INTEGRATION.md S4 gives about 4.7 cm/s at base 6 from lap times.
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
- **Barcode hold (`LINE_BARCODE_HOLD`, default 0):** with barcodes printed
  across the line (1), a read holds rl/steer/error at 0 and sets the word's
  barcode bit, so motion holds its heading over the bars. On the demo course
  the barcodes sit beside the line (the prof's course photos, 19 Nov), so the
  line sensors never see a bar and nothing is held (0). Holding there blinded
  line following for about 2 s after every junction (the crossbar under GP7
  opens a read too) and made motion read "line seen" on white.
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
  - **No approved table exists yet** (MQTT_SCHEMA.md §12 item 9). Until a
    7-byte map arrives, every letter is rejected as unknown. Set
    `BARCODE_DEFAULT_TABLE_SET` / `BARCODE_DEFAULT_TABLE` once the mapping
    is approved.
  - Maps are applied only between reads.
  - A map of the wrong length is rejected and the working table kept.
- **The stored command:** it survives a pause and is cleared on a new
  mission, Finished or Fault. nav takes it once, at the next junction while
  Running. With none stored, nav sends nothing and motion follows straight
  on.

## Console

```
[line] raw ADC GP26=312 GP27=298 norm 41/38 err=0 | barcode DO GP7=0 (white) | missed 0 ticks (0 since boot)
[line] calibration OK: GP26 ok (white 300, rise 2500) GP27 ok (...)
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
| `test_line_track` | error direction, J_n debounce, one event, rearm, J_n against the course bars and crossbar at `MOTION_LF_BASE` (bars ±2 mm), noise, barcode hold, noisy loss, stuck sensors |
| `test_barcode_pipeline` | every letter, both directions, 3 speeds, from 1 ms samples through capture, messages, decode and table; malformed, unknown, multi-letter, END/ABORT, W_long, W_max, timeout, cancel, stale `read_id`, lost widths |
| `test_barcode_action` | the table, map apply and reject, the command word |
| `test_line_state` | the word, and the motion statechart contract |

These are software tests only. Thresholds, the barcode sensor at speed, the
sensor spacing against the artwork, and calibration on the real start pose
are not verified on hardware.

## Open items for the team

- **Buddy 2:** nav_cmd_mbf, reading `line_state`, the drive and the turns.
- **Buddy 1:** call `barcode_map_submit()` from the `map` command (M8), and
  report `barcode_map_version()` as `S.mpv`.
- **Team:** approve the action codes and a default table.
