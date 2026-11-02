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
  `LINE_JUNCTION_SAMPLES` (J_n = 80, provisional) samples running gives one
  event. That is longer than any barcode bar and shorter than the crossbar.
  It rearms only after GP7 has read white for `LINE_JUNCTION_REARM_SAMPLES`
  samples running. A barcode read doesn't block it, because the crossbar
  starts a capture of its own.
- **Mission change:** the junction is disarmed and a partial barcode
  aborted.

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
[line] raw ADC GP26=312 GP27=298 norm 41/38 err=0 | barcode DO GP7=0 (white)
[line] calibration OK: GP26 ok (white 300, rise 2500) GP27 ok (...)
[barcode] 'C' -> RIGHT (dir=rev), stored for the next junction
[barcode] REJECTED: malformed (bad_length)
[line] junction (GP7 black 80 ms) -> nav
```

`LINE_COMMISSIONING_MODE=1` prints the heartbeat every 100 ms instead of
2 s. Nothing prints while a barcode is under the car.

## Tests

```sh
docker compose run --rm build bash -lc "make -C tests/line"
```

| Suite | What it covers |
|---|---|
| `test_ir_cal` | calibration pass and fail (rail, contrast), hysteresis, normalising, both polarities |
| `test_line_track` | error direction, J_n debounce, one event, rearm, noise, barcode hold, noisy loss, stuck sensors |
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
