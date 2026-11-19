#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

/* Measure over white and at the midpoint of the 18 mm black line before use;
   set midpoint thresholds with a suitable hysteresis gap. */
#define LINE_SENSOR_LEFT_GPIO                    (26u)
#define LINE_SENSOR_LEFT_ADC_CHANNEL             (0u)
#define LINE_SENSOR_LEFT_BLACK_IS_HIGH           (1u)
/* measured 2026-10-29: white 175-300, centred black 2410-2493;
   2026-10-31 (sensors raised): white 150-530, centred black 1250-1330.
   These are only the boot guess and the white/black split for a
   one-colour calibration: once calibrated, the thresholds follow the
   measured levels (see Calibration below). */
#define LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD  (900u)
#define LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD (700u)

#define LINE_SENSOR_RIGHT_GPIO                    (27u)
#define LINE_SENSOR_RIGHT_ADC_CHANNEL             (1u)
#define LINE_SENSOR_RIGHT_BLACK_IS_HIGH           (1u)
#define LINE_SENSOR_RIGHT_BLACK_RISING_THRESHOLD  (900u)
#define LINE_SENSOR_RIGHT_BLACK_FALLING_THRESHOLD (700u)

#define BARCODE_SENSOR_GPIO                       (7u)
/* Verify the digital DO polarity on hardware. */
#define BARCODE_BLACK_IS_HIGH                     (1u)

/* GP28 remains reserved for battery sensing (ADC2). */
#define BATTERY_SENSE_GPIO                        (28u)
#define BATTERY_SENSE_ADC_CHANNEL                 (2u)

#define BARCODE_MAX_CHARACTERS                    (32u)
#define BARCODE_MAX_ELEMENTS                      ((BARCODE_MAX_CHARACTERS * 10u) + 19u)
#define BARCODE_MAX_WIDTHS                        (512u)
#define BARCODE_WIDE_NARROW_MIN_RATIO_X100        (150u)
/* 1 (default): when one narrow/wide split for all nine elements of a
   symbol fails, the decoder tries again with bars and spaces split apart
   (barcode_decode.c).  A sensor that reads every bar wider reads every
   space narrower by the same amount.  The one split fails with wide 2 x
   narrow from about a quarter of the narrow width (0.75 mm on 3 mm
   bars), and with 3 x from about a third (1 mm); split apart, they decode
   (tests/test_barcode_decode.c, up to a third).  No wrong letter in the
   sim sweeps (the checker's 21060 reads; 42120 more, half read
   backwards).  BARCODE_SPLIT_MAX_RATIO: the widest bar (space) at most
   this many times the narrowest bar (space) of the symbol.  This default
   changed without car data.  0: the one split only, as before
   (-DBARCODE_DECODE_SPLIT=0). */
#ifndef BARCODE_DECODE_SPLIT
#define BARCODE_DECODE_SPLIT                      (1u)
#endif
#define BARCODE_SPLIT_MAX_RATIO                   (8u)
#define BARCODE_GAP_MAX_NARROW_RATIO_X100         (300u)
#define BARCODE_SCAN_END_GAP_US                   (50000u)
#define BARCODE_SCAN_IDLE_US                      BARCODE_SCAN_END_GAP_US
#define BARCODE_SCAN_MAX_US                       (10000000u)

/* ---- Barcode capture in ir_sense_task, in 1 ms sampling ticks ----
   Q: a read ends after this many white ticks, or BARCODE_Q_WIDTH_FACTOR x
   the narrowest width seen, whichever is longer.  W_max: widths per read
   (32 letters).  W_long: any single bar or space longer than this aborts
   the read.  PROVISIONAL until measured on the car.
   Factor 6 (was 4): a wide space is 2-3 narrow modules, and a sensor that
   reads bars 0.5 mm wide (spaces 0.5 mm short) with GP7's speed 10 % off
   makes the course's 9 mm wide space 4.2 times its 2.5 mm narrow space
   (C), so 4 x narrowest ended the read inside the barcode (J3).  Sim, the
   course sheet (3 mm, 3:1) at base 6 with GP7's speed +/-10 % and the
   bars 0.5 mm off: 4 decodes 7 and 58 of 104 reads, 6 all 104.  A valid
   read no longer waits for Q (BARCODE_END_ON_STOP), so the longer Q only
   delays the end of a read that does not decode.
   The cost of 6: a GP7 black that does not decode (a brush of the line,
   a seam, a sheet that fails, the crossbar in a rule 0 image) keeps its
   read open 6 x its width instead of 4 x, and with LINE_BARCODE_HOLD 1
   the steering hold lasts as long (the width, Q, then
   BARCODE_JUNCTION_HOLDOFF_MS 300): for the 124 ms blip of 4 Nov
   (trials_batch1_2026-11-04.txt:2559) 1168 ms blind instead of 920 ms,
   about 250 ms or 12 mm more at base 6 (C).  Before the H1 fix it also
   widened the stretch in which a failed read swallowed the crossbar (18
   mm instead of 12 for 3 mm bars, C); now that crossbar still fires
   (BARCODE_NOT_BAR_RATIO_X10).  Re-run with the H1 fix (sim, the 63180
   reads of the size envelope there): factor 4 decodes 33679 and fires 1132 junctions inside barcodes
   (reads end at a wide space, and the rest of the barcode opens a read
   of its own); 6 decodes 49217 with none.  So 6 stays.  This default
   changed without car data.  -DBARCODE_Q_WIDTH_FACTOR=4 builds the old
   value. */
#define BARCODE_Q_TICKS                (BARCODE_SCAN_END_GAP_US / 1000u)
#ifndef BARCODE_Q_WIDTH_FACTOR
#define BARCODE_Q_WIDTH_FACTOR                    (6u)
#endif
#define BARCODE_W_MAX                             BARCODE_MAX_ELEMENTS
#define BARCODE_W_LONG_TICKS                      (2000u)
#define BARCODE_READ_MAX_TICKS         (BARCODE_SCAN_MAX_US / 1000u)

/* Glitch filter (bar_capture.c): every GP7 sample goes through it before
   the capture and, with LINE_JUNCTION_RULE 1, before the junction side.
   A black or white run this many samples or shorter counts as the level
   either side of it.  GP7 drops out inside a black run: "B63 w4 B24", 4
   ms of white inside 91 ms of black, on 4 Nov
   (tests/scan_bench/logs/trials_batch2_2026-11-04.txt:196).  6 merges
   that logged 4 ms dropout (3 did not: the checker, 19 Nov) and is still
   below the narrowest real bar or space: 2 mm at base 8 is 31 ms, and
   1 mm (a 2 mm bar read 1 mm thin) with GP7 20 % fast is 13 ms (C).  The
   checker's size envelope decodes the same at 3 and 6.  Every edge is
   seen this many samples late (0.4 mm at base 8, C); widths are
   unchanged.  A longer dropout is left to "not a bar" below.  0: no
   filter, as before (-DBARCODE_GLITCH_TICKS=0). */
#ifndef BARCODE_GLITCH_TICKS
#define BARCODE_GLITCH_TICKS                      (6u)
#endif

/* Not a bar (bar_capture.c; it matters with LINE_JUNCTION_RULE 1).
   Before, no black run inside an open read with 1-28 widths was a
   junction, so a crossbar that started inside one never fired, armed or
   not (H1, checker, 19 Nov): after a dropout inside the crossbar longer
   than the filter, a speck or sheet edge in the quiet zone before it, or
   an unreadable sheet close before it.  Now a black run inside an open
   read is not a bar once it is longer than BARCODE_NOT_BAR_RATIO_X10 / 10
   times the widest width of the read before the space in front of it
   (that space may be the gap to the crossbar, any length up to the quiet
   zone, so it is left out).  The junction rules then apply to the run as
   to any other (armed: LINE_ARMED_CONFIRM_MM; otherwise J_n), and when a
   junction fires the read ENDs there: barcode_task judges and prints the
   widths before the crossbar (malformed), and the rest of the crossbar
   opens no read.
   Known limit (checker, 19 Nov): a real course sheet that fails to
   decode because widths were lost (a space taped over, a bar not seen,
   bars merged by blur) still has a 9 mm wide element, so the threshold
   is 2.5 x 9 = 22.5 mm, longer than the 18 mm crossbar: a crossbar 0-15
   mm behind such a sheet is swallowed (0 of 26 letters fire in the sim,
   as before this fix; rule 0 fires).  Armed, the command then runs at
   the next junction; unarmed there is no junction event.  Fixed only for
   reads of equal-width bars (a speck, a dropout, merged small bars).
   2.5 (the checker's figure) from a read's 11th width on: by then the
   widths before the last space include a wide one (from any bar of *X*,
   either way round, at most 8 narrow elements come before a wide one, C),
   and no bar long enough for either rule (7.1 mm: J_n at base 4 with GP7
   20 % slow, C) is more than 2.1 times a wide element (wide bar 1 mm
   wider, wide space 1 mm narrower, GP7's speed +/-20 %, C).
   Before the 11th width, BARCODE_NOT_BAR_EARLY_X10, 7: the widest width
   so far may be a narrow one and the run a wide bar (the reversed start
   character's wide bar is its 3rd width), up to 6.3 times a narrow bar
   (3 x, the bars 1 mm thin, GP7 +/-20 %, C).  So a speck before the
   crossbar must be under 18 / 7 = 2.6 mm (C), and the junction then
   fires 7 x the speck into the crossbar: 14 mm for a 2 mm speck (host
   test).  Except the 3rd width after a dropout, a space under
   1 / BARCODE_DROPOUT_DIV of the bar before it, which no barcode's space
   is (at least 0.22 of it: a 2 mm bar read 3 mm wide, its space 1 mm,
   GP7 +/-20 %, C): there the ratio is 2.5.  Armed, a 4 ms dropout up to
   5 mm into the crossbar fires (sim, base 4/6/8); later ones leave less
   than 10 mm after them.  By the length rule, up to 4-5 mm at base 4 and
   6, and never at base 8, where J_n is the whole crossbar (C).
   Sim, the size envelope (63180 reads: narrow 2-6 mm, wide 2-3 x, base
   4/6/8, GP7's speed +/-0/10/20 %, blur -1..+1 mm, 26 letters), 42120
   more with half of them read backwards, and two sheets before one
   junction: the same reads decode and 0 junctions fire inside barcodes,
   with and without it.  0: off, as before
   (-DBARCODE_NOT_BAR_RATIO_X10=0). */
#ifndef BARCODE_NOT_BAR_RATIO_X10
#define BARCODE_NOT_BAR_RATIO_X10                 (25u)
#endif
#ifndef BARCODE_NOT_BAR_EARLY_X10
#define BARCODE_NOT_BAR_EARLY_X10                 (70u)
#endif
#ifndef BARCODE_DROPOUT_DIV
#define BARCODE_DROPOUT_DIV                       (5u)
#endif

/* Compiled-in letter -> movement table, used until a valid map arrives
   (MQTT_SCHEMA.md 8.3).  The letters are only given on the day, so it is
   off: until a map is loaded every letter is rejected as unknown and no
   junction command is sent.  To use one, set _SET to 1
   and give 26 characters, 'A' first: F forward, L left, R right, U U-turn,
   e.g. "LRFUFFFFFFFFFFFFFFFFFFFFFF" for the writeup's example (A left,
   B right, C straight, D U-turn).  For a car image edit the two lines
   here: an in-place rebuild does not recompile on a changed -D.  The host
   tests set them with -D (tests/line/Makefile).  A wrong length stops the
   build; any character other than F, L, R, U leaves the table empty, and
   the boot line says so (docs/BUDDY3_LINE_BARCODE.md, "Commands on the
   day"). */
#ifndef BARCODE_DEFAULT_TABLE_SET
#define BARCODE_DEFAULT_TABLE_SET                 (0)
#endif
#ifndef BARCODE_DEFAULT_TABLE
#define BARCODE_DEFAULT_TABLE                     ""
#endif
#if (BARCODE_DEFAULT_TABLE_SET != 0) && (BARCODE_DEFAULT_TABLE_SET != 1)
#error "BARCODE_DEFAULT_TABLE_SET must be 0 or 1"
#endif

/* Barcode hold.  1 (default, today's behaviour): while a barcode is being
   read, and BARCODE_JUNCTION_HOLDOFF_MS after, line_state holds
   rl/steer/error at 0 and sets its barcode bit, and motion_task holds its
   heading.  0: nothing is held.  The hold was written for bars that cross
   the line sensors; on the demo course the sheets sit beside the line (the
   prof's course photos, 19 Nov), so GP26/GP27 never see a bar.  Which
   setting is better there is NOT decided yet:
   - holding keeps the heading, so GP7 passes the bars at a constant
     speed, which the decoder needs: with GP7's speed varying +/-20 %
     today's decoder decodes 79-91 of 208 reads, with +/-10 % all 208
     (sim);
   - not holding avoids blind straight driving: up to ~3.4 s (~158 mm) per
     sheet, and ~2.2 s per crossbar, because the crossbar under GP7 opens a
     read too (sim).
   Avani and Hanis choose after the car session C1 A/B; build the other
   side with -DLINE_BARCODE_HOLD=0.  Held or not, the dark time and the
   lost bit follow the real line sensors (line_track.c), so a held car on
   white is not "line seen" and is lost after LINE_LOST_MS. */
#ifndef LINE_BARCODE_HOLD
#define LINE_BARCODE_HOLD                         (1u)
#endif

/* How long the hold lasts after a read ends (LINE_BARCODE_HOLD 1 only).
   It no longer gates junctions: those come from GP7's run length (below). */
#define BARCODE_JUNCTION_HOLDOFF_MS               (300u)

/* ---- Junction detection (ticks of 1 ms) ----
   GP26 and GP27 both sit inside the 18 mm line, so both read black (rl 11)
   whenever the car is centred, and a cross-junction looks the same to them:
   they cannot see one.  The junction comes from the barcode sensor (GP7)
   instead: a crossbar is one black run far longer than any barcode bar.
   J_n: GP7 black this many samples running = one junction.  It must be
   above the widest barcode bar and below the crossbar at the speed GP7
   passes them, taken as motion's line-following base (MOTION_LF_BASE,
   motion_cfg.h), so J_n must move with MOTION_LF_BASE: change both
   together.  Nothing in the firmware checks this; only the host test
   does, with the default headers.
   Base 6 is 47.6 mm/s (C: 6 counts a 10 ms tick, 64 mm wheel, 2535 counts
   a turn; INTEGRATION.md S4 gives ~4.7 cm/s from lap times).  There the
   course's 9 mm wide bar is 189 ms and the 18 mm crossbar 378 ms (C), and
   280 sits about midway.  The old 80 was sized for ~12 cm/s: at base 6 it
   is 3.8 mm of black, so every wide bar fired a junction (6 per sheet,
   sim) that motion accepts in LINE_FOLLOW.
   280 at a constant GP7 speed, bars and crossbar -2..+2 mm off their
   printed width: at base 6 no bar fires and every crossbar does (sim,
   host test).  At base 8 no bar fires either, but the crossbar is only
   284 ms (C), so one seen 1 mm or more narrow is missed; at base 4 a wide
   bar is 284 ms (C) and fires (host test).
   These margins hold only while GP7 moves at about the base speed: a 9 mm
   bar fires if GP7 is slower than ~32 mm/s, an 18 mm crossbar is missed
   if it is faster than ~64 mm/s (C).  Steering moves GP7's speed: at the
   41 deg/s logged in line following (4 Nov, trials_batch1 line 2554), by
   about +/-32 mm/s if GP7 is 45 mm to the side of the centreline (C; the
   45 mm is assumed, the mount is not measured).  So the gate for 280
   is the car session C1 count of junction lines per sheet and per
   crossbar.  Rearm: GP7 white this many samples running first.
   PROVISIONAL; -DLINE_JUNCTION_SAMPLES=80 builds the old value. */
#ifndef LINE_JUNCTION_SAMPLES
#define LINE_JUNCTION_SAMPLES                     (280u)
#endif
#define LINE_JUNCTION_REARM_SAMPLES               (20u)

/* ---- The barcode-armed junction (the team's rule, 19 Nov) ----
   The course puts a barcode sheet before or at each junction, with bars
   of different sizes, so no single length J_n can tell every bar from the
   crossbar.  1 (default): once GP7 has read a complete, valid barcode and
   barcode_task has stored its command, the next black run GP7 sees is the
   junction, whatever its length (junction_arm.h).  While the bars of a
   read arrive, nothing is a junction.  With no barcode armed, the length
   rule above is the fallback.  This default changed without car data
   (Hanis's decision, 19 Nov), and its three values below are guesses,
   none measured, to confirm on the car: 10 mm (LINE_ARMED_CONFIRM_MM),
   150 mm (LINE_ARMED_HOLD_MM) and 50 ms (LINE_ARMED_VERDICT_WAIT_MS).
   0: the length rule alone, as before (-DLINE_JUNCTION_RULE=0, for the
   A/B).  The rule needs BARCODE_END_ON_STOP 1.
   Every switch back to the behaviour before these changes:
   -DLINE_JUNCTION_RULE=0 -DBARCODE_END_ON_STOP=0 -DBARCODE_Q_WIDTH_FACTOR=4
   -DBARCODE_DECODE_SPLIT=0 -DBARCODE_GLITCH_TICKS=0
   -DBARCODE_NOT_BAR_RATIO_X10=0 (tests/line OLD_RULE).
   A -D car image needs a clean tree and make GCC="arm-none-eabi-gcc
   -D...": CFLAGS is set with := (build_make/pico_rp2040.mk:257). */
#ifndef LINE_JUNCTION_RULE
#define LINE_JUNCTION_RULE                        (1u)
#endif

/* 1 (default): a one-letter read ENDs as soon as the stop character's
   last bar ends, if its 29 widths decode as a valid *X*, instead of
   after the quiet zone.  A crossbar closer behind the sheet than the
   quiet zone (BARCODE_Q_WIDTH_FACTOR x the narrowest width: 18 mm for
   the 3 mm bars, C) no longer joins the read and spoils it, and the
   command is stored before the car reaches the crossbar.  This default
   changed without car data.  0: always wait for the quiet zone, as before
   (-DBARCODE_END_ON_STOP=0, with -DLINE_JUNCTION_RULE=0). */
#ifndef BARCODE_END_ON_STOP
#define BARCODE_END_ON_STOP                       (1u)
#endif

/* Armed: the black run is the junction once it has lasted
   LINE_ARMED_MIN_SAMPLES samples (noise) and LINE_ARMED_CONFIRM_MM of
   travel (distance_state, the wheels' average).  10 mm is longer than the
   first bar of any barcode in the size range (every Code 39 barcode
   starts with a narrow bar, at most 6 mm printed, so a second sheet
   before the same junction does not fire), and shorter than the 18 mm
   crossbar, also with the bars read 1 mm off and GP7's speed 20 % off the
   car's (sim, tests/line).  Where the junction fires, which
   MOTION_TURN_FWD_MM counts from: about 10 mm into the crossbar at base 6
   and 8 (J_n 280 is 13.3 and 17.8 mm there, C); at base 4 J_n 280 is 8.9
   mm (C), so the length rule fires first, and the armed junction line
   prints "GP7 black 8-9 mm" (host test: 8.9-10.3 mm at a steady GP7
   speed).  With GP7's speed +/-0-20 % off the car's and blur +/-1 mm, by
   either rule: 7.6-10.3 mm at base 4, 7.8-15.5 at base 6, 7.4-19.1 at
   base 8 (sim, the size envelope under BARCODE_NOT_BAR_RATIO_X10).  A
   speck or dropout before or in the crossbar can move it later, up to the
   crossbar's end.  PROVISIONAL: 10 mm is a guess to confirm on the car.
   0 mm: the first LINE_ARMED_MIN_SAMPLES of black fire. */
#ifndef LINE_ARMED_MIN_SAMPLES
#define LINE_ARMED_MIN_SAMPLES                    (5u)
#endif
#ifndef LINE_ARMED_CONFIRM_MM
#define LINE_ARMED_CONFIRM_MM                     (10u)
#endif

/* D_hold (week6_design_review.md, assumption 2): an armed junction is
   disarmed after this much travel with no junction (the stored command
   stays for nav; only the junction falls back to the length rule).
   150 mm is a GUESS, not measured, to confirm on the car: it must be
   longer than the gap from the last bar to the crossbar on the course. */
#ifndef LINE_ARMED_HOLD_MM
#define LINE_ARMED_HOLD_MM                        (150u)
#endif

/* After a complete read, a junction that is due waits at most this long
   (ms) for barcode_task (core 0) to judge the read and store its command,
   so nav finds it.  barcode_task wakes on the message; 50 ms is 2.4 mm at
   base 6 (C).  50 ms is a GUESS (barcode_task's real latency on the car
   is not measured), to confirm on the car. */
#ifndef LINE_ARMED_VERDICT_WAIT_MS
#define LINE_ARMED_VERDICT_WAIT_MS                (50u)
#endif

/* ---- Line loss: no confirmed black for this long (ms) = lost ---- */
#define LINE_LOST_MS                              (1500u)

/* ---- Calibration (CAL_REQ from nav_ctrl_task), ADC counts ----
   IR_CAL_SAMPLES ticks are recorded per sensor with the car still.  If a
   sensor saw both black and white (span >= IR_CAL_MIN_CONTRAST), those are
   its levels.  If it saw one colour only, the thresholds above decide which:
   black (the start pose: centred, both sensors inside the line) must sit at
   least IR_CAL_WHITE_MARGIN above the rising one, white as far below the
   falling one; the missing level is guessed (white = IR_CAL_WHITE_GUESS_PCT
   of black, or black = white + 2 x IR_CAL_MIN_CONTRAST).  A sensor between
   the two while the other clearly saw black is on the line edge (car not
   quite centred): it passes as "edge" from the defaults and learns its
   levels while driving (ir_cal_finish_pair).
   Thresholds are always the level midpoint +/- IR_CAL_HYST_PCT of the
   span.  From then on, every sample moves its colour's level 1/2^
   IR_ADAPT_SHIFT of the way towards it (2048 samples, about 2 s), so the
   thresholds follow sensor height, tape and floor; a "white" sample far
   darker than white pulls black down 2^IR_ADAPT_STRANDED_SHIFT times
   slower.  The levels never come closer than IR_CAL_MIN_CONTRAST.
   A sensor reading at an ADC rail (<= IR_RAIL_LOW or >= IR_RAIL_HIGH) for
   most of the window, or for IR_RAIL_FAULT_SAMPLES running in use, is
   stuck.  PROVISIONAL. */
#define IR_CAL_SAMPLES                            (1000u)
#define IR_CAL_MIN_CONTRAST                       (400u)
#define IR_CAL_HYST_PCT                           (10u)
#define IR_CAL_WHITE_MARGIN                       (100u)
#define IR_CAL_WHITE_GUESS_PCT                    (25u)
#define IR_ADAPT_SHIFT                            (11u)
#define IR_ADAPT_STRANDED_SHIFT                   (2u)
#define IR_RAIL_LOW                               (2u)
#define IR_RAIL_HIGH                              (4093u)
#define IR_RAIL_FAULT_SAMPLES                     (200u)

/* Compatibility names retained for the other board-configuration users. */
#define LINE_LEFT_GPIO                            LINE_SENSOR_LEFT_GPIO
#define LINE_RIGHT_GPIO                           LINE_SENSOR_RIGHT_GPIO
#define LINE_BLACK_IS_HIGH                        LINE_SENSOR_LEFT_BLACK_IS_HIGH
#define BARCODE_GPIO                              BARCODE_SENSOR_GPIO
#define KY033_DARK_LEVEL                          BARCODE_BLACK_IS_HIGH
#define BARCODE_SENSOR_AVAILABLE                  (1u)

#define MOTOR_DRIVER_GPIO_FIRST                   (8u)
#define MOTOR_DRIVER_GPIO_LAST                    (11u)

#if ((LINE_SENSOR_LEFT_GPIO >= MOTOR_DRIVER_GPIO_FIRST) && \
     (LINE_SENSOR_LEFT_GPIO <= MOTOR_DRIVER_GPIO_LAST)) || \
    ((LINE_SENSOR_RIGHT_GPIO >= MOTOR_DRIVER_GPIO_FIRST) && \
     (LINE_SENSOR_RIGHT_GPIO <= MOTOR_DRIVER_GPIO_LAST))
#define LINE_MOTOR_PIN_CONFLICT (1u)
#warning "A line sensor GPIO overlaps the motor-driver GPIO range"
#else
#define LINE_MOTOR_PIN_CONFLICT (0u)
#endif

#if (LINE_SENSOR_LEFT_GPIO > 29u) || (LINE_SENSOR_RIGHT_GPIO > 29u) || \
    (BARCODE_SENSOR_GPIO > 29u) || (BATTERY_SENSE_GPIO > 29u)
#error "Configured RP2040 GPIO numbers must be in the range 0..29"
#endif
#if LINE_SENSOR_LEFT_GPIO == LINE_SENSOR_RIGHT_GPIO
#error "The left and right line sensors must use distinct GPIOs"
#endif
#if (BARCODE_SENSOR_GPIO == LINE_SENSOR_LEFT_GPIO) || \
    (BARCODE_SENSOR_GPIO == LINE_SENSOR_RIGHT_GPIO)
#error "The barcode GPIO must not duplicate a line-sensor GPIO"
#endif
#if LINE_SENSOR_LEFT_GPIO != (26u + LINE_SENSOR_LEFT_ADC_CHANNEL)
#error "The left line GPIO and ADC channel do not match"
#endif
#if LINE_SENSOR_RIGHT_GPIO != (26u + LINE_SENSOR_RIGHT_ADC_CHANNEL)
#error "The right line GPIO and ADC channel do not match"
#endif
#if BATTERY_SENSE_GPIO != (26u + BATTERY_SENSE_ADC_CHANNEL)
#error "The battery-sense GPIO and ADC channel do not match"
#endif
#if (LINE_BARCODE_HOLD != 0u) && (LINE_BARCODE_HOLD != 1u)
#error "LINE_BARCODE_HOLD must be 0 or 1"
#endif
#if (LINE_JUNCTION_RULE != 0u) && (LINE_JUNCTION_RULE != 1u)
#error "LINE_JUNCTION_RULE must be 0 or 1"
#endif
#if (BARCODE_DECODE_SPLIT != 0u) && (BARCODE_DECODE_SPLIT != 1u)
#error "BARCODE_DECODE_SPLIT must be 0 or 1"
#endif
#if (BARCODE_END_ON_STOP != 0u) && (BARCODE_END_ON_STOP != 1u)
#error "BARCODE_END_ON_STOP must be 0 or 1"
#endif
#if (LINE_JUNCTION_RULE != 0u) && (BARCODE_END_ON_STOP == 0u)
#error "LINE_JUNCTION_RULE 1 needs BARCODE_END_ON_STOP 1"
#endif
#if (LINE_ARMED_MIN_SAMPLES < 1u) || (LINE_ARMED_VERDICT_WAIT_MS < 1u)
#error "LINE_ARMED_MIN_SAMPLES and LINE_ARMED_VERDICT_WAIT_MS must be at least 1"
#endif
#if BARCODE_GLITCH_TICKS > 10u
#error "BARCODE_GLITCH_TICKS must be 0..10: the narrowest real bar is about 13 ms"
#endif
#if (BARCODE_NOT_BAR_RATIO_X10 != 0u) && \
    ((BARCODE_NOT_BAR_RATIO_X10 < 20u) || (BARCODE_NOT_BAR_EARLY_X10 < BARCODE_NOT_BAR_RATIO_X10) || \
     (BARCODE_DROPOUT_DIV < 1u))
#error "BARCODE_NOT_BAR_RATIO_X10 must be 0 or at least 20, _EARLY_X10 at least as big, BARCODE_DROPOUT_DIV at least 1"
#endif

#endif
