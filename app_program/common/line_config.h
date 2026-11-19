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
   (tests/test_barcode_decode.c, up to a third).  BARCODE_SPLIT_MAX_RATIO:
   the widest bar (space) at most this many times the narrowest bar
   (space) of the symbol.  0: the one split only, as before. */
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
   the narrowest width seen, whichever is longer (the existing 50 ms /
   4x-narrowest rule).  W_max: widths per read (32 letters).  W_long: any
   single bar or space longer than this aborts the read.  PROVISIONAL until
   measured on the car. */
#define BARCODE_Q_TICKS                (BARCODE_SCAN_END_GAP_US / 1000u)
#define BARCODE_Q_WIDTH_FACTOR                    (4u)
#define BARCODE_W_MAX                             BARCODE_MAX_ELEMENTS
#define BARCODE_W_LONG_TICKS                      (2000u)
#define BARCODE_READ_MAX_TICKS         (BARCODE_SCAN_MAX_US / 1000u)

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
#if (BARCODE_DECODE_SPLIT != 0u) && (BARCODE_DECODE_SPLIT != 1u)
#error "BARCODE_DECODE_SPLIT must be 0 or 1"
#endif

#endif
