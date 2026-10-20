/*
 * buddy3_config.h - every tunable value for Buddy 3 (line + barcode).
 *
 * EDIT THIS FILE FIRST. Items marked TUNE are the bracketed values in the
 * report ([K_min], [L_min], [J_n], [M], [Q], [W_max], [W_long], [R] ...).
 * Set them from your A3 / A7 test data, then copy the final numbers back
 * into Section 6.3 and Section 7 of the report.
 */
#ifndef BUDDY3_CONFIG_H
#define BUDDY3_CONFIG_H

/* ---------- Pins (Section 2 pin map) ---------- */
#define B3_LINE_A_GPIO            6u    /* Left line reader DO */
#define B3_LINE_B_GPIO            8u    /* Centre line reader DO */
#ifndef B3_BARCODE_GPIO
#define B3_BARCODE_GPIO           26u   /* Barcode reader DO */
#endif
#define B3_BATT_GPIO              28u   /* VBAT_SENSE, ADC2 */
#define B3_BATT_ADC_CH            2u

/* ---------- Task identity (Section 3) ---------- */
#define B3_IR_TASK_PRIORITY       10u   /* core 1 */
#define B3_BARCODE_TASK_PRIORITY  14u   /* core 0 */

/* ---------- Timing (NFR-11) ---------- */
#define B3_SAMPLE_PERIOD_MS       1u    /* ir_sense_task runs every 1 ms */
#define B3_BATT_PERIOD_TICKS      1000u /* battery sample every 1 s */

/* Uncomment and set to a SPARE GPIO to measure ir_sense_task WCET with a
 * logic analyser (pin goes high at tick start, low at tick end). */
/* #define B3_PROFILE_PIN         15u */

/* ---------- Sensor polarity (check on the real modules) ---------- */
#ifndef B3_LINE_DARK_IS_HIGH
#ifdef KY033_DARK_LEVEL
#define B3_LINE_DARK_IS_HIGH      KY033_DARK_LEVEL
#else
#define B3_LINE_DARK_IS_HIGH      0     /* Set after testing the DO polarity */
#endif
#endif
#ifndef B3_BARCODE_BAR_LEVEL
#define B3_BARCODE_BAR_LEVEL      B3_LINE_DARK_IS_HIGH
#endif
#define B3_LINE_ERROR_SIGN        1     /* TUNE: +1 = positive error when
                                           line is under sensor B. Agree the
                                           sign convention with Buddy 2. */

/* ---------- ADC ---------- */
#define B3_ADC_MAX                4095u /* 12-bit */
#define B3_ADC_VREF_MV            3300u
#define B3_ADC_RAIL_MARGIN        20u   /* within this of 0 / 4095 = a
                                           disconnected sensor (calibration
                                           fails) */

/* ---------- Calibration (6.3 Calibrating) ---------- */
#define B3_CAL_WINDOW_MS          3000u /* min/max recorded over this window */
#define B3_CAL_MIN_CONTRAST       800u  /* TUNE [K_min], raw ADC counts */

/* ---------- Line tracking (6.3 Tracking) ---------- */
#define B3_NORM_FULL              1000  /* normalised darkness 0..1000 */
#define B3_LINE_MIN_SIGNAL        300   /* TUNE [L_min]: dark_a + dark_b */
#define B3_LINE_LOST_MS           5u    /* TUNE [M] */
#define B3_JUNCTION_DARK          700   /* TUNE: both sensors >= this */
#define B3_JUNCTION_RELEASE       500   /* TUNE: re-arm when either < this */
#define B3_JUNCTION_DEBOUNCE      3u    /* TUNE [J_n], samples (= ms) */

/* ---------- Barcode capture (6.3 Region B) ---------- */
#define B3_BAR_DEBOUNCE_SAMPLES   2u    /* TUNE: equal delay on both edges,
                                           so widths are not affected */
/* [Q]: no edge for this long ends a barcode. It must be LONGER than the
 * widest space INSIDE a barcode at your slowest speed:
 *   Q > 1.5 * (wide width / v_min)
 * and SHORTER than the gap to the junction: Q < (D_bar - length) / v_max. */
#define B3_BAR_GAP_END_MS         250u  /* TUNE [Q] */
#define B3_BAR_MAX_WIDTHS         32u   /* TUNE [W_max]; *A* through *Z* in
                                           Code 39 uses 29 widths */
#define B3_BAR_LONG_MS            400u  /* TUNE [W_long] */

/* ---------- Narrow / wide decision (6.3 Decoding) ----------
 * ratio = width * 100 / shortest width in the same barcode
 *   ratio <= NARROW_MAX          -> narrow
 *   ratio >= WIDE_MIN  ([R])     -> wide
 *   in between                   -> ambiguous, barcode rejected          */
#define B3_RATIO_NARROW_MAX_X100  160u  /* TUNE */
#define B3_RATIO_WIDE_MIN_X100    200u  /* TUNE [R] */
#define B3_RATIO_SEPARATE_BAR_SPACE 1u

/* ---------- Barcode table ---------- */
#define B3_BARCODE_MAX_ELEMENTS   B3_BAR_MAX_WIDTHS
#define B3_BARCODE_MAX_ENTRIES    32u   /* report uses 26 + FINISH */

/* ---------- bar_mbf (Section 4) ---------- */
#define B3_BAR_MBF_RECORDS        64u   /* 2 x W_max */
#define B3_BAR_MBF_BYTES_PER_REC  8u    /* 2-byte record + kernel overhead.
                                           CONFIRM the overhead in your port. */

/* ---------- Battery (pin-table GP28 row) ---------- */
#define B3_BATT_JUMPER_SOLDERED   0     /* TUNE: set 1 ONLY if the VBAT jumper
                                           on the back of the Robo Pico is
                                           soldered. Otherwise GP28 floats. */
#define B3_BATT_DIVIDER           2u    /* GP28 reads VBAT / 2 */

#endif /* BUDDY3_CONFIG_H */
