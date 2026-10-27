#ifndef ROBOCAR_ROBOT_CONFIG_H
#define ROBOCAR_ROBOT_CONFIG_H

/* Buddy 3 line / barcode board settings: pins, ADC thresholds, and
 * compile-time behaviour flags for line_task.
 *
 * SAMPLE: written by Buddy 4 so the firmware builds after line_task was
 * merged.  The values come from docs/BUDDY3_LINE_BARCODE.md; anything
 * the doc doesn't give is marked [GUESS].  Buddy 3 to check every value
 * against the real board and replace this file with theirs.
 *
 * Every value is #ifndef-guarded so a build can override it with -D. */

/* ---- Line sensors (digital) ---------------------------------------- */

/* GP8 is also M1 (left motor) in motor_task, GP8-GP11.  line_task sets
 * GP8 to a SIO input, which takes it away from the PWM.  Pick another
 * pin before line_task runs alongside motor_task. */
#ifndef LINE_LEFT_GPIO
#define LINE_LEFT_GPIO			8u
#endif
#ifndef LINE_RIGHT_GPIO
#define LINE_RIGHT_GPIO			6u
#endif

/* 1 = a HIGH pin level means black. */
#ifndef LINE_BLACK_IS_HIGH
#define LINE_BLACK_IS_HIGH		1u
#endif

/* Task loop delay, in kernel ticks (1 tick = 1 ms on RP2040). */
#ifndef LINE_SAMPLE_PERIOD_MS
#define LINE_SAMPLE_PERIOD_MS		1u
#endif

/* Samples a new line state must hold before it is accepted; 00 (no
 * line) needs more so a short gap isn't reported as lost. */
#ifndef LINE_CONFIRM_SAMPLES
#define LINE_CONFIRM_SAMPLES		6u
#endif
#ifndef LINE_LOST_SAMPLES
#define LINE_LOST_SAMPLES		8u
#endif

/* Raw-pin heartbeat print period. */
#ifndef LINE_HEARTBEAT_MS
#define LINE_HEARTBEAT_MS		2000u
#endif

/* 1 = report 00 as LOST / STOP instead of STRAIGHT / DRIVE. */
#ifndef LINE_STOP_ON_LOST
#define LINE_STOP_ON_LOST		0
#endif

/* 1 = swap the STEER_LEFT / STEER_RIGHT suggestions. */
#ifndef LINE_STEER_INVERT
#define LINE_STEER_INVERT		0
#endif

/* 1 = at boot, sample white then black for 5 s each and print the pins. */
#ifndef LINE_COMMISSIONING_MODE
#define LINE_COMMISSIONING_MODE		0
#endif
#ifndef LINE_COMMISSION_SAMPLE_PERIOD_MS
#define LINE_COMMISSION_SAMPLE_PERIOD_MS	100u	/* [GUESS] */
#endif
#ifndef LINE_COMMISSION_SAMPLE_COUNT
#define LINE_COMMISSION_SAMPLE_COUNT	50u	/* 50 x 100 ms = 5 s */
#endif

/* ---- Barcode sensor (analog) --------------------------------------- */

#ifndef BARCODE_GPIO
#define BARCODE_GPIO			26u	/* GP26 = ADC0 */
#endif
#ifndef BARCODE_ADC_CHANNEL
#define BARCODE_ADC_CHANNEL		0u
#endif

/* ADC readings averaged per sample. [GUESS] */
#ifndef BARCODE_ADC_SAMPLES
#define BARCODE_ADC_SAMPLES		4u
#endif

/* Hysteresis on the 12-bit ADC: black ~4045, white ~135 measured. */
#ifndef BARCODE_BLACK_RISING_THRESHOLD
#define BARCODE_BLACK_RISING_THRESHOLD	3000u
#endif
#ifndef BARCODE_WHITE_FALLING_THRESHOLD
#define BARCODE_WHITE_FALLING_THRESHOLD	1000u
#endif
#ifndef BARCODE_BLACK_IS_HIGH
#define BARCODE_BLACK_IS_HIGH		1u
#endif

/* An unfinished scan ends after this much white (or 4x the narrowest
 * run, whichever is longer), and after BARCODE_SCAN_MAX_US in total. */
#ifndef BARCODE_SCAN_IDLE_US
#define BARCODE_SCAN_IDLE_US		50000ULL
#endif
#ifndef BARCODE_SCAN_MAX_US
#define BARCODE_SCAN_MAX_US		10000000ULL
#endif

/* Bar/space widths captured per scan, and decoded letters kept. */
#ifndef BARCODE_MAX_WIDTHS
#define BARCODE_MAX_WIDTHS		512u
#endif
#ifndef BARCODE_MAX_CHARACTERS
#define BARCODE_MAX_CHARACTERS		48u
#endif

/* ---- Reserved ------------------------------------------------------ */

/* Battery sense, not used yet. */
#ifndef BATTERY_GPIO
#define BATTERY_GPIO			28u	/* GP28 = ADC2 */
#endif
#ifndef BATTERY_ADC_CHANNEL
#define BATTERY_ADC_CHANNEL		2u
#endif

#endif /* ROBOCAR_ROBOT_CONFIG_H */
