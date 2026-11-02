/*
 * line_state.h - line_state: the word ir_sense_task (Buddy 3) publishes
 * every 1 ms sample for motion_task (Buddy 2).
 *
 * This is the only file a reader needs.  Read the word with
 * line_state_word() (line_task.h) and take the fields out with the helpers
 * below, never with your own shifts.
 *
 * Layout [PROPOSED, agree with Buddy 2]:
 *
 *   bits  0-1   rl      filtered 0bRL.  Both sensors are closer together
 *                       than the 18 mm line, so: 3 (both black) = centred
 *                       on the line, normal following; 1 = only GP26
 *                       black, 2 = only GP27 black (the line is on that
 *                       side); 0 (both white) = off the line.  The macro
 *                       names LINE_RL_CENTRED (0) and LINE_RL_JUNCTION (3)
 *                       predate this geometry: 3 is NOT a junction.
 *                       Junctions come from GP7 (NAV_FLG_JUNCTION).
 *   bits  2-3   steer   which way to correct: 0 none, 1 left, 2 right.
 *                       Already includes LINE_STEER_INVERT; 0 when
 *                       centred (rl 3) or off the line.
 *   bit   4     barcode a barcode is being read (or was, within
 *                       BARCODE_JUNCTION_HOLDOFF_MS).  Its bars cross the line
 *                       sensors, so rl, steer and error are held at 0: hold
 *                       the heading and a constant speed.
 *   bit   5     lost    no confirmed black for LINE_LOST_MS (noise-filtered)
 *   bit   6     fault   a line sensor is stuck at an ADC rail, or the ADC
 *                       failed: treat the line as lost
 *   bit   7     cal     calibration has passed (0 = configured thresholds)
 *   bits  8-15  counter +1 on every publish, modulo 256.  The same value
 *                       for LINE_ST_STALE_TICKS of motion's 10 ms ticks means
 *                       ir_sense_task has stopped publishing.
 *   bits 16-23  error   line-position error, -100..+100, stored + 128:
 *                       (right - left) normalised darkness / 10, negated
 *                       by LINE_STEER_INVERT (1 on this car: sensors inside
 *                       the line).  POSITIVE = correct LEFT: with the
 *                       invert, the left sensor is darker, the line is on
 *                       the GP26 side (counter-clockwise), the same sign
 *                       as the S.trn turn rate.  Near 0 when centred; 0
 *                       when held.
 *   bits 24-31  dark    time since the last confirmed black, in 10 ms,
 *                       saturating at 255
 *
 * motion_sm inputs: steering from error (or steer); b_line_seen = rl != 0
 * or dark < a few units; b_line_missing = lost or fault or a stale counter;
 * b_line_found = rl != 0 for [N_line] ticks.  Centred reads 11; 00 is off
 * the line, and lost after LINE_LOST_MS of it.
 *
 * One writer (ir_sense_task, core 1) builds the whole word and publishes it
 * with one aligned 32-bit store; motion_task (core 1) reads it with one
 * load (Tribunal 1: one writer, whole-word store).  No bit-fields
 * (Barr-C 5.5); the signed error is offset-encoded.
 */

#ifndef ROBOCAR_LINE_STATE_H
#define ROBOCAR_LINE_STATE_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_ST_RL_SHIFT      (0u)
#define LINE_ST_RL_MASK       (0x3u)
#define LINE_ST_STEER_SHIFT   (2u)
#define LINE_ST_STEER_MASK    (0x3u)
#define LINE_ST_BARCODE_BIT   (1u << 4)
#define LINE_ST_LOST_BIT      (1u << 5)
#define LINE_ST_FAULT_BIT     (1u << 6)
#define LINE_ST_CAL_BIT       (1u << 7)
#define LINE_ST_CTR_SHIFT     (8u)
#define LINE_ST_CTR_MASK      (0xFFu)
#define LINE_ST_ERR_SHIFT     (16u)
#define LINE_ST_ERR_MASK      (0xFFu)
#define LINE_ST_ERR_OFFSET    (128)
#define LINE_ST_ERR_MAX       (100)
#define LINE_ST_DARK_SHIFT    (24u)
#define LINE_ST_DARK_MASK     (0xFFu)
#define LINE_ST_DARK_UNIT_MS  (10u)

/* A reader seeing the same counter this many 10 ms ticks running: stale.
   (One UART console line can hold ir_sense_task for ~7 ms.) */
#define LINE_ST_STALE_TICKS   (5u)

/* rl values */
#define LINE_RL_CENTRED       (0u)
#define LINE_RL_LEFT_DEV      (1u)
#define LINE_RL_RIGHT_DEV     (2u)
#define LINE_RL_JUNCTION      (3u)

/* steer values */
#define LINE_STEER_NONE       (0u)
#define LINE_STEER_LEFT       (1u)
#define LINE_STEER_RIGHT      (2u)

/* Fields of one publish. */
typedef struct {
	uint32_t rl;
	uint32_t steer;
	bool barcode;
	bool lost;
	bool fault;
	bool cal;
	uint32_t counter;
	int32_t error;		/* -100..100 */
	uint32_t dark_ms;
} line_state_fields_t;

/* Before the first publish: lost, line never seen, error 0. */
#define LINE_STATE_INITIAL \
	(LINE_ST_LOST_BIT | ((uint32_t)LINE_ST_ERR_OFFSET << LINE_ST_ERR_SHIFT) | \
	 (LINE_ST_DARK_MASK << LINE_ST_DARK_SHIFT))

static inline uint32_t line_state_pack(const line_state_fields_t *f)
{
	int32_t error = f->error;
	uint32_t dark = f->dark_ms / LINE_ST_DARK_UNIT_MS;

	if (error > LINE_ST_ERR_MAX) {
		error = LINE_ST_ERR_MAX;
	} else if (error < -LINE_ST_ERR_MAX) {
		error = -LINE_ST_ERR_MAX;
	} else {
		/* in range */
	}
	if (dark > LINE_ST_DARK_MASK) {
		dark = LINE_ST_DARK_MASK;
	}
	return ((f->rl & LINE_ST_RL_MASK) << LINE_ST_RL_SHIFT) |
	       ((f->steer & LINE_ST_STEER_MASK) << LINE_ST_STEER_SHIFT) |
	       (f->barcode ? LINE_ST_BARCODE_BIT : 0u) |
	       (f->lost ? LINE_ST_LOST_BIT : 0u) |
	       (f->fault ? LINE_ST_FAULT_BIT : 0u) |
	       (f->cal ? LINE_ST_CAL_BIT : 0u) |
	       ((f->counter & LINE_ST_CTR_MASK) << LINE_ST_CTR_SHIFT) |
	       ((uint32_t)(error + LINE_ST_ERR_OFFSET) << LINE_ST_ERR_SHIFT) |
	       (dark << LINE_ST_DARK_SHIFT);
}

static inline uint32_t line_state_rl(uint32_t word)
{
	return (word >> LINE_ST_RL_SHIFT) & LINE_ST_RL_MASK;
}

static inline uint32_t line_state_steer(uint32_t word)
{
	return (word >> LINE_ST_STEER_SHIFT) & LINE_ST_STEER_MASK;
}

static inline bool line_state_barcode(uint32_t word)
{
	return (word & LINE_ST_BARCODE_BIT) != 0u;
}

static inline bool line_state_lost(uint32_t word)
{
	return (word & LINE_ST_LOST_BIT) != 0u;
}

static inline bool line_state_fault(uint32_t word)
{
	return (word & LINE_ST_FAULT_BIT) != 0u;
}

static inline bool line_state_cal(uint32_t word)
{
	return (word & LINE_ST_CAL_BIT) != 0u;
}

static inline uint32_t line_state_counter(uint32_t word)
{
	return (word >> LINE_ST_CTR_SHIFT) & LINE_ST_CTR_MASK;
}

/* -100..+100, positive = correct left (counter-clockwise). */
static inline int32_t line_state_error(uint32_t word)
{
	return (int32_t)((word >> LINE_ST_ERR_SHIFT) & LINE_ST_ERR_MASK)
	       - LINE_ST_ERR_OFFSET;
}

static inline uint32_t line_state_dark_ms(uint32_t word)
{
	return ((word >> LINE_ST_DARK_SHIFT) & LINE_ST_DARK_MASK)
	       * LINE_ST_DARK_UNIT_MS;
}

/* rl -> steer.  Uninverted (sensors straddling the line): black on GP26
   corrects right, black on GP27 corrects left.  Inverted (this car, both
   sensors inside the line): only GP27 black means the line is on the
   right, so it corrects right, and only GP26 black corrects left. */
static inline uint32_t line_state_steer_for(uint32_t rl, bool invert)
{
	uint32_t steer = LINE_STEER_NONE;

	if (rl == LINE_RL_LEFT_DEV) {
		steer = invert ? LINE_STEER_LEFT : LINE_STEER_RIGHT;
	} else if (rl == LINE_RL_RIGHT_DEV) {
		steer = invert ? LINE_STEER_RIGHT : LINE_STEER_LEFT;
	} else {
		/* centred or junction: no correction */
	}
	return steer;
}

#endif
