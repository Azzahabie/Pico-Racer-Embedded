/*
 * ir_cal.c - per-sensor calibration, classification and normalisation.
 */

#include "ir_cal.h"

static uint16_t darkness(bool black_is_high, uint16_t raw)
{
	if (raw > IR_ADC_MAX) {
		raw = IR_ADC_MAX;
	}
	return black_is_high ? raw : (uint16_t)(IR_ADC_MAX - raw);
}

/* Levels in x256 fixed point, so a 1-count change survives the averaging */
#define IR_LEVEL_Q	(8u)

/* rise/fall from the levels: the midpoint +/- IR_CAL_HYST_PCT of the span */
static void set_thresholds(ir_cal_sensor_t *cal)
{
	uint32_t white = cal->white_q >> IR_LEVEL_Q;
	uint32_t black = cal->black_q >> IR_LEVEL_Q;
	uint32_t span = black - white;
	uint32_t mid = white + span / 2u;
	uint32_t hyst = (span * IR_CAL_HYST_PCT) / 100u;

	cal->white = (uint16_t)white;
	cal->black = (uint16_t)black;
	cal->rise = (uint16_t)(mid + hyst);
	cal->fall = (uint16_t)(mid - hyst);
}

static void set_levels(ir_cal_sensor_t *cal, uint32_t white, uint32_t black)
{
	cal->white_q = white << IR_LEVEL_Q;
	cal->black_q = black << IR_LEVEL_Q;
	set_thresholds(cal);
}

ir_cal_sensor_t ir_cal_default(bool black_is_high, uint16_t rise_raw,
			       uint16_t fall_raw)
{
	ir_cal_sensor_t cal;
	uint16_t rise = darkness(black_is_high, rise_raw);
	uint16_t fall = darkness(black_is_high, fall_raw);
	uint32_t mid;

	/* line_config.h names them for black-is-high; flip for the other */
	if (rise < fall) {
		uint16_t swap = rise;
		rise = fall;
		fall = swap;
	}
	mid = ((uint32_t)rise + fall) / 2u;
	cal.black_is_high = black_is_high;
	cal.rise = rise;
	cal.fall = fall;
	/* the midpoint convention: threshold halfway between white and black */
	cal.white = 0u;
	cal.black = (uint16_t)(2u * mid);
	cal.white_q = (uint32_t)cal.white << IR_LEVEL_Q;
	cal.black_q = (uint32_t)cal.black << IR_LEVEL_Q;
	return cal;
}

void ir_cal_begin(ir_cal_acc_t *acc, bool black_is_high)
{
	acc->min_dark = UINT16_MAX;
	acc->max_dark = 0u;
	acc->samples = 0u;
	acc->rail_samples = 0u;
	acc->black_is_high = black_is_high;
}

bool ir_at_rail(uint16_t raw)
{
	return (raw <= IR_RAIL_LOW) || (raw >= IR_RAIL_HIGH);
}

void ir_cal_add(ir_cal_acc_t *acc, uint16_t raw)
{
	uint16_t dark;

	acc->samples++;
	if (ir_at_rail(raw)) {
		acc->rail_samples++;
		return;		/* a rail reading says nothing about the surface */
	}
	dark = darkness(acc->black_is_high, raw);
	if (dark < acc->min_dark) {
		acc->min_dark = dark;
	}
	if (dark > acc->max_dark) {
		acc->max_dark = dark;
	}
}

ir_cal_result_t ir_cal_finish(const ir_cal_acc_t *acc,
			      const ir_cal_sensor_t *defaults,
			      ir_cal_sensor_t *out)
{
	ir_cal_sensor_t cal = *defaults;
	uint32_t span;

	if (acc->samples == 0u) {
		return IR_CAL_NO_SAMPLES;
	}
	if ((acc->rail_samples * 2u) > acc->samples) {
		return IR_CAL_RAIL_STUCK;
	}
	span = (uint32_t)acc->max_dark - acc->min_dark;
	if (span >= IR_CAL_MIN_CONTRAST) {
		/* both colours seen: thresholds from the measured midpoint */
		set_levels(&cal, acc->min_dark, acc->max_dark);
	} else if (((uint32_t)acc->max_dark + IR_CAL_WHITE_MARGIN) <=
		   defaults->fall) {
		/* one colour, white (sensors off the line): black is guessed a
		   safe step above it, and found properly once driving */
		set_levels(&cal, acc->min_dark,
			   (uint32_t)acc->min_dark + (2u * IR_CAL_MIN_CONTRAST));
	} else if ((uint32_t)acc->min_dark >=
		   ((uint32_t)defaults->rise + IR_CAL_WHITE_MARGIN)) {
		/* one colour, black: the normal start pose, centred with both
		   sensors inside the 18 mm line.  White is guessed as a fraction
		   of black (both scale with the sensor height), and found
		   properly the first time a sensor leaves the line */
		set_levels(&cal,
			   ((uint32_t)acc->max_dark * IR_CAL_WHITE_GUESS_PCT) / 100u,
			   acc->max_dark);
	} else {
		/* one colour, between the thresholds: neither clearly */
		return IR_CAL_LOW_CONTRAST;
	}
	*out = cal;
	return IR_CAL_OK;
}

/* One step of a running average, 1 / 2^shift of the way to dark */
static void follow(uint32_t *level_q, uint16_t dark, uint32_t shift)
{
	int32_t diff = (int32_t)((uint32_t)dark << IR_LEVEL_Q) - (int32_t)*level_q;

	*level_q = (uint32_t)((int32_t)*level_q + (diff / (int32_t)(1u << shift)));
}

void ir_cal_adapt(ir_cal_sensor_t *cal, uint16_t raw, bool black)
{
	uint32_t white_q = cal->white_q;
	uint32_t black_q = cal->black_q;
	uint16_t dark;

	if (ir_at_rail(raw)) {
		return;
	}
	dark = darkness(cal->black_is_high, raw);
	if (black) {
		follow(&black_q, dark, IR_ADAPT_SHIFT);
	} else if ((uint32_t)dark >= ((uint32_t)cal->white + IR_CAL_MIN_CONTRAST)) {
		/* "white", but far darker than white: black reads lower than
		   the thresholds expect */
		follow(&black_q, dark, IR_ADAPT_SHIFT + IR_ADAPT_STRANDED_SHIFT);
	} else {
		follow(&white_q, dark, IR_ADAPT_SHIFT);
	}
	if ((black_q >> IR_LEVEL_Q) <
	    ((white_q >> IR_LEVEL_Q) + IR_CAL_MIN_CONTRAST)) {
		return;		/* too close to tell apart: keep the last levels */
	}
	cal->white_q = white_q;
	cal->black_q = black_q;
	set_thresholds(cal);
}

const char *ir_cal_result_name(ir_cal_result_t result)
{
	switch (result) {
	case IR_CAL_OK:
		return "ok";
	case IR_CAL_NO_SAMPLES:
		return "no_samples";
	case IR_CAL_RAIL_STUCK:
		return "rail_stuck";
	case IR_CAL_LOW_CONTRAST:
		return "low_contrast";
	default:
		return "?";
	}
}

bool ir_classify(const ir_cal_sensor_t *cal, uint16_t raw, bool was_black)
{
	uint16_t dark = darkness(cal->black_is_high, raw);

	return was_black ? (dark > cal->fall) : (dark >= cal->rise);
}

uint16_t ir_normalise(const ir_cal_sensor_t *cal, uint16_t raw)
{
	uint16_t dark = darkness(cal->black_is_high, raw);
	uint32_t range;

	if (dark <= cal->white) {
		return 0u;
	}
	if ((dark >= cal->black) || (cal->black <= cal->white)) {
		return IR_NORM_BLACK;
	}
	range = (uint32_t)cal->black - cal->white;
	return (uint16_t)((((uint32_t)dark - cal->white) * IR_NORM_BLACK) / range);
}
