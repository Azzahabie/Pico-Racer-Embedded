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
	uint32_t mid;
	uint32_t hyst;

	if (acc->samples == 0u) {
		return IR_CAL_NO_SAMPLES;
	}
	if ((acc->rail_samples * 2u) > acc->samples) {
		return IR_CAL_RAIL_STUCK;
	}
	span = (uint32_t)acc->max_dark - acc->min_dark;
	if (span >= IR_CAL_MIN_CONTRAST) {
		/* both colours seen: thresholds from the measured midpoint */
		mid = acc->min_dark + span / 2u;
		hyst = (span * IR_CAL_HYST_PCT) / 100u;
		cal.white = acc->min_dark;
		cal.black = acc->max_dark;
		cal.rise = (uint16_t)(mid + hyst);
		cal.fall = (uint16_t)(mid - hyst);
	} else if (((uint32_t)acc->max_dark + IR_CAL_WHITE_MARGIN) <=
		   defaults->fall) {
		/* one colour, white (sensors off the line): keep the tested
		   thresholds; black mirrors white about the threshold midpoint.
		   It may lie above 4095 (darkness units), so the midpoint stays
		   at 500 */
		mid = ((uint32_t)defaults->rise + defaults->fall) / 2u;
		cal.white = acc->min_dark;
		cal.black = (uint16_t)((2u * mid) - acc->min_dark);
	} else if ((uint32_t)acc->min_dark >=
		   ((uint32_t)defaults->rise + IR_CAL_WHITE_MARGIN)) {
		/* one colour, black: the normal start pose, centred with both
		   sensors inside the 18 mm line.  Same rule mirrored: white
		   mirrors black about the midpoint, floored at 0 */
		mid = ((uint32_t)defaults->rise + defaults->fall) / 2u;
		cal.black = acc->max_dark;
		cal.white = ((2u * mid) > acc->max_dark)
			    ? (uint16_t)((2u * mid) - acc->max_dark) : 0u;
	} else {
		/* one colour, between the thresholds: neither clearly */
		return IR_CAL_LOW_CONTRAST;
	}
	*out = cal;
	return IR_CAL_OK;
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
