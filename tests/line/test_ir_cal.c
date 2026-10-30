/* Host test for ir_cal.c: calibration success and failure, classification
   with hysteresis, normalisation, both polarities. */

#include "check.h"
#include "../../app_program/line_task/ir_cal.h"

/* Fixed test thresholds, not line_config.h: the numbers below are worked
   out for 2500/2000, and the config is retuned on the car. */
#define TEST_RISE	(2500u)
#define TEST_FALL	(2000u)

static ir_cal_sensor_t defaults_high(void)
{
	return ir_cal_default(true, TEST_RISE, TEST_FALL);
}

static void test_both_colours(void)
{
	ir_cal_sensor_t def = defaults_high();
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	uint32_t i;

	ir_cal_begin(&acc, true);
	for (i = 0u; i < 500u; ++i) {
		ir_cal_add(&acc, (uint16_t)(150u + (i % 7u)));	/* white, noisy */
		ir_cal_add(&acc, (uint16_t)(3900u - (i % 5u)));	/* black */
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_OK);
	CHECK(cal.white == 150u);
	CHECK(cal.black == 3900u);
	CHECK((cal.rise > 2025u) && (cal.fall < 2025u));	/* midpoint 2025 */
	CHECK(ir_normalise(&cal, 150u) == 0u);
	CHECK(ir_normalise(&cal, 3900u) == IR_NORM_BLACK);
	CHECK(ir_normalise(&cal, 2025u) == 500u);
	CHECK(ir_normalise(&cal, 4000u) == IR_NORM_BLACK);	/* clamped */
	CHECK(ir_normalise(&cal, 10u) == 0u);
}

static void test_white_only_keeps_tested_thresholds(void)
{
	ir_cal_sensor_t def = defaults_high();
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	uint32_t i;

	ir_cal_begin(&acc, true);
	for (i = 0u; i < IR_CAL_SAMPLES; ++i) {
		ir_cal_add(&acc, (uint16_t)(200u + (i % 30u)));
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_OK);
	CHECK(cal.rise == TEST_RISE);
	CHECK(cal.fall == TEST_FALL);
	CHECK(cal.white == 200u);
	CHECK(ir_normalise(&cal, 200u) == 0u);
	CHECK(ir_normalise(&cal, 2250u) == 500u);	/* threshold midpoint */
}

/* The start pose: centred, both sensors inside the 18 mm line, so black
   only.  It must pass (else CAL_FAIL puts nav in Fault), keeping the tested
   thresholds. */
static void test_black_only_keeps_tested_thresholds(void)
{
	ir_cal_sensor_t def = defaults_high();
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	uint32_t i;

	ir_cal_begin(&acc, true);
	for (i = 0u; i < IR_CAL_SAMPLES; ++i) {
		ir_cal_add(&acc, (uint16_t)(3800u - (i % 30u)));
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_OK);
	CHECK(cal.rise == TEST_RISE);
	CHECK(cal.fall == TEST_FALL);
	CHECK(cal.black == 3800u);
	CHECK(cal.white == 700u);			/* 2 x 2250 - 3800 */
	CHECK(ir_normalise(&cal, 3800u) == IR_NORM_BLACK);
	CHECK(ir_normalise(&cal, 2250u) == 500u);	/* threshold midpoint */
	CHECK(ir_classify(&cal, 3800u, false));
}

static void test_failures(void)
{
	ir_cal_sensor_t def = defaults_high();
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	uint32_t i;

	/* no samples */
	ir_cal_begin(&acc, true);
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_NO_SAMPLES);

	/* stuck at the 0 rail (disconnected) and at the top rail (shorted) */
	ir_cal_begin(&acc, true);
	for (i = 0u; i < 100u; ++i) {
		ir_cal_add(&acc, 0u);
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_RAIL_STUCK);
	ir_cal_begin(&acc, true);
	for (i = 0u; i < 100u; ++i) {
		ir_cal_add(&acc, 4095u);
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_RAIL_STUCK);
	CHECK(ir_at_rail(0u) && ir_at_rail(4095u) && !ir_at_rail(2000u));

	/* one colour, too close to the falling threshold: low contrast */
	ir_cal_begin(&acc, true);
	for (i = 0u; i < 100u; ++i) {
		ir_cal_add(&acc, (uint16_t)(1900u + (i % 3u)));
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_LOW_CONTRAST);

	/* black only, but too close to the rising threshold: low contrast */
	ir_cal_begin(&acc, true);
	for (i = 0u; i < 100u; ++i) {
		ir_cal_add(&acc, (uint16_t)(2600u + (i % 3u)));
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_LOW_CONTRAST);

	/* a failure leaves *out untouched */
	CHECK((cal.rise == def.rise) && (cal.fall == def.fall) && (cal.white == def.white));
}

static void test_classify_hysteresis(void)
{
	ir_cal_sensor_t cal = defaults_high();	/* rise 2500, fall 2000 */

	CHECK(!ir_classify(&cal, 2400u, false));	/* below rise: still white */
	CHECK(ir_classify(&cal, 2500u, false));		/* reaches rise: black */
	CHECK(ir_classify(&cal, 2100u, true));		/* above fall: stays black */
	CHECK(!ir_classify(&cal, 2000u, true));		/* at fall: white again */
}

static void test_black_is_low(void)
{
	ir_cal_sensor_t def = ir_cal_default(false, TEST_RISE, TEST_FALL);
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	uint32_t i;

	CHECK(ir_classify(&def, 1900u, false));		/* low raw = black */
	CHECK(!ir_classify(&def, 3000u, false));
	ir_cal_begin(&acc, false);
	for (i = 0u; i < 200u; ++i) {
		ir_cal_add(&acc, 3800u);			/* white reads high */
		ir_cal_add(&acc, 300u);				/* black reads low */
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_OK);
	CHECK(ir_normalise(&cal, 300u) == IR_NORM_BLACK);
	CHECK(ir_normalise(&cal, 3800u) == 0u);
	CHECK(ir_classify(&cal, 300u, false));
	CHECK(!ir_classify(&cal, 3800u, true));
}

int main(void)
{
	test_both_colours();
	test_white_only_keeps_tested_thresholds();
	test_black_only_keeps_tested_thresholds();
	test_failures();
	test_classify_hysteresis();
	test_black_is_low();
	return check_report("ir calibration");
}
