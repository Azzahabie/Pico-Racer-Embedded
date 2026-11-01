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

/* White only (sensors off the line): black is guessed 2 x
   IR_CAL_MIN_CONTRAST above white, and the thresholds sit between them */
static void test_white_only_guesses_black(void)
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
	CHECK(cal.white == 200u);
	CHECK(cal.black == 200u + (2u * IR_CAL_MIN_CONTRAST));
	CHECK((cal.rise > 600u) && (cal.fall < 600u));	/* midpoint 600 */
	CHECK(ir_normalise(&cal, 200u) == 0u);
	CHECK(ir_normalise(&cal, 600u) == 500u);
	CHECK(!ir_classify(&cal, 229u, false));
}

/* The start pose: centred, both sensors inside the 18 mm line, so black
   only.  It must pass (else CAL_FAIL puts nav in Fault); white is guessed
   as IR_CAL_WHITE_GUESS_PCT of black, not taken from the config. */
static void test_black_only_guesses_white(void)
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
	CHECK(cal.black == 3800u);
	CHECK(cal.white == (3800u * IR_CAL_WHITE_GUESS_PCT) / 100u);	/* 950 */
	CHECK((cal.rise > 2375u) && (cal.fall < 2375u));	/* midpoint 2375 */
	CHECK(ir_normalise(&cal, 3800u) == IR_NORM_BLACK);
	CHECK(ir_normalise(&cal, 2375u) == 500u);
	CHECK(ir_classify(&cal, 3771u, false));
	CHECK(!ir_classify(&cal, 300u, true));		/* the real floor is white */
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
		ir_cal_add(&acc, (uint16_t)(2550u + (i % 3u)));
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

/* ---- Running adaptation ---- */

/* n samples of one reading through classify + adapt, like line_track */
static bool run(ir_cal_sensor_t *cal, uint16_t raw, uint32_t n, bool black)
{
	uint32_t i;

	for (i = 0u; i < n; ++i) {
		black = ir_classify(cal, raw, black);
		ir_cal_adapt(cal, raw, black);
	}
	return black;
}

/* calibration at rest on the line, with the real config thresholds */
static ir_cal_sensor_t cal_on_line(uint16_t black_raw)
{
	ir_cal_sensor_t def = ir_cal_default(true,
		LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
		LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	uint32_t i;

	ir_cal_begin(&acc, true);
	for (i = 0u; i < IR_CAL_SAMPLES; ++i) {
		ir_cal_add(&acc, (uint16_t)(black_raw - 20u + (i % 40u)));
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_OK);
	return cal;
}

/* S4 attempt 5 (31 Oct): black 1,250-1,330, both sensors dipped to about
   1,050 for 1.5 s, white 150-320.  The fixed 1400/1100 called the dip white
   and fired LOST; the measured levels must not. */
static void test_attempt5_weak_black(void)
{
	ir_cal_sensor_t cal = cal_on_line(1300u);
	bool black;

	CHECK(cal.rise < 1050u);
	black = run(&cal, 1300u, 2000u, true);
	CHECK(black);
	black = run(&cal, 1050u, 1500u, black);		/* the dip */
	CHECK(black);
	black = run(&cal, 250u, 50u, black);		/* off the line */
	CHECK(!black);
	black = run(&cal, 250u, 5000u, black);		/* white found */
	CHECK((cal.white > 240u) && (cal.white < 330u));
	CHECK(run(&cal, 1300u, 20u, black));		/* back on it */
}

/* Same code, sensors low (demo day may differ): black 3,500, white 250 */
static void test_strong_black(void)
{
	ir_cal_sensor_t cal = cal_on_line(3500u);
	bool black;

	black = run(&cal, 3500u, 1000u, true);
	CHECK(black);
	black = run(&cal, 250u, 20u, black);
	CHECK(!black);
	black = run(&cal, 250u, 10000u, black);
	CHECK(cal.white < 300u);			/* guess 875 replaced */
	CHECK(run(&cal, 3500u, 20u, black));
	CHECK(!run(&cal, 900u, 20u, true));		/* now far from black */
}

/* Black fades slowly (sensor rising over the course, worn tape): 1,300 to
   700 over 30 s, white 250.  Every sample must stay black. */
static void test_slow_drift(void)
{
	ir_cal_sensor_t cal = cal_on_line(1300u);
	bool black = true;
	bool always = true;
	uint32_t i;

	(void)run(&cal, 250u, 5000u, false);		/* white measured */
	for (i = 0u; i < 30000u; ++i) {
		uint16_t raw = (uint16_t)(1300u - ((600u * i) / 30000u));

		black = run(&cal, raw, 1u, black);
		always = always && black;
	}
	CHECK(always);
	CHECK(cal.black < 800u);
	CHECK(!run(&cal, 250u, 20u, true));
}

/* Car lifted (black rises to 4,000), then put back on a 1,300 line: black
   is stranded above every reading.  The slow pull must bring it back. */
static void test_stranded_black(void)
{
	ir_cal_sensor_t cal = cal_on_line(1300u);
	bool black;
	uint32_t ms = 0u;

	(void)run(&cal, 250u, 5000u, false);
	black = run(&cal, 4000u, 20000u, true);		/* lifted */
	CHECK(cal.black > 3900u);
	black = run(&cal, 1300u, 1u, black);
	CHECK(!black);					/* stranded */
	while (!black && (ms < 30000u)) {
		black = run(&cal, 1300u, 1u, black);
		ms++;
	}
	CHECK(black);
	CHECK(ms < 15000u);				/* about 11 s */
	CHECK(cal.white < 330u);			/* white untouched */
}

/* The levels never close up: a long grey patch does not turn the floor
   into black, and rail readings change nothing */
static void test_contrast_floor_and_rails(void)
{
	ir_cal_sensor_t cal = cal_on_line(1300u);
	ir_cal_sensor_t before;

	(void)run(&cal, 250u, 5000u, false);
	(void)run(&cal, 700u, 60000u, false);		/* grey, classed white */
	CHECK(cal.black >= (cal.white + IR_CAL_MIN_CONTRAST));
	before = cal;
	ir_cal_adapt(&cal, 0u, false);
	ir_cal_adapt(&cal, 4095u, true);
	CHECK((cal.white_q == before.white_q) && (cal.black_q == before.black_q));
}

static void test_adapt_black_is_low(void)
{
	ir_cal_sensor_t def = ir_cal_default(false, TEST_RISE, TEST_FALL);
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	bool black;
	uint32_t i;

	ir_cal_begin(&acc, false);
	for (i = 0u; i < 200u; ++i) {
		ir_cal_add(&acc, 3800u);
		ir_cal_add(&acc, 300u);
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_OK);
	black = run(&cal, 1500u, 10000u, true);		/* weaker black, low */
	CHECK(black);
	CHECK(!run(&cal, 3800u, 20u, black));
}

/* ---- Both sensors together: the line edge ---- */

static void fill(ir_cal_acc_t *acc, uint16_t raw)
{
	uint32_t i;

	ir_cal_begin(acc, true);
	for (i = 0u; i < IR_CAL_SAMPLES; ++i) {
		ir_cal_add(acc, (uint16_t)(raw - 15u + (i % 30u)));
	}
}

/* S5 attempt 1 (31 Oct): GP27 on black (~2,040), GP26 on the line edge
   (~715, between the white and black limits).  It failed 3 times in 5;
   now GP26 is "edge", starts from the defaults and learns its levels. */
static void test_pair_edge(void)
{
	ir_cal_sensor_t def = ir_cal_default(true,
		LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
		LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);
	ir_cal_sensor_t l = def;
	ir_cal_sensor_t r = def;
	ir_cal_acc_t al;
	ir_cal_acc_t ar;
	ir_cal_result_t rl;
	ir_cal_result_t rr;
	bool black;

	fill(&al, 715u);
	fill(&ar, 2040u);
	CHECK(ir_cal_finish_pair(&al, &ar, &def, &def, &l, &r, &rl, &rr));
	CHECK(rl == IR_CAL_EDGE);
	CHECK(rr == IR_CAL_OK);
	CHECK((l.rise == def.rise) && (l.fall == def.fall));	/* defaults */
	CHECK(r.black >= 2040u);
	CHECK(ir_cal_result_name(IR_CAL_EDGE)[0] == 'e');

	/* driving: GP26 sees its real black (1,065) and floor (250) */
	black = run(&l, 715u, 3000u, false);			/* parked */
	black = run(&l, 1065u, 2000u, black);
	CHECK(black);
	black = run(&l, 250u, 4000u, black);
	CHECK(!black);
	black = run(&l, 1065u, 4000u, black);
	CHECK(black);
	CHECK((l.white < 330u) && (l.black > 950u) && (l.black < 1200u));
	CHECK(!run(&l, 300u, 50u, true));			/* floor is white */

	/* mirrored: GP27 on the edge */
	l = def;
	r = def;
	fill(&al, 1500u);
	fill(&ar, 760u);
	CHECK(ir_cal_finish_pair(&al, &ar, &def, &def, &l, &r, &rl, &rr));
	CHECK((rl == IR_CAL_OK) && (rr == IR_CAL_EDGE));
}

/* What must still fail: both on the edge, edge + off the line, a stuck
   sensor; and a failure leaves both outputs untouched. */
static void test_pair_failures(void)
{
	ir_cal_sensor_t def = ir_cal_default(true,
		LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
		LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);
	ir_cal_sensor_t l = def;
	ir_cal_sensor_t r = def;
	ir_cal_acc_t al;
	ir_cal_acc_t ar;
	ir_cal_result_t rl;
	ir_cal_result_t rr;
	uint32_t i;

	l.white = 11u;					/* marker: must survive */
	fill(&al, 715u);
	fill(&ar, 800u);
	CHECK(!ir_cal_finish_pair(&al, &ar, &def, &def, &l, &r, &rl, &rr));
	CHECK((rl == IR_CAL_LOW_CONTRAST) && (rr == IR_CAL_LOW_CONTRAST));

	fill(&al, 715u);
	fill(&ar, 250u);				/* off the line: white */
	CHECK(!ir_cal_finish_pair(&al, &ar, &def, &def, &l, &r, &rl, &rr));
	CHECK(rl == IR_CAL_LOW_CONTRAST);

	fill(&al, 715u);
	ir_cal_begin(&ar, true);
	for (i = 0u; i < 100u; ++i) {
		ir_cal_add(&ar, 4095u);			/* stuck at the rail */
	}
	CHECK(!ir_cal_finish_pair(&al, &ar, &def, &def, &l, &r, &rl, &rr));
	CHECK(rr == IR_CAL_RAIL_STUCK);
	CHECK(l.white == 11u);
}

int main(void)
{
	test_both_colours();
	test_white_only_guesses_black();
	test_black_only_guesses_white();
	test_failures();
	test_classify_hysteresis();
	test_black_is_low();
	test_attempt5_weak_black();
	test_strong_black();
	test_slow_drift();
	test_stranded_black();
	test_contrast_floor_and_rails();
	test_adapt_black_is_low();
	test_pair_edge();
	test_pair_failures();
	return check_report("ir calibration");
}
