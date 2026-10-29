/* Host test for line_track.c: line detection and error direction, junction
   debounce and rearming, noisy line loss, barcode hold, stuck sensors. */

#include "check.h"
#include "../../app_program/line_task/line_track.h"

#define W  (300u)	/* white reading  */
#define B  (3500u)	/* black reading  */

static line_track_t trk;
static line_track_out_t out;

static void setup(void)
{
	ir_cal_sensor_t cal = ir_cal_default(true,
		LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
		LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);

	line_track_init(&trk, &cal, &cal, false);
}

/* n samples, GP7 (the junction mark) as given; returns how many junction
   events they produced */
static unsigned feed_mark(unsigned n, uint16_t left, uint16_t right, bool hold,
			  bool mark)
{
	unsigned events = 0u;
	unsigned i;

	for (i = 0u; i < n; ++i) {
		line_track_step(&trk, left, right, true, hold, mark, &out);
		if (out.junction) {
			events++;
		}
	}
	return events;
}

/* n samples with GP7 white */
static unsigned feed(unsigned n, uint16_t left, uint16_t right, bool hold)
{
	return feed_mark(n, left, right, hold, false);
}

static void test_direction(void)
{
	setup();
	feed(LINE_CONFIRM_SAMPLES, B, W, false);	/* GP26 black: veered left */
	CHECK(line_state_rl(out.word) == LINE_RL_LEFT_DEV);
	CHECK(line_state_steer(out.word) == LINE_STEER_RIGHT);
	CHECK(line_state_error(out.word) < -50);	/* negative = correct right */

	feed(LINE_CONFIRM_SAMPLES, W, B, false);	/* GP27 black: veered right */
	CHECK(line_state_rl(out.word) == LINE_RL_RIGHT_DEV);
	CHECK(line_state_steer(out.word) == LINE_STEER_LEFT);
	CHECK(line_state_error(out.word) > 50);		/* positive = correct left */

	feed(LINE_LOST_SAMPLES, W, W, false);
	CHECK(line_state_rl(out.word) == LINE_RL_CENTRED);
	CHECK(line_state_error(out.word) == 0);
	CHECK(!line_state_lost(out.word));		/* just seen: not lost */

	/* a debounce: 5 samples are not enough to change state */
	feed(LINE_CONFIRM_SAMPLES - 1u, B, W, false);
	CHECK(line_state_rl(out.word) == LINE_RL_CENTRED);

	/* LINE_STEER_INVERT flips both steer and the error sign */
	{
		ir_cal_sensor_t cal = ir_cal_default(true, 2500u, 2000u);
		line_track_init(&trk, &cal, &cal, true);
		feed(LINE_CONFIRM_SAMPLES, B, W, false);
		CHECK(line_state_steer(out.word) == LINE_STEER_LEFT);
		CHECK(line_state_error(out.word) > 50);
	}
}

static void test_junction(void)
{
	setup();
	CHECK(feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false) == 0u);	/* arms */

	/* centred on the line (both sensors inside it, rl 11) for a long
	   time, wobbling either side: never a junction */
	CHECK(feed(2000u, B, B, false) == 0u);
	CHECK(line_state_rl(out.word) == LINE_RL_JUNCTION);	/* 11 = centred */
	CHECK(!line_state_lost(out.word));
	{
		unsigned events = 0u;
		unsigned i;
		for (i = 0u; i < 50u; ++i) {
			events += feed(20u, B, W, false);
			events += feed(20u, B, B, false);
			events += feed(20u, W, B, false);
			events += feed(20u, B, B, false);
		}
		CHECK(events == 0u);
	}

	/* GP7: J_n - 1 samples: nothing; J_n-th: exactly one event */
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES - 1u, B, B, false, true) == 0u);
	CHECK(!out.crossing);
	CHECK(feed_mark(1u, B, B, false, true) == 1u);
	CHECK(out.crossing);
	CHECK(feed_mark(200u, B, B, false, true) == 0u);	/* still on the bar */

	/* off it, but not long enough to rearm, then back on: no event */
	CHECK(feed(LINE_JUNCTION_REARM_SAMPLES - 1u, B, B, false) == 0u);
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, false, true) == 0u);

	/* properly off it: rearmed, the next junction counts */
	CHECK(feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false) == 0u);
	CHECK(!out.crossing);
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, false, true) == 1u);

	/* barcode bars (short GP7 runs) never make a junction */
	feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false);
	{
		unsigned events = 0u;
		unsigned i;
		for (i = 0u; i < 100u; ++i) {
			events += feed_mark(LINE_JUNCTION_SAMPLES / 2u, B, B, true, true);
			events += feed_mark(5u, B, B, true, false);
		}
		CHECK(events == 0u);
	}

	/* a barcode under the car: line state held centred, but a crossbar
	   under GP7 is still a junction (it starts a capture itself) */
	feed(LINE_JUNCTION_REARM_SAMPLES, W, W, false);
	CHECK(feed(100u, B, B, true) == 0u);
	CHECK(line_state_barcode(out.word));
	CHECK(line_state_rl(out.word) == LINE_RL_CENTRED);
	CHECK(line_state_error(out.word) == 0);
	CHECK(!line_state_lost(out.word));		/* the bars count as seen */
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, true, true) == 1u);

	/* mission change while on a junction: it does not fire again */
	feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false);
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, false, true) == 1u);
	line_track_disarm_junction(&trk);
	CHECK(feed_mark(50u, B, B, false, true) == 0u);
	feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false);
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, false, true) == 1u);
}

static void test_line_loss(void)
{
	unsigned i;

	setup();
	CHECK(line_state_lost(LINE_STATE_INITIAL));
	feed(LINE_CONFIRM_SAMPLES, B, W, false);	/* line seen */
	CHECK(!line_state_lost(out.word));
	/* the debounce holds the state LINE_LOST_SAMPLES - 1 more samples,
	   then the dark time starts counting */
	feed((LINE_LOST_SAMPLES - 1u) + (LINE_LOST_MS - 1u), W, W, false);
	CHECK(!line_state_lost(out.word));
	feed(1u, W, W, false);
	CHECK(line_state_lost(out.word));

	/* noise: short black blips (under the debounce) do not "find" it */
	for (i = 0u; i < 50u; ++i) {
		feed(2u, B, W, false);
		feed(20u, W, W, false);
	}
	CHECK(line_state_lost(out.word));

	/* a real line does */
	feed(LINE_CONFIRM_SAMPLES, W, B, false);
	CHECK(!line_state_lost(out.word));
	CHECK(line_state_dark_ms(out.word) == 0u);
}

static void test_faults(void)
{
	setup();
	feed(LINE_CONFIRM_SAMPLES, B, W, false);
	feed(IR_RAIL_FAULT_SAMPLES - 1u, 0u, W, false);	/* GP26 at the 0 rail */
	CHECK(!line_state_fault(out.word));
	feed(1u, 0u, W, false);
	CHECK(line_state_fault(out.word));
	CHECK(line_state_lost(out.word));			/* a fault is never "on line" */
	CHECK(feed(100u, 0u, 4095u, false) == 0u);		/* no junction from rails */
	feed(1u, W, W, false);
	CHECK(!line_state_fault(out.word));

	line_track_step(&trk, W, W, false, false, false, &out);	/* ADC read failed */
	CHECK(line_state_fault(out.word));
}

static void test_word(void)
{
	ir_cal_sensor_t cal = ir_cal_default(true, 2500u, 2000u);
	uint32_t before;

	setup();
	feed(1u, W, W, false);
	before = line_state_counter(out.word);
	feed(1u, W, W, false);
	CHECK(line_state_counter(out.word) == ((before + 1u) & LINE_ST_CTR_MASK));
	CHECK(!line_state_cal(out.word));
	line_track_set_cal(&trk, &cal, &cal);
	feed(1u, W, W, false);
	CHECK(line_state_cal(out.word));
}

int main(void)
{
	test_direction();
	test_junction();
	test_line_loss();
	test_faults();
	test_word();
	return check_report("line tracking");
}
