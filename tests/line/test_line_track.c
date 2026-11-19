/* Host test for line_track.c: line detection and error direction, junction
   debounce and rearming, J_n against the course barcodes, noisy line loss,
   barcode hold (and the dark time under it), stuck sensors. */

#include "check.h"
#include "../../app_program/line_task/line_track.h"
#include "../../app_program/motor_task/motion_cfg.h"	/* line-following speed */

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

/* n samples with the barcode side's gate (line_track_step_gate) */
static unsigned feed_gate(unsigned n, bool mark, bool in_read, bool armed_due,
			  bool wait)
{
	line_junction_gate_t gate = { in_read, armed_due, wait, false };
	unsigned events = 0u;
	unsigned i;

	for (i = 0u; i < n; ++i) {
		line_track_step_gate(&trk, B, B, true, false, mark, &gate, &out);
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
	CHECK(!line_state_lost(out.word));		/* GP26/GP27 on the line */
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, true, true) == 1u);

	/* mission change while on a junction: it does not fire again */
	feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false);
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, false, true) == 1u);
	line_track_disarm_junction(&trk);
	CHECK(feed_mark(50u, B, B, false, true) == 0u);
	feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false);
	CHECK(feed_mark(LINE_JUNCTION_SAMPLES, B, B, false, true) == 1u);
}

/* GP7 black or white for mm of travel at motion's line-following base
   speed: MOTION_LF_BASE counts a tick, MOTION_COUNTS_PER_REV a wheel turn
   (47.6 mm/s at base 6, calculated) */
static unsigned ms_for_mm(int mm)
{
	const double mm_per_s = ((double)MOTION_LF_BASE * 1000.0 /
				 (double)MOTION_TICK_MS) *
				(3.14159265358979 * (double)MOTION_WHEEL_DIA_MM) /
				(double)MOTION_COUNTS_PER_REV;

	return (unsigned)(((double)mm * 1000.0 / mm_per_s) + 0.5);
}

/* The course: Code 39 sheets beside the line, narrow 3 mm, wide 9 mm, about
   141 mm (3 symbols, 15 bars), at least 12 mm before the 18 mm crossbar.
   GP7 may see the bars up to 2 mm wider or narrower than printed (the
   spaces the other way).  No sheet bar may be a junction or a crossing,
   every crossbar exactly one, and it rearms for the next.  GP7 moves at a
   constant MOTION_LF_BASE speed here; on the car steering changes GP7's
   speed (line_config.h), which this test does not model.  With J_n 280 it
   passes at base 5 to 7 and fails at base 4 or below (a 9 mm bar fires:
   284 ms at base 4) and at base 8 or above (a crossbar seen 1 mm narrow
   is missed), so J_n must move with MOTION_LF_BASE.  The old J_n 80 fails
   at base 6: it fired on every wide bar. */
static void test_junction_course(void)
{
	int err;

	for (err = -2; err <= 2; ++err) {
		unsigned events = 0u;
		bool crossing = false;
		unsigned bar;

		setup();
		feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false);	/* armed */
		/* worse than any letter: every bar wide */
		for (bar = 0u; bar < 15u; ++bar) {
			events += feed_mark(ms_for_mm(9 + err), B, B, false, true);
			crossing = crossing || out.crossing;
			events += feed(ms_for_mm(3 - err), B, B, false);
		}
		CHECK(events == 0u);
		CHECK(!crossing);

		feed(ms_for_mm(12), B, B, false);
		CHECK(feed_mark(ms_for_mm(18 + err), B, B, false, true) == 1u);
		CHECK(out.crossing);
		CHECK(feed(LINE_JUNCTION_REARM_SAMPLES, B, B, false) == 0u);
		CHECK(!out.crossing);
		CHECK(feed_mark(ms_for_mm(18 + err), B, B, false, true) == 1u);
	}
}

/* The gate (LINE_JUNCTION_RULE 1): a black run while the bars of a read
   arrive is never a junction however long (the length rule fired on it);
   armed and confirmed fires before J_n; a junction due while a complete
   read waits for its verdict fires once the wait ends, even if GP7 has
   left the black by then; a NULL gate is the length rule alone. */
static void test_junction_gate(void)
{
	setup();
	CHECK(feed_gate(LINE_JUNCTION_REARM_SAMPLES, false, false, false, false) == 0u);

	/* bars arriving: 3 x J_n of black, no event, no crossing */
	CHECK(feed_gate(3u * LINE_JUNCTION_SAMPLES, true, true, false, false) == 0u);
	CHECK(!out.crossing);
	/* the same run once the read is over (or never was): the length rule */
	CHECK(feed_gate(1u, true, false, false, false) == 1u);
	CHECK(out.crossing);

	/* armed: fires the sample it is confirmed, long before J_n */
	CHECK(feed_gate(LINE_JUNCTION_REARM_SAMPLES, false, false, false, false) == 0u);
	CHECK(feed_gate(10u, true, false, false, false) == 0u);
	CHECK(feed_gate(1u, true, false, true, false) == 1u);
	CHECK(out.crossing);			/* on that crossbar */
	CHECK(feed_gate(LINE_JUNCTION_SAMPLES, true, false, true, false) == 0u);	/* once */

	/* armed, but bars arriving: never */
	CHECK(feed_gate(LINE_JUNCTION_REARM_SAMPLES, false, false, false, false) == 0u);
	CHECK(feed_gate(50u, true, true, true, false) == 0u);

	/* due while waiting for the verdict: held, then fires, even on white;
	   held past the rearm time, it fires once, and the next black within
	   the rearm time after it is not another junction */
	CHECK(feed_gate(LINE_JUNCTION_REARM_SAMPLES, false, false, false, false) == 0u);
	CHECK(feed_gate(LINE_JUNCTION_SAMPLES + 20u, true, false, false, true) == 0u);
	CHECK(feed_gate(LINE_JUNCTION_REARM_SAMPLES + 5u, false, false, false, true) == 0u);
	CHECK(feed_gate(1u, false, false, false, false) == 1u);
	CHECK(!out.crossing);			/* GP7 already off it */
	CHECK(feed_gate(LINE_JUNCTION_SAMPLES, true, false, true, false) == 0u);
	CHECK(feed_gate(100u, false, false, false, false) == 0u);

	/* a mission change drops a held junction */
	CHECK(feed_gate(1u, true, false, true, true) == 0u);
	line_track_disarm_junction(&trk);
	CHECK(feed_gate(50u, false, false, false, false) == 0u);
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

/* A barcode hold blanks rl, steer and error, not the dark time: that
   follows the real (debounced) line sensors.  Motion reads "line seen" as
   rl != 0 or dark < MOTION_LF_SEEN_MS, and rl is 00 while held, so a dark
   time held at 0 read "line seen" with both sensors on white, and the
   line was never lost (sim: a turn that lands off the arm went on to
   LINE_FOLLOW instead of LINE_SEARCH). */
static void test_hold_dark(void)
{
	setup();
	feed(LINE_CONFIRM_SAMPLES, B, B, false);	/* centred on the line */

	/* held on the line: rl reads 00, but the line is seen */
	feed(1000u, B, B, true);
	CHECK(line_state_barcode(out.word));
	CHECK(line_state_rl(out.word) == LINE_RL_CENTRED);
	CHECK(line_state_dark_ms(out.word) == 0u);
	CHECK(!line_state_lost(out.word));

	/* held, both sensors white: the debounce keeps 11 for
	   LINE_LOST_SAMPLES - 1 samples, then the dark time counts */
	feed((LINE_LOST_SAMPLES - 1u) + MOTION_LF_SEEN_MS, W, W, true);
	CHECK(line_state_barcode(out.word));
	CHECK(line_state_rl(out.word) == LINE_RL_CENTRED);
	CHECK(line_state_dark_ms(out.word) >= MOTION_LF_SEEN_MS);	/* not seen */
	CHECK(!line_state_lost(out.word));
	feed(LINE_LOST_MS - 1u - MOTION_LF_SEEN_MS, W, W, true);
	CHECK(out.fields.dark_ms == LINE_LOST_MS - 1u);
	CHECK(!line_state_lost(out.word));
	feed(1u, W, W, true);
	CHECK(line_state_barcode(out.word));
	CHECK(line_state_lost(out.word));

	/* back on the line, still held: found, dark 0 again */
	feed(LINE_CONFIRM_SAMPLES, B, B, true);
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

	/* S.faults bit 4 (FAULT_LINE_SENSOR) follows line_state's fault bit.
	   The glue that sends it (fault_set/fault_clear on the bit's edges in
	   line_task.c, before the console's barcode check) is kernel code and
	   is NOT tested here.  The checks below cover line_track only, and
	   they pass on the line_track.c from before the bit was wired: they
	   show the fault bit cannot stick, not that the glue is right.  The
	   console check depends on bar_capture_reading(), not on the hold, so
	   hold=true below says nothing about the glue's order either. */
	line_track_step(&trk, W, W, false, false, false, &out);	/* ADC read failed */
	CHECK(line_state_fault(out.word));
	line_track_step(&trk, W, W, true, false, false, &out);	/* the next good read ends it */
	CHECK(!line_state_fault(out.word));

	/* a rail fault under a barcode hold starts and ends the same way: the
	   hold does not mask the fault bit */
	feed(IR_RAIL_FAULT_SAMPLES, 4095u, W, true);		/* GP26 at the top rail */
	CHECK(line_state_fault(out.word));
	feed(1u, W, W, true);
	CHECK(!line_state_fault(out.word));
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

/* S4 attempt 5 end to end: calibrated on a weak 1,300 line, both sensors
   dip to 1,050 for 2 s.  The car must not be reported lost. */
static void test_weak_black_not_lost(void)
{
	ir_cal_sensor_t def = ir_cal_default(true,
		LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
		LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);
	ir_cal_sensor_t cal = def;
	ir_cal_acc_t acc;
	bool ever_lost = false;
	unsigned i;

	setup();
	ir_cal_begin(&acc, true);
	for (i = 0u; i < IR_CAL_SAMPLES; ++i) {
		ir_cal_add(&acc, (uint16_t)(1280u + (i % 40u)));
	}
	CHECK(ir_cal_finish(&acc, &def, &cal) == IR_CAL_OK);
	line_track_set_cal(&trk, &cal, &cal);
	feed(1000u, 1300u, 1300u, false);
	CHECK(!line_state_lost(out.word));
	for (i = 0u; i < 2000u; ++i) {
		feed(1u, 1050u, 1050u, false);
		ever_lost = ever_lost || line_state_lost(out.word);
	}
	CHECK(!ever_lost);
	CHECK(out.fields.rl == LINE_RL_JUNCTION);	/* rl 11: both black */
	feed(LINE_LOST_SAMPLES + LINE_LOST_MS, 250u, 250u, false);
	CHECK(line_state_lost(out.word));		/* real loss still seen */
}

int main(void)
{
	test_direction();
	test_weak_black_not_lost();
	test_junction();
	test_junction_course();
	test_junction_gate();
	test_line_loss();
	test_hold_dark();
	test_faults();
	test_word();
	return check_report("line tracking");
}
