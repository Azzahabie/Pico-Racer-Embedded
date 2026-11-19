/* Host test for the scan gate (LINE_SCAN_GATE 1, scan_gate.c) and the two
   places it acts: the barcode capture (bar_capture_set_gate) and the
   length rule (line_junction_gate_t.turning in line_track.c).

   distance_state is built with motion_ds_pack() as motion_task builds it:
   one publish every 10 ms, the counter +1 each time, the turn rate in
   whole deg/s.  Turn rates (C, at base 6, 47.6 mm/s): a curve of radius
   150 mm is v / R = 18.2 deg/s, 300 mm is 9.1 deg/s; a junction pivot
   about 43 deg/s (MOTION_TURN_SPEED 6), a fast pivot 90 deg/s; the
   weave on a straight +/-45 deg/s at about 1 Hz (41 deg/s logged in line
   following, trials_batch1_2026-11-04.txt:2554).  None measured for this
   test. */

#include <math.h>
#include <string.h>

#include "check.h"
#include "../../app_program/line_task/scan_gate.h"
#include "../../app_program/line_task/bar_capture.h"
#include "../../app_program/line_task/line_track.h"
#include "../../app_program/motor_task/motion_ds.h"

#define PI (3.14159265358979)
#define B (3500u)		/* line sensors: on the line */

/* ---- the gate, fed as motion publishes ---- */
static scan_gate_t g;
static uint32_t ds_word;
static uint32_t ds_ctr;
static unsigned now;		/* ms */
static bool motion_up;
static bool publishing;

/* a turn rate profile: base + amp x sin(2 pi hz t + ph), from t0 (ms) */
static double base_dps;
static double amp_dps;
static double hz;
static double ph;

static void gate_reset(void)
{
	now = 0u;
	ds_ctr = 0u;
	ds_word = motion_ds_pack(0, MOTION_DS_STEADY, 0u, 0);
	scan_gate_init(&g, ds_word);
	motion_up = true;
	publishing = true;
	base_dps = 0.0;
	amp_dps = 0.0;
	hz = 1.0;
	ph = 0.0;
}

static double rate_now(void)
{
	return base_dps + amp_dps * sin(2.0 * PI * hz * (double)now / 1000.0 + ph);
}

/* one 1 ms sample; returns the gate */
static bool step1(void)
{
	bool open;

	if (publishing && ((now % 10u) == 0u)) {
		ds_ctr++;
		ds_word = motion_ds_pack(0, MOTION_DS_STEADY, ds_ctr,
					 (int32_t)lround(rate_now()));
	}
	open = scan_gate_step(&g, ds_word, motion_up, 1u);
	now++;
	return open;
}

/* n ms; returns how many of them were shut */
static unsigned run(unsigned n)
{
	unsigned shut = 0u;

	while (n-- > 0u) {
		shut += step1() ? 0u : 1u;
	}
	return shut;
}

/* ms until the gate first shuts (n if it never does) */
static unsigned until_shut(unsigned n)
{
	unsigned i;

	for (i = 0u; i < n; ++i) {
		if (!step1()) {
			return i;
		}
	}
	return n;
}

/* ms until it first opens */
static unsigned until_open(unsigned n)
{
	unsigned i;

	for (i = 0u; i < n; ++i) {
		if (step1()) {
			return i;
		}
	}
	return n;
}

static void test_straight(void)
{
	gate_reset();
	CHECK(run(10000u) == 0u);
	CHECK(g.available && g.open);
	CHECK(scan_gate_avg_x10(&g) == 0);
	/* a steady 5 deg/s (under 6): open, the average reaches 5.0 */
	base_dps = 5.0;
	CHECK(run(10000u) == 0u);
	CHECK(scan_gate_avg_x10(&g) == 50);
	/* 6 deg/s is turning; -6 too */
	base_dps = 6.0;
	CHECK(run(10000u) > 0u);
	CHECK(!g.open && (scan_gate_avg_x10(&g) == 60));
	base_dps = -6.0;
	(void)run(10000u);
	CHECK(!g.open && (scan_gate_avg_x10(&g) == -60));
}

/* The weave on a straight: +/-45 deg/s, 0.7-2 Hz, 8 phases.  After the
   first 6 s (the car has been following the line a while) the gate
   never shuts, and the average stays within the limit with room. */
static void test_weave(void)
{
	static const double hzs[] = { 0.7, 0.8, 1.0, 1.25, 1.5, 2.0 };
	unsigned f;
	unsigned k;
	unsigned shut = 0u;
	unsigned shut_start = 0u;
	int32_t worst[6] = { 0, 0, 0, 0, 0, 0 };

	for (f = 0u; f < 6u; ++f) {
		for (k = 0u; k < 8u; ++k) {
			unsigned i;

			gate_reset();
			amp_dps = 45.0;
			hz = hzs[f];
			ph = 2.0 * PI * (double)k / 8.0;
			shut_start += run(6000u);
			for (i = 0u; i < 10000u; ++i) {
				int32_t a;

				shut += step1() ? 0u : 1u;
				a = scan_gate_avg_x10(&g);
				a = (a < 0) ? -a : a;
				if (a > worst[f]) {
					worst[f] = a;
				}
			}
		}
	}
	(void)printf("  weave +/-45 deg/s from a standing start, 8 phases, after 6 s: worst average 0.7 Hz %d.%d, 0.8 Hz %d.%d, "
		     "1 Hz %d.%d, 1.25 Hz %d.%d, 1.5 Hz %d.%d, 2 Hz %d.%d deg/s; shut %u ms "
		     "(%u ms in the first 6 s)\n",
		     worst[0] / 10, worst[0] % 10, worst[1] / 10, worst[1] % 10,
		     worst[2] / 10, worst[2] % 10, worst[3] / 10, worst[3] % 10,
		     worst[4] / 10, worst[4] % 10, worst[5] / 10, worst[5] % 10,
		     shut, shut_start);
	CHECK(shut == 0u);
	CHECK(worst[2] < 40);	/* 1 Hz: under 4 deg/s */
}

/* Curves (a step to v / R) and pivots: how soon the gate shuts, that it
   stays shut, and how soon it opens again once the turn ends. */
static void test_curves_pivots(void)
{
	static const double dps[] = { 18.2, 9.1, 43.0, 90.0, -18.2, -43.0 };
	static const unsigned limit[] = { 1000u, 2000u, 600u, 400u, 1000u, 600u };
	unsigned shut_ms[6];
	unsigned reopen;
	unsigned k;

	for (k = 0u; k < 6u; ++k) {
		gate_reset();
		CHECK(run(3000u) == 0u);
		base_dps = dps[k];
		shut_ms[k] = until_shut(5000u);
		CHECK(shut_ms[k] < limit[k]);
		CHECK(run(5000u) == 5000u);		/* stays shut */
		CHECK((scan_gate_avg_x10(&g) > 0) == (dps[k] > 0.0));
	}

	/* a 90 deg junction pivot at 43 deg/s (2.1 s), then straight */
	gate_reset();
	(void)run(3000u);
	base_dps = 43.0;
	(void)run(2100u);
	base_dps = 0.0;
	reopen = until_open(10000u);
	(void)printf("  shut after: 150 mm curve %u ms, 300 mm %u ms, pivot 43 deg/s %u ms, "
		     "90 deg/s %u ms; open again %u ms after a 90 deg pivot at 43 deg/s\n",
		     shut_ms[0], shut_ms[1], shut_ms[2], shut_ms[3], reopen);
	CHECK(reopen < 3000u);
	CHECK(run(5000u) == 0u);

	/* a 150 mm curve with the weave on top: shut all through, from 2 s */
	for (k = 0u; k < 8u; ++k) {
		gate_reset();
		amp_dps = 45.0;
		ph = 2.0 * PI * (double)k / 8.0;
		(void)run(6000u);
		base_dps = 18.2;
		(void)run(2000u);
		CHECK(run(8000u) == 8000u);
	}
}

/* No distance_state: open whatever the turn rate, and the average starts
   again from 0 when it comes back. */
static void test_fail_open(void)
{
	/* motor_task_flag() 0 (a bench image) */
	gate_reset();
	motion_up = false;
	base_dps = 90.0;
	CHECK(run(5000u) == 0u);
	CHECK(!g.available && (scan_gate_avg_x10(&g) == 0));

	/* the counter stops (motion_task not publishing) during a pivot */
	gate_reset();
	base_dps = 90.0;
	(void)run(2000u);
	CHECK(!g.open && g.available);
	publishing = false;
	/* the last publish was up to 9 ms before */
	CHECK(run(LINE_SCAN_STALE_MS - 10u) == LINE_SCAN_STALE_MS - 10u);
	(void)run(10u);
	CHECK(g.open && !g.available);			/* stale: open */
	CHECK(run(5000u) == 0u);
	/* publishing again, still pivoting: available at once, the average
	   from 0, so open until it has risen again */
	publishing = true;
	(void)step1();
	CHECK(g.available);
	CHECK(until_shut(2000u) < 400u);

	/* the very first word (counter 0, before motion's first publish)
	   counts as fresh for LINE_SCAN_STALE_MS, with turn rate 0 */
	gate_reset();
	publishing = false;
	CHECK(run(LINE_SCAN_STALE_MS - 1u) == 0u);
	CHECK(g.available);
	(void)run(1u);
	CHECK(!g.available && g.open);
}

/* elapsed_ms > 1 (missed ticks) steps the average as often as 1 ms calls
   would; extreme rates do not overflow (UBSan). */
static void test_elapsed_and_range(void)
{
	scan_gate_t a;
	scan_gate_t b;
	uint32_t w;
	unsigned i;
	unsigned j;

	w = motion_ds_pack(0, MOTION_DS_STEADY, 0u, 0);
	scan_gate_init(&a, w);
	scan_gate_init(&b, w);
	for (i = 1u; i <= 300u; ++i) {
		w = motion_ds_pack(0, MOTION_DS_STEADY, i, (int32_t)(i % 37u) - 10);
		(void)scan_gate_step(&a, w, true, 10u);
		for (j = 0u; j < 10u; ++j) {
			(void)scan_gate_step(&b, w, true, 1u);
		}
	}
	CHECK(a.stage1 == b.stage1);
	CHECK(a.stage2 == b.stage2);

	scan_gate_init(&a, w);
	for (i = 1u; i <= 3000u; ++i) {
		w = motion_ds_pack(0, MOTION_DS_STEADY, i, MOTION_DS_TURN_MAX);
		(void)scan_gate_step(&a, w, true, 10u);
	}
	CHECK(scan_gate_avg_x10(&a) > 20400);
	for (i = 1u; i <= 3000u; ++i) {
		w = motion_ds_pack(0, MOTION_DS_STEADY, i, MOTION_DS_TURN_MIN);
		(void)scan_gate_step(&a, w, true, 10u);
	}
	CHECK(scan_gate_avg_x10(&a) < -20400);
	CHECK(!a.open);
}

/* ---- the capture with the gate ---- */
static bar_capture_t cap;
static unsigned cap_widths;
static unsigned cap_ends;
static unsigned cap_aborts;

static void cap_ticks(unsigned n, bool black)
{
	uint16_t w = 0u;

	while (n-- > 0u) {
		bar_event_t ev = bar_capture_step(&cap, black, &w);

		cap_widths += (ev == BAR_EV_WIDTH) ? 1u : 0u;
		cap_ends += (ev == BAR_EV_END) ? 1u : 0u;
		cap_aborts += (ev == BAR_EV_ABORT) ? 1u : 0u;
	}
}

static void cap_reset(void)
{
	bar_capture_init(&cap);
	cap_widths = 0u;
	cap_ends = 0u;
	cap_aborts = 0u;
	cap_ticks(100u, false);
}

/* *A* at 20/60 ticks, gate as given after its first bar */
static void cap_sheet(bool open_after_first)
{
	static const char *const pat[3] = { "NWNNWNWNN", "WNNNNWNNW", "NWNNWNWNN" };
	unsigned s;
	unsigned i;

	for (s = 0u; s < 3u; ++s) {
		for (i = 0u; i < 9u; ++i) {
			cap_ticks((pat[s][i] == 'W') ? 60u : 20u, (i % 2u) == 0u);
			if ((s == 0u) && (i == 0u)) {
				bar_capture_set_gate(&cap, open_after_first);
			}
		}
		if (s < 2u) {
			cap_ticks(20u, false);
		}
	}
	cap_ticks(200u, false);
}

static void test_capture_gate(void)
{
	/* shut: a black opens no read, nothing is sent, no hold */
	cap_reset();
	CHECK(cap.gate_open);			/* open after init */
	bar_capture_set_gate(&cap, false);
	cap_ticks(400u, true);
	CHECK(!bar_capture_reading(&cap) && (cap.ignored == 1u));
	CHECK((cap_widths + cap_ends + cap_aborts) == 0u);
	/* nor the rest of it once the gate opens: a quiet zone first (white
	   is seen BARCODE_GLITCH_TICKS + 1 samples late) */
	bar_capture_set_gate(&cap, true);
	cap_ticks(300u, true);
	CHECK(!bar_capture_reading(&cap) && (cap.ignored == 1u));
	cap_ticks(BARCODE_GLITCH_TICKS + BARCODE_Q_TICKS - 1u, false);
	CHECK(bar_capture_busy(&cap));
	cap_ticks(1u, false);
	CHECK(!bar_capture_busy(&cap));
	cap_ticks(20u, true);
	CHECK(bar_capture_reading(&cap));	/* open: the next black reads */

	/* several blacks within a quiet zone of each other count once */
	cap_reset();
	bar_capture_set_gate(&cap, false);
	cap_ticks(30u, true);
	cap_ticks(20u, false);
	cap_ticks(30u, true);
	CHECK(cap.ignored == 1u);
	cap_ticks(BARCODE_GLITCH_TICKS + BARCODE_Q_TICKS, false);
	cap_ticks(30u, true);
	CHECK(cap.ignored == 2u);

	/* a read already open goes on: the whole sheet decodes with the gate
	   shut after its first bar */
	cap_reset();
	cap_sheet(false);
	CHECK((cap_ends == 1u) && (cap_aborts == 0u) && (cap_widths == 29u));
	CHECK(cap.complete && (cap.letter == 'A') && (cap.ignored == 0u));

	/* and with it open, the same (as before the gate) */
	cap_reset();
	cap_sheet(true);
	CHECK((cap_ends == 1u) && cap.complete && (cap.letter == 'A'));
}

/* ---- the length rule with the gate ---- */
static line_track_t trk;
static line_track_out_t out;

static unsigned feed(unsigned n, bool mark, bool turning, bool armed_due)
{
	line_junction_gate_t gate = { false, armed_due, false, turning };
	unsigned events = 0u;

	while (n-- > 0u) {
		line_track_step_gate(&trk, B, B, true, false, mark, &gate, &out);
		events += out.junction ? 1u : 0u;
	}
	return events;
}

static void test_length_rule_gate(void)
{
	ir_cal_sensor_t cal = ir_cal_default(true,
		LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
		LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);

	line_track_init(&trk, &cal, &cal, true);
	CHECK(feed(LINE_JUNCTION_REARM_SAMPLES, false, false, false) == 0u);

	/* a run that begins while turning never fires, and is no crossing */
	CHECK(feed(3u * LINE_JUNCTION_SAMPLES, true, true, false) == 0u);
	CHECK(!out.crossing);
	/* nor once the gate opens during it: the gate at its start holds */
	CHECK(feed(3u * LINE_JUNCTION_SAMPLES, true, false, false) == 0u);
	CHECK(!out.crossing);

	/* the next run, begun going straight, fires at J_n as before, even
	   if the gate shuts during it */
	CHECK(feed(LINE_JUNCTION_REARM_SAMPLES, false, false, false) == 0u);
	CHECK(feed(1u, true, false, false) == 0u);
	CHECK(feed(LINE_JUNCTION_SAMPLES - 2u, true, true, false) == 0u);
	CHECK(feed(1u, true, true, false) == 1u);
	CHECK(out.crossing);

	/* the armed junction fires while turning */
	CHECK(feed(LINE_JUNCTION_REARM_SAMPLES, false, false, false) == 0u);
	CHECK(feed(10u, true, true, false) == 0u);
	CHECK(feed(1u, true, true, true) == 1u);

	/* a mission change mid-run: the next black sample starts a run */
	CHECK(feed(LINE_JUNCTION_REARM_SAMPLES, false, false, false) == 0u);
	CHECK(feed(10u, true, true, false) == 0u);
	line_track_disarm_junction(&trk);
	CHECK(!trk.run_turning);
}

int main(void)
{
	(void)printf("scan gate: LINE_SCAN_GATE %u, limit %u deg/s, two stages of 1/%u every 10 ms, "
		     "stale after %u ms\n", (unsigned)LINE_SCAN_GATE,
		     (unsigned)LINE_SCAN_MAX_TURN_DPS, (unsigned)LINE_SCAN_AVG_DIV,
		     (unsigned)LINE_SCAN_STALE_MS);
	test_straight();
	test_weave();
	test_curves_pivots();
	test_fail_open();
	test_elapsed_and_range();
	test_capture_gate();
	test_length_rule_gate();
	return check_report("scan gate");
}
