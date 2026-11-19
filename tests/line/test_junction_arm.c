/* Host test for the barcode-armed junction (LINE_JUNCTION_RULE 1), end to
   end, one 1 ms sample at a time in line_task.c's order: bar_capture.c
   (END at the stop character) -> the 4-byte bar_mbf message ->
   barcode_rx.c -> barcode_task's store (modelled: the command first, then
   the judged word, after a latency) -> junction_arm.c -> line_track.c's
   gate -> NAV_FLG_JUNCTION, where nav's check (barcode_cmd_pending) runs.

   GP7 passes Code 39 sheets in the course format (*X*) at the sizes the
   course may use, then an 18 mm crossbar.  Positions are in mm on GP7's
   path; the car moves at motion's base speed (C: base x 100 counts/s,
   pi x 64 mm / 2535 counts); distance_state is whole mm every 10 ms.
   Sensor blur e: every bar (and the crossbar) reads e mm wider, every
   space e mm narrower.  None of these inputs is measured on the car.

   Built twice: the real rule, and, as test_junction_old (not in the
   default run), the branch's rule before it (length only, END after the
   quiet zone, quiet factor 4, one split in the decoder), which fails the
   envelope checks: that is what the new rule fixes.

   The scan gate (LINE_SCAN_GATE, scan_gate.c) runs in line_task.c's
   place, on distance_state built as motion publishes it (every 10 ms,
   the counter +1, the turn rate of the model below in whole deg/s).  It
   is also built with -DLINE_SCAN_GATE=0 (test_junction_arm_nogate, in the
   default run): every test before the gate passes unchanged, and the
   gate tests check the behaviour before it.  junction_scan_off (not in
   the default run) builds the gate out but expects it: those tests
   fail. */

#include <math.h>
#include <string.h>

#include "check.h"
#include "../../app_program/line_task/bar_capture.h"
#include "../../app_program/line_task/barcode_rx.h"
#include "../../app_program/line_task/junction_arm.h"
#include "../../app_program/line_task/line_track.h"
#include "../../app_program/line_task/scan_gate.h"
#include "../../app_program/motor_task/motion_ds.h"

/* The gate tests expect what the build has, unless told otherwise. */
#ifndef TEST_EXPECT_GATE
#define TEST_EXPECT_GATE LINE_SCAN_GATE
#endif

#define RAW_B (3500u)		/* line sensors: both on the line */
#define MM_PER_COUNT (3.14159265358979 * 64.0 / 2535.0)
#define CROSSBAR_MM (18.0)

static const char *const patterns[27] = {
	"WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",
	"WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",
	"NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",
	"WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",
	"NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",
	"WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",
	"WWNNWNNNN", "NWWNWNNNN", "NWNNWNWNN"	/* '*' */
};

/* ---- the course: black segments on GP7's path ---- */
typedef struct { double a; double b; } seg_t;
static seg_t segs[48];
static unsigned nsegs;
static double crossbar_a;	/* crossbar near edge as GP7 sees it, or < 0 */
static double crossbar_b;

/* One sheet *L* from x0 (mm): returns its last bar's printed end. */
static double sheet(char letter, double n, double r, double e, double x0)
{
	char text[3] = { '*', letter, '*' };
	double x = x0;
	unsigned k;
	unsigned i;

	for (k = 0u; k < 3u; ++k) {
		const char *p = patterns[(text[k] == '*') ? 26 : (text[k] - 'A')];

		for (i = 0u; i < 9u; ++i) {
			double w = (p[i] == 'W') ? n * r : n;

			if ((i % 2u) == 0u) {
				segs[nsegs].a = x - e / 2.0;
				segs[nsegs].b = x + w + e / 2.0;
				nsegs++;
			}
			x += w;
		}
		if (k < 2u) {
			x += n;		/* inter-character gap */
		}
	}
	return x;
}

static void black_run(double a, double b, double e)
{
	segs[nsegs].a = a - e / 2.0;
	segs[nsegs].b = b + e / 2.0;
	nsegs++;
}

static void crossbar(double at, double e)
{
	crossbar_a = at - e / 2.0;
	crossbar_b = at + CROSSBAR_MM + e / 2.0;
	black_run(at, at + CROSSBAR_MM, e);
}

static bool black_at(double x)
{
	unsigned i;

	for (i = 0u; i < nsegs; ++i) {
		if ((x >= segs[i].a) && (x < segs[i].b)) {
			return true;
		}
	}
	return false;
}

/* ---- the chain ---- */
typedef struct { unsigned t; uint8_t msg[BAR_MSG_SIZE]; } qmsg_t;

static bar_capture_t cap;
static barcode_rx_t rx;
static barcode_table_t table;
static line_track_t trk;
static line_track_out_t out;
#if LINE_JUNCTION_RULE
static junction_arm_t arm;
static line_junction_gate_t gate;
#endif
static qmsg_t queue[512];
static unsigned q_head;
static unsigned q_n;
static unsigned latency;	/* barcode_task, ms */
static int bt_dead;		/* barcode_task never answers */
static uint32_t cmd_word;	/* barcode_task's g_cmd */
static uint32_t cmd_seq;
static uint32_t judged;		/* barcode_task's g_judged */
static uint32_t nav_taken;	/* nav's last used sequence */
static unsigned now;		/* ms */
static uint32_t ds_mm;		/* distance_state, whole mm */
static uint32_t ds_prev;
static double gp7_x;		/* mm */

/* distance_state as motion_task publishes it, and the turn rate model:
   curve_dps from GP7 at curve_x on, plus a weave of weave_amp deg/s at
   weave_hz (sine, phase weave_ph) */
static scan_gate_t sg;
static uint32_t ds_word;
static uint32_t ds_ctr;
static bool motion_up;		/* motor_task_flag() > 0 */
static bool ds_frozen;		/* motion_task stopped publishing */
static double curve_x;
static double curve_dps;
static double weave_amp;
static double weave_hz;
static double weave_ph;
static unsigned held_ms;	/* samples with a read open (the hold) */

/* results */
static unsigned verdicts;
static barcode_verdict_t first_verdict;
static unsigned junctions;
static unsigned j_before;	/* junction events before the crossbar */
static unsigned j_cross;	/* on the crossbar (or up to 30 mm past) */
static bool j_cmd_ok;		/* first crossbar junction: right command pending */
static char j_letter;		/* the letter nav would use there, '-' none */
static double j_into;		/* mm into the crossbar */
static unsigned t_store;	/* ms of the last command store */
static unsigned t_junction;	/* ms of the first crossbar junction */
static char want;		/* the letter the command should carry */
static unsigned ev_armed;
static unsigned ev_rejected;
static unsigned ev_no_verdict;
static unsigned ev_disarm_dist;
static unsigned junction_ends;	/* reads ENDed by a junction (widths before it) */

static void chain_reset(bool table_loaded)
{
	ir_cal_sensor_t cal = ir_cal_default(true,
		LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
		LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);
	unsigned i;

	bar_capture_init(&cap);
	barcode_rx_init(&rx);
	barcode_table_init(&table);
	if (table_loaded) {
		/* A..Z cycle FORWARD, LEFT, RIGHT, U_TURN */
		(void)barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUFL");
	}
	line_track_init(&trk, &cal, &cal, true);
#if LINE_JUNCTION_RULE
	junction_arm_init(&arm);
#endif
	q_head = 0u;
	q_n = 0u;
	latency = 1u;
	bt_dead = 0;
	cmd_word = BARCODE_CMD_NONE;
	cmd_seq = 0u;
	judged = 0u;
	nav_taken = 0u;
	now = 0u;
	ds_mm = 0u;
	ds_prev = 0u;
	verdicts = 0u;
	(void)memset(&first_verdict, 0, sizeof(first_verdict));
	junctions = 0u;
	j_before = 0u;
	j_cross = 0u;
	j_cmd_ok = false;
	j_letter = '-';
	j_into = -1.0;
	t_store = 0u;
	t_junction = 0u;
	ev_armed = 0u;
	ev_rejected = 0u;
	ev_no_verdict = 0u;
	ev_disarm_dist = 0u;
	junction_ends = 0u;
	nsegs = 0u;
	crossbar_a = -1.0;
	crossbar_b = -1.0;
	ds_ctr = 0u;
	ds_word = motion_ds_pack(0, MOTION_DS_STEADY, 0u, 0);
	scan_gate_init(&sg, ds_word);
	motion_up = true;
	ds_frozen = false;
	curve_x = 1.0e9;
	curve_dps = 0.0;
	weave_amp = 0.0;
	weave_hz = 1.0;
	weave_ph = 0.0;
	held_ms = 0u;
	for (i = 0u; i < 300u; ++i) {		/* on the line, GP7 white */
		line_track_step(&trk, RAW_B, RAW_B, true, false, false, &out);
	}
}

static void q_send(bar_event_t ev, uint16_t value)
{
	qmsg_t *m = &queue[(q_head + q_n) % 512u];

	m->t = now;
	bar_msg_pack(m->msg, ev, cap.read_id, value);
	q_n++;
}

/* barcode_task: judge, store the command, then the judged word */
static void barcode_task_model(void)
{
	while ((q_n > 0u) && !bt_dead && ((queue[q_head].t + latency) <= now)) {
		barcode_rx_event_t re = barcode_rx_feed(&rx, queue[q_head].msg);

		q_head = (q_head + 1u) % 512u;
		q_n--;
		if (re != BARCODE_RX_NONE) {
			barcode_verdict_t v;

			barcode_rx_evaluate(&rx, re, &table, &v);
			if (v.accepted) {
				cmd_seq++;
				cmd_word = barcode_cmd_pack(barcode_letter_index(v.letter),
							    (uint32_t)v.action, cmd_seq);
				t_store = now;
			}
			judged = barcode_judged_pack(rx.read_id);
			if (verdicts == 0u) {
				first_verdict = v;
			}
			verdicts++;
		}
	}
}

/* One 1 ms sample, as line_task.c makes it. */
static void sample(bool black)
{
	uint16_t width = 0u;
	bar_event_t ev;
#if LINE_SCAN_GATE
	/* the scan gate, before the capture */
	bool scan_open = scan_gate_step(&sg, ds_word, motion_up, 1u);

	bar_capture_set_gate(&cap, scan_open);
#endif
	ev = bar_capture_step(&cap, black, &width);

	if (ev == BAR_EV_WIDTH) {
		q_send(ev, width);
	} else if (ev == BAR_EV_END) {
#if LINE_JUNCTION_RULE
		uint32_t before = cmd_word;	/* read before the END goes */

		q_send(ev, cap.count);
		if (cap.complete) {
			junction_arm_read_done(&arm, cap.read_id, cap.letter, before);
		}
#else
		q_send(ev, cap.count);
#endif
	} else if (ev == BAR_EV_ABORT) {
		q_send(ev, (uint16_t)cap.reason);
	} else {
		/* nothing finished */
	}
	barcode_task_model();
#if LINE_JUNCTION_RULE
	{
		/* the junction side counts GP7 after the capture's filter */
		bool seen = bar_capture_black(&cap);
		jarm_event_t je = junction_arm_step(&arm, judged, cmd_word, seen,
						    ds_mm - ds_prev,
						    bar_capture_in_read(&cap), &gate);

		ds_prev = ds_mm;
		ev_armed += (je == JARM_EV_ARMED) ? 1u : 0u;
		ev_rejected += (je == JARM_EV_REJECTED) ? 1u : 0u;
		ev_no_verdict += (je == JARM_EV_NO_VERDICT) ? 1u : 0u;
		ev_disarm_dist += (je == JARM_EV_DISARM_DISTANCE) ? 1u : 0u;
#if LINE_SCAN_GATE
		gate.turning = !scan_open;
#endif
		line_track_step_gate(&trk, RAW_B, RAW_B, true, false, seen, &gate, &out);
	}
#elif LINE_SCAN_GATE
	{
		line_junction_gate_t g0 = { false, false, false, !scan_open };

		line_track_step_gate(&trk, RAW_B, RAW_B, true, false, black, &g0, &out);
	}
#else
	line_track_step(&trk, RAW_B, RAW_B, true, false, black, &out);
#endif
	held_ms += bar_capture_reading(&cap) ? 1u : 0u;
	if (out.junction) {
		bool pending = barcode_cmd_pending(cmd_word, nav_taken);

#if LINE_JUNCTION_RULE
		bar_event_t jev;

		(void)junction_arm_used(&arm);
		jev = bar_capture_junction(&cap);	/* the crossbar is not a bar */
		if (jev != BAR_EV_NONE) {
			q_send(jev, (jev == BAR_EV_END) ? cap.count
						      : (uint16_t)BAR_ABORT_JUNCTION);
			junction_ends += (jev == BAR_EV_END) ? 1u : 0u;
		}
#endif
		junctions++;
		if ((crossbar_a >= 0.0) && (gp7_x >= crossbar_a) &&
		    (gp7_x < crossbar_b + 30.0)) {
			j_cross++;
			if (j_cross == 1u) {
				j_letter = pending ? (char)('A' + barcode_cmd_letter(cmd_word))
						   : '-';
				j_cmd_ok = pending && (j_letter == want);
				j_into = gp7_x - crossbar_a;
				t_junction = now;
			}
		} else if ((crossbar_a < 0.0) || (gp7_x < crossbar_a)) {
			j_before++;
		} else {
			/* past the crossbar */
		}
		if (pending) {
			nav_taken = barcode_cmd_seq(cmd_word);	/* nav used it */
		}
	}
}

/* motion_task's 10 ms publish of distance_state, with this turn rate */
static void publish(double dps)
{
	if (!ds_frozen) {
		ds_ctr++;
		ds_word = motion_ds_pack((int32_t)ds_mm, MOTION_DS_STEADY, ds_ctr,
					 (int32_t)lround(dps));
	}
}

static double turn_model(void)
{
	return ((gp7_x >= curve_x) ? curve_dps : 0.0) +
	       weave_amp * sin(2.0 * 3.14159265358979 * weave_hz * (double)now / 1000.0 +
			       weave_ph);
}

/* Drive GP7 from x0 to x1 at the base speed; amp: GP7's speed varies
   +/-amp (a triangle wave of period 700 ms) while the car's does not. */
static void drive(double x0, double x1, unsigned base, double amp)
{
	double v = (double)base * 100.0 * MM_PER_COUNT / 1000.0;	/* mm/ms */
	unsigned t0 = now;

	gp7_x = x0;
	while (gp7_x < x1) {
		unsigned ph = (now - t0) % 700u;
		double tri = (ph < 350u) ? ((double)ph / 175.0 - 1.0)
					 : (3.0 - (double)ph / 175.0);

		if ((now % 10u) == 0u) {
			ds_mm = (uint32_t)(v * (double)now);
			publish(turn_model());
		}
		sample(black_at(gp7_x));
		gp7_x += v * (1.0 + amp * tri);
		now++;
	}
}

/* ---- the size envelope ---- */
typedef struct {
	unsigned runs;
	unsigned decoded;
	unsigned j_inside;
	unsigned cross_once;
	unsigned cmd_ok;
	double into_min;	/* armed junctions: mm into the crossbar */
	double into_max;
} tally_t;

/* One sheet, a gap, the crossbar, 100 mm on. */
static void one(char letter, double n, double r, double e, unsigned base,
		double amp, double gap, tally_t *t)
{
	double end;

	chain_reset(true);
	want = letter;
	end = sheet(letter, n, r, e, 0.0);
	crossbar(end + gap, e);
	drive(-40.0, crossbar_b + 100.0, base, amp);
	t->runs++;
	t->decoded += (first_verdict.accepted && (first_verdict.letter == letter)) ? 1u : 0u;
	t->j_inside += j_before;
	t->cross_once += ((j_cross == 1u) && (junctions == 1u)) ? 1u : 0u;
	t->cmd_ok += j_cmd_ok ? 1u : 0u;
	if (j_cmd_ok) {
		if ((t->runs == 1u) || (j_into < t->into_min)) {
			t->into_min = j_into;
		}
		if ((t->runs == 1u) || (j_into > t->into_max)) {
			t->into_max = j_into;
		}
	}
}

/* Every size in the envelope: narrow 2-6 mm, wide:narrow 2-3, base 4, 6
   and 8, blur -0.5..+0.5 mm, steady GP7 speed: each read decodes, nothing
   fires inside the barcode, the crossbar fires exactly once, with the
   command.  The course sheet (3 mm, 3:1) with all 26 letters; the other
   sizes with 8 letters that cover every wide-bar position. */
static void test_envelope(void)
{
	static const double ns[] = { 2.0, 2.5, 3.0, 4.0, 5.0, 6.0 };
	static const double rs[] = { 2.0, 2.5, 3.0 };
	static const unsigned bases[] = { 4u, 6u, 8u };
	static const double es[] = { -0.5, 0.0, 0.5 };
	static const char few[] = "ABEKMRUZ";
	tally_t t;
	unsigned a;
	unsigned b;
	unsigned c;
	unsigned d;

	(void)memset(&t, 0, sizeof(t));
	for (a = 0u; a < 6u; ++a) {
		for (b = 0u; b < 3u; ++b) {
			for (c = 0u; c < 3u; ++c) {
				for (d = 0u; d < 3u; ++d) {
					bool course = (ns[a] == 3.0) && (rs[b] == 3.0);
					unsigned k;
					unsigned count = course ? 26u : 8u;

					for (k = 0u; k < count; ++k) {
						char letter = course ? (char)('A' + k) : few[k];

						one(letter, ns[a], rs[b], es[d], bases[c],
						    0.0, 40.0, &t);
					}
				}
			}
		}
	}
	(void)printf("  envelope: %u runs, decoded %u, junctions inside a barcode %u, "
		     "crossbar exactly once %u, command at the junction %u, "
		     "%.1f-%.1f mm into the crossbar\n",
		     t.runs, t.decoded, t.j_inside, t.cross_once, t.cmd_ok,
		     t.into_min, t.into_max);
	CHECK(t.decoded == t.runs);
	CHECK(t.j_inside == 0u);
	CHECK(t.cross_once == t.runs);
	CHECK(t.cmd_ok == t.runs);
#if LINE_JUNCTION_RULE
	/* armed: LINE_ARMED_CONFIRM_MM of travel on the crossbar, at any
	   speed (distance_state is whole mm every 10 ms) */
	CHECK(t.into_min > (double)LINE_ARMED_CONFIRM_MM - 1.5);
	CHECK(t.into_max < (double)LINE_ARMED_CONFIRM_MM + 1.5);
#endif
}

/* The course sheet with GP7's speed +/-10 % and +/-20 % off the car's
   (steering), and blur up to 1 mm: still one junction with the command
   wherever the read decodes, never one inside. */
static void test_course_sheet_steering(void)
{
	static const double amps[] = { 0.1, 0.2 };
	static const double es[] = { -1.0, -0.5, 0.0, 0.5, 1.0 };
	tally_t t;
	unsigned a;
	unsigned d;
	char letter;

	(void)memset(&t, 0, sizeof(t));
	for (a = 0u; a < 2u; ++a) {
		for (d = 0u; d < 5u; ++d) {
			for (letter = 'A'; letter <= 'Z'; ++letter) {
				one(letter, 3.0, 3.0, es[d], 6u, amps[a], 40.0, &t);
			}
		}
	}
	(void)printf("  course sheet, GP7 speed +/-10..20 %%, blur -1..+1 mm, base 6: %u runs, "
		     "decoded %u, inside %u, crossbar once %u, command %u\n",
		     t.runs, t.decoded, t.j_inside, t.cross_once, t.cmd_ok);
	CHECK(t.j_inside == 0u);
	CHECK(t.cmd_ok == t.decoded);
	CHECK(t.decoded * 10u >= t.runs * 9u);	/* at least 90 % (247 of 260 here) */
}

/* The crossbar right behind the sheet (gap from the last bar's end, as
   printed): 0 (the last bar printed against it), 5, 10, 20, 40, 80 mm. */
static void test_crossbar_gaps(void)
{
	static const double gaps[] = { 0.0, 5.0, 10.0, 20.0, 40.0, 80.0 };
	static const unsigned bases[] = { 4u, 6u, 8u };
	tally_t t;
	unsigned g;
	unsigned b;
	char letter;

	(void)memset(&t, 0, sizeof(t));
	for (g = 0u; g < 6u; ++g) {
		for (b = 0u; b < 3u; ++b) {
			for (letter = 'A'; letter <= 'Z'; ++letter) {
				one(letter, 3.0, 3.0, 0.0, bases[b], 0.0, gaps[g], &t);
			}
		}
	}
	(void)printf("  crossbar 0-80 mm after the sheet: %u runs, decoded %u, crossbar once %u, "
		     "command %u\n", t.runs, t.decoded, t.cross_once, t.cmd_ok);
	CHECK(t.decoded == t.runs);
	CHECK(t.cross_once == t.runs);
	CHECK(t.cmd_ok == t.runs);

	/* the big sheet (6 mm bars) printed against the crossbar */
	(void)memset(&t, 0, sizeof(t));
	for (letter = 'A'; letter <= 'Z'; ++letter) {
		one(letter, 6.0, 3.0, 0.0, 4u, 0.0, 0.0, &t);
	}
	CHECK(t.cmd_ok == t.runs);
}

#if LINE_JUNCTION_RULE
/* The command is stored before the junction flag, with barcode_task up to
   40 ms slow, even with the crossbar right behind the sheet; and nav's
   check at the flag finds it.  barcode_task dead: after
   LINE_ARMED_VERDICT_WAIT_MS the junction goes anyway (straight on). */
static void test_store_before_flag(void)
{
	static const unsigned lats[] = { 0u, 1u, 5u, 20u, 40u };
	unsigned i;
	double end;

	for (i = 0u; i < 5u; ++i) {
		chain_reset(true);
		latency = lats[i];
		want = 'L';
		end = sheet('L', 3.0, 3.0, 0.0, 0.0);
		crossbar(end, 0.0);			/* printed against it */
		drive(-40.0, crossbar_b + 50.0, 6u, 0.0);
		CHECK(ev_armed == 1u);
		CHECK(j_cross == 1u);
		CHECK(j_cmd_ok);
		CHECK(t_store < t_junction);
	}

	chain_reset(true);
	bt_dead = 1;
	want = 'L';
	end = sheet('L', 3.0, 3.0, 0.0, 0.0);
	crossbar(end, 0.0);
	drive(-40.0, crossbar_b + 50.0, 6u, 0.0);
	CHECK(ev_no_verdict == 1u);
	CHECK(ev_armed == 0u);
	CHECK(j_cross == 1u);			/* the length rule */
	CHECK(!j_cmd_ok && (j_letter == '-'));
}

/* A crossbar black only 12 mm long (shorter than J_n at base 6: 252 ms)
   after the sheet: it fires only when armed. */
static unsigned short_bar_junctions(double from)
{
	unsigned before = junctions;

	black_run(from, from + 12.0, 0.0);
	drive(gp7_x, from + 40.0, 6u, 0.0);
	return junctions - before;
}

/* Noise, a rejected read and an unknown letter do not arm. */
static void test_no_arm(void)
{
	double x;
	unsigned i;

	/* noise: 2 ms blips of black */
	chain_reset(true);
	gp7_x = -40.0;
	for (i = 0u; i < 20u; ++i) {
		black_run(gp7_x + 5.0 + (double)i * 6.0, gp7_x + 5.1 + (double)i * 6.0, 0.0);
	}
	drive(-40.0, 100.0, 6u, 0.0);
	CHECK(ev_armed == 0u);
	CHECK(short_bar_junctions(120.0) == 0u);

	/* malformed: 15 equal bars and spaces (no wide element) */
	chain_reset(true);
	for (i = 0u; i < 15u; ++i) {
		black_run((double)i * 6.0, (double)i * 6.0 + 3.0, 0.0);
	}
	drive(-40.0, 150.0, 6u, 0.0);
	CHECK((verdicts == 1u) && !first_verdict.accepted);
	CHECK(ev_armed == 0u);
	CHECK(short_bar_junctions(160.0) == 0u);

	/* a valid *C*, but no table: decoded, rejected as unknown */
	chain_reset(false);
	x = sheet('C', 3.0, 3.0, 0.0, 0.0);
	drive(-40.0, x + 20.0, 6u, 0.0);
	CHECK((verdicts == 1u) && (first_verdict.reject == BARCODE_REJECT_UNKNOWN));
	CHECK(ev_rejected == 1u);
	CHECK(ev_armed == 0u);
	CHECK(short_bar_junctions(x + 40.0) == 0u);

	/* the same with the table: armed, so the short black fires */
	chain_reset(true);
	x = sheet('C', 3.0, 3.0, 0.0, 0.0);
	drive(-40.0, x + 20.0, 6u, 0.0);
	CHECK(ev_armed == 1u);
	CHECK(short_bar_junctions(x + 40.0) == 1u);
}

/* D_hold: armed, then LINE_ARMED_HOLD_MM with no black: disarmed (the
   command stays for nav); a mission change disarms too. */
static void test_disarm(void)
{
	double x;

	chain_reset(true);
	x = sheet('R', 3.0, 3.0, 0.0, 0.0);
	drive(-40.0, x + (double)LINE_ARMED_HOLD_MM - 20.0, 6u, 0.0);
	CHECK((ev_armed == 1u) && (ev_disarm_dist == 0u));
	drive(gp7_x, x + (double)LINE_ARMED_HOLD_MM + 20.0, 6u, 0.0);
	CHECK(ev_disarm_dist == 1u);
	CHECK(short_bar_junctions(gp7_x + 10.0) == 0u);
	CHECK(barcode_cmd_pending(cmd_word, nav_taken));	/* kept for nav */

	chain_reset(true);
	x = sheet('R', 3.0, 3.0, 0.0, 0.0);
	drive(-40.0, x + 20.0, 6u, 0.0);
	CHECK(arm.phase == JARM_ARMED);
	CHECK(junction_arm_cancel(&arm) == JARM_EV_DISARM_MISSION);	/* line_task.c */
	CHECK(junction_arm_cancel(&arm) == JARM_EV_NONE);		/* once */
	CHECK(short_bar_junctions(x + 40.0) == 0u);
}

/* Two sheets before one junction: the second's command is the one used,
   and its first bar is not the junction (it is narrow, under
   LINE_ARMED_CONFIRM_MM).  Sizes: same, and a 6 mm sheet after a 2 mm. */
static void test_two_sheets(void)
{
	double x;

	chain_reset(true);
	want = 'U';
	x = sheet('A', 3.0, 3.0, 0.0, 0.0);
	x = sheet('U', 3.0, 3.0, 0.0, x + 20.0);
	crossbar(x + 30.0, 0.0);
	drive(-40.0, crossbar_b + 60.0, 6u, 0.0);
	CHECK(ev_armed == 2u);
	CHECK(j_before == 0u);
	CHECK((j_cross == 1u) && j_cmd_ok && (j_letter == 'U'));

	chain_reset(true);
	want = 'R';
	x = sheet('E', 2.0, 3.0, 0.0, 0.0);
	x = sheet('R', 6.0, 3.0, 0.5, x + 20.0);
	crossbar(x + 30.0, 0.5);
	drive(-40.0, crossbar_b + 60.0, 4u, 0.0);
	CHECK(ev_armed == 2u);
	CHECK(j_before == 0u);
	CHECK((j_cross == 1u) && j_cmd_ok);

	/* the second read gets no verdict (barcode_task stuck): still armed
	   by the first, whose command nav uses */
	chain_reset(true);
	want = 'A';
	x = sheet('A', 3.0, 3.0, 0.0, 0.0);
	drive(-40.0, x + 10.0, 6u, 0.0);
	CHECK(ev_armed == 1u);
	bt_dead = 1;
	x = sheet('U', 3.0, 3.0, 0.0, x + 20.0);
	crossbar(x + 30.0, 0.0);
	drive(gp7_x, crossbar_b + 60.0, 6u, 0.0);
	CHECK(ev_no_verdict == 1u);
	CHECK(j_before == 0u);
	CHECK((j_cross == 1u) && j_cmd_ok && (j_letter == 'A'));
}

/* No barcode: the length rule still fires on the crossbar, once; and a
   crossbar after an unreadable sheet (the read swallows it) still fires
   once the read has passed the 29 widths of a one-letter barcode. */
static void test_fallback(void)
{
	unsigned i;

	chain_reset(true);
	crossbar(50.0, 0.0);
	drive(-40.0, 50.0 + 14.0, 6u, 0.0);
	CHECK((junctions == 1u) && (j_cross == 1u) && (j_letter == '-'));
	CHECK(ev_armed == 0u);
	/* fired at J_n: 280 ms into the crossbar at 47.6 mm/s (C: 13.3 mm),
	   plus the glitch filter's lag (6 samples, 0.3 mm) */
	CHECK((j_into > 12.5) && (j_into < 14.0));
	/* and its own read is dropped there: no line_state hold, no reject */
	CHECK(!bar_capture_reading(&cap));
	drive(gp7_x, 200.0, 6u, 0.0);
	CHECK((verdicts == 1u) && (first_verdict.reject == BARCODE_REJECT_ABORTED));
	CHECK(rx.abort_reason == BAR_ABORT_JUNCTION);
	CHECK(junctions == 1u);

	/* 15 equal bars (unreadable), then the crossbar 5 mm after */
	chain_reset(true);
	for (i = 0u; i < 15u; ++i) {
		black_run((double)i * 6.0, (double)i * 6.0 + 3.0, 0.0);
	}
	crossbar(87.0 + 5.0, 0.0);
	drive(-40.0, crossbar_b + 100.0, 6u, 0.0);
	CHECK((junctions == 1u) && (j_cross == 1u));
}
#endif

#if LINE_JUNCTION_RULE
/* H1 (checker, 19 Nov): a crossbar that starts inside an open read with
   1-28 widths fired nothing, armed or not, because no black run inside a
   read was a junction.  Each case below must give exactly one junction,
   at the crossbar, and none before it. */
typedef struct {
	unsigned runs;
	unsigned once;		/* one junction, at the crossbar, none before */
	unsigned letter_ok;	/* nav found the expected command (or none) */
	double into_max;
} h1_tally_t;

/* A white gap of exactly n samples at about x on GP7's path (drive()
   from -40 mm at a steady base): it starts half a sample past a sample,
   so the n samples it covers do not depend on rounding. */
static void white_gap(double c1, double x, unsigned n, unsigned base)
{
	double v = (double)base * 100.0 * MM_PER_COUNT / 1000.0;
	double a = -40.0 + ((double)(unsigned long)((x + 40.0) / v) + 0.5) * v;

	black_run(c1, a, 0.0);
	black_run(a + (double)n * v, c1 + CROSSBAR_MM, 0.0);
	crossbar_a = c1;
	crossbar_b = c1 + CROSSBAR_MM;
}

static void h1_count(h1_tally_t *t)
{
	t->runs++;
	t->once += ((junctions == 1u) && (j_cross == 1u) && (j_before == 0u)) ? 1u : 0u;
	t->letter_ok += (j_letter == want) ? 1u : 0u;
	if (j_into > t->into_max) {
		t->into_max = j_into;
	}
}

/* Sheet B (LEFT in this table) to arm the junction, or none; returns
   where the crossbar starts: 40 mm after the sheet, or at 60 mm. */
static double h1_start(bool armed)
{
	chain_reset(true);
	want = armed ? 'B' : '-';
	return armed ? sheet('B', 3.0, 3.0, 0.0, 0.0) + 40.0 : 60.0;
}

static void test_h1_crossbar_in_read(void)
{
	static const unsigned bases[] = { 4u, 6u, 8u };
	static const unsigned blip_ms[] = { 1u, 2u, 3u };
	static const double blip_at[] = { 0.5, 12.0, 12.0 };
	h1_tally_t drop, speck, blip, unread;
	double into = 0.0;
	unsigned b;
	unsigned k;
	int armed;
	double c1;

	(void)memset(&drop, 0, sizeof(drop));
	(void)memset(&speck, 0, sizeof(speck));
	(void)memset(&blip, 0, sizeof(blip));
	(void)memset(&unread, 0, sizeof(unread));

	/* 1. armed sheet, a white dropout 3 mm into the crossbar one sample
	   longer than a glitch the filter merges (the GP7 log has a 4 ms one:
	   trials_batch2_2026-11-04.txt:196, B63 w4 B24; see 1b for that one).
	   Then a second crossroad 300 mm on, no sheet: the command was used at
	   the first, so nav finds none there. */
	for (b = 0u; b < 3u; ++b) {
		c1 = h1_start(true);
		white_gap(c1, c1 + 3.0, BARCODE_GLITCH_TICKS + 1u, bases[b]);
		drive(-40.0, c1 + 100.0, bases[b], 0.0);
		h1_count(&drop);
		CHECK(junction_ends == 1u);	/* the 2-width read ENDed there */
		CHECK(verdicts == 2u);		/* the sheet, then that read */
		crossbar(c1 + 300.0, 0.0);
		j_cross = 0u;
		j_letter = '?';
		drive(gp7_x, c1 + 400.0, bases[b], 0.0);
		CHECK((j_cross == 1u) && (j_letter == '-'));
	}

	/* 1b. the logged 4 ms dropout itself (B63 w4 B24): within the filter
	   (BARCODE_GLITCH_TICKS 6), so it is merged: the junction fires with
	   the sheet's command and no 2-width read is ENDed inside the
	   crossbar; only the crossbar's own read, dropped as usual. */
	for (b = 0u; b < 3u; ++b) {
		c1 = h1_start(true);
		white_gap(c1, c1 + 3.0, 4u, bases[b]);
		drive(-40.0, c1 + 100.0, bases[b], 0.0);
		h1_count(&drop);
		CHECK(junction_ends == 0u);	/* merged: no 2-width read */
		CHECK(verdicts == 2u);		/* the sheet, then the crossbar's
						   own read, dropped (aborted) */
	}

	/* 2. a 2 mm speck (a brush of the line, a sheet edge) 5 and 10 mm
	   before the crossbar, armed and not */
	for (armed = 1; armed >= 0; --armed) {
		for (k = 0u; k < 2u; ++k) {
			double gap = (k == 0u) ? 5.0 : 10.0;

			for (b = 0u; b < 3u; ++b) {
				c1 = h1_start(armed != 0);
				black_run(c1 - gap - 2.0, c1 - gap, 0.0);
				crossbar(c1, 0.0);
				drive(-40.0, c1 + 100.0, bases[b], 0.0);
				h1_count(&speck);
				CHECK(junction_ends == 1u);
			}
		}
	}

	/* 3. a 1 ms white blip 0.5 mm into the crossbar, armed and not; and
	   2 or 3 ms at 12 mm in, where the rest of the crossbar is shorter
	   than either rule: the junction side counts GP7 after the filter */
	for (armed = 1; armed >= 0; --armed) {
		for (k = 0u; k < 3u; ++k) {
			for (b = 0u; b < 3u; ++b) {
				c1 = h1_start(armed != 0);
				white_gap(c1, c1 + blip_at[k], blip_ms[k], bases[b]);
				drive(-40.0, c1 + 100.0, bases[b], 0.0);
				h1_count(&blip);
				CHECK(junction_ends == 0u);	/* one run: the crossbar's own read */
			}
		}
	}

	/* 4. an unreadable sheet (tape over it, merged bars): 9, 11 or 13
	   equal 3 mm bars (17, 21, 25 widths), the crossbar 5 or 10 mm
	   after, inside the read's quiet zone (6 x 3 mm) */
	for (k = 0u; k < 6u; ++k) {
		unsigned nb = 9u + 2u * (k % 3u);
		double gap = (k < 3u) ? 5.0 : 10.0;
		unsigned i;

		for (b = 0u; b < 3u; ++b) {
			chain_reset(true);
			want = '-';
			for (i = 0u; i < nb; ++i) {
				black_run((double)i * 6.0, (double)i * 6.0 + 3.0, 0.0);
			}
			crossbar((double)(nb - 1u) * 6.0 + 3.0 + gap, 0.0);
			drive(-40.0, crossbar_b + 100.0, bases[b], 0.0);
			/* base 8: the length rule's J_n 280 is 17.8 mm, at the
			   crossbar's very end (C); it fires there or not at all */
			if (bases[b] != 8u) {
				h1_count(&unread);
				CHECK(junction_ends == 1u);
				CHECK((verdicts == 1u) && (first_verdict.reject == BARCODE_REJECT_DECODE));
			} else {
				CHECK((j_before == 0u) && (junctions <= 1u));
			}
		}
	}

	into = (drop.into_max > into) ? drop.into_max : into;
	into = (speck.into_max > into) ? speck.into_max : into;
	into = (blip.into_max > into) ? blip.into_max : into;
	into = (unread.into_max > into) ? unread.into_max : into;
	(void)printf("  H1, crossbar inside an open read (base 4/6/8): dropout %u/%u, speck %u/%u, "
		     "blip %u/%u, unreadable sheet %u/%u fire once at the crossbar "
		     "(the latest %.1f mm in)\n",
		     drop.once, drop.runs, speck.once, speck.runs, blip.once, blip.runs,
		     unread.once, unread.runs, into);
	CHECK((drop.once == drop.runs) && (drop.letter_ok == drop.runs));
	CHECK((speck.once == speck.runs) && (speck.letter_ok == speck.runs));
	CHECK((blip.once == blip.runs) && (blip.letter_ok == blip.runs));
	CHECK(unread.once == unread.runs);
}

/* What the H1 fix must not do: fire inside a barcode.  A second sheet
   before the same junction, read either way round, at 2-6 mm bars, 2-3 x,
   base 4/6/8, blur -1..+1 mm, GP7's speed steady and +/-20 %: no
   junction before the crossbar, and the second sheet's command is used
   wherever it decoded.  Its first black runs are where a bar can be
   several times every width before it (the reversed start character's
   wide bar is its 3rd width, and a read that began part way through a
   barcode can have 8 narrow widths before a wide bar). */
static void test_h1_no_junction_in_barcodes(void)
{
	static const double ns[] = { 2.0, 3.0, 4.0, 6.0 };
	static const double rs[] = { 2.0, 3.0 };
	static const unsigned bases[] = { 4u, 6u, 8u };
	static const double es[] = { -1.0, 0.0, 1.0 };
	static const double amps[] = { 0.0, 0.2 };
	unsigned runs = 0u;
	unsigned inside = 0u;
	unsigned a, b, c, d, f;
	int rev;

	for (rev = 0; rev <= 1; ++rev)
	for (a = 0u; a < 4u; ++a) for (b = 0u; b < 2u; ++b) for (c = 0u; c < 3u; ++c)
	for (d = 0u; d < 3u; ++d) for (f = 0u; f < 2u; ++f) {
		double x;
		uint16_t w[29];
		unsigned m = 0u;
		unsigned i;
		unsigned s;
		char text[3] = { '*', 'H', '*' };

		chain_reset(true);
		want = 'H';
		x = sheet('A', 3.0, 3.0, 0.0, 0.0) + 20.0;
		for (s = 0u; s < 3u; ++s) {
			const char *p = patterns[(text[s] == '*') ? 26 : (text[s] - 'A')];

			for (i = 0u; i < 9u; ++i) {
				w[m++] = (uint16_t)((p[i] == 'W') ? 2u : 1u);
			}
			if (s < 2u) {
				w[m++] = 1u;
			}
		}
		for (i = 0u; i < 29u; ++i) {
			double wi = ((rev ? w[28u - i] : w[i]) == 2u) ? ns[a] * rs[b] : ns[a];

			if ((i % 2u) == 0u) {
				black_run(x, x + wi, es[d]);
			}
			x += wi;
		}
		crossbar(x + 30.0, es[d]);
		drive(-40.0, crossbar_b + 60.0, bases[c], amps[f]);
		runs++;
		inside += j_before;
	}
	(void)printf("  a second sheet either way round before the junction: %u runs, "
		     "junctions inside a barcode %u\n", runs, inside);
	CHECK(inside == 0u);
}
#endif

#if LINE_JUNCTION_RULE
/* ---- The scan gate (the prof's tip, 19 Nov: barcodes are only on
   straight segments) ---- */

/* base 6: v / R in deg/s (C) */
static double curve_rate(double radius_mm)
{
	double v = 6.0 * 100.0 * MM_PER_COUNT;	/* mm/s */

	return v / radius_mm * 180.0 / 3.14159265358979;
}

/* The course sheet, the crossbar 40 mm after, from x0 at base 6. */
static void sheet_run(char letter, double x0)
{
	double end;

	want = letter;
	end = sheet(letter, 3.0, 3.0, 0.0, 0.0);
	crossbar(end + 40.0, 0.0);
	drive(x0, crossbar_b + 100.0, 6u, 0.0);
}

/* A sheet on a straight, and on a straight where the car weaves +/-45
   deg/s at 0.8, 1 and 1.25 Hz, 8 phases, after 300 mm of line
   following: every read decodes, arms, and the crossbar fires with its
   command; the gate never turns a black away. */
static void test_scan_straight(void)
{
	static const double hzs[] = { 0.8, 1.0, 1.25 };
	static const char letters[] = "ABEKMRUZ";
	unsigned runs = 0u;
	unsigned ok = 0u;
	unsigned ignored = 0u;
	unsigned f;
	unsigned k;
	unsigned l;

	chain_reset(true);
	sheet_run('C', -300.0);
	CHECK((ev_armed == 1u) && (j_cross == 1u) && j_cmd_ok);
	CHECK(sg.available && sg.open && (cap.ignored == 0u));

	for (f = 0u; f < 3u; ++f) {
		for (k = 0u; k < 8u; ++k) {
			for (l = 0u; l < 8u; ++l) {
				chain_reset(true);
				weave_amp = 45.0;
				weave_hz = hzs[f];
				weave_ph = 2.0 * 3.14159265358979 * (double)k / 8.0;
				sheet_run(letters[l], -300.0);
				runs++;
				ok += ((ev_armed == 1u) && (j_cross == 1u) && (junctions == 1u) &&
				       j_cmd_ok) ? 1u : 0u;
				ignored += cap.ignored;
			}
		}
	}
	(void)printf("  scan gate, weaving straight (+/-45 deg/s, 0.8-1.25 Hz): %u runs, "
		     "decoded, armed and fired with the command %u, blacks turned away %u\n",
		     runs, ok, ignored);
	CHECK(ok == runs);
	CHECK(ignored == 0u);
}

typedef struct {
	unsigned opened;	/* reads opened */
	unsigned fired;		/* junction events */
	unsigned held;		/* ms with a read open (the steering hold) */
	unsigned judged;	/* verdicts (rejects printed) */
} sweep_t;

/* GP7 sweeps the line on a curve: one black run of 3, 9, 15, 25 or
   50 mm (63 ms to 1.05 s at base 6, C; 15 mm and more is past J_n 280),
   each in a run of its own, d0 mm into the curve.  The curve (turn rate
   dps) begins at x = 100 after 400 mm of straight line following.  Where
   GP7 meets the line on a curve depends on its mount, not measured (S3);
   the gate needs the averaged rate to rise first.  amp, hz, ph: a weave
   on top (amp 0: none).  sweep_ds: 0 distance_state as usual, 1
   motor_task_flag() 0, 2 motion_task not publishing. */
static unsigned sweep_ds;

static void curve_sweeps(double dps, double d0, double amp, double hz,
			 double ph, sweep_t *r)
{
	static const double lens[] = { 3.0, 9.0, 15.0, 25.0, 50.0 };
	unsigned i;

	(void)memset(r, 0, sizeof(*r));
	for (i = 0u; i < 5u; ++i) {
		uint8_t id0;
		double x = 100.0 + d0;

		chain_reset(true);
		want = '-';
		weave_amp = amp;
		weave_hz = hz;
		weave_ph = ph;
		motion_up = (sweep_ds != 1u);
		ds_frozen = (sweep_ds == 2u);
		curve_x = 100.0;
		curve_dps = dps;
		black_run(x, x + lens[i], 0.0);
		id0 = cap.read_id;
		drive(-300.0, x + lens[i] + 80.0, 6u, 0.0);
		r->opened += (uint8_t)(cap.read_id - id0);
		r->fired += junctions;
		r->held += held_ms;
		r->judged += verdicts;
	}
}

static void test_scan_curves(void)
{
	static const double radii[] = { 150.0, 300.0 };
	sweep_t r;
	double from[2] = { 0.0, 0.0 };	/* the larger of the two directions */
	unsigned c;
	int dir;
	unsigned k;

	for (c = 0u; c < 2u; ++c) {
		for (dir = -1; dir <= 1; dir += 2) {
			double dps = (double)dir * curve_rate(radii[c]);
			double d0;
			double clear = -1.0;

			/* from 100 mm into the curve (2.1 s at base 6) */
			for (d0 = 100.0; d0 <= 160.0; d0 += 20.0) {
				curve_sweeps(dps, d0, 0.0, 1.0, 0.0, &r);
				if (TEST_EXPECT_GATE) {
					CHECK((r.opened == 0u) && (r.fired == 0u));
					CHECK((r.held == 0u) && (r.judged == 0u));
				} else {
					/* before the gate: every black opens a read, and
					   the three past J_n fire junctions */
					CHECK((r.opened == 5u) && (r.fired == 3u));
					CHECK(r.held > 0u);
				}
			}
			/* report: from how far in (5 mm steps) nothing opens */
			for (d0 = 0.0; d0 <= 100.0; d0 += 5.0) {
				curve_sweeps(dps, d0, 0.0, 1.0, 0.0, &r);
				if ((r.opened == 0u) && (r.fired == 0u) && (clear < 0.0)) {
					clear = d0;
				} else if ((r.opened != 0u) || (r.fired != 0u)) {
					clear = -1.0;
				} else {
					/* still nothing */
				}
			}
			if ((clear < 0.0) || (from[c] < 0.0)) {
				from[c] = -1.0;
			} else if (clear > from[c]) {
				from[c] = clear;
			} else {
				/* the other direction cleared later */
			}
		}
	}
	(void)printf("  scan gate, GP7 sweeping the line in a curve, both ways: nothing opens from "
		     "%.0f mm into a 150 mm curve (%.1f deg/s), %.0f mm into a 300 mm curve "
		     "(%.1f deg/s) (-1: something opens up to 100 mm in)\n",
		     from[0], curve_rate(150.0), from[1], curve_rate(300.0));

	/* a curve with the +/-45 deg/s weave on top, 8 phases, from 100 mm:
	   the 150 mm curve stays shut; the 300 mm one (9.1 deg/s) is near
	   the limit, so it is reported, not checked */
	{
		unsigned bad150 = 0u;
		unsigned bad300 = 0u;
		unsigned bad300_08 = 0u;

		for (k = 0u; k < 8u; ++k) {
			double phs = 2.0 * 3.14159265358979 * (double)k / 8.0;

			curve_sweeps(curve_rate(150.0), 100.0, 45.0, 1.0, phs, &r);
			bad150 += r.opened + r.fired;
			curve_sweeps(curve_rate(300.0), 100.0, 45.0, 1.0, phs, &r);
			bad300 += r.opened + r.fired;
			curve_sweeps(curve_rate(300.0), 100.0, 45.0, 0.8, phs, &r);
			bad300_08 += r.opened + r.fired;
		}
		(void)printf("  scan gate, curve plus the +/-45 deg/s weave, 8 phases, from 100 mm "
			     "(64 without the gate: 40 reads, 24 junctions): reads opened + junctions "
			     "150 mm 1 Hz %u, 300 mm 1 Hz %u, 300 mm 0.8 Hz %u\n",
			     bad150, bad300, bad300_08);
		if (TEST_EXPECT_GATE) {
			CHECK(bad150 == 0u);
		} else {
			CHECK(bad150 > 0u);
		}
	}
}
/* The car turning on the spot at dps for ms (distance_state's distance
   stays, its turn rate is dps); GP7 black from on[i] for len[i] ms. */
static void spin(unsigned ms, double dps, const unsigned *on,
		 const unsigned *len, unsigned n)
{
	unsigned t0 = now;

	while ((now - t0) < ms) {
		unsigned t = now - t0;
		bool b = false;
		unsigned i;

		for (i = 0u; i < n; ++i) {
			b = b || ((t >= on[i]) && (t < on[i] + len[i]));
		}
		if ((now % 10u) == 0u) {
			publish(dps);
		}
		sample(b);
		now++;
	}
}

/* A U-turn pivot after 400 mm of line following, at the junction pivot's
   43 deg/s (C, MOTION_TURN_SPEED 6) and at 90 deg/s.  GP7 (assumed 50 mm
   ahead of the axle and 45 mm aside: 67 mm from it) crosses the lines of
   a cross junction 42 and 132 deg into the pivot, for 18 mm / (67 mm x
   the rate) each: 358 ms at 43 deg/s, past J_n 280; 171 ms at 90 (C).
   Nothing opens and nothing fires. */
static void test_scan_pivot(void)
{
	static const double rates[] = { 43.0, 90.0 };
	unsigned k;

	for (k = 0u; k < 2u; ++k) {
		double w = rates[k] * 3.14159265358979 / 180.0;	/* rad/s */
		unsigned on[2];
		unsigned len[2];
		uint8_t id0;

		on[0] = (unsigned)(42.0 / rates[k] * 1000.0);
		on[1] = (unsigned)(132.0 / rates[k] * 1000.0);
		len[0] = (unsigned)(18.0 / (67.0 * w) * 1000.0);
		len[1] = len[0];
		chain_reset(true);
		want = '-';
		drive(-400.0, 0.0, 6u, 0.0);
		id0 = cap.read_id;
		held_ms = 0u;
		spin((unsigned)(180.0 / rates[k] * 1000.0) + 200u, rates[k], on, len, 2u);
		if (TEST_EXPECT_GATE) {
			CHECK((uint8_t)(cap.read_id - id0) == 0u);
			CHECK((junctions == 0u) && (held_ms == 0u) && (verdicts == 0u));
			CHECK(cap.ignored == 2u);
		} else {
			/* before the gate: reads open (and hold the steering);
			   at 43 deg/s both crossings fire junctions */
			CHECK((uint8_t)(cap.read_id - id0) >= 1u);
			CHECK(held_ms > 0u);
			CHECK(junctions == ((k == 0u) ? 2u : 0u));
		}
	}
}

/* The armed junction is not gated: a sheet read on a straight, then the
   car turns (20 deg/s from 5 mm after the sheet), and the crossbar
   80 mm after the sheet still fires with the command.  The crossbar's
   own read is turned away. */
static void test_scan_armed_turning(void)
{
	double end;

	chain_reset(true);
	want = 'B';
	end = sheet('B', 3.0, 3.0, 0.0, 0.0);
	crossbar(end + 80.0, 0.0);
	curve_x = end + 5.0;
	curve_dps = 20.0;
	drive(-300.0, crossbar_b + 60.0, 6u, 0.0);
	CHECK(ev_armed == 1u);
	CHECK((j_cross == 1u) && (junctions == 1u) && j_cmd_ok && (j_letter == 'B'));
	CHECK((cap.ignored == 1u) == (TEST_EXPECT_GATE != 0));
}

/* No distance_state: the gate fails open, so the curve sweeps behave as
   before the gate.  motor_task_flag() 0 (a bench image), and motion_task
   not publishing (the counter never moves). */
static void test_scan_fail_open(void)
{
	sweep_t r;

	for (sweep_ds = 1u; sweep_ds <= 2u; ++sweep_ds) {
		curve_sweeps(curve_rate(150.0), 100.0, 0.0, 1.0, 0.0, &r);
		CHECK((r.opened == 5u) && (r.fired == 3u) && (r.held > 0u));
		if (LINE_SCAN_GATE) {
			CHECK(!sg.available && sg.open);
		}
	}
	sweep_ds = 0u;
}
#endif

/* While the bars of a read arrive, a long bar is never a junction: the
   6 mm sheet's 18 mm wide bars at base 4 (567 ms of black, C) under the
   length rule alone fired one per wide bar. */
static void test_big_bars(void)
{
	tally_t t;
	char letter;

	(void)memset(&t, 0, sizeof(t));
	for (letter = 'A'; letter <= 'Z'; ++letter) {
		one(letter, 6.0, 3.0, 0.0, 4u, 0.0, 40.0, &t);
	}
	(void)printf("  6 mm bars at base 4: junctions inside the barcode %u (26 sheets)\n",
		     t.j_inside);
	CHECK(t.j_inside == 0u);
	CHECK(t.cross_once == t.runs);
}

int main(void)
{
	(void)printf("scan gate: LINE_SCAN_GATE %u (tests expect %u), limit %u deg/s, "
		     "average 2 x 1/%u every 10 ms\n", (unsigned)LINE_SCAN_GATE,
		     (unsigned)TEST_EXPECT_GATE, (unsigned)LINE_SCAN_MAX_TURN_DPS,
		     (unsigned)LINE_SCAN_AVG_DIV);
#if LINE_JUNCTION_RULE
	(void)printf("junction rule: barcode-armed (LINE_JUNCTION_RULE 1), confirm %u mm, "
		     "D_hold %u mm, quiet factor %u, decoder split %u\n",
		     (unsigned)LINE_ARMED_CONFIRM_MM, (unsigned)LINE_ARMED_HOLD_MM,
		     (unsigned)BARCODE_Q_WIDTH_FACTOR, (unsigned)BARCODE_DECODE_SPLIT);
#else
	(void)printf("junction rule: length only (LINE_JUNCTION_RULE 0), J_n %u, "
		     "quiet factor %u, decoder split %u, END on stop %u\n",
		     (unsigned)LINE_JUNCTION_SAMPLES, (unsigned)BARCODE_Q_WIDTH_FACTOR,
		     (unsigned)BARCODE_DECODE_SPLIT, (unsigned)BARCODE_END_ON_STOP);
#endif
	test_envelope();
	test_course_sheet_steering();
	test_crossbar_gaps();
	test_big_bars();
#if LINE_JUNCTION_RULE
	test_store_before_flag();
	test_no_arm();
	test_disarm();
	test_two_sheets();
	test_fallback();
	test_h1_crossbar_in_read();
	test_h1_no_junction_in_barcodes();
	test_scan_straight();
	test_scan_curves();
	test_scan_pivot();
	test_scan_armed_turning();
	test_scan_fail_open();
#endif
	return check_report("barcode-armed junction");
}
