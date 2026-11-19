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
   envelope checks: that is what the new rule fixes. */

#include <string.h>

#include "check.h"
#include "../../app_program/line_task/bar_capture.h"
#include "../../app_program/line_task/barcode_rx.h"
#include "../../app_program/line_task/junction_arm.h"
#include "../../app_program/line_task/line_track.h"

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
	nsegs = 0u;
	crossbar_a = -1.0;
	crossbar_b = -1.0;
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
	bar_event_t ev = bar_capture_step(&cap, black, &width);

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
		jarm_event_t je = junction_arm_step(&arm, judged, cmd_word, black,
						    ds_mm - ds_prev,
						    bar_capture_in_read(&cap), &gate);

		ds_prev = ds_mm;
		ev_armed += (je == JARM_EV_ARMED) ? 1u : 0u;
		ev_rejected += (je == JARM_EV_REJECTED) ? 1u : 0u;
		ev_no_verdict += (je == JARM_EV_NO_VERDICT) ? 1u : 0u;
		ev_disarm_dist += (je == JARM_EV_DISARM_DISTANCE) ? 1u : 0u;
	}
	line_track_step_gate(&trk, RAW_B, RAW_B, true, false, black, &gate, &out);
#else
	line_track_step(&trk, RAW_B, RAW_B, true, false, black, &out);
#endif
	if (out.junction) {
		bool pending = barcode_cmd_pending(cmd_word, nav_taken);

#if LINE_JUNCTION_RULE
		(void)junction_arm_used(&arm);
		if (bar_capture_reading(&cap) && (cap.count == 0u)) {	/* the crossbar's read */
			q_send(bar_capture_abort(&cap, BAR_ABORT_JUNCTION),
			       (uint16_t)BAR_ABORT_JUNCTION);
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
	drive(-40.0, 50.0 + 13.6, 6u, 0.0);
	CHECK((junctions == 1u) && (j_cross == 1u) && (j_letter == '-'));
	CHECK(ev_armed == 0u);
	/* fired at J_n: 280 ms into the crossbar at 47.6 mm/s (C: 13.3 mm) */
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
#endif
	return check_report("barcode-armed junction");
}
