/* Host test: the barcode-armed junction with the analog barcode sensor
   (BARCODE_SENSOR_ANALOG 1, LINE_JUNCTION_RULE 1), end to end, one 1 ms
   sample at a time in line_task.c's order:
     the reading (bc_sim.h) -> bc_analog_front.c -> barcode_task's reply
     polled -> Boon's capture (bc_analog_capture.c) -> LETTER or ANALOG to
     barcode_task (modelled: after a latency it decodes, stores the
     command, the judged word, then the reply) -> the arming rule ->
     junction_arm.c -> line_track.c's gate -> the junction, where nav's
     check (barcode_cmd_pending) runs.
   line_task.c's analog glue (analog_letter, analog_hand_off,
   analog_reply, the arm-due rule, the verdict wait) is restated here
   line for line, as test_junction_arm.c restates the GP7 one.

   The car drives at base 4 (32 mm/s, M 19 Nov); distance_state is whole
   mm every 10 ms.  Sheets *X* at the course's three scales (narrow 3.4,
   5.1, 6.9 mm; wide 2.5 x), then an 18 mm crossbar at a gap behind the
   last bar; a square 4 mm sensor spot (sim, bc_sim.h). */

#include <stdio.h>
#include <string.h>

#include "check.h"
#include "bc_sim.h"
#include "../../app_program/line_task/bc_analog_front.h"
#include "../../app_program/line_task/bc_analog_capture.h"
#include "../../app_program/line_task/bc_analog_link.h"
#include "../../app_program/line_task/barcode_action.h"
#include "../../app_program/line_task/barcode_rx.h"
#include "../../app_program/line_task/junction_arm.h"
#include "../../app_program/line_task/line_track.h"
#include "../../app_program/motor_task/motion_ds.h"

#define RAW_B		(3500u)		/* line sensors: both on the line */
#define SPEED		(32.0)		/* mm/s, base 4 */
#define CROSSBAR_MM	(18.0)

/* the chain */
static bc_front_t f;
static bc_capture_t bc;
static junction_arm_t arm;
static line_junction_gate_t gate;
static line_track_t trk;
static line_track_out_t out;
static barcode_table_t table;

/* barcode_task model */
static unsigned latency;		/* ms from a message to its verdict */
static unsigned latency_cfg = 3u;	/* what reset() sets */
static int msg_pending;			/* 0 none, 1 LETTER, 2 ANALOG */
static unsigned msg_at;
static uint8_t msg_id;
static char msg_letter;
static bool msg_cut;
static uint32_t cmd_word;
static uint32_t cmd_seq;
static uint32_t judged;
static uint32_t reply;
static uint32_t nav_taken;

/* line_task.c's analog glue state */
static bool handoff;
static bool cancelled;
static unsigned handoff_ms;
static uint32_t cmd_before_handoff;
static bool arm_due;
static uint8_t arm_id;
static char arm_letter;
static uint32_t arm_cmd;

/* the run */
static unsigned now;
static double x;			/* sensor position, mm */
static double dist;
static uint32_t ds_word;
static uint32_t ds_ctr;
static uint32_t ds_prev;
static double cross_a;			/* the crossbar, or < 0 */
static double cross_b;

/* results */
static unsigned junctions;
static unsigned j_before;
static unsigned j_cross;
static char j_letter;
static double j_into;
static unsigned stored;

static void reset(void)
{
	ir_cal_sensor_t cal = ir_cal_default(true, LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
					     LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);
	unsigned i;

	bc_front_init(&f);
	bc_init(&bc);
	junction_arm_init(&arm);
	line_track_init(&trk, &cal, &cal, true);
	barcode_table_init(&table);
	(void)barcode_table_from_string(&table, "FLRUFLRUFLRUFLRUFLRUFLRUFL");
	latency = latency_cfg;
	msg_pending = 0;
	cmd_word = BARCODE_CMD_NONE;
	cmd_seq = 0u;
	judged = 0u;
	reply = 0u;
	nav_taken = 0u;
	handoff = false;
	cancelled = false;
	arm_due = false;
	now = 0u;
	x = 0.0;
	dist = 1000.0;
	ds_ctr = 0u;
	ds_word = motion_ds_pack(1000, MOTION_DS_STEADY, 0u, 0);
	ds_prev = ds_word;
	cross_a = -1.0;
	cross_b = -1.0;
	junctions = 0u;
	j_before = 0u;
	j_cross = 0u;
	j_letter = '-';
	j_into = -1.0;
	stored = 0u;
	bc_sim_clear();
	for (i = 0u; i < 300u; ++i) {
		line_track_step(&trk, RAW_B, RAW_B, true, false, false, &out);
	}
}

/* barcode_task: store the command, the judged word, then the reply */
static void barcode_task_model(void)
{
	barcode_action_t action;
	bc_out_t o;
	bc_result_t r;

	if ((msg_pending == 0) || ((msg_at + latency) > now)) {
		return;
	}
	if (msg_pending == 1) {
		if (barcode_table_lookup(&table, msg_letter, &action)) {
			cmd_seq++;
			cmd_word = barcode_cmd_pack(barcode_letter_index(msg_letter), action, cmd_seq);
			stored++;
		}
		judged = barcode_judged_pack(msg_id);
	} else {
		r = bc_finish(&bc, &o);
		if ((r == BC_VALID) && !msg_cut) {
			if (barcode_table_lookup(&table, o.ch, &action)) {
				cmd_seq++;
				cmd_word = barcode_cmd_pack(barcode_letter_index(o.ch), action, cmd_seq);
				stored++;
			}
			judged = barcode_judged_pack(msg_id);
		} else if (r == BC_REJECT) {
			judged = barcode_judged_pack(msg_id);
		} else {
			/* nothing stored */
		}
		reply = bc_link_reply_pack(msg_id, r, o.ch);
	}
	msg_pending = 0;
}

static void send(int kind, char letter, bool cut)
{
	msg_pending = kind;
	msg_at = now;
	msg_id = bc.read_id;
	msg_letter = letter;
	msg_cut = cut;
}

/* line_task.c: analog_hand_off() */
static void hand_off(bool cut)
{
	cmd_before_handoff = cmd_word;
	send(2, '\0', cut);
	handoff = true;
	cancelled = false;
	handoff_ms = 0u;
}

/* One 1 ms sample, in line_task.c's order. */
static void sample(void)
{
	bc_out_t o;
	bc_result_t r;
	bool seen;
	int32_t moved;
	jarm_event_t je;

	if ((now % 10u) == 0u) {
		ds_ctr++;
		ds_word = motion_ds_pack((int32_t)dist, MOTION_DS_STEADY, ds_ctr, 0);
	}
	bc_front_step(&f, bc_sim_read(x), ds_word, true, 1u);
	/* analog_reply() */
	if (handoff) {
		if (bc_link_reply_is(reply, bc.read_id)) {
			r = bc_link_reply_result(reply);
			if ((r == BC_VALID) && !bc.b_cut && !cancelled) {
				junction_arm_read_done(&arm, bc.read_id, bc_link_reply_letter(reply),
						       cmd_before_handoff);
			}
			bc_resume(&bc, r);
			handoff = false;
		} else {
			handoff_ms++;
		}
	}
	/* analog_capture() */
	r = bc_sample(&bc, f.norm, bc_front_take_step(&f), &o);
	if (r == BC_VALID) {
		arm_cmd = cmd_word;
		send(1, o.ch, false);
		arm_due = true;
		arm_id = bc.read_id;
		arm_letter = o.ch;
	} else if (r == BC_DONE) {
		hand_off(false);
	} else {
		/* nothing finished */
	}
	barcode_task_model();
	/* the junction side */
	moved = motion_ds_delta_mm(ds_word, ds_prev);
	ds_prev = ds_word;
	seen = f.seen;
	if (arm_due && (!seen || bc.b_black)) {
		junction_arm_read_done(&arm, arm_id, arm_letter, arm_cmd);
		arm_due = false;
	}
	je = junction_arm_step(&arm, judged, cmd_word, seen,
			       (uint32_t)((moved < 0) ? -moved : moved),
			       bc_in_read(&bc) || arm_due, &gate);
	(void)je;
	gate.wait = gate.wait || (handoff && (handoff_ms < BC_ANALOG_VERDICT_WAIT_MS));
	gate.turning = false;
	line_track_step_gate(&trk, RAW_B, RAW_B, true, false, seen, &gate, &out);
	if (out.junction) {
		bool pending = barcode_cmd_pending(cmd_word, nav_taken);

		(void)junction_arm_used(&arm);
		arm_due = false;
		if (bc_junction(&bc) == BC_DONE) {
			hand_off(true);
		}
		junctions++;
		if ((cross_a >= 0.0) && (x >= cross_a) && (x < cross_b + 30.0)) {
			j_cross++;
			if (j_cross == 1u) {
				j_letter = pending ? (char)('A' + barcode_cmd_letter(cmd_word)) : '-';
				j_into = x - cross_a;
			}
		} else if ((cross_a < 0.0) || (x < cross_a)) {
			j_before++;
		} else {
			/* past it */
		}
		if (pending) {
			nav_taken = barcode_cmd_seq(cmd_word);
		}
	}
	x += SPEED / 1000.0;
	dist += SPEED / 1000.0;
	now++;
}

static void drive_to(double x1)
{
	while (x < x1) {
		sample();
	}
}

/* A sheet then a crossbar gap mm behind its last bar (gap < 0: none). */
static void course(char letter, double n, int rev, double thin, double gap)
{
	double end;

	reset();
	end = bc_sim_sheet(letter, n, 2.5, rev, 80.0, thin);
	if (gap >= 0.0) {
		cross_a = end + gap;
		cross_b = cross_a + CROSSBAR_MM;
		bc_sim_black(cross_a, cross_b, BC_SIM_BLACK);
	}
	drive_to(((gap >= 0.0) ? cross_b : end) + 200.0);
}

int main(void)
{
	static const double narrow[3] = { 3.4, 5.1, 6.9 };
	static const double gaps[6] = { 0.0, 5.0, 15.0, 30.0, 60.0, 120.0 };
	static const char letters[] = "AFKRWZ";
	unsigned s;
	unsigned g;
	unsigned l;
	unsigned runs = 0u;
	unsigned ok = 0u;
	unsigned bad_before = 0u;
	unsigned bad_cmd = 0u;
	unsigned missed = 0u;
	double into_min = 1e9;
	double into_max = -1.0;

	/* full-contrast sheets, every gap: one junction, on the crossbar,
	   with the sheet's command */
	for (s = 0u; s < 3u; ++s) {
		for (g = 0u; g < 6u; ++g) {
			for (l = 0u; l < sizeof(letters) - 1u; ++l) {
				char c = letters[l];
				char want = (char)('A' + barcode_letter_index(c));

				course(c, narrow[s], (int)(l & 1u), BC_SIM_BLACK, gaps[g]);
				runs++;
				bad_before += (j_before > 0u) ? 1u : 0u;
				missed += (j_cross == 0u) ? 1u : 0u;
				bad_cmd += ((j_cross > 0u) && (j_letter != want)) ? 1u : 0u;
				if ((j_before == 0u) && (j_cross == 1u) && (junctions == 1u) &&
				    (j_letter == want)) {
					ok++;
					into_min = (j_into < into_min) ? j_into : into_min;
					into_max = (j_into > into_max) ? j_into : into_max;
				}
			}
		}
	}
	(void)printf("analog chain: %u courses (3 scales, gaps 0-120 mm, 6 letters both ways): "
		     "%u right, %u with a junction before the crossbar, %u crossbars missed, "
		     "%u wrong command; fired %.1f-%.1f mm into the crossbar\n",
		     runs, ok, bad_before, missed, bad_cmd, into_min, into_max);
	CHECK(ok == runs);

	/* faint thin bars: no stop END, decoded at Boon's end on core 0 (the
	   reply arms the junction), so the crossbar must come after his 40 mm
	   end of a code */
	{
		unsigned fok = 0u;
		unsigned fbefore = 0u;
		unsigned fn = 0u;

		for (s = 1u; s < 3u; ++s) {
			for (l = 0u; l < sizeof(letters) - 1u; ++l) {
				char c = letters[l];

				course(c, narrow[s], 0, 900.0, 120.0);
				fn++;
				fbefore += (j_before > 0u) ? 1u : 0u;
				fok += ((j_cross == 1u) && (j_letter == (char)('A' + barcode_letter_index(c))))
				       ? 1u : 0u;
			}
		}
		(void)printf("analog chain: faint thin bars (900), crossbar 120 mm behind: %u of %u "
			     "right, %u with a junction before the crossbar\n", fok, fn, fbefore);
		CHECK(fbefore == 0u);
		CHECK(fok == fn);
	}

	/* barcode_task slower than the verdict wait (300 ms > 200): a faint
	   sheet decoded at Boon's end, the crossbar 45 mm behind its last bar
	   (5 mm after his end): one junction, on the crossbar, with the
	   sheet's command */
	latency_cfg = 300u;
	course('K', 5.1, 0, 900.0, 45.0);
	(void)printf("analog chain: barcode_task 300 ms late, crossbar 5 mm after Boon's end: "
		     "%u junction(s) on the crossbar, %u before, command '%c'\n",
		     j_cross, j_before, j_letter);
	CHECK((j_cross == 1u) && (j_before == 0u) && (j_letter == 'K'));
	latency_cfg = 3u;

	/* a sheet with no crossbar: no junction at all */
	course('A', 3.4, 0, BC_SIM_BLACK, -1.0);
	CHECK(junctions == 0u);
	CHECK(stored == 1u);

	return check_report("test_bc_analog_chain");
}
