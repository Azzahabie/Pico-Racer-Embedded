/*
 * line_task.c - ir_sense_task: samples the two line sensors (GP26, GP27)
 * and the barcode sensor (GP7) every 1 ms, on core 1.
 *
 * Every sample it publishes line_state for motion_task, steps the junction
 * detector (NAV_FLG_JUNCTION to nav_ctrl_task) and steps the barcode
 * capture, sending widths to barcode_task on bar_mbf with TMO_POL.  It never
 * waits for decoding, the table or telemetry, and never drives the motors.
 * On CAL_REQ from nav_ctrl_task it calibrates over IR_CAL_SAMPLES samples
 * without stopping the 1 ms loop.  A line sensor fault sets
 * FAULT_LINE_SENSOR (S.faults bit 4) and clears it when it ends.
 *
 * Junctions (LINE_JUNCTION_RULE 1): a complete barcode read ENDs at its
 * stop character; once barcode_task has judged it and stored its command
 * (barcode_judged_word, barcode_cmd_word), the next black under GP7 is
 * the junction (junction_arm.h).  The travel for that comes from
 * distance_state (motion_ds.h, one aligned load, core 1 like imu_task).
 * The junction side counts GP7's runs after the capture's glitch filter
 * (bar_capture_black), so both see the same runs.
 *
 * The scan gate (LINE_SCAN_GATE 1, scan_gate.h): barcodes are only on
 * straight segments (the prof's tip, 19 Nov), so a GP7 black that starts
 * while motion is in a manoeuvre (TURNING, BYPASS, LINE_SEARCH,
 * REVERSING: motion_state_read(), one aligned load, core 1), or while
 * its averaged turn rate is at or above LINE_SCAN_MAX_TURN_DPS, opens no
 * read and fires no junction by the length rule.  The average starts
 * again from 0 when a manoeuvre ends.  The armed junction is not gated.
 * Without distance_state it fails open.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "../usb_console_compat.h"
#include "../common/messages.h"
#include "../comms_task/comms_task.h"
#include "../nav_task/nav_ctrl.h"
#include "line_task.h"
#include "line_logic.h"
#include "line_state.h"
#include "line_track.h"
#include "ir_cal.h"
#include "bar_capture.h"
#include "barcode_scanner.h"
#include "barcode_task.h"
#include "barcode_action.h"
#include "junction_arm.h"
#include "scan_gate.h"
#include "../motor_task/motion_ds.h"
#include "../motor_task/motion_sm.h"
#include "../motor_task/motor_task.h"

#define IR_SAMPLE_MS		1	/* the 1 ms cadence (CNF_TIMER_PERIOD 1) */
#define IR_COMMISSION_MS	100	/* print period with LINE_COMMISSIONING_MODE */
#define IR_GATE_PRINT_MS	1000	/* at most one "ignored" line this often */

#if LINE_JUNCTION_RULE
#define IR_RULE_TEXT	"barcode-armed, length rule as fallback (LINE_JUNCTION_RULE 1)"
#else
#define IR_RULE_TEXT	"length only (LINE_JUNCTION_RULE 0)"
#endif

/* COMM_W_KIND_LINE states */
#define TELE_LINE_ON		0U
#define TELE_LINE_LOST		1U
#define TELE_LINE_CROSSING	2U
#define TELE_LINE_UNSENT	3U	/* never sent: forces the first record */

LOCAL ID		g_ir_flgid;
LOCAL ID		g_ir_tid;
LOCAL volatile uint32_t	g_line_state = LINE_STATE_INITIAL;	/* one store, this task only */

LOCAL line_track_t	g_trk;
LOCAL bar_capture_t	g_cap;
#if LINE_JUNCTION_RULE
LOCAL junction_arm_t	g_arm;
#endif
#if LINE_SCAN_GATE
LOCAL scan_gate_t	g_gate;
#endif

EXPORT ID ir_sense_init(void)
{
	T_CFLG	cflg = { .exinf = NULL, .flgatr = TA_TFIFO | TA_WSGL, .iflgptn = 0U };
	ID	objid;

	if(0 < g_ir_flgid) {
		return g_ir_flgid;
	}
	objid = tk_cre_flg(&cflg);
	if(0 < objid) {
		g_ir_flgid = objid;
	}
	return objid;
}

EXPORT uint32_t line_state_word(void)
{
	return g_line_state;
}

LOCAL void ir_tick(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(g_ir_tid);
}

LOCAL ER start_ticks(void)
{
	T_CCYC	ccyc;
	ID	cycid;

	g_ir_tid = tk_get_tid();
	ccyc.exinf  = NULL;
	ccyc.cycatr = TA_HLNG | TA_STA;
	ccyc.cychdr = (FP)ir_tick;
	ccyc.cyctim = IR_SAMPLE_MS;
	ccyc.cycphs = IR_SAMPLE_MS;
	cycid = tk_cre_cyc(&ccyc);
	return (cycid < E_OK) ? (ER)cycid : E_OK;
}

/* Sleep until the next tick.  Returns the ticks that went by unserved. */
LOCAL UINT wait_tick(void)
{
	INT	missed;

	(void)tk_slp_tsk(TMO_FEVR);
	missed = tk_can_wup(TSK_SELF);
	return (missed > 0) ? (UINT)missed : 0U;
}

LOCAL void tell_nav(UINT bits)
{
	if(0 < nav_flag()) {
		(void)tk_set_flg(nav_flag(), bits);
	}
}

LOCAL BOOL cal_requested(void)
{
	UINT	ptn;

	return (0 < g_ir_flgid) &&
	       (E_OK == tk_wai_flg(g_ir_flgid, IR_FLG_CAL_REQ, TWF_ORW | TWF_BITCLR,
				   &ptn, TMO_POL));
}

/* Send one capture event; a full bar_mbf abandons the read (an ABORT is
   tried, and the next read's new read_id keeps it clean either way). */
LOCAL void send_bar(bar_event_t event, uint16_t value)
{
	UB	msg[BAR_MSG_SIZE];
	ID	mbfid = barcode_bar_mbf();

	if((BAR_EV_NONE == event) || (0 >= mbfid)) {
		return;
	}
	bar_msg_pack(msg, event, g_cap.read_id, value);
	if((E_OK != tk_snd_mbf(mbfid, msg, BAR_MSG_SIZE, TMO_POL)) &&
	   (BAR_EV_ABORT != event)) {
		(void)bar_capture_abort(&g_cap, BAR_ABORT_OVERFLOW);
		bar_msg_pack(msg, BAR_EV_ABORT, g_cap.read_id, (uint16_t)BAR_ABORT_OVERFLOW);
		(void)tk_snd_mbf(mbfid, msg, BAR_MSG_SIZE, TMO_POL);
	}
}

LOCAL void capture_step(BOOL black)
{
	bar_event_t	event;
	uint16_t	width = 0u;

	event = bar_capture_step(&g_cap, black ? true : false, &width);
	if(BAR_EV_WIDTH == event) {
		send_bar(event, width);
	} else if(BAR_EV_END == event) {
#if LINE_JUNCTION_RULE
		/* before the END goes: barcode_task (core 0) may store at once */
		uint32_t	cmd_before = barcode_cmd_word();

		send_bar(event, g_cap.count);
		if(g_cap.complete) {
			junction_arm_read_done(&g_arm, g_cap.read_id, g_cap.letter,
					       cmd_before);
		}
#else
		send_bar(event, g_cap.count);
#endif
	} else if(BAR_EV_ABORT == event) {
		send_bar(event, (uint16_t)g_cap.reason);
	} else {
		/* nothing finished */
	}
}

LOCAL ir_cal_sensor_t default_cal(BOOL black_is_high, uint16_t rise, uint16_t fall)
{
	return ir_cal_default(black_is_high ? true : false, rise, fall);
}

#if LINE_JUNCTION_RULE
/* The barcode-armed junction's console lines (junction_arm.h). */
LOCAL void print_arm(jarm_event_t event, mission_state_t mission)
{
	UB	stored = (UB)('A' + barcode_cmd_letter(g_arm.cmd));
	const char	*after = (JARM_ARMED == g_arm.phase)
				 ? "still armed by the barcode before"
				 : "junction by length only";

	switch(event) {
	case JARM_EV_ARMED:
		tm_printf((UB *)"[line] junction armed: barcode '%c' -> %s, the next black under GP7 is the junction\n",
			  stored, barcode_action_name(barcode_cmd_action(g_arm.cmd)));
		break;
	case JARM_EV_REJECTED:
		tm_printf((UB *)"[line] barcode '%c' read, no command stored: %s\n",
			  g_arm.letter, after);
		break;
	case JARM_EV_NO_VERDICT:
		tm_printf((UB *)"[line] barcode '%c' read, no verdict from barcode_task in %u ms: %s\n",
			  g_arm.letter, (UINT)LINE_ARMED_VERDICT_WAIT_MS, after);
		break;
	case JARM_EV_DISARM_DISTANCE:
		tm_printf((UB *)"[line] junction disarmed: no junction within %u mm of barcode '%c' (command kept for nav)\n",
			  (UINT)LINE_ARMED_HOLD_MM, stored);
		break;
	case JARM_EV_DISARM_MISSION:
		tm_printf((UB *)"[line] junction disarmed: mission %s\n",
			  nav_state_name(mission));
		break;
	case JARM_EV_NONE:
	default:
		break;
	}
}
#endif

#if LINE_SCAN_GATE
/* A GP7 black the scan gate turned away; more = others since the last
   such line, not printed (IR_GATE_PRINT_MS); motion = the motion state
   now. */
LOCAL void print_ignored(UINT more, uint32_t motion)
{
	INT	avg = (INT)scan_gate_avg_x10(&g_gate);
	UINT	size = (UINT)((avg < 0) ? -avg : avg);

	if(g_gate.manoeuvre) {
		tm_printf((UB *)"[line] GP7 black ignored: motion %s (scan gate): no read, no length-rule junction (%u more since the last line)\n",
			  motion_sm_name(motion), more);
	} else {
		tm_printf((UB *)"[line] GP7 black ignored: turning %u.%u deg/s %s (scan gate, limit %u): no read, no length-rule junction (%u more since the last line)\n",
			  size / 10U, size % 10U, (avg < 0) ? "right" : "left",
			  (UINT)LINE_SCAN_MAX_TURN_DPS, more);
	}
}

/* Blacks turned away since the last line and not printed yet: printed
   once IR_GATE_PRINT_MS has passed with no new one to print them with. */
LOCAL void print_ignored_rest(UINT more)
{
	tm_printf((UB *)"[line] GP7 black ignored: %u more since the last line (scan gate)\n",
		  more);
}

/* distance_state lost or back: the gate fails open without it. */
LOCAL void print_ds(BOOL available, BOOL motion_up)
{
	if(available) {
		tm_printf((UB *)"[line] scan gate: distance_state back, no read starts while turning\n");
	} else {
		tm_printf((UB *)"[line] scan gate: no distance_state (%s): GP7 scans while turning too (fails open)\n",
			  motion_up ? "counter stopped" : "motor_task_flag 0, bench image?");
	}
}
#endif

/* rl 11 is the centred car (both sensors inside the line): ON, not
   CROSSING.  CROSSING is GP7 on a junction's crossbar. */
LOCAL UINT tele_line_state(const line_track_out_t *o)
{
	if(o->fields.lost) {
		return TELE_LINE_LOST;
	}
	return o->crossing ? TELE_LINE_CROSSING : TELE_LINE_ON;
}

EXPORT void ir_sense_task(INT stacd, void *exinf)
{
	ir_cal_sensor_t	left_def = default_cal(LINE_SENSOR_LEFT_BLACK_IS_HIGH != 0u,
					       LINE_SENSOR_LEFT_BLACK_RISING_THRESHOLD,
					       LINE_SENSOR_LEFT_BLACK_FALLING_THRESHOLD);
	ir_cal_sensor_t	right_def = default_cal(LINE_SENSOR_RIGHT_BLACK_IS_HIGH != 0u,
						LINE_SENSOR_RIGHT_BLACK_RISING_THRESHOLD,
						LINE_SENSOR_RIGHT_BLACK_FALLING_THRESHOLD);
	ir_cal_sensor_t	left_new;
	ir_cal_sensor_t	right_new;
	ir_cal_acc_t	left_acc;
	ir_cal_acc_t	right_acc;
	ir_cal_result_t	left_res;
	ir_cal_result_t	right_res;
	BOOL		cal_ok;
	line_track_out_t	out;
	line_reader_raw_t	raw;
	mission_state_t	mission = nav_mission_state();
	UINT		cal_left = 0U;
	UINT		missed;
	UINT		missed_beat = 0U;	/* NFR-11: since the last heartbeat */
	UINT		missed_total = 0U;	/* and since boot */
	UINT		holdoff = 0U;
	UINT		print_ms = 0U;
	UINT		tele_state = TELE_LINE_UNSENT;	/* S.lin defaults to ON */
	UINT		last_rl = LINE_RL_CENTRED;
#if LINE_JUNCTION_RULE || LINE_SCAN_GATE
	line_junction_gate_t	gate;
	uint32_t	ds_now;
#endif
#if LINE_JUNCTION_RULE
	jarm_event_t	arm_event;
	jarm_event_t	cancel_event;
	bar_event_t	cap_event;
	BOOL		seen;		/* GP7 after the glitch filter */
	uint32_t	judged;
	uint32_t	cmd_now;
	uint32_t	ds_prev = motion_ds_read();
	int32_t		moved;
	BOOL		armed_junction = FALSE;
#endif
#if LINE_SCAN_GATE
	BOOL		scan_open;	/* a GP7 black may open a read */
	BOOL		motion_up;
	uint32_t	motion_now;	/* motion_state_read() */
	BOOL		ds_reported = TRUE;	/* distance_state as last printed */
	uint16_t	ignored_seen = 0U;	/* g_cap.ignored as last counted */
	UINT		ignored_more = 0U;	/* turned away, not yet printed */
	UINT		gate_quiet = 0U;	/* ms until the next such line */
	UINT		elapsed;
#endif
	BOOL		black;
	BOOL		adc_ok;
	BOOL		fault_reported = FALSE;
	BOOL		fault_flagged = FALSE;	/* FAULT_LINE_SENSOR as last sent */
	BOOL		lost_reported = TRUE;	/* LINE_STATE_INITIAL is lost */
#if LINE_GP7_DIAG
	UINT		gp7_ms = 0U;
#endif
	UB		body[COMM_W_INTAKE_SIZE_LINE];
	char		bits[3];
	ER		ercd;

	(void)stacd;
	(void)exinf;

	line_track_init(&g_trk, &left_def, &right_def, LINE_STEER_INVERT != 0);
	bar_capture_init(&g_cap);
#if LINE_JUNCTION_RULE
	junction_arm_init(&g_arm);
#endif
#if LINE_SCAN_GATE
	scan_gate_init(&g_gate, motion_ds_read());
#endif
	ercd = line_reader_init(NULL);
	if(E_OK > ercd) {
		tm_printf((UB *)"[line] ADC initialization failed, er=%d\n", ercd);
		fault_set(FAULT_LINE_SENSOR);	/* S.faults bit 4, for good */
		tell_nav(NAV_FLG_CAL_FAIL);
		while(1) {
			(void)tk_slp_tsk(TMO_FEVR);
		}
	}
	(void)barcode_scanner_init(NULL);
	(void)ir_sense_init();
	ercd = start_ticks();
	if(E_OK > ercd) {
		tm_printf((UB *)"[line] tk_cre_cyc failed, er=%d: stopped\n", ercd);
		tell_nav(NAV_FLG_CAL_FAIL);
		while(1) {
			(void)tk_slp_tsk(TMO_FEVR);
		}
	}
	tm_printf((UB *)"[line] ir_sense_task: line ADC GP%d/GP%d, barcode DO GP%d, 1 ms\n",
		  (INT)LINE_SENSOR_LEFT_GPIO, (INT)LINE_SENSOR_RIGHT_GPIO,
		  (INT)BARCODE_SENSOR_GPIO);
#if LINE_SCAN_GATE
	tm_printf((UB *)"[line] junction rule: " IR_RULE_TEXT " | scan gate on: GP7 black opens no read and no length-rule junction in TURNING, BYPASS, LINE_SEARCH or REVERSING, or while the averaged turn rate is %u deg/s or more (LINE_SCAN_GATE 1)\n",
		  (UINT)LINE_SCAN_MAX_TURN_DPS);
#else
	tm_printf((UB *)"[line] junction rule: " IR_RULE_TEXT " | scan gate off (LINE_SCAN_GATE 0)\n");
#endif

	while(1) {
		missed = wait_tick();
		missed_beat += missed;
		missed_total += missed;
#if LINE_SCAN_GATE
		elapsed = 1U + missed;
#endif

		/* sample first, so the 1 ms cadence does not depend on the work */
		raw = line_reader_read_raw();
		adc_ok = (E_OK <= line_reader_last_error()) ? TRUE : FALSE;
		black = barcode_scanner_read_black() ? TRUE : FALSE;
#if LINE_JUNCTION_RULE || LINE_SCAN_GATE
		/* distance_state, one load: the scan gate's turn rate and the
		   armed junction's travel */
		ds_now = motion_ds_read();
#endif
#if LINE_SCAN_GATE
		/* before the capture: no read starts while the car turns or
		   is in a manoeuvre (motion's state, one load) */
		motion_up = (0 < motor_task_flag()) ? TRUE : FALSE;
		motion_now = motion_state_read();
		scan_open = scan_gate_step(&g_gate, ds_now, motion_now,
					   motion_up ? true : false,
					   (uint32_t)elapsed) ? TRUE : FALSE;
		bar_capture_set_gate(&g_cap, scan_open ? true : false);
#endif

		/* barcode: ticks missed (a long console line) still count as time */
		capture_step(black);
		while(0U < missed) {
			capture_step(black);
			missed--;
		}

		/* a mission change drops any partial read and stale junction */
#if LINE_JUNCTION_RULE
		cancel_event = JARM_EV_NONE;
#endif
		if(nav_mission_state() != mission) {
			mission = nav_mission_state();
			send_bar(bar_capture_abort(&g_cap, BAR_ABORT_CANCELLED),
				 (uint16_t)BAR_ABORT_CANCELLED);
			line_track_disarm_junction(&g_trk);
#if LINE_JUNCTION_RULE
			cancel_event = junction_arm_cancel(&g_arm);
#endif
		}

#if LINE_JUNCTION_RULE
		/* the barcode-armed junction: travel from distance_state.
		   The judged word first, then the command: barcode_task (core
		   0) stores them the other way round, so a read judged here
		   always comes with its command (as function arguments the
		   two loads would have no order) */
		moved = motion_ds_delta_mm(ds_now, ds_prev);
		ds_prev = ds_now;
		judged = barcode_judged_word();
		cmd_now = barcode_cmd_word();
		seen = bar_capture_black(&g_cap) ? TRUE : FALSE;
		arm_event = junction_arm_step(&g_arm, judged, cmd_now,
					      seen ? true : false,
					      (uint32_t)((moved < 0) ? -moved : moved),
					      bar_capture_in_read(&g_cap), &gate);
#endif

		/* reading, not busy: after a width_too_long abort GP7 may stay
		   black for as long as the car is parked on it */
		if(bar_capture_reading(&g_cap)) {
			holdoff = BARCODE_JUNCTION_HOLDOFF_MS;
		} else if(0U < holdoff) {
			holdoff--;
		} else {
			/* not held */
		}
#if LINE_JUNCTION_RULE
		/* the scan gate shuts the length rule, never the armed junction */
#if LINE_SCAN_GATE
		gate.turning = scan_open ? false : true;
#else
		gate.turning = false;
#endif
		line_track_step_gate(&g_trk, raw.left_guard, raw.right_guard,
				     adc_ok ? true : false,
				     (0U != LINE_BARCODE_HOLD) && (0U < holdoff),
				     seen ? true : false, &gate, &out);
#elif LINE_SCAN_GATE
		gate.in_read = false;
		gate.armed_due = false;
		gate.wait = false;
		gate.turning = scan_open ? false : true;
		line_track_step_gate(&g_trk, raw.left_guard, raw.right_guard,
				     adc_ok ? true : false,
				     (0U != LINE_BARCODE_HOLD) && (0U < holdoff),
				     black ? true : false, &gate, &out);
#else
		line_track_step(&g_trk, raw.left_guard, raw.right_guard,
				adc_ok ? true : false,
				(0U != LINE_BARCODE_HOLD) && (0U < holdoff),
				black ? true : false, &out);
#endif
		g_line_state = out.word;			/* one store */

		if(out.junction) {
#if LINE_JUNCTION_RULE
			armed_junction = junction_arm_used(&g_arm) ? TRUE : FALSE;
			/* the crossbar is not a bar: a read it opened itself is
			   dropped, so it neither holds line_state for its quiet
			   zone nor prints a reject; a read with widths before it
			   (a speck, a dropout, merged bars) ENDs there,
			   and barcode_task prints those widths */
			cap_event = bar_capture_junction(&g_cap);
			send_bar(cap_event, (BAR_EV_END == cap_event)
				 ? g_cap.count : (uint16_t)BAR_ABORT_JUNCTION);
#endif
			tell_nav(NAV_FLG_JUNCTION);
		}

		/* S.faults bit 4 follows line_state's fault bit, on its edges.
		   Only this task's bit; before the console, so a barcode read
		   never holds it back */
		if((out.fields.fault ? TRUE : FALSE) != fault_flagged) {
			fault_flagged = out.fields.fault ? TRUE : FALSE;
			if(fault_flagged) {
				fault_set(FAULT_LINE_SENSOR);
			} else {
				fault_clear(FAULT_LINE_SENSOR);
			}
		}

		/* calibration runs inside the loop: sampling never stops */
		if(cal_requested()) {
			ir_cal_begin(&left_acc, LINE_SENSOR_LEFT_BLACK_IS_HIGH != 0u);
			ir_cal_begin(&right_acc, LINE_SENSOR_RIGHT_BLACK_IS_HIGH != 0u);
			cal_left = IR_CAL_SAMPLES;
		}
		if(0U < cal_left) {
			if(adc_ok) {
				ir_cal_add(&left_acc, raw.left_guard);
				ir_cal_add(&right_acc, raw.right_guard);
			}
			if(0U == --cal_left) {
				left_new = g_trk.left_cal;	/* kept if it fails */
				right_new = g_trk.right_cal;
				cal_ok = ir_cal_finish_pair(&left_acc, &right_acc,
							    &left_def, &right_def,
							    &left_new, &right_new,
							    &left_res, &right_res)
					 ? TRUE : FALSE;
				if(cal_ok) {
					line_track_set_cal(&g_trk, &left_new, &right_new);
					tell_nav(NAV_FLG_CAL_OK_IR);
				} else {
					tell_nav(NAV_FLG_CAL_FAIL);
				}
				tm_printf((UB *)"[line] calibration %s: GP26 %s (white %u, black %u, rise %u) GP27 %s (white %u, black %u, rise %u)\n",
					  cal_ok ? "OK" : "FAILED",
					  ir_cal_result_name(left_res), (UINT)left_new.white,
					  (UINT)left_new.black, (UINT)left_new.rise,
					  ir_cal_result_name(right_res), (UINT)right_new.white,
					  (UINT)right_new.black, (UINT)right_new.rise);
			}
		}

		/* telemetry on edges only; comms_tele_send never blocks */
		if(tele_line_state(&out) != tele_state) {
			tele_state = tele_line_state(&out);
			(void)comms_tele_send(body, (INT)comm_w_pack_line(body,
				tele_state, out.fields.error));
		}

		/* the crossbar under GP7 is itself a capture, so this line must
		   come before the barcode check below */
#if LINE_JUNCTION_RULE
		if(out.junction && armed_junction) {
			tm_printf((UB *)"[line] junction (armed by barcode '%c' %s, GP7 black %u mm) -> nav\n",
				  (UB)('A' + barcode_cmd_letter(g_arm.cmd)),
				  barcode_action_name(barcode_cmd_action(g_arm.cmd)),
				  (UINT)g_arm.black_mm);
		} else if(out.junction) {
			tm_printf((UB *)"[line] junction (GP7 black %u ms, length rule) -> nav\n",
				  (UINT)LINE_JUNCTION_SAMPLES);
		} else {
			/* no junction this sample */
		}
		print_arm(cancel_event, mission);
		print_arm(arm_event, mission);
#else
		if(out.junction) {
			tm_printf((UB *)"[line] junction (GP7 black %u ms) -> nav\n",
				  (UINT)LINE_JUNCTION_SAMPLES);
		}
#endif

#if LINE_SCAN_GATE
		/* the scan gate's lines: a GP7 black turned away (at most one a
		   second, counting the rest, which are printed on their own
		   once the second is over and no read is open), and
		   distance_state lost or back */
		gate_quiet = (gate_quiet > elapsed) ? (gate_quiet - elapsed) : 0U;
		if(g_cap.ignored != ignored_seen) {
			ignored_more += (UINT)(uint16_t)(g_cap.ignored - ignored_seen);
			ignored_seen = g_cap.ignored;
			if(0U == gate_quiet) {
				print_ignored(ignored_more - 1U, motion_now);
				ignored_more = 0U;
				gate_quiet = IR_GATE_PRINT_MS;
			}
		} else if((0U == gate_quiet) && (0U < ignored_more) &&
			  !bar_capture_reading(&g_cap)) {
			print_ignored_rest(ignored_more);
			ignored_more = 0U;
			gate_quiet = IR_GATE_PRINT_MS;
		} else {
			/* nothing to print */
		}
		if((g_gate.available ? TRUE : FALSE) != ds_reported) {
			ds_reported = g_gate.available ? TRUE : FALSE;
			print_ds(ds_reported, motion_up);
		}
#endif

#if LINE_GP7_DIAG
		/* before the barcode check below: it must print even while GP7
		   looks like a bar */
		if(++gp7_ms >= 500U) {
			uint16_t	pd, pu, none;

			gp7_ms = 0U;
			barcode_scanner_probe(&pd, &pu, &none);
			tm_printf((UB *)"[gp7] pull-down=%u pull-up=%u no-pull=%u -> %s\n",
				  (UINT)pd, (UINT)pu, (UINT)none,
				  (pd != pu) ? "NOT CONNECTED (follows the pull)"
				  : (0U != none) ? "driven HIGH (sensor says black)"
						 : "driven LOW (sensor says white)");
		}
#endif

		/* console: never while a barcode is being read (an aborted one
		   being discarded does not count) */
		if(bar_capture_reading(&g_cap)) {
			continue;
		}
		if(out.fields.fault && !fault_reported) {
			tm_printf((UB *)"[line] SENSOR FAULT: ADC failed or a sensor is at a rail (GP26=%u GP27=%u)\n",
				  (UINT)raw.left_guard, (UINT)raw.right_guard);
		}
		fault_reported = out.fields.fault ? TRUE : FALSE;
		if(out.fields.rl != last_rl) {
			last_rl = out.fields.rl;
			line_reader_format_2bit((UB)last_rl, bits);
			tm_printf((UB *)"[line] RL=%s err=%d\n", bits, (INT)out.fields.error);
		}
		if((out.fields.lost ? TRUE : FALSE) != lost_reported) {
			lost_reported = out.fields.lost ? TRUE : FALSE;
			tm_printf(lost_reported
				  ? (UB *)"[line] LOST: RL=00 for %u ms\n"
				  : (UB *)"[line] line found (LOST_MS %u)\n",
				  (UINT)LINE_LOST_MS);
		}
		print_ms++;
		if(print_ms >= ((LINE_COMMISSIONING_MODE != 0) ? IR_COMMISSION_MS
							       : LINE_HEARTBEAT_MS)) {
			print_ms = 0U;
			tm_printf((UB *)"[line] raw ADC GP26=%u GP27=%u norm %u/%u err=%d%s%s | barcode DO GP%d=%u (%s) | missed %u ticks (%u since boot)\n",
				  (UINT)raw.left_guard, (UINT)raw.right_guard,
				  (UINT)out.left_norm, (UINT)out.right_norm,
				  (INT)out.fields.error,
				  out.fields.lost ? " LOST" : "",
				  out.fields.cal ? "" : " uncal",
				  (INT)BARCODE_SENSOR_GPIO,
				  (UINT)barcode_scanner_get_last_raw(),
				  black ? "black" : "white",
				  missed_beat, missed_total);
			missed_beat = 0U;
			tm_printf((UB *)"[line] dbg GP26 raw=%u min=%u max=%u norm=%u thr=%u/%u on_line=%d"
				  " | GP27 raw=%u min=%u max=%u norm=%u thr=%u/%u on_line=%d | dark_ms=%u lost=%d\n",
				  (UINT)raw.left_guard, (UINT)g_trk.left_cal.white,
				  (UINT)g_trk.left_cal.black, (UINT)out.left_norm,
				  (UINT)g_trk.left_cal.rise, (UINT)g_trk.left_cal.fall,
				  g_trk.left_black ? 1 : 0,
				  (UINT)raw.right_guard, (UINT)g_trk.right_cal.white,
				  (UINT)g_trk.right_cal.black, (UINT)out.right_norm,
				  (UINT)g_trk.right_cal.rise, (UINT)g_trk.right_cal.fall,
				  g_trk.right_black ? 1 : 0,
				  (UINT)out.fields.dark_ms, out.fields.lost ? 1 : 0);
		}
	}
}
