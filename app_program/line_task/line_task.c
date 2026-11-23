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
 *
 * BARCODE_SENSOR_ANALOG 1 (line_config.h): the barcode sensor is an analog
 * IR module on GP28 (ADC2), read every 1 ms after the line sensors and fed
 * to the other team's capture (bc_analog_front.h, bc_analog_capture.h); reads cross
 * to barcode_task as bc_analog_link.h says.  The junction side counts the
 * front end's floor-relative black (after the glitch filter) where it
 * counts GP7 above; a read that ENDed at its stop character arms the
 * junction once that black has ended (or the next bar has begun), and one
 * decoded on core 0 arms it when barcode_task's reply comes.  GP7 is not
 * read.
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
#if BARCODE_SENSOR_ANALOG
#include "adc_hw.h"
#include "bc_analog_front.h"
#include "bc_analog_capture.h"
#include "bc_analog_link.h"
#endif

#define IR_SAMPLE_MS		1	/* the 1 ms cadence (CNF_TIMER_PERIOD 1) */
#define IR_COMMISSION_MS	100	/* print period with LINE_COMMISSIONING_MODE */
#define IR_GATE_PRINT_MS	1000	/* at most one "ignored" line this often */

#if LINE_JUNCTION_RULE
#define IR_RULE_TEXT	"barcode-armed, length rule as fallback (LINE_JUNCTION_RULE 1)"
#else
#define IR_RULE_TEXT	"length only (LINE_JUNCTION_RULE 0)"
#endif

/* The barcode sensor in console lines */
#if BARCODE_SENSOR_ANALOG
#define BC_NAME		"GP28"
#else
#define BC_NAME		"GP7"
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
#if !BARCODE_SENSOR_ANALOG
LOCAL bar_capture_t	g_cap;
#endif
#if LINE_JUNCTION_RULE
LOCAL junction_arm_t	g_arm;
#endif
#if LINE_SCAN_GATE
LOCAL scan_gate_t	g_gate;
#endif
#if BARCODE_SENSOR_ANALOG
LOCAL bc_front_t	g_bcf;
LOCAL bc_capture_t	g_bc;	/* barcode_task's while BC_ST_DECODING */
LOCAL BOOL		g_bc_handoff;	/* ANALOG sent, reply awaited */
LOCAL BOOL		g_bc_cancelled;	/* a mission change since it was sent */
LOCAL UINT		g_bc_handoff_ms;
LOCAL uint32_t		g_bc_cmd_before;	/* barcode_cmd_word() before the send */
LOCAL UINT		g_bc_lost;	/* messages bar_mbf had no room for */
#if LINE_JUNCTION_RULE
LOCAL BOOL		g_bc_arm_due;	/* a read ENDed at its stop: arm soon */
LOCAL uint8_t		g_bc_arm_id;
LOCAL char		g_bc_arm_letter;
LOCAL uint32_t		g_bc_arm_cmd;
#endif
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

#if BARCODE_SENSOR_ANALOG
EXPORT const bc_capture_t *line_bc_capture(void)
{
	return &g_bc;
}
#endif

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

#if !BARCODE_SENSOR_ANALOG
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
#endif

#if BARCODE_SENSOR_ANALOG
/* One analog-path message to barcode_task (bc_analog_link.h); false if
   bar_mbf had no room. */
LOCAL BOOL send_analog(uint8_t type, uint16_t value)
{
	UB	msg[BAR_MSG_SIZE];
	ID	mbfid = barcode_bar_mbf();

	if(0 >= mbfid) {
		return FALSE;
	}
	bar_msg_pack(msg, (bar_event_t)type, g_bc.read_id, value);
	BC_LINK_BARRIER();	/* the capture is written before it goes */
	if(E_OK != tk_snd_mbf(mbfid, msg, BAR_MSG_SIZE, TMO_POL)) {
		g_bc_lost++;
		return FALSE;
	}
	return TRUE;
}

/* The other team's end of a code (or a junction cut): barcode_task decodes the
   capture.  Without room in bar_mbf the read is lost and the capture
   cools down as after a reject. */
LOCAL void analog_hand_off(BOOL cut)
{
	g_bc_cmd_before = barcode_cmd_word();
	if(send_analog(BAR_MSG_ANALOG, cut ? (uint16_t)BAR_ANALOG_CUT : 0u)) {
		g_bc_handoff = TRUE;
		g_bc_cancelled = FALSE;
		g_bc_handoff_ms = 0U;
	} else {
		bc_resume(&g_bc, BC_REJECT);
	}
}

/* A read ENDed at its stop character, decoded here (BC_END_ON_STOP). */
LOCAL void analog_letter(const bc_out_t *o)
{
	uint32_t	cmd_before = barcode_cmd_word();

	if(send_analog(BAR_MSG_LETTER, bc_link_letter_value(o->ch,
			o->b_reversed)) && (0U != LINE_JUNCTION_RULE)) {
#if LINE_JUNCTION_RULE
		g_bc_arm_due = TRUE;
		g_bc_arm_id = g_bc.read_id;
		g_bc_arm_letter = o->ch;
		g_bc_arm_cmd = cmd_before;
#else
		(void)cmd_before;
#endif
	}
}

/* n samples of the capture (1 + ticks missed), each with its travel. */
LOCAL void analog_capture(UINT n)
{
	bc_out_t	o;
	bc_result_t	r;

	while(0U < n) {
		r = bc_sample(&g_bc, g_bcf.norm, bc_front_take_step(&g_bcf), &o);
		if(BC_VALID == r) {
			analog_letter(&o);
		} else if(BC_DONE == r) {
			analog_hand_off(FALSE);
		} else {
			/* nothing finished */
		}
		n--;
	}
}

/* barcode_task's reply to the ANALOG message: the capture comes back.  A
   read that decoded (not cut, no mission change since) arms the junction:
   barcode_task stored its command and judged word before the reply. */
LOCAL void analog_reply(UINT elapsed)
{
	uint32_t	reply;
	bc_result_t	r;

	if(!g_bc_handoff) {
		return;
	}
	reply = barcode_analog_reply_word();
	if(!bc_link_reply_is(reply, g_bc.read_id)) {
		g_bc_handoff_ms += elapsed;
		return;
	}
	BC_LINK_BARRIER();	/* core 0 is done with the capture */
	r = bc_link_reply_result(reply);
#if LINE_JUNCTION_RULE
	if((BC_VALID == r) && !g_bc.b_cut && !g_bc_cancelled) {
		junction_arm_read_done(&g_arm, g_bc.read_id,
				       bc_link_reply_letter(reply), g_bc_cmd_before);
	}
#endif
	bc_resume(&g_bc, r);
	g_bc_handoff = FALSE;
}
#endif

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
		tm_printf((UB *)"[line] junction armed: barcode '%c' -> %s, the next black under " BC_NAME " is the junction\n",
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
		tm_printf((UB *)"[line] " BC_NAME " black ignored: motion %s (scan gate): no read, no length-rule junction (%u more since the last line)\n",
			  motion_sm_name(motion), more);
	} else {
		tm_printf((UB *)"[line] " BC_NAME " black ignored: turning %u.%u deg/s %s (scan gate, limit %u): no read, no length-rule junction (%u more since the last line)\n",
			  size / 10U, size % 10U, (avg < 0) ? "right" : "left",
			  (UINT)LINE_SCAN_MAX_TURN_DPS, more);
	}
}

/* Blacks turned away since the last line and not printed yet: printed
   once IR_GATE_PRINT_MS has passed with no new one to print them with. */
LOCAL void print_ignored_rest(UINT more)
{
	tm_printf((UB *)"[line] " BC_NAME " black ignored: %u more since the last line (scan gate)\n",
		  more);
}

/* distance_state lost or back: the gate fails open without it. */
LOCAL void print_ds(BOOL available, BOOL motion_up)
{
	if(available) {
		tm_printf((UB *)"[line] scan gate: distance_state back, no read starts while turning\n");
	} else {
		tm_printf((UB *)"[line] scan gate: no distance_state (%s): " BC_NAME " scans while turning too (fails open)\n",
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
#elif BARCODE_SENSOR_ANALOG
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
#if BARCODE_SENSOR_ANALOG
	uint16_t	bc_raw;
#endif
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
#if BARCODE_SENSOR_ANALOG
	bc_front_init(&g_bcf);
	bc_init(&g_bc);
#else
	bar_capture_init(&g_cap);
#endif
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
#if BARCODE_SENSOR_ANALOG
	adc_hw_pin_init(BC_ANALOG_GPIO);	/* after line_reader_init: ADC up */
#else
	(void)barcode_scanner_init(NULL);
#endif
	(void)ir_sense_init();
	ercd = start_ticks();
	if(E_OK > ercd) {
		tm_printf((UB *)"[line] tk_cre_cyc failed, er=%d: stopped\n", ercd);
		tell_nav(NAV_FLG_CAL_FAIL);
		while(1) {
			(void)tk_slp_tsk(TMO_FEVR);
		}
	}
#if BARCODE_SENSOR_ANALOG
	tm_printf((UB *)"[line] ir_sense_task: line ADC GP%d/GP%d, barcode AO GP%d (ADC%d, analog: the other team's barcode code, BARCODE_SENSOR_ANALOG 1), 1 ms; span %u, black +%u/+%u over the floor\n",
		  (INT)LINE_SENSOR_LEFT_GPIO, (INT)LINE_SENSOR_RIGHT_GPIO,
		  (INT)BC_ANALOG_GPIO, (INT)BC_ANALOG_ADC_CHANNEL,
		  (UINT)BC_ANALOG_SPAN_RAW, (UINT)BC_ANALOG_ON_DELTA,
		  (UINT)BC_ANALOG_OFF_DELTA);
#else
	tm_printf((UB *)"[line] ir_sense_task: line ADC GP%d/GP%d, barcode DO GP%d, 1 ms\n",
		  (INT)LINE_SENSOR_LEFT_GPIO, (INT)LINE_SENSOR_RIGHT_GPIO,
		  (INT)BARCODE_SENSOR_GPIO);
#endif
#if LINE_SCAN_GATE
	tm_printf((UB *)"[line] junction rule: " IR_RULE_TEXT " | scan gate on: " BC_NAME " black opens no read and no length-rule junction in TURNING, BYPASS, LINE_SEARCH or REVERSING, or while the averaged turn rate is %u deg/s or more (LINE_SCAN_GATE 1)\n",
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
#if BARCODE_SENSOR_ANALOG
		/* the analog barcode sensor, then its level and its travel
		   (distance_state, one load, as for the junction below) */
		bc_raw = adc_hw_read(BC_ANALOG_ADC_CHANNEL);
		ds_now = motion_ds_read();
		bc_front_step(&g_bcf, bc_raw, ds_now, 0 < motor_task_flag(),
			      (uint32_t)(1U + missed));
		black = g_bcf.black ? TRUE : FALSE;
#else
		black = barcode_scanner_read_black() ? TRUE : FALSE;
#endif
#if (LINE_JUNCTION_RULE || LINE_SCAN_GATE) && !BARCODE_SENSOR_ANALOG
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
#if BARCODE_SENSOR_ANALOG
		bc_set_gate(&g_bc, scan_open ? true : false);
#else
		bar_capture_set_gate(&g_cap, scan_open ? true : false);
#endif
#endif

#if BARCODE_SENSOR_ANALOG
		/* barcode_task's reply first, then the capture: ticks missed
		   still count as time and travel */
		analog_reply(1U + missed);
		analog_capture(1U + missed);
		missed = 0U;
#else
		/* barcode: ticks missed (a long console line) still count as time */
		capture_step(black);
		while(0U < missed) {
			capture_step(black);
			missed--;
		}
#endif

		/* a mission change drops any partial read and stale junction */
#if LINE_JUNCTION_RULE
		cancel_event = JARM_EV_NONE;
#endif
		if(nav_mission_state() != mission) {
			mission = nav_mission_state();
#if BARCODE_SENSOR_ANALOG
			if(bc_abort(&g_bc)) {
				(void)send_analog(BAR_MSG_ABORT, (uint16_t)BAR_ABORT_CANCELLED);
			}
			g_bc_cancelled = TRUE;	/* a decode under way arms nothing */
#if LINE_JUNCTION_RULE
			g_bc_arm_due = FALSE;
#endif
#else
			send_bar(bar_capture_abort(&g_cap, BAR_ABORT_CANCELLED),
				 (uint16_t)BAR_ABORT_CANCELLED);
#endif
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
#if BARCODE_SENSOR_ANALOG
		seen = g_bcf.seen ? TRUE : FALSE;
		/* a read that ENDed at its stop arms once the sensor has left
		   its last bar, or the next bar has begun (a crossbar close
		   behind it): never on that bar */
		if(g_bc_arm_due && (!seen || g_bc.b_black)) {
			junction_arm_read_done(&g_arm, g_bc_arm_id, g_bc_arm_letter,
					       g_bc_arm_cmd);
			g_bc_arm_due = FALSE;
		}
		judged = barcode_judged_word();
		cmd_now = barcode_cmd_word();
		arm_event = junction_arm_step(&g_arm, judged, cmd_now,
					      seen ? true : false,
					      (uint32_t)((moved < 0) ? -moved : moved),
					      bc_in_read(&g_bc) || g_bc_arm_due, &gate);
		/* a due junction waits for barcode_task's decode, at most
		   BC_ANALOG_VERDICT_WAIT_MS */
		gate.wait = gate.wait ||
			    (g_bc_handoff && (g_bc_handoff_ms < BC_ANALOG_VERDICT_WAIT_MS));
		gate.moved_mm = (uint32_t)((moved < 0) ? -moved : moved);
#else
		judged = barcode_judged_word();
		cmd_now = barcode_cmd_word();
		seen = bar_capture_black(&g_cap) ? TRUE : FALSE;
		arm_event = junction_arm_step(&g_arm, judged, cmd_now,
					      seen ? true : false,
					      (uint32_t)((moved < 0) ? -moved : moved),
					      bar_capture_in_read(&g_cap), &gate);
#endif
#endif

		/* reading, not busy: after a width_too_long abort GP7 may stay
		   black for as long as the car is parked on it */
#if BARCODE_SENSOR_ANALOG
		if(bc_busy(&g_bc)) {
#else
		if(bar_capture_reading(&g_cap)) {
#endif
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
		gate.manoeuvre = g_gate.manoeuvre || (0u < g_gate.hold_mm);
#else
		gate.turning = false;
		gate.manoeuvre = false;
#endif
		line_track_step_gate(&g_trk, raw.left_guard, raw.right_guard,
				     adc_ok ? true : false,
				     (0U != LINE_BARCODE_HOLD) && (0U < holdoff),
				     seen ? true : false, &gate, &out);
#elif LINE_SCAN_GATE
		gate.in_read = false;
		gate.armed_due = false;
		gate.wait = false;
		gate.armed = false;
		gate.moved_mm = 0u;
		gate.manoeuvre = g_gate.manoeuvre || (0u < g_gate.hold_mm);
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
#if BARCODE_SENSOR_ANALOG
			/* the crossbar is not a bar: a capture it started is
			   dropped; one with elements before it is cut there and
			   judged only (no command: this junction has fired) */
			(void)cap_event;
			g_bc_arm_due = FALSE;
			if(BC_DONE == bc_junction(&g_bc)) {
				analog_hand_off(TRUE);
			}
#else
			/* the crossbar is not a bar: a read it opened itself is
			   dropped, so it neither holds line_state for its quiet
			   zone nor prints a reject; a read with widths before it
			   (a speck, a dropout, merged bars) ENDs there,
			   and barcode_task prints those widths */
			cap_event = bar_capture_junction(&g_cap);
			send_bar(cap_event, (BAR_EV_END == cap_event)
				 ? g_cap.count : (uint16_t)BAR_ABORT_JUNCTION);
#endif
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
			tm_printf((UB *)"[line] junction (armed by barcode '%c' %s, " BC_NAME " black %u mm) -> nav\n",
				  (UB)('A' + barcode_cmd_letter(g_arm.cmd)),
				  barcode_action_name(barcode_cmd_action(g_arm.cmd)),
				  (UINT)g_arm.black_mm);
		} else if(out.junction) {
#if defined(LINE_JUNCTION_PAIR) && LINE_JUNCTION_PAIR
			tm_printf((UB *)"[line] junction (both line sensors black, " BC_NAME " black just before) -> nav\n");
#else
			tm_printf((UB *)"[line] junction (" BC_NAME " black %u ms, length rule) -> nav\n",
				  (UINT)LINE_JUNCTION_SAMPLES);
#endif
		} else {
			/* no junction this sample */
		}
		print_arm(cancel_event, mission);
		print_arm(arm_event, mission);
#if defined(LINE_JUNCTION_PAIR) && LINE_JUNCTION_PAIR
		/* debug, also during a read: black runs of 5 mm or more */
		if(g_trk.dbg_pair_mm >= 5u) {
			tm_printf((UB *)"[line] pair: BOTH black %u mm (straight %s)\n",
				  (UINT)g_trk.dbg_pair_mm, g_trk.dbg_turning ? "NO" : "yes");
		}
		if(g_trk.dbg_l_mm >= 5u) {
			tm_printf((UB *)"[line] pair: LEFT black %u mm\n", (UINT)g_trk.dbg_l_mm);
		}
		if(g_trk.dbg_r_mm >= 5u) {
			tm_printf((UB *)"[line] pair: RIGHT black %u mm\n", (UINT)g_trk.dbg_r_mm);
		}
		g_trk.dbg_pair_mm = 0u;
		g_trk.dbg_l_mm = 0u;
		g_trk.dbg_r_mm = 0u;
#endif
#else
		if(out.junction) {
			tm_printf((UB *)"[line] junction (" BC_NAME " black %u ms) -> nav\n",
				  (UINT)LINE_JUNCTION_SAMPLES);
		}
#endif

#if LINE_SCAN_GATE
		/* the scan gate's lines: a GP7 black turned away (at most one a
		   second, counting the rest, which are printed on their own
		   once the second is over and no read is open), and
		   distance_state lost or back */
		gate_quiet = (gate_quiet > elapsed) ? (gate_quiet - elapsed) : 0U;
#if BARCODE_SENSOR_ANALOG
		if(g_bc.ignored != ignored_seen) {
			ignored_more += (UINT)(uint16_t)(g_bc.ignored - ignored_seen);
			ignored_seen = g_bc.ignored;
#else
		if(g_cap.ignored != ignored_seen) {
			ignored_more += (UINT)(uint16_t)(g_cap.ignored - ignored_seen);
			ignored_seen = g_cap.ignored;
#endif
			if(0U == gate_quiet) {
				print_ignored(ignored_more - 1U, motion_now);
				ignored_more = 0U;
				gate_quiet = IR_GATE_PRINT_MS;
			}
#if BARCODE_SENSOR_ANALOG
		} else if((0U == gate_quiet) && (0U < ignored_more) &&
			  !bc_busy(&g_bc)) {
#else
		} else if((0U == gate_quiet) && (0U < ignored_more) &&
			  !bar_capture_reading(&g_cap)) {
#endif
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
#if BARCODE_SENSOR_ANALOG
		if(bc_busy(&g_bc)) {
			continue;
		}
		if(0U < g_bc_lost) {
			tm_printf((UB *)"[line] analog barcode: %u message(s) lost (bar_mbf full): read(s) dropped\n",
				  g_bc_lost);
			g_bc_lost = 0U;
		}
#else
		if(bar_capture_reading(&g_cap)) {
			continue;
		}
#endif
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
#if BARCODE_SENSOR_ANALOG
			tm_printf((UB *)"[line] raw ADC GP26=%u GP27=%u norm %u/%u err=%d%s%s | barcode AO GP%d=%u floor %u norm %u (%s) %u mm/s%s | missed %u ticks (%u since boot)\n",
				  (UINT)raw.left_guard, (UINT)raw.right_guard,
				  (UINT)out.left_norm, (UINT)out.right_norm,
				  (INT)out.fields.error,
				  out.fields.lost ? " LOST" : "",
				  out.fields.cal ? "" : " uncal",
				  (INT)BC_ANALOG_GPIO, (UINT)g_bcf.dark,
				  (UINT)g_bcf.floor_v, (UINT)g_bcf.norm,
				  black ? "black" : "white",
				  (UINT)bc_front_speed_mm_s(&g_bcf),
				  g_bcf.ds_ok ? "" : " (no distance_state)",
				  missed_beat, missed_total);
#else
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
#endif
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
