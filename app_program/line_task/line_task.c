/*
 * line_task.c - ir_sense_task: samples the two line sensors (GP26, GP27)
 * and the barcode sensor (GP7) every 1 ms, on core 1.
 *
 * Every sample it publishes line_state for motion_task, steps the junction
 * detector (NAV_FLG_JUNCTION to nav_ctrl_task) and steps the barcode
 * capture, sending widths to barcode_task on bar_mbf with TMO_POL.  It never
 * waits for decoding, the table or telemetry, and never drives the motors.
 * On CAL_REQ from nav_ctrl_task it calibrates over IR_CAL_SAMPLES samples
 * without stopping the 1 ms loop.
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

#define IR_SAMPLE_MS		1	/* the 1 ms cadence (CNF_TIMER_PERIOD 1) */
#define IR_COMMISSION_MS	100	/* print period with LINE_COMMISSIONING_MODE */

/* COMM_W_KIND_LINE states */
#define TELE_LINE_ON		0U
#define TELE_LINE_LOST		1U
#define TELE_LINE_CROSSING	2U

LOCAL ID		g_ir_flgid;
LOCAL ID		g_ir_tid;
LOCAL volatile uint32_t	g_line_state = LINE_STATE_INITIAL;	/* one store, this task only */

LOCAL line_track_t	g_trk;
LOCAL bar_capture_t	g_cap;

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
		send_bar(event, g_cap.count);
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

LOCAL UINT tele_line_state(const line_state_fields_t *f)
{
	if(f->lost) {
		return TELE_LINE_LOST;
	}
	return (LINE_RL_JUNCTION == f->rl) ? TELE_LINE_CROSSING : TELE_LINE_ON;
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
	line_track_out_t	out;
	line_reader_raw_t	raw;
	mission_state_t	mission = nav_mission_state();
	UINT		cal_left = 0U;
	UINT		missed;
	UINT		holdoff = 0U;
	UINT		print_ms = 0U;
	UINT		tele_state = TELE_LINE_LOST;
	UINT		last_rl = LINE_RL_CENTRED;
	BOOL		black;
	BOOL		adc_ok;
	BOOL		fault_reported = FALSE;
	UB		body[COMM_W_INTAKE_SIZE_LINE];
	char		bits[3];
	ER		ercd;

	(void)stacd;
	(void)exinf;

	line_track_init(&g_trk, &left_def, &right_def, LINE_STEER_INVERT != 0);
	bar_capture_init(&g_cap);
	ercd = line_reader_init(NULL);
	if(E_OK > ercd) {
		tm_printf((UB *)"[line] ADC initialization failed, er=%d\n", ercd);
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

	while(1) {
		missed = wait_tick();

		/* sample first, so the 1 ms cadence does not depend on the work */
		raw = line_reader_read_raw();
		adc_ok = (E_OK <= line_reader_last_error()) ? TRUE : FALSE;
		black = barcode_scanner_read_black() ? TRUE : FALSE;

		/* barcode: ticks missed (a long console line) still count as time */
		capture_step(black);
		while(0U < missed) {
			capture_step(black);
			missed--;
		}

		/* a mission change drops any partial read and stale junction */
		if(nav_mission_state() != mission) {
			mission = nav_mission_state();
			send_bar(bar_capture_abort(&g_cap, BAR_ABORT_CANCELLED),
				 (uint16_t)BAR_ABORT_CANCELLED);
			line_track_disarm_junction(&g_trk);
		}

		if(bar_capture_busy(&g_cap)) {
			holdoff = BARCODE_JUNCTION_HOLDOFF_MS;
		} else if(0U < holdoff) {
			holdoff--;
		} else {
			/* not held */
		}
		line_track_step(&g_trk, raw.left_guard, raw.right_guard,
				adc_ok ? true : false, 0U < holdoff, &out);
		g_line_state = out.word;			/* one store */

		if(out.junction) {
			tell_nav(NAV_FLG_JUNCTION);
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
				left_res = ir_cal_finish(&left_acc, &left_def, &left_new);
				right_res = ir_cal_finish(&right_acc, &right_def, &right_new);
				if((IR_CAL_OK == left_res) && (IR_CAL_OK == right_res)) {
					line_track_set_cal(&g_trk, &left_new, &right_new);
					tell_nav(NAV_FLG_CAL_OK_IR);
				} else {
					tell_nav(NAV_FLG_CAL_FAIL);
				}
				tm_printf((UB *)"[line] calibration %s: GP26 %s (white %u, rise %u) GP27 %s (white %u, rise %u)\n",
					  ((IR_CAL_OK == left_res) && (IR_CAL_OK == right_res))
						  ? "OK" : "FAILED",
					  ir_cal_result_name(left_res), (UINT)left_new.white,
					  (UINT)left_new.rise,
					  ir_cal_result_name(right_res), (UINT)right_new.white,
					  (UINT)right_new.rise);
			}
		}

		/* telemetry on edges only; comms_tele_send never blocks */
		if(tele_line_state(&out.fields) != tele_state) {
			tele_state = tele_line_state(&out.fields);
			(void)comms_tele_send(body, (INT)comm_w_pack_line(body,
				tele_state, out.fields.error));
		}

		/* console: never while a barcode is under the car */
		if(bar_capture_busy(&g_cap)) {
			continue;
		}
		if(out.fields.fault && !fault_reported) {
			tm_printf((UB *)"[line] SENSOR FAULT: ADC failed or a sensor is at a rail (GP26=%u GP27=%u)\n",
				  (UINT)raw.left_guard, (UINT)raw.right_guard);
		}
		fault_reported = out.fields.fault ? TRUE : FALSE;
		if(out.junction) {
			tm_printf((UB *)"[line] junction -> nav\n");
		}
		if(out.fields.rl != last_rl) {
			last_rl = out.fields.rl;
			line_reader_format_2bit((UB)last_rl, bits);
			tm_printf((UB *)"[line] RL=%s err=%d\n", bits, (INT)out.fields.error);
		}
		print_ms++;
		if(print_ms >= ((LINE_COMMISSIONING_MODE != 0) ? IR_COMMISSION_MS
							       : LINE_HEARTBEAT_MS)) {
			print_ms = 0U;
			tm_printf((UB *)"[line] raw ADC GP26=%u GP27=%u norm %u/%u err=%d%s%s | barcode DO GP%d=%u (%s)\n",
				  (UINT)raw.left_guard, (UINT)raw.right_guard,
				  (UINT)out.left_norm, (UINT)out.right_norm,
				  (INT)out.fields.error,
				  out.fields.lost ? " LOST" : "",
				  out.fields.cal ? "" : " uncal",
				  (INT)BARCODE_SENSOR_GPIO,
				  (UINT)barcode_scanner_get_last_raw(),
				  black ? "black" : "white");
		}
	}
}
