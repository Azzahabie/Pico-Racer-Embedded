/*
 *	nav_ctrl.c
 *	nav_ctrl_task, MINIMAL version: the System-Level Statechart (Section 5)
 *	without junction / barcode navigation yet.
 *
 *	  Initialising -> Calibrating -> Ready -> Running <-> Paused
 *	                       ^                     |
 *	                       |      Fault <--------+ (IMU / motion fault)
 *	                       +----- (fault reset)   Finished (operator stop)
 *
 *	Buttons (BTN_ISR on core 0, active low, debounced in the task):
 *	  GP20  start (Ready), resume (Paused), reset (Finished),
 *	        fault reset (Fault -> Calibrating)
 *	  GP21  pause (Running)
 *	Events:
 *	  BLOCKED (scan_task, after t_block)  Running -> Paused, buzzer 3 beeps
 *	  LINE_LOST (motion_task)             Running -> Paused, buzzer 3 beeps
 *	  IMU_FAULT / MOTION_FAULT            -> Fault, long beep
 *	  MQTT START / PAUSE / RESUME / RESET / FINISH via mission_cmd_mbf (FR-24)
 *	Calibration (FR-02): CAL_REQ to ir_sense_task and imu_task, wait for
 *	both CAL_OK (10 s), else Fault. If neither task is bound yet (bench),
 *	calibration is SKIPPED and says so on the console. After a fault,
 *	FAULT_CLEAR is sent only after both calibrations pass.
 *
 *	NOT YET (needs Buddies 2 and 3): junction + barcode commands (FR-08),
 *	MOVE_DONE / CMD_REJECTED handling, end-of-course trigger (owner
 *	unassigned), RGB LED on GP18 (WS2812, needs PIO).
 *
 *	Barr-C (safety subset): braces on every body, constant on the left of
 *	==/!=, lowercase _t type names, names of 3+ characters, g_ globals,
 *	p_ pointers, _isr suffix, bool from stdbool.h, final else.
 *	Tabs/brace placement follow the repo.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>
#include <tm/tmonitor.h>

#include "nav_ctrl.h"
#include "../common/gpio_irq.h"
#include "../obstacle_task/scan_if.h"	/* NAV_FLG_BLOCKED */

#define BTN_START_PIN		20U	/* Button 1 */
#define BTN_PAUSE_PIN		21U	/* Button 2 */
#define BUZZER_PIN		22U	/* PWM slice 3, channel A */

#define BTN_DEBOUNCE_MS		30
#define BTN_LOCKOUT_MS		300	/* ignore bounces after a press */
#define CAL_TIMEOUT_MS		10000	/* [TBD with Buddies 3 and 4] */
#define HEARTBEAT_MS		1000	/* wake at least this often */

/* Buzzer: 125 MHz / (div 1) / (wrap + 1) = 2 kHz */
#define BUZZER_SLICE		3U
#define BUZZER_DIV_REG		(PWM_BASE + PWM_CHx_DIV + (BUZZER_SLICE * 0x14))
#define BUZZER_WRAP		62499U

#define NAV_WAIT_BITS		(NAV_FLG_BTN | NAV_FLG_BLOCKED | NAV_FLG_LINE_LOST | \
				 NAV_FLG_LINE_FOUND | NAV_FLG_IMU_FAULT | \
				 NAV_FLG_MOTION_FAULT | NAV_FLG_MISSION_CMD | \
				 NAV_FLG_JUNCTION | NAV_FLG_MOVE_DONE | \
				 NAV_FLG_CMD_REJECTED | NAV_FLG_CONN | \
				 NAV_FLG_MOTION_STOPPED)

/* FAULT_CLEAR opcode for nav_cmd_mbf [PROPOSED, agree with Buddy 2] */
#define NAV_CMD_FAULT_CLEAR	0x10U
#define FAULT_CLEAR_IMU		0x01U
#define FAULT_CLEAR_MOTION	0x02U

LOCAL volatile uint32_t	g_mission = (uint32_t)MS_INITIALISING;	/* one store */
LOCAL ID		g_nav_flgid = 0;
LOCAL nav_bind_t	g_nav_bind;

/* ------------------------------------------------------------------ */
/* Readers                                                            */
/* ------------------------------------------------------------------ */
EXPORT mission_state_t nav_mission_state(void)
{
	return (mission_state_t)g_mission;		/* one aligned load */
}

EXPORT bool nav_mission_running(void)
{
	return ((uint32_t)MS_RUNNING == g_mission);
}

EXPORT ID nav_flag(void)
{
	return g_nav_flgid;
}

EXPORT const char *nav_state_name(mission_state_t state)
{
	const char *p_name;

	switch(state) {
	case MS_INITIALISING:	p_name = "INITIALISING";	break;
	case MS_CALIBRATING:	p_name = "CALIBRATING";		break;
	case MS_READY:		p_name = "READY";		break;
	case MS_RUNNING:	p_name = "RUNNING";		break;
	case MS_PAUSED:		p_name = "PAUSED";		break;
	case MS_FINISHED:	p_name = "FINISHED";		break;
	case MS_FAULT:		p_name = "FAULT";		break;
	default:		p_name = "?";			break;
	}
	return p_name;
}

/* ------------------------------------------------------------------ */
/* Set-up                                                             */
/* ------------------------------------------------------------------ */
EXPORT ER nav_ctrl_init(void)
{
	T_CFLG	cflg = { .exinf = NULL, .flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0U };
	ID	objid;

	if(0 < g_nav_flgid) {
		return E_OK;
	}
	objid = tk_cre_flg(&cflg);
	if(0 >= objid) {
		return (ER)objid;
	}
	g_nav_flgid = objid;
	return E_OK;
}

EXPORT void nav_bind(const nav_bind_t *p_bind)
{
	if(NULL != p_bind) {
		g_nav_bind = *p_bind;
	}
}

/* BTN_ISR: only sets a bit; the task debounces (Section 4). */
LOCAL void nav_btn_isr(UINT pin, UW events, UW now_us)
{
	(void)pin;
	(void)events;
	(void)now_us;
	(void)tk_set_flg(g_nav_flgid, NAV_FLG_BTN);
}

LOCAL void button_init(UINT pin)
{
	UW pad;

	gpio_set_pin(pin, GPIO_MODE_IN);
	pad = in_w(GPIO(pin));
	pad &= ~GPIO_PDE;
	pad |= (GPIO_PUE | GPIO_IE);			/* active low: pull-up */
	out_w(GPIO(pin), pad);
}

LOCAL bool pressed(UINT pin)
{
	return (0U == gpio_get_val(pin));
}

LOCAL void buzzer_init(void)
{
	/* same rule as servo_init(): release the PWM block, never re-reset it */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while(0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) {
		;
	}
	pwm_set_pin(BUZZER_PIN);
	out_w(BUZZER_DIV_REG, 1U << 4);			/* divider 1 */
	pwm_set_wrap(BUZZER_PIN, BUZZER_WRAP);
	pwm_set_cc(BUZZER_PIN, 0U);			/* silent */
	pwm_set_enabled(BUZZER_PIN, TRUE);
}

LOCAL void beep(INT count, INT on_ms)
{
	INT idx;

	for(idx = 0; idx < count; idx++) {
		pwm_set_cc(BUZZER_PIN, (BUZZER_WRAP + 1U) / 2U);	/* 50 % duty */
		tk_dly_tsk(on_ms);
		pwm_set_cc(BUZZER_PIN, 0U);
		tk_dly_tsk(100);
	}
}

/* ------------------------------------------------------------------ */
/* Statechart                                                         */
/* ------------------------------------------------------------------ */
LOCAL void set_state(mission_state_t next)
{
	mission_state_t prev = nav_mission_state();

	g_mission = (uint32_t)next;			/* the only writer */
	tm_printf((UB *)"[nav] mission %s -> %s\n", nav_state_name(prev),
		  nav_state_name(next));
}

LOCAL void send_fault_clear(UB which)
{
	UB	msg[2];
	ER	ercd;

	msg[0] = (UB)NAV_CMD_FAULT_CLEAR;
	msg[1] = which;
	if(0 < g_nav_bind.nav_cmd_mbfid) {
		ercd = tk_snd_mbf(g_nav_bind.nav_cmd_mbfid, msg, (INT)sizeof(msg), TMO_POL);
		if(E_OK != ercd) {
			tm_printf((UB *)"[nav] nav_cmd_mbf full: FAULT_CLEAR not sent\n");
		}
	} else {
		tm_printf((UB *)"[nav] -> nav_cmd_mbf: FAULT_CLEAR(%s) (motion_task not bound)\n",
			  (FAULT_CLEAR_IMU == which) ? "IMU" : "MOTION");
	}
}

/* Calibrating: both must pass (FR-02). Returns true on success. */
LOCAL bool calibrate(void)
{
	UINT	need = 0U;
	UINT	got = 0U;
	UINT	flgptn;
	ER	ercd;
	bool	ok = true;
	bool	done = false;

	if(0 < g_nav_bind.ir_flgid) {
		need |= NAV_FLG_CAL_OK_IR;
		(void)tk_set_flg(g_nav_bind.ir_flgid, g_nav_bind.cal_req_bit);
	}
	if(0 < g_nav_bind.imu_flgid) {
		need |= NAV_FLG_CAL_OK_IMU;
		(void)tk_set_flg(g_nav_bind.imu_flgid, g_nav_bind.cal_req_bit);
	}
	if(0U == need) {
		tm_printf((UB *)"[nav] no ir_sense/imu task bound: calibration SKIPPED "
			  "(bench only, FR-02 not met until they are)\n");
		return true;
	}

	while(!done) {
		ercd = tk_wai_flg(g_nav_flgid, need | NAV_FLG_CAL_FAIL,
				  TWF_ORW | TWF_BITCLR, &flgptn, CAL_TIMEOUT_MS);
		if(E_OK != ercd) {
			tm_printf((UB *)"[nav] calibration timed out\n");
			ok = false;
			done = true;
		} else if(0U != (flgptn & NAV_FLG_CAL_FAIL)) {
			tm_printf((UB *)"[nav] calibration FAILED\n");
			ok = false;
			done = true;
		} else {
			got |= (flgptn & need);
			done = (need == got);
		}
	}
	return ok;
}

LOCAL void enter_calibrating(bool after_fault)
{
	set_state(MS_CALIBRATING);
	if(calibrate()) {
		if(after_fault) {
			send_fault_clear(FAULT_CLEAR_IMU);	/* only after both pass */
			send_fault_clear(FAULT_CLEAR_MOTION);
		}
		set_state(MS_READY);
		beep(1, 100);
	} else {
		set_state(MS_FAULT);
		beep(1, 800);
	}
}

/* One debounced button press: start/resume/reset or pause. */
LOCAL void on_button(void)
{
	bool			start_btn;
	bool			pause_btn;
	mission_state_t		state = nav_mission_state();

	tk_dly_tsk(BTN_DEBOUNCE_MS);
	start_btn = pressed(BTN_START_PIN);
	pause_btn = pressed(BTN_PAUSE_PIN);

	if(start_btn) {
		tm_printf((UB *)"[nav] button GP20\n");
		if((MS_READY == state) || (MS_PAUSED == state)) {
			set_state(MS_RUNNING);		/* new mission / resume */
		} else if(MS_FINISHED == state) {
			set_state(MS_READY);		/* reset for a new mission */
		} else if(MS_FAULT == state) {
			enter_calibrating(true);	/* fault reset */
		} else {
			/* ignored in this state */
		}
	} else if(pause_btn) {
		tm_printf((UB *)"[nav] button GP21\n");
		if(MS_RUNNING == state) {
			set_state(MS_PAUSED);
		}
	} else {
		/* bounce or release: nothing pressed */
	}
	tk_dly_tsk(BTN_LOCKOUT_MS);
	(void)tk_clr_flg(g_nav_flgid, ~NAV_FLG_BTN);	/* drop bounce edges */
}

LOCAL void on_mission_cmd(void)
{
	UB		cmd = 0U;
	INT		size;
	mission_state_t	state;

	if(0 >= g_nav_bind.mission_cmd_mbfid) {
		return;
	}
	size = tk_rcv_mbf(g_nav_bind.mission_cmd_mbfid, &cmd, TMO_POL);
	while(0 < size) {
		state = nav_mission_state();
		tm_printf((UB *)"[nav] MQTT command %d\n", (INT)cmd);
		if((MCMD_START == cmd) && (MS_READY == state)) {
			set_state(MS_RUNNING);
		} else if((MCMD_PAUSE == cmd) && (MS_RUNNING == state)) {
			set_state(MS_PAUSED);
		} else if((MCMD_RESUME == cmd) && (MS_PAUSED == state)) {
			set_state(MS_RUNNING);
		} else if((MCMD_RESET == cmd) && (MS_FINISHED == state)) {
			set_state(MS_READY);
		} else if((MCMD_RESET == cmd) && (MS_FAULT == state)) {
			enter_calibrating(true);
		} else if((MCMD_FINISH == cmd) &&
			  ((MS_RUNNING == state) || (MS_PAUSED == state))) {
			set_state(MS_FINISHED);
		} else {
			tm_printf((UB *)"[nav] command not valid in %s: ignored\n",
				  nav_state_name(state));
		}
		size = tk_rcv_mbf(g_nav_bind.mission_cmd_mbfid, &cmd, TMO_POL);
	}
}

EXPORT void nav_ctrl_task(INT stacd, void *exinf)
{
	UINT	bits;
	UINT	flgptn;
	ER	ercd;

	(void)stacd;
	(void)exinf;

	if(E_OK != nav_ctrl_init()) {
		tm_printf((UB *)"[nav] nav_flg not created: stopped\n");
		while(1) {
			(void)tk_slp_tsk(TMO_FEVR);
		}
	}

	/* Initialising: devices up, motors braked by motion_task's own init */
	set_state(MS_INITIALISING);
	buzzer_init();
	button_init(BTN_START_PIN);
	button_init(BTN_PAUSE_PIN);
	(void)gpio_irq_init();				/* core 0 (BTN_ISR) */
	(void)gpio_irq_attach(BTN_START_PIN, GPIO_IRQ_EDGE_FALL, nav_btn_isr);
	(void)gpio_irq_attach(BTN_PAUSE_PIN, GPIO_IRQ_EDGE_FALL, nav_btn_isr);

	enter_calibrating(false);
	tm_printf((UB *)"[nav] GP20 = start/resume/reset, GP21 = pause\n");

	while(1) {
		ercd = tk_wai_flg(g_nav_flgid, NAV_WAIT_BITS, TWF_ORW | TWF_BITCLR,
				  &flgptn, HEARTBEAT_MS);
		bits = (E_OK == ercd) ? flgptn : 0U;

		/* faults first: they override everything (NFR-13, FR-30) */
		if(0U != (bits & (NAV_FLG_IMU_FAULT | NAV_FLG_MOTION_FAULT))) {
			tm_printf((UB *)"[nav] %s fault\n",
				  (0U != (bits & NAV_FLG_IMU_FAULT)) ? "IMU" : "MOTION");
			if(MS_FAULT != nav_mission_state()) {
				set_state(MS_FAULT);
				beep(1, 800);
			}
		}
		if((0U != (bits & (NAV_FLG_BLOCKED | NAV_FLG_LINE_LOST))) &&
		   (MS_RUNNING == nav_mission_state())) {
			tm_printf((UB *)"[nav] %s: pausing the mission\n",
				  (0U != (bits & NAV_FLG_BLOCKED)) ? "path blocked / sensor lost"
								   : "line lost");
			set_state(MS_PAUSED);
			beep(3, 100);
		}
		if(0U != (bits & NAV_FLG_MISSION_CMD)) {
			on_mission_cmd();
		}
		if(0U != (bits & NAV_FLG_BTN)) {
			on_button();
		}
		if(0U != (bits & (NAV_FLG_JUNCTION | NAV_FLG_MOVE_DONE |
				  NAV_FLG_CMD_REJECTED))) {
			/* junction navigation (FR-08) comes with Buddies 2 and 3 */
			tm_printf((UB *)"[nav] junction event 0x%x: not handled yet\n",
				  (INT)(bits & (NAV_FLG_JUNCTION | NAV_FLG_MOVE_DONE |
						NAV_FLG_CMD_REJECTED)));
		}
	}
}
