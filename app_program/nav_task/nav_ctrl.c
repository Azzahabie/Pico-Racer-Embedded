/*
 *	nav_ctrl.c
 *	nav_ctrl_task: the System-Level Statechart (Section 5) and the
 *	junction / barcode navigation.
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
 *	FAULT_CLEAR is sent only after both calibrations pass.  A GP20 fault
 *	reset first waits NAV_CAL_SETTLE_MS, hands off (the press rocks the car).
 *	MQTT commands sent while calibrating are dropped, never applied later.
 *
 *	JUNCTION (ir_sense_task, FR-08): barcode_task's stored command goes to
 *	motion_task as NAV_CMD_JUNCTION on nav_cmd_mbf; no stored command means
 *	no command (straight on).  motion_task turns and answers MOVE_DONE, or
 *	CMD_REJECTED when it is not following the line (a bypass, a search);
 *	the stored command counts as used only on MOVE_DONE, or when the
 *	mission pauses during the turn (NAV_KEEP_CMD_ON_REFUSAL 1, default).
 *	With 0 it is used up at the junction where it is sent, as before.
 *
 *	End of the run: the demo ends by lifting and inverting the car
 *	(the prof, 19 Nov), which imu_task turns into an IMU fault.
 *	NOT YET: RGB LED on GP18 (WS2812, needs PIO).
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
#include "../line_task/barcode_task.h"		/* barcode_cmd_word() */
#include "../line_task/barcode_action.h"
#include "../common/messages.h"		/* comm_w_pack_mission() */
#include "../comms_task/comms_task.h"	/* comms_tele_send() */

#define BTN_START_PIN		20U	/* Button 1 */
#define BTN_PAUSE_PIN		21U	/* Button 2 */
#define BUZZER_PIN		22U	/* PWM slice 3, channel A; no motor on slice 3 */

#define BTN_DEBOUNCE_MS		30

/* S5 turn test without barcodes: a junction with no barcode command uses
   this action (barcode_action_t: 0 forward, 1 left, 2 right, 3 U-turn)
   instead of going straight on.  -1, the default, is the real behaviour;
   never set it for the demo build.  A TEST image says so at boot. */
#ifndef NAV_TEST_JUNCTION_ACTION
#define NAV_TEST_JUNCTION_ACTION	(-1)
#endif
/* What a refused JUNCTION (CMD_REJECTED) does to the barcode command.
   1, the default: the command is kept for the next junction.  During a
   bypass or a line search GP7 crosses lines that read as junctions (the 4
   Nov logs show 0-7 per bypass, M: trials_batch1/2, fr20_bypass_rejoin),
   motion refuses each one, and the first of them used to lose the command
   before the real junction after the box.  This goes against the design
   rule "a junction command is only valid at the junction where it was
   issued" (Week 6 review, section 8.2): with the box just past a
   crossroad, the crossroad is crossed during the bypass, refused, and the
   command then runs at the next junction, the wrong place.
   0: the command is used up at the junction where it is sent, whatever
   motion answers and also when nav_cmd_mbf is full (the rule, and the
   behaviour before 19 Nov).  The owner chooses. */
#ifndef NAV_KEEP_CMD_ON_REFUSAL
#define NAV_KEEP_CMD_ON_REFUSAL	1
#endif
#define BTN_LOCKOUT_MS		300	/* ignore bounces after a press */
#define CAL_TIMEOUT_MS		10000	/* [TBD with Buddies 3 and 4] */
/* Fault reset by GP20: the press rocks the car, and the line calibration
   (1 s, no stillness check) and the IMU's first try started at once.  On
   4 Nov 13 of 29 calibrations failed with line "low_contrast" or IMU "not
   still" / "car moving", e.g. GP20 at 04:17:12.841, then GP27 low_contrast
   and IMU "not still" 1 s later (trials_batch1:2299-2305).  So after a
   GP20 fault reset nav waits this long, hands off, before CAL_REQ.  Boot
   and an MQTT reset start at once: nobody touches the car.  0 = no wait,
   as before. */
#ifndef NAV_CAL_SETTLE_MS
#define NAV_CAL_SETTLE_MS	1000
#endif
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

LOCAL volatile uint32_t	g_mission = (uint32_t)MS_INITIALISING;	/* one store */
LOCAL ID		g_nav_flgid = 0;
LOCAL ID		g_fault_flgid = 0;
LOCAL nav_bind_t	g_nav_bind;
LOCAL bool		g_move_open = false;	/* JUNCTION sent, no reply yet */
LOCAL uint32_t		g_cmd_taken;		/* seq of the last barcode command used */
LOCAL bool		g_cmd_sent = false;	/* the open JUNCTION carries a barcode command */
LOCAL uint32_t		g_cmd_sent_seq;		/* ... with this seq */

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

	if((0 < g_nav_flgid) && (0 < g_fault_flgid)) {
		return E_OK;
	}
	if(0 >= g_nav_flgid) {
		objid = tk_cre_flg(&cflg);
		if(0 >= objid) {
			return (ER)objid;
		}
		g_nav_flgid = objid;
	}
	objid = tk_cre_flg(&cflg);			/* fault_flg, all bits clear */
	if(0 >= objid) {
		return (ER)objid;
	}
	g_fault_flgid = objid;
	return E_OK;
}

/* ------------------------------------------------------------------ */
/* Shared fault bitmask                                               */
/* ------------------------------------------------------------------ */
EXPORT void fault_set(UINT bits)
{
	if(0 < g_fault_flgid) {
		(void)tk_set_flg(g_fault_flgid, bits & FAULT_ALL);
	}
}

EXPORT void fault_clear(UINT bits)
{
	if(0 < g_fault_flgid) {
		(void)tk_clr_flg(g_fault_flgid, ~(bits & FAULT_ALL));	/* keeps the rest */
	}
}

EXPORT UINT fault_bits(void)
{
	T_RFLG	rflg;
	UINT	bits = 0U;

	if((0 < g_fault_flgid) && (E_OK == tk_ref_flg(g_fault_flgid, &rflg))) {
		bits = rflg.flgptn & FAULT_ALL;
	}
	return bits;
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

/* Silent: GP22 as a plain output driven low, off the PWM.  Slice 3 is
   shared only with GP6/GP7/GP23 (no motor: those are slices 4 and 5, the
   servo 6), and GP7 (barcode DO) and GP23 (CYW43 WL_ON) are SIO pins, so
   the slice itself sounds nothing unless GP22 is routed to it. */
LOCAL void buzzer_off(void)
{
	pwm_set_enabled(BUZZER_PIN, FALSE);
	(void)gpio_set_val(BUZZER_PIN, 0U);		/* low before it drives */
	(void)gpio_set_pin(BUZZER_PIN, GPIO_MODE_OUT);	/* FUNCSEL SIO */
}

LOCAL void buzzer_init(void)
{
	/* same rule as servo_init(): release the PWM block, never re-reset it */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while(0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) {
		;
	}
	buzzer_off();					/* silent by default */
	out_w(BUZZER_DIV_REG, 1U << 4);			/* divider 1 */
	pwm_set_wrap(BUZZER_PIN, BUZZER_WRAP);
	pwm_set_cc(BUZZER_PIN, (BUZZER_WRAP + 1U) / 2U);	/* 50 %, used only by beep() */
}

/* The only place the buzzer sounds: GP22 is on the PWM for each on_ms
   and back on a driven-low GPIO otherwise. */
LOCAL void beep(INT count, INT on_ms)
{
	INT idx;

	tm_printf((UB *)"[nav] buzzer: %d x %d ms\n", count, on_ms);
	for(idx = 0; idx < count; idx++) {
		pwm_set_enabled(BUZZER_PIN, TRUE);
		pwm_set_pin(BUZZER_PIN);
		tk_dly_tsk(on_ms);
		buzzer_off();
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
	if((MS_RUNNING != next) && (MS_PAUSED != next)) {
		g_move_open = false;	/* no turn survives Finished, Fault or a new mission */
		g_cmd_sent = false;
	} else if((MS_RUNNING == prev) && (MS_PAUSED == next) && g_cmd_sent) {
		/* Paused (GP21, MQTT, BLOCKED, LINE_LOST) during a barcode turn:
		   motion drops the turn and answers CMD_REJECTED.  The turn was
		   started at the junction the command was for, so it counts as
		   used: kept, it would turn the car at a later junction, or pivot
		   a full turn from the half-turned pose when GP7 reads the same
		   junction again after the resume. */
		g_cmd_taken = g_cmd_sent_seq;
		g_cmd_sent = false;
		tm_printf((UB *)"[nav] paused during a barcode turn: command used\n");
	} else {
		/* Running, or Paused with no barcode turn open: nothing to end */
	}
	tm_printf((UB *)"[nav] mission %s -> %s\n", nav_state_name(prev),
		  nav_state_name(next));

	/* E mission: one record per change, so a run can be rebuilt from the
	   broker (FR-23).  Never blocks; a full queue counts a drop.  S.mis
	   does not depend on it: comms_task reads the state itself. */
	{
		UB	body[COMM_W_INTAKE_SIZE_MISSION];

		(void)comms_tele_send(body, (INT)comm_w_pack_mission(body, (uint32_t)next));
	}
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

	/* A reply that arrived after an earlier attempt timed out must not
	 * answer this one: drop stale CAL_OK / CAL_FAIL before asking. */
	(void)tk_clr_flg(g_nav_flgid, ~(NAV_FLG_CAL_OK_IR | NAV_FLG_CAL_OK_IMU |
					NAV_FLG_CAL_FAIL));

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
			if(0U != (flgptn & NAV_FLG_CAL_OK_IMU)) {
				/* imu_task cleared its own fault when it answered OK:
				   an IMU_FAULT it raised before that (e.g. no data at
				   boot) is stale, and left set it would drop Ready
				   straight back to Fault */
				(void)tk_clr_flg(g_nav_flgid, ~NAV_FLG_IMU_FAULT);
				tm_printf((UB *)"[nav] CAL_OK_IMU: stale IMU_FAULT cleared\n");
			}
			got |= (flgptn & need);
			done = (need == got);
		}
	}
	return ok;
}

/* MQTT commands that queued while calibrating are dropped, not applied
   later: a START sent during calibration used to start the run the moment
   Ready was reached, with nobody at the car.  The flag is cleared first,
   so a command placed during the drain is never left behind unseen. */
LOCAL void drop_mission_cmds(void)
{
	UB	cmd = 0U;

	if(0 >= g_nav_bind.mission_cmd_mbfid) {
		return;
	}
	(void)tk_clr_flg(g_nav_flgid, ~NAV_FLG_MISSION_CMD);
	while(0 < tk_rcv_mbf(g_nav_bind.mission_cmd_mbfid, &cmd, TMO_POL)) {
		tm_printf((UB *)"[nav] MQTT command %d sent while CALIBRATING: dropped\n",
			  (INT)cmd);
	}
}

/* settle_ms: wait this long before CAL_REQ (a GP20 press moved the car) */
LOCAL void enter_calibrating(bool after_fault, INT settle_ms)
{
	bool	cal_ok;

	set_state(MS_CALIBRATING);
	if(0 < settle_ms) {
		tm_printf((UB *)"[nav] hands off the car: calibration starts in %d ms\n",
			  settle_ms);
		tk_dly_tsk(settle_ms);
	}
	cal_ok = calibrate();
	drop_mission_cmds();
	if(cal_ok) {
		if(after_fault) {
			send_fault_clear(FAULT_CLEAR_IMU);	/* only after both pass */
			send_fault_clear(FAULT_CLEAR_MOTION);
		}
		/* junctions seen while calibrating are not this mission's */
		(void)tk_clr_flg(g_nav_flgid, ~NAV_FLG_JUNCTION);
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
			enter_calibrating(true, NAV_CAL_SETTLE_MS);	/* fault reset */
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
			enter_calibrating(true, 0);	/* nobody touches the car */
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

/* NAV_FLG_JUNCTION from ir_sense_task (FR-08): the barcode command stored
   since the last junction goes to motion_task as NAV_CMD_JUNCTION.  It is
   used up only when motion turns (on_motion_reply), or here with
   NAV_KEEP_CMD_ON_REFUSAL 0.  With none stored, no command: motion_task
   follows the line straight on. */
LOCAL void on_junction(void)
{
	uint32_t	cmd = barcode_cmd_word();	/* one aligned load */
	uint32_t	action = barcode_cmd_action(cmd);
	bool		b_barcode = false;
	UB		msg[2];
	ER		ercd;

	if(MS_RUNNING != nav_mission_state()) {
		tm_printf((UB *)"[nav] junction while %s: ignored\n",
			  nav_state_name(nav_mission_state()));
		return;
	}
	if(g_move_open) {
		/* the sensors cross the junction's lines again during a turn */
		tm_printf((UB *)"[nav] junction during a move: ignored\n");
		return;
	}
	if(barcode_cmd_pending(cmd, g_cmd_taken)) {
#if NAV_KEEP_CMD_ON_REFUSAL
		b_barcode = true;		/* used once motion has turned */
#else
		g_cmd_taken = barcode_cmd_seq(cmd);	/* this junction used it */
#endif
	} else {
#if NAV_TEST_JUNCTION_ACTION >= 0
		action = (uint32_t)NAV_TEST_JUNCTION_ACTION;
		tm_printf((UB *)"[nav] junction, no barcode command: TEST action %s\n",
			  barcode_action_name(action));
#else
		tm_printf((UB *)"[nav] junction, no barcode command: straight on\n");
		return;
#endif
	}
	msg[0] = (UB)NAV_CMD_JUNCTION;
	msg[1] = (UB)action;
	if(0 < g_nav_bind.nav_cmd_mbfid) {
		ercd = tk_snd_mbf(g_nav_bind.nav_cmd_mbfid, msg, (INT)sizeof(msg), TMO_POL);
		if(E_OK == ercd) {
			g_move_open = true;
			g_cmd_sent = b_barcode;
			g_cmd_sent_seq = barcode_cmd_seq(cmd);
			tm_printf((UB *)"[nav] -> motion: JUNCTION(%s)\n",
				  barcode_action_name(action));
		} else {
			tm_printf((UB *)"[nav] nav_cmd_mbf full: JUNCTION(%s) not sent\n",
				  barcode_action_name(action));
		}
	} else {
		tm_printf((UB *)"[nav] -> nav_cmd_mbf: JUNCTION(%s) (motion_task not bound)\n",
			  barcode_action_name(action));
	}
}

/* MOVE_DONE or CMD_REJECTED for the open JUNCTION.  The barcode command it
   carried is used up only when motion turned (MOVE_DONE; with
   NAV_KEEP_CMD_ON_REFUSAL 0 it was used when sent).  motion_task
   refuses a JUNCTION outside Line Following, and during a bypass or a
   search GP7 crosses lines that read as junctions: a refusal there used to
   lose the command before the real junction.  Now it waits for the next
   junction.  (A turn dropped half way by an obstacle also answers
   CMD_REJECTED, so its command is used at the next junction instead.  A
   turn dropped by a pause was already counted as used in set_state.) */
LOCAL void on_motion_reply(UINT bits)
{
	bool	b_done = (0U != (bits & NAV_FLG_MOVE_DONE));
	bool	b_kept = (!b_done) && g_cmd_sent;

	if(b_done && g_cmd_sent) {
		g_cmd_taken = g_cmd_sent_seq;		/* this junction used it */
	}
	g_cmd_sent = false;
	g_move_open = false;				/* ready for the next junction */
	tm_printf((UB *)"[nav] motion: %s%s\n", b_done ? "MOVE_DONE" : "CMD_REJECTED",
		  b_kept ? ", barcode command kept for the next junction" : "");
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

	enter_calibrating(false, 0);
	tm_printf((UB *)"[nav] GP20 = start/resume/reset, GP21 = pause\n");
#if NAV_TEST_JUNCTION_ACTION >= 0
	tm_printf((UB *)"[nav] TEST image: NAV_TEST_JUNCTION_ACTION %d, a junction with "
		  "no barcode command takes %s (not for the demo)\n",
		  (INT)NAV_TEST_JUNCTION_ACTION,
		  barcode_action_name((uint32_t)NAV_TEST_JUNCTION_ACTION));
#endif

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
		if(0U != (bits & NAV_FLG_JUNCTION)) {
			on_junction();
		}
		if(0U != (bits & (NAV_FLG_MOVE_DONE | NAV_FLG_CMD_REJECTED))) {
			on_motion_reply(bits);
		}
	}
}
