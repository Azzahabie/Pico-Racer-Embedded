/*
 *	scan_if.c
 *	scan_task's links to the rest of the system. See scan_if.h.
 *
 *	Barr-C (safety subset): braces on every body, constant on the left of
 *	==/!=, lowercase _t type names, names of 3+ characters, g_ globals,
 *	p_ pointers, bool from stdbool.h. Tabs/brace placement follow the repo.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "scan_if.h"
#include "../motor_task/motor_task.h"

LOCAL scan_if_bind_t	g_bind;			/* all 0 / NULL = standalone */
LOCAL ID		g_scan_flgid = 0;
LOCAL ID		g_tick_cycid = 0;
LOCAL UW		g_tele_drops = 0U;

/* 60 ms cyclic handler: runs outside any task, only sets a bit (Section 4) */
LOCAL void scan_tick_cyc(void *p_exinf)
{
	(void)p_exinf;
	(void)tk_set_flg(g_scan_flgid, SCAN_FLG_TICK);
}

EXPORT void scan_if_bind(const scan_if_bind_t *p_bind)
{
	if(NULL != p_bind) {
		g_bind = *p_bind;
	}
}

EXPORT ER scan_if_init(void)
{
	T_CFLG	cflg = {
		.exinf = NULL, .flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0U
	};
	T_CCYC	ccyc = {
		.exinf = NULL, .cycatr = TA_HLNG | TA_STA,
		.cychdr = (FP)scan_tick_cyc,
		.cyctim = SCAN_TICK_MS, .cycphs = SCAN_TICK_MS
	};				/* other fields 0: no name, any core */
	ID	objid;

	if(0 < g_scan_flgid) {
		return E_OK;				/* already done */
	}

	objid = tk_cre_flg(&cflg);
	if(0 >= objid) {
		return (ER)objid;
	}
	g_scan_flgid = objid;

	objid = tk_cre_cyc(&ccyc);
	if(0 >= objid) {
		return (ER)objid;
	}
	g_tick_cycid = objid;

	if(scan_if_standalone()) {
		tm_printf((UB *)"[scan_if] INCOMPLETE BINDING: motion_flg=%d "
			  "avoid_mbf=%d; avoid commands fail closed\n",
			  (INT)g_bind.motion_flgid, (INT)g_bind.avoid_mbfid);
	}
	return E_OK;
}

EXPORT ID scan_if_flag(void)
{
	return g_scan_flgid;
}

EXPORT bool scan_if_standalone(void)
{
	return (0 >= g_bind.motion_flgid) || (0 >= g_bind.avoid_mbfid);
}

/* Wait for any of bits; clears only the bits that ended the wait. */
EXPORT UINT scan_if_wait(UINT bits, TMO tmout)
{
	UINT	flgptn = 0U;
	ER	ercd;

	ercd = tk_wai_flg(g_scan_flgid, bits, TWF_ORW | TWF_BITCLR, &flgptn, tmout);
	if(E_OK != ercd) {
		flgptn = 0U;				/* timeout */
	}
	return flgptn & bits;
}

EXPORT void scan_if_clear(UINT bits)
{
	(void)tk_clr_flg(g_scan_flgid, ~bits);	/* clr_flg keeps the bits given */
}

/* ---- outgoing signals ---- */

EXPORT void scan_if_stop_set(void)
{
	ER ercd;

	if(0 < g_bind.motion_flgid) {
		motion_stop_stamp(MOTION_EVT_STOP_SCAN);
		ercd = tk_set_flg(g_bind.motion_flgid, MOTION_EVT_STOP_SCAN);
		if(E_OK != ercd) {
			tm_printf((UB *)"[scan_if] STOP_SCAN request failed, er=%d\n",
				  (INT)ercd);
		}
	} else {
		tm_printf((UB *)"[scan_if] STOP_SCAN requested but motion flag is "
			  "not bound; no motor brake is sent\n");
	}
}

EXPORT void scan_if_stop_clear(void)
{
	ER ercd;

	if(0 < g_bind.motion_flgid) {
		ercd = tk_clr_flg(g_bind.motion_flgid, ~MOTION_EVT_STOP_SCAN);
		if(E_OK != ercd) {
			tm_printf((UB *)"[scan_if] STOP_SCAN clear failed, er=%d\n",
				  (INT)ercd);
		}
	} else {
		tm_printf((UB *)"[scan_if] STOP_SCAN cleared locally; motion flag "
			  "is not bound\n");
	}
}

/* Place the command, then set CMD (Section 4 order). Never blocks. */
EXPORT bool scan_if_send_cmd(const avoid_cmd_t *p_cmd)
{
	bool	sent = false;
	ER	ercd;
	uint16_t packed_cmd;

	if((NULL != p_cmd) && !scan_if_standalone()) {
		if(!motion_avoid_cmd_pack(p_cmd->type, p_cmd->side, p_cmd->dist_mm,
					  &packed_cmd)) {
			tm_printf((UB *)"[scan_if] invalid avoid command rejected\n");
			return false;
		}
		ercd = tk_snd_mbf(g_bind.avoid_mbfid, &packed_cmd,
				  (INT)sizeof(packed_cmd), TMO_POL);
		if(E_OK != ercd) {
			sent = false;			/* full: caller retries next tick */
		} else {
			sent = true;
			ercd = tk_set_flg(g_bind.motion_flgid, MOTION_EVT_CMD);
			if(E_OK != ercd) {
				tm_printf((UB *)"[scan_if] command queued but motion wake "
					  "failed, er=%d\n", (INT)ercd);
			}
		}
	}
	return sent;
}

EXPORT void scan_if_set_blocked(void)
{
	if(0 < g_bind.nav_flgid) {
		(void)tk_set_flg(g_bind.nav_flgid, NAV_FLG_BLOCKED);
	} else {
		tm_printf((UB *)"[scan_if] -> nav_flg: BLOCKED (nav_ctrl_task pauses the mission)\n");
	}
}

EXPORT void scan_if_send_tele(const scan_tele_t *p_rec)
{
	ER ercd;

	if(0 < g_bind.tele_mbfid) {
		ercd = tk_snd_mbf(g_bind.tele_mbfid, p_rec, (INT)sizeof(*p_rec), TMO_POL);
		if(E_OK != ercd) {
			g_tele_drops++;			/* never wait on telemetry */
		}
	} else if(TELE_SCAN_FAULT == p_rec->type) {
		tm_printf((UB *)"[scan_if] -> telemetry: SENSOR FAULT record\n");
	} else {
		tm_printf((UB *)"[scan_if] -> telemetry: profile d_c=%d @%d W=%d "
			  "x=%d..%d clr L=%d R=%d act=%d\n",
			  (INT)p_rec->d_close_mm, (INT)p_rec->close_deg,
			  (INT)p_rec->width_mm, (INT)p_rec->x_left_mm,
			  (INT)p_rec->x_right_mm, (INT)p_rec->clear_left_mm,
			  (INT)p_rec->clear_right_mm, (INT)p_rec->action);
	}
}

EXPORT UW scan_if_tele_drops(void)
{
	return g_tele_drops;
}

/* ---- shared state, read through the owner's callbacks ---- */

EXPORT bool scan_if_mission_running(void)
{
	return (NULL == g_bind.p_mission_running) ? true : g_bind.p_mission_running();
}

EXPORT bool scan_if_hump_active(void)
{
	return (NULL == g_bind.p_hump_active) ? false : g_bind.p_hump_active();
}
