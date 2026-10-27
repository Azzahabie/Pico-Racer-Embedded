/*
 *	obstacle_task.c
 *	Buddy 5: scan_task (design Section 5, Statechart 5; FR-13 to FR-18).
 *	Priority 13, pinned to core 1 (app_main.c).
 *
 *	OBSTACLE_TEST_MODE:
 *	  0 = ultrasonic range test (A5, NFR-06): 20 pings per round
 *	  1 = servo sweep, no sensor
 *	  2 = servo mounting calibration
 *	  3 = one coarse + fine scan every 3 s, printed (NFR-20 width trials)
 *	  4 = scan_task as designed: the full Statechart 5  <- default
 *
 *	Statechart 5 (mode 4):
 *	  Idle        mission not Running: servo centred, STOP_SCAN clear.
 *	  Monitoring  ping ahead every 60 ms (TICK). 2 readings in a row below
 *	              D_stop (4 on a hump) -> set STOP_SCAN -> Coarse Scan.
 *	  Coarse Scan 5 angles. Nothing within D_obs -> false alarm: clear
 *	              STOP_SCAN -> Monitoring. Else -> Fine Scan.
 *	  Fine Scan   5 deg steps out from each coarse edge, median of 3 at the
 *	              edge angles, capped to keep the scan inside 5.5 s (NFR-07).
 *	              Profile -> telemetry -> Planning.
 *	  Planning    continue / bypass / reverse / blocked (FR-17). Bypass:
 *	              clear replies, place command, set CMD, clear STOP_SCAN ->
 *	              Avoiding. Reverse (once): STOP_SCAN stays set -> Reversing.
 *	              Buffer full: keep STOP_SCAN, retry next tick.
 *	  Reversing   wait REVERSE_DONE -> Coarse Scan with D_obs + reverse.
 *	  Avoiding    keep pinging; 2 close -> STOP_SCAN -> Coarse Scan (re-plan).
 *	              LINE_FOUND -> Monitoring; LINE_LOST -> Idle.
 *	  Blocked     STOP_SCAN stays. 3 clear in a row -> Coarse Scan;
 *	              only a confirmed false alarm or LINE_FOUND resumes
 *	              monitoring.
 *	              After t_block -> BLOCKED on nav_flg (mission pauses).
 *	  Sensor Lost 3 no-echo pings in a row (any active state): STOP_SCAN,
 *	              fault record. 3 valid echoes -> Coarse Scan. t_block ->
 *	              BLOCKED on nav_flg.
 *	  Any state -> Idle when the mission is not Running.
 *
 *	STOP_SCAN uses the bound motion flag. Avoid commands fail closed until
 *	the real motion command buffer is bound; replies are never simulated.
 *
 *	Barr-C (safety subset): braces on every body, constant on the left of
 *	==/!=, lowercase _t type names, names of 3+ characters, g_ globals,
 *	p_ pointers, bool from stdbool.h, no continue, final else.
 *	Tabs/brace placement follow the repo.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <tm/tmonitor.h>

#include "obstacle_task.h"
#include "ultrasonic.h"
#include "servo.h"
#include "scan_logic.h"
#include "scan_if.h"
#include "../common/messages.h"

#define OBSTACLE_TEST_MODE	4

#define SAMPLES_PER_ROUND	20U
#define NUM_COARSE		5
#define NO_HIT			999	/* "no coarse hit yet" marker angle */

#define SCAN_BUDGET_MS		5500	/* leaves ramp time inside NFR-07's 7 s budget */
#define FINE_MAX_STEPS		5	/* new angles per edge: the coarse miss is
					   6 fine steps away and already measured */
#define LOOK_EST_MS		200	/* settle 150 + one ping (budget check)  */
#define MEDIAN_EST_MS		360	/* settle 150 + 3 pings 60 ms apart       */
#define T_BLOCK_MS		10000	/* [t_block] proposed                     */
#define REVERSE_TIMEOUT_MS	5000	/* no REVERSE_DONE by then: Blocked [TBD] */

_Static_assert(SL_MAX_OFFSET_MM <= MOTION_AVOID_DIST_MAX,
	       "bypass offset exceeds the packed avoid-command distance");
_Static_assert(SL_REV_DIST_MM <= MOTION_AVOID_DIST_MAX,
	       "reverse distance exceeds the packed avoid-command distance");

LOCAL const INT g_coarse_deg[NUM_COARSE] = { -45, -25, 0, 25, 45 };

/* ------------------------------------------------------------------ */
/* Mode 1: servo sweep                                                */
/* ------------------------------------------------------------------ */
LOCAL void servo_go(INT deg)
{
	UW pulse_us = servo_move_to(deg);

	tm_printf((UB *)"[servo] angle %d deg -> pulse %d us\n", deg, (INT)pulse_us);
	tk_dly_tsk(500);
}

LOCAL void servo_test(void)
{
	INT idx;
	INT deg;

	servo_init();
	tm_printf((UB *)"[servo] SG90 test on GP%d\n", (INT)SERVO_PIN);
	tk_dly_tsk(1000);

	while(1) {
		tm_printf((UB *)"[servo] -- centre --\n");
		servo_go(0);
		tk_dly_tsk(1000);

		tm_printf((UB *)"[servo] -- coarse scan --\n");
		for(idx = 0; idx < NUM_COARSE; idx++) {
			servo_go(g_coarse_deg[idx]);
		}

		tm_printf((UB *)"[servo] -- fine scan, 5 deg steps --\n");
		for(deg = -20; deg <= 20; deg += 5) {
			servo_go(deg);
		}

		tk_dly_tsk(2000);
	}
}

/* ------------------------------------------------------------------ */
/* Mode 2: mounting calibration                                       */
/* ------------------------------------------------------------------ */
LOCAL void servo_hold(INT deg, const char *p_where, INT secs)
{
	UW pulse_us = servo_move_to(deg);

	tm_printf((UB *)"[calib] angle %d deg (pulse %d us): sensor should point %s\n",
		  deg, (INT)pulse_us, p_where);
	tk_dly_tsk(secs * 1000);
}

LOCAL void servo_calibrate(void)
{
	servo_init();
	tm_printf((UB *)"[calib] trim %d us, dir %d. Look at the sensor from above.\n",
		  (INT)SERVO_TRIM_US, (INT)SERVO_DIR);
	while(1) {
		servo_hold(0,   "STRAIGHT AHEAD", 5);
		servo_hold(-45, "LEFT (45 deg)",  3);
		servo_hold(0,   "STRAIGHT AHEAD", 3);
		servo_hold(45,  "RIGHT (45 deg)", 3);
	}
}


/* ------------------------------------------------------------------ */
/* Statechart 5                                                       */
/* ------------------------------------------------------------------ */
typedef enum {
	ST_IDLE = 0,
	ST_MONITORING,
	ST_COARSE,
	ST_FINE,
	ST_PLANNING,
	ST_REVERSING,
	ST_AVOIDING,
	ST_BLOCKED,
	ST_SENSOR_LOST
} scan_state_t;

/* scan_ctx_t.bits (minimum-bit requirement). The context is private to
 * scan_task: no other task or ISR reads or writes it, so the packed
 * read-modify-writes below need no lock.
 *   bits 0-3  state, 9 values (Statechart 5 plus Reversing)
 *   bit  4    reverse_tried  kept until LINE_FOUND
 *   bit  5    after_reverse  rescan with D_obs + reverse
 *   bit  6    blocked_sent   BLOCKED raised once per entry
 *   bit  7    abort_scan     mission left Running mid-scan
 *   bit  8    lost_in_scan   3 no-echo pings mid-scan
 *   bits 9-10 scan_no_echo   no-echo pings in a row, saturates at 3
 * The scan grid stays uint16_t per slot (Tribunal 1 decision): 12 bits
 * would cover the range but save only ~9 bytes for a shift on every ping. */
#define CTX_STATE_MASK		0x000FU
#define CTX_REVERSE_TRIED	(1U << 4)
#define CTX_AFTER_REVERSE	(1U << 5)
#define CTX_BLOCKED_SENT	(1U << 6)
#define CTX_ABORT_SCAN		(1U << 7)
#define CTX_LOST_IN_SCAN	(1U << 8)
#define CTX_NO_ECHO_SHIFT	9U
#define CTX_NO_ECHO_MAX		3U
#define CTX_NO_ECHO_MASK	(CTX_NO_ECHO_MAX << CTX_NO_ECHO_SHIFT)

_Static_assert(ST_SENSOR_LOST <= CTX_STATE_MASK, "scan state needs more than 4 bits");
_Static_assert(SL_N_NO_ECHO <= CTX_NO_ECHO_MAX, "scan_no_echo field too narrow");

typedef struct {
	uint16_t	bits;		/* state + flags, see above       */
	uint16_t	grid[SL_SLOTS];
	sl_profile_t	prof;
	sl_action_t	action;
	sl_runs_t	runs;		/* forward-ping counters          */
	INT		left_hit;	/* outermost coarse hits          */
	INT		right_hit;
	UW		state_us;	/* when the state was entered     */
	UW		scan_us;	/* when the coarse scan started   */
} scan_ctx_t;

LOCAL scan_state_t ctx_state(const scan_ctx_t *p_ctx)
{
	return (scan_state_t)(p_ctx->bits & CTX_STATE_MASK);
}

LOCAL void ctx_set_state(scan_ctx_t *p_ctx, scan_state_t state)
{
	p_ctx->bits = (uint16_t)((p_ctx->bits & ~CTX_STATE_MASK) |
				 ((uint16_t)state & CTX_STATE_MASK));
}

LOCAL bool ctx_flag(const scan_ctx_t *p_ctx, uint16_t mask)
{
	return (0U != (p_ctx->bits & mask));
}

LOCAL void ctx_set_flag(scan_ctx_t *p_ctx, uint16_t mask, bool on)
{
	if(on) {
		p_ctx->bits = (uint16_t)(p_ctx->bits | mask);
	} else {
		p_ctx->bits = (uint16_t)(p_ctx->bits & ~mask);
	}
}

LOCAL uint8_t ctx_no_echo(const scan_ctx_t *p_ctx)
{
	return (uint8_t)((p_ctx->bits & CTX_NO_ECHO_MASK) >> CTX_NO_ECHO_SHIFT);
}

LOCAL void ctx_set_no_echo(scan_ctx_t *p_ctx, uint8_t count)
{
	if(count > CTX_NO_ECHO_MAX) {
		count = (uint8_t)CTX_NO_ECHO_MAX;	/* saturate */
	}
	p_ctx->bits = (uint16_t)((p_ctx->bits & ~CTX_NO_ECHO_MASK) |
				 ((uint16_t)count << CTX_NO_ECHO_SHIFT));
}

LOCAL const char *state_name(scan_state_t state)
{
	const char *p_name;

	switch(state) {
	case ST_IDLE:		p_name = "IDLE";		break;
	case ST_MONITORING:	p_name = "MONITORING";		break;
	case ST_COARSE:		p_name = "COARSE_SCAN";		break;
	case ST_FINE:		p_name = "FINE_SCAN";		break;
	case ST_PLANNING:	p_name = "PLANNING";		break;
	case ST_REVERSING:	p_name = "REVERSING";		break;
	case ST_AVOIDING:	p_name = "AVOIDING";		break;
	case ST_BLOCKED:	p_name = "BLOCKED";		break;
	case ST_SENSOR_LOST:	p_name = "SENSOR_LOST";		break;
	default:		p_name = "?";			break;
	}
	return p_name;
}

LOCAL UW now_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

LOCAL UW ms_since(UW start_us)
{
	return (now_us() - start_us) / 1000U;		/* wrap-safe */
}

LOCAL INT d_obs_mm(const scan_ctx_t *p_ctx)
{
	return ctx_flag(p_ctx, CTX_AFTER_REVERSE) ? (SL_D_OBS_MM + SL_REV_DIST_MM)
						  : SL_D_OBS_MM;
}

LOCAL bool is_hit(const scan_ctx_t *p_ctx, uint16_t dist_mm)
{
	return (SL_FAR != dist_mm) && (SL_NOT_MEASURED != dist_mm) &&
	       ((INT)dist_mm <= d_obs_mm(p_ctx));
}

LOCAL uint8_t n_stop(void)
{
	return scan_if_hump_active() ? (uint8_t)SL_N_STOP_HUMP : (uint8_t)SL_N_STOP;
}

LOCAL uint16_t to_grid(UW dist_mm)
{
	uint16_t result;

	if(0U == dist_mm) {
		result = 1U;				/* 0 means "far" in the grid */
	} else if(dist_mm > 0xFFFEU) {
		result = 0xFFFEU;
	} else {
		result = (uint16_t)dist_mm;
	}
	return result;
}

LOCAL void servo_centre(void)
{
	servo_move_to(0);
	tk_dly_tsk(SERVO_SETTLE_MS);
}

/* State entry actions (Statechart 5). */
LOCAL void enter(scan_ctx_t *p_ctx, scan_state_t next)
{
	scan_state_t prev = ctx_state(p_ctx);

	tm_printf((UB *)"[scan] %s -> %s\n", state_name(prev), state_name(next));
	if((ST_SENSOR_LOST == prev) && (ST_SENSOR_LOST != next)) {
		scan_if_sensor_lost(false);	/* our fault bit, cleared by us */
	}
	ctx_set_state(p_ctx, next);
	p_ctx->state_us = now_us();
	ctx_set_flag(p_ctx, CTX_BLOCKED_SENT, false);
	sl_runs_reset(&p_ctx->runs);

	switch(next) {
	case ST_IDLE:
		scan_if_stop_clear();		/* no stale stop survives */
		ctx_set_flag(p_ctx, CTX_REVERSE_TRIED | CTX_AFTER_REVERSE, false);
		servo_centre();
		break;
	case ST_SENSOR_LOST:
		scan_if_stop_set();		/* cannot see: must not drive */
		scan_if_sensor_lost(true);	/* FAULT_SCAN_SENSOR_LOST -> S record */
		servo_centre();
		break;
	case ST_MONITORING:
	case ST_AVOIDING:
	case ST_BLOCKED:
		servo_centre();			/* forward pings */
		break;
	default:
		break;				/* scans move the servo themselves */
	}
	/* A TICK that queued up during a scan must not fire a ping sooner
	 * than 60 ms after the last one. */
	scan_if_clear(SCAN_FLG_TICK);
}

/* One forward ping for Monitoring / Avoiding / Blocked / Sensor Lost. */
LOCAL sl_ping_t fwd_ping(scan_ctx_t *p_ctx)
{
	UW		dist_mm = 0;
	UW		width_us = 0;
	us_status_t	status = us_ping(&dist_mm, &width_us);
	sl_ping_t	ping;

	ping = sl_classify(US_NO_ECHO == status,
			   (US_OK == status) ? (uint32_t)dist_mm : SL_FAR);
	sl_runs_add(&p_ctx->runs, ping);
	return ping;
}

/* One ping during a scan, as a grid value. No echo is NOT a clear
 * reading: it stays "not measured", and 3 in a row abort to Sensor Lost. */
LOCAL uint16_t scan_ping(scan_ctx_t *p_ctx)
{
	UW		dist_mm = 0;
	UW		width_us = 0;
	us_status_t	status = us_ping(&dist_mm, &width_us);
	uint16_t	result;

	if(US_NO_ECHO == status) {
		result = SL_NOT_MEASURED;
		ctx_set_no_echo(p_ctx, (uint8_t)(ctx_no_echo(p_ctx) + 1U));	/* saturates */
		if(ctx_no_echo(p_ctx) >= SL_N_NO_ECHO) {
			ctx_set_flag(p_ctx, CTX_LOST_IN_SCAN, true);
		}
	} else {
		ctx_set_no_echo(p_ctx, 0U);
		result = (US_OK == status) ? to_grid(dist_mm) : SL_FAR;
	}
	if(!scan_if_mission_running()) {
		ctx_set_flag(p_ctx, CTX_ABORT_SCAN, true);
	}
	return result;
}

LOCAL bool scan_ok(const scan_ctx_t *p_ctx)
{
	return !ctx_flag(p_ctx, CTX_ABORT_SCAN | CTX_LOST_IN_SCAN);
}

LOCAL uint16_t look(scan_ctx_t *p_ctx, INT deg)
{
	servo_move_to(deg);
	tk_dly_tsk(SERVO_SETTLE_MS);	/* >= 60 ms since the last ping too */
	return scan_ping(p_ctx);
}

/* Median of 3 pings: only at the edge angles (NFR-07). */
LOCAL uint16_t median3(scan_ctx_t *p_ctx)
{
	uint16_t ping_lo = scan_ping(p_ctx);
	uint16_t ping_mid;
	uint16_t ping_hi;
	uint16_t tmp;

	tk_dly_tsk(US_CYCLE_MS);
	ping_mid = scan_ping(p_ctx);
	tk_dly_tsk(US_CYCLE_MS);
	ping_hi = scan_ping(p_ctx);

	if(ping_lo > ping_mid) {
		tmp = ping_lo; ping_lo = ping_mid; ping_mid = tmp;
	}
	if(ping_mid > ping_hi) {
		tmp = ping_mid; ping_mid = ping_hi; ping_hi = tmp;
	}
	if(ping_lo > ping_mid) {
		tmp = ping_lo; ping_lo = ping_mid; ping_mid = tmp;
	}
	/* FAR (0) sorts first and NOT_MEASURED (0xFFFF) last, so one stray
	 * of either is ignored */
	return ping_mid;
}

LOCAL void coarse_scan(scan_ctx_t *p_ctx)
{
	INT		idx;
	INT		deg;
	uint16_t	dist_mm;

	for(idx = 0; idx < SL_SLOTS; idx++) {
		p_ctx->grid[idx] = SL_NOT_MEASURED;
	}
	ctx_set_flag(p_ctx, CTX_ABORT_SCAN | CTX_LOST_IN_SCAN, false);
	ctx_set_no_echo(p_ctx, 0U);
	p_ctx->left_hit = NO_HIT;
	p_ctx->right_hit = NO_HIT;
	p_ctx->scan_us = now_us();

	for(idx = 0; (idx < NUM_COARSE) && scan_ok(p_ctx); idx++) {
		deg = g_coarse_deg[idx];
		dist_mm = look(p_ctx, deg);
		p_ctx->grid[SL_IDX(deg)] = dist_mm;
		if(is_hit(p_ctx, dist_mm)) {
			if(NO_HIT == p_ctx->left_hit) {
				p_ctx->left_hit = deg;
			}
			p_ctx->right_hit = deg;
		}
	}
}

/* Room left in the 4.2 s scan budget for one more look, keeping time for
 * the median(s) still to come. */
LOCAL bool fits_budget(const scan_ctx_t *p_ctx, INT medians_left)
{
	return (ms_since(p_ctx->scan_us) + LOOK_EST_MS +
		((UW)medians_left * MEDIAN_EST_MS)) <= SCAN_BUDGET_MS;
}

/* Step from start_deg outward (dir = -1 left, +1 right) in 5 deg steps
 * until the first miss, the scan limit, FINE_MAX_STEPS or the budget.
 * Re-measure the last hit angle with a median of 3. */
LOCAL void fine_edge(scan_ctx_t *p_ctx, INT start_deg, INT dir, INT medians_left)
{
	INT		deg = start_deg + (dir * SL_STEP_DEG);
	INT		last_hit = start_deg;
	INT		steps = 0;
	bool		edge_found = false;
	uint16_t	dist_mm;

	while(!edge_found && scan_ok(p_ctx) && (steps < FINE_MAX_STEPS) &&
	      (deg >= -SL_LIMIT_DEG) && (deg <= SL_LIMIT_DEG) &&
	      fits_budget(p_ctx, medians_left)) {
		dist_mm = p_ctx->grid[SL_IDX(deg)];
		if(SL_NOT_MEASURED == dist_mm) {	/* else: coarse-scanned */
			dist_mm = look(p_ctx, deg);
			p_ctx->grid[SL_IDX(deg)] = dist_mm;
			steps++;
		}

		if(is_hit(p_ctx, dist_mm)) {
			last_hit = deg;
			deg += dir * SL_STEP_DEG;
		} else if(SL_NOT_MEASURED == dist_mm) {
			deg += dir * SL_STEP_DEG;	/* no echo here: keep going */
		} else {
			edge_found = true;
		}
	}

	if(scan_ok(p_ctx)) {
		servo_move_to(last_hit);
		tk_dly_tsk(SERVO_SETTLE_MS);
		p_ctx->grid[SL_IDX(last_hit)] = median3(p_ctx);
	}
}

LOCAL void fine_scan(scan_ctx_t *p_ctx)
{
	/* left edge first (2 medians still to come), then the right edge */
	fine_edge(p_ctx, p_ctx->left_hit, -1, 2);
	fine_edge(p_ctx, p_ctx->right_hit, +1, 1);
}

LOCAL void print_scan(const scan_ctx_t *p_ctx, UW scan_ms)
{
	const sl_profile_t	*p_prof = &p_ctx->prof;
	INT			idx;

	tm_printf((UB *)"[scan] ---- sweep (negative = left), obstacle <= %d mm ----\n",
		  (INT)d_obs_mm(p_ctx));
	for(idx = 0; idx < SL_SLOTS; idx++) {
		if(SL_NOT_MEASURED == p_ctx->grid[idx]) {
			/* not pinged (or no echo): nothing to print */
		} else if(SL_FAR == p_ctx->grid[idx]) {
			tm_printf((UB *)"[scan] angle %d: far\n", (INT)SL_DEG(idx));
		} else {
			tm_printf((UB *)"[scan] angle %d: %d mm%s\n", (INT)SL_DEG(idx),
				  (INT)p_ctx->grid[idx],
				  is_hit(p_ctx, p_ctx->grid[idx]) ? "  <- obstacle" : "");
		}
	}
	if(p_prof->found) {
		tm_printf((UB *)"[scan] profile: closest %d mm @ %d deg, edges %d..%d deg, "
			  "x %d..%d mm, width %d mm, %s\n",
			  (INT)p_prof->d_close_mm, (INT)p_prof->close_deg,
			  (INT)p_prof->left_deg, (INT)p_prof->right_deg,
			  (INT)p_prof->x_left_mm, (INT)p_prof->x_right_mm,
			  (INT)p_prof->width_mm,
			  p_prof->in_path ? "IN the path" : "beside the path");
		tm_printf((UB *)"[scan] clearance L %d R %d mm (need %d), offset L %d R %d mm\n",
			  (INT)p_prof->clear_left_mm, (INT)p_prof->clear_right_mm,
			  (INT)SL_C_PASS_MM,
			  (INT)p_prof->offset_left_mm, (INT)p_prof->offset_right_mm);
	}
	tm_printf((UB *)"[scan] ACTION: %s%s   (scan took %d ms, budget %d ms%s)\n",
		  sl_action_name(p_ctx->action),
		  ctx_flag(p_ctx, CTX_REVERSE_TRIED) ? " (reverse already tried)" : "",
		  (INT)scan_ms, (INT)SCAN_BUDGET_MS,
		  (scan_ms > (UW)SCAN_BUDGET_MS) ? ", OVER" : "");
}

/* Fine Scan done: profile, action, telemetry record (FR-16, FR-23).
 * The obstacle body is 13 bits today (width, bypass side); the rest of
 * the profile is on the console until Buddy 1 takes a wider body. */
LOCAL void make_profile(scan_ctx_t *p_ctx)
{
	uint32_t	bypass;
	uint32_t	width_mm;

	sl_profile(p_ctx->grid, d_obs_mm(p_ctx), &p_ctx->prof);
	p_ctx->action = sl_decide(&p_ctx->prof, ctx_flag(p_ctx, CTX_REVERSE_TRIED));
	print_scan(p_ctx, ms_since(p_ctx->scan_us));

	if(SL_BYPASS_LEFT == p_ctx->action) {
		bypass = SCAN_TELE_BYPASS_LEFT;
	} else if(SL_BYPASS_RIGHT == p_ctx->action) {
		bypass = SCAN_TELE_BYPASS_RIGHT;
	} else {
		bypass = SCAN_TELE_BYPASS_NONE;
	}
	width_mm = (0 < p_ctx->prof.width_mm) ? (uint32_t)p_ctx->prof.width_mm : 0U;
	scan_if_send_profile(width_mm, bypass);
}

/* After an aborted or failed scan: where to go. Returns true if handled. */
LOCAL bool scan_failed(scan_ctx_t *p_ctx)
{
	bool handled = true;

	if(ctx_flag(p_ctx, CTX_ABORT_SCAN)) {
		enter(p_ctx, ST_IDLE);
	} else if(ctx_flag(p_ctx, CTX_LOST_IN_SCAN)) {
		enter(p_ctx, ST_SENSOR_LOST);
	} else {
		handled = false;
	}
	return handled;
}

LOCAL void do_planning(scan_ctx_t *p_ctx)
{
	avoid_cmd_t	cmd;
	sl_profile_t	*p_prof = &p_ctx->prof;
	int32_t		offset_mm;

	switch(p_ctx->action) {
	case SL_CONTINUE:
		tm_printf((UB *)"[scan] false alarm / not in the path: continue\n");
		scan_if_stop_clear();
		ctx_set_flag(p_ctx, CTX_AFTER_REVERSE, false);
		enter(p_ctx, ST_MONITORING);
		break;

	case SL_BYPASS_LEFT:
	case SL_BYPASS_RIGHT:
		offset_mm = (SL_BYPASS_LEFT == p_ctx->action) ?
			    p_prof->offset_left_mm : p_prof->offset_right_mm;
		cmd.type = (uint8_t)AVOID_BYPASS;
		cmd.side = (SL_BYPASS_LEFT == p_ctx->action) ?
			   (uint8_t)AVOID_SIDE_LEFT : (uint8_t)AVOID_SIDE_RIGHT;
		cmd.dist_mm = (uint16_t)offset_mm;	/* <= SL_MAX_OFFSET_MM */
		scan_if_clear(SCAN_FLG_REPLIES);	/* old replies cannot end this */
		if(scan_if_send_cmd(&cmd)) {
			scan_if_stop_clear();		/* AFTER the command (Section 4) */
			ctx_set_flag(p_ctx, CTX_AFTER_REVERSE, false);
			enter(p_ctx, ST_AVOIDING);
		} else {
			tm_printf((UB *)"[scan] avoid_cmd_mbf unavailable/full: "
				  "STOP_SCAN kept, retry\n");
			(void)scan_if_wait(SCAN_FLG_TICK, TMO_FEVR);
		}
		break;

	case SL_REVERSE:
		cmd.type = (uint8_t)AVOID_REVERSE;
		cmd.side = (uint8_t)AVOID_SIDE_NONE;
		cmd.dist_mm = (uint16_t)SL_REV_DIST_MM;
		scan_if_clear(SCAN_FLG_REPLIES);
		if(scan_if_send_cmd(&cmd)) {		/* STOP_SCAN stays set */
			ctx_set_flag(p_ctx, CTX_REVERSE_TRIED, true);
			ctx_set_flag(p_ctx, CTX_AFTER_REVERSE, true);
			enter(p_ctx, ST_REVERSING);
		} else {
			tm_printf((UB *)"[scan] avoid_cmd_mbf unavailable/full: "
				  "STOP_SCAN kept, retry\n");
			(void)scan_if_wait(SCAN_FLG_TICK, TMO_FEVR);
		}
		break;

	case SL_BLOCKED:
	default:
		enter(p_ctx, ST_BLOCKED);
		break;
	}
}

LOCAL void scan_task_run(void)
{
	scan_ctx_t	ctx;
	UINT		bits;
	ER		ercd;

	servo_init();
	ercd = scan_if_init();
	if(E_OK != ercd) {
		tm_printf((UB *)"[scan] scan_flg / 60 ms tick not created (%d): stopped\n",
			  (INT)ercd);
		while(1) {
			(void)tk_slp_tsk(TMO_FEVR);	/* do not drive blind */
		}
	}
	us_init();
	us_use_flag(scan_if_flag(), SCAN_FLG_ECHO);	/* ECHO_ISR -> scan_flg */
	tm_printf((UB *)"[scan] scan_task: servo GP%d, HC-SR04 TRIG GP%d ECHO GP%d, "
		  "D_stop %d D_obs %d C_pass %d D_rev %d mm\n",
		  (INT)SERVO_PIN, (INT)US_TRIG_PIN, (INT)US_ECHO_PIN,
		  (INT)SL_D_STOP_MM, (INT)SL_D_OBS_MM, (INT)SL_C_PASS_MM,
		  (INT)SL_D_REV_MM);

	ctx.bits = 0U;				/* Idle, every flag clear */
	ctx.action = SL_CONTINUE;
	ctx.scan_us = now_us();
	enter(&ctx, ST_IDLE);			/* entry actions: centre, clear */

	while(1) {
		/* every state returns to Idle when the mission is not Running */
		if((ST_IDLE != ctx_state(&ctx)) && !scan_if_mission_running()) {
			enter(&ctx, ST_IDLE);
		}

		switch(ctx_state(&ctx)) {
		case ST_IDLE:
			(void)scan_if_wait(SCAN_FLG_TICK, TMO_FEVR);
			if(scan_if_mission_running()) {
				enter(&ctx, ST_MONITORING);
			}
			break;

		case ST_MONITORING:
			(void)scan_if_wait(SCAN_FLG_TICK, TMO_FEVR);
			(void)fwd_ping(&ctx);
			if(sl_runs_no_echo(&ctx.runs) >= SL_N_NO_ECHO) {
				enter(&ctx, ST_SENSOR_LOST);
			} else if(sl_runs_close(&ctx.runs) >= n_stop()) {
				scan_if_stop_set();		/* motion_task brakes (NFR-18) */
				enter(&ctx, ST_COARSE);
			} else {
				/* path clear: keep monitoring */
			}
			break;

		case ST_COARSE:
			coarse_scan(&ctx);
			if(scan_failed(&ctx)) {
				/* handled */
			} else if(NO_HIT == ctx.left_hit) {
				ctx.action = SL_CONTINUE;
				enter(&ctx, ST_PLANNING);	/* false alarm */
			} else {
				enter(&ctx, ST_FINE);
			}
			break;

		case ST_FINE:
			fine_scan(&ctx);
			if(!scan_failed(&ctx)) {
				make_profile(&ctx);
				enter(&ctx, ST_PLANNING);
			}
			break;

		case ST_PLANNING:
			do_planning(&ctx);
			break;

		case ST_REVERSING:
			bits = scan_if_wait(SCAN_FLG_TICK | SCAN_FLG_REVERSE_DONE, TMO_FEVR);
			if(0U != (bits & SCAN_FLG_REVERSE_DONE)) {
				enter(&ctx, ST_COARSE);		/* rescan, D_obs + reverse */
			} else if(ms_since(ctx.state_us) >= (UW)REVERSE_TIMEOUT_MS) {
				tm_printf((UB *)"[scan] no REVERSE_DONE: treat as blocked\n");
				enter(&ctx, ST_BLOCKED);
			} else {
				/* still reversing */
			}
			break;

		case ST_AVOIDING:
			bits = scan_if_wait(SCAN_FLG_TICK | SCAN_FLG_LINE_FOUND |
					    SCAN_FLG_LINE_LOST, TMO_FEVR);
			if(0U != (bits & SCAN_FLG_LINE_FOUND)) {
				ctx_set_flag(&ctx, CTX_REVERSE_TRIED, false);	/* this avoidance is over */
				enter(&ctx, ST_MONITORING);
			} else if(0U != (bits & SCAN_FLG_LINE_LOST)) {
				enter(&ctx, ST_IDLE);		/* nav_ctrl_task decides */
			} else {
				(void)fwd_ping(&ctx);
				if(sl_runs_no_echo(&ctx.runs) >= SL_N_NO_ECHO) {
					enter(&ctx, ST_SENSOR_LOST);
				} else if(sl_runs_close(&ctx.runs) >= n_stop()) {
					scan_if_stop_set();	/* re-plan */
					enter(&ctx, ST_COARSE);
				} else {
					/* bypass going on */
				}
			}
			break;

		case ST_BLOCKED:
			(void)scan_if_wait(SCAN_FLG_TICK, TMO_FEVR);
			(void)fwd_ping(&ctx);
			if(sl_runs_clear(&ctx.runs) >= SL_N_CLEAR) {
				tm_printf((UB *)"[scan] readings clear; rescan while "
					  "STOP_SCAN remains set\n");
				enter(&ctx, ST_COARSE);
			} else if(!ctx_flag(&ctx, CTX_BLOCKED_SENT) && (ms_since(ctx.state_us) >= (UW)T_BLOCK_MS)) {
				scan_if_set_blocked();
				ctx_set_flag(&ctx, CTX_BLOCKED_SENT, true);
			} else {
				/* still blocked: STOP_SCAN stays set */
			}
			break;

		case ST_SENSOR_LOST:
			(void)scan_if_wait(SCAN_FLG_TICK, TMO_FEVR);
			(void)fwd_ping(&ctx);
			if(sl_runs_echo(&ctx.runs) >= SL_N_ECHO) {
				tm_printf((UB *)"[scan] sensor answering again: path unknown, rescan\n");
				enter(&ctx, ST_COARSE);		/* not Monitoring */
			} else if(!ctx_flag(&ctx, CTX_BLOCKED_SENT) && (ms_since(ctx.state_us) >= (UW)T_BLOCK_MS)) {
				scan_if_set_blocked();
				ctx_set_flag(&ctx, CTX_BLOCKED_SENT, true);
			} else {
				/* still lost: STOP_SCAN stays set */
			}
			break;

		default:
			enter(&ctx, ST_IDLE);
			break;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Mode 3: one coarse + fine scan every 3 s (width trials, NFR-20)    */
/* ------------------------------------------------------------------ */
LOCAL void scan_test(void)
{
	scan_ctx_t ctx;

	servo_init();
	us_init();
	tm_printf((UB *)"[scan] mode 3: scan every 3 s, servo GP%d, TRIG GP%d ECHO GP%d\n",
		  (INT)SERVO_PIN, (INT)US_TRIG_PIN, (INT)US_ECHO_PIN);

	ctx.bits = 0U;				/* Idle, every flag clear */
	ctx.action = SL_CONTINUE;
	while(1) {
		coarse_scan(&ctx);
		if(ctx_flag(&ctx, CTX_LOST_IN_SCAN)) {
			tm_printf((UB *)"[scan] 3 pings with no echo: SENSOR LOST (check wiring)\n");
		} else if(NO_HIT == ctx.left_hit) {
			tm_printf((UB *)"[scan] nothing within %d mm: CONTINUE (scan took %d ms)\n",
				  (INT)SL_D_OBS_MM, (INT)ms_since(ctx.scan_us));
		} else {
			fine_scan(&ctx);
			if(ctx_flag(&ctx, CTX_LOST_IN_SCAN)) {
				tm_printf((UB *)"[scan] 3 pings with no echo: SENSOR LOST (check wiring)\n");
			} else {
				make_profile(&ctx);
			}
		}
		servo_centre();
		tk_dly_tsk(3000);			/* time to move the object */
	}
}

/* ------------------------------------------------------------------ */
/* Mode 0: ultrasonic range test (A5)                                 */
/* ------------------------------------------------------------------ */
LOCAL void range_test(void)
{
	UW round_num = 0;

	us_init();
	tm_printf((UB *)"[obstacle] HC-SR04 TRIG=GP%d ECHO=GP%d, echo by %s\n",
		  (INT)US_TRIG_PIN, (INT)US_ECHO_PIN, US_USE_IRQ ? "interrupt" : "polling");

	while(1) {
		UW	sum_mm = 0;
		UW	min_mm = 0xFFFFFFFFU;
		UW	max_mm = 0;
		UW	n_ok = 0;
		UW	n_far = 0;
		UW	n_noecho = 0;
		UW	idx;

		for(idx = 0; idx < SAMPLES_PER_ROUND; idx++) {
			UW dist_mm = 0;
			UW width_us = 0;

			switch(us_ping(&dist_mm, &width_us)) {
			case US_OK:
				n_ok++;
				sum_mm += dist_mm;
				if(dist_mm < min_mm) {
					min_mm = dist_mm;
				}
				if(dist_mm > max_mm) {
					max_mm = dist_mm;
				}
				break;
			case US_OUT_OF_RANGE:
				n_far++;
				break;
			case US_NO_ECHO:
			default:
				n_noecho++;
				break;
			}
			tk_dly_tsk(US_CYCLE_MS);
		}

		round_num++;
		if(n_ok > 0U) {
			tm_printf((UB *)"[obstacle] round %d: mean %d mm  min %d  max %d  "
				  "spread %d mm  (ok %d, out-of-range %d, no-echo %d)\n",
				  (INT)round_num, (INT)(sum_mm / n_ok), (INT)min_mm,
				  (INT)max_mm, (INT)(max_mm - min_mm),
				  (INT)n_ok, (INT)n_far, (INT)n_noecho);
		} else {
			tm_printf((UB *)"[obstacle] round %d: no valid readings  "
				  "(out-of-range %d, no-echo %d)\n",
				  (INT)round_num, (INT)n_far, (INT)n_noecho);
		}
	}
}


/* ------------------------------------------------------------------ */
EXPORT void obstacle_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	switch(OBSTACLE_TEST_MODE) {		/* each one loops forever */
	case 1:
		servo_test();
		break;
	case 2:
		servo_calibrate();
		break;
	case 3:
		scan_test();
		break;
	case 4:
		scan_task_run();
		break;
	default:
		range_test();
		break;
	}
}
