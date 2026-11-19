/* Host test for scan_task (app_program/obstacle_task/obstacle_task.c,
 * Statechart 5): the real obstacle_task.c and scan_logic.c on the motion
 * sim's kernel stubs, with fakes for scan_if (motion, nav, telemetry),
 * the HC-SR04 and the servo.  make -C tests/obstacle
 *
 * The world: a fake clock (every tk_dly_tsk and every 60 ms TICK move
 * it), the angle the servo was last sent to, and what the sensor reads at
 * each angle (g_world_mm, the same 5 deg slots as the scan grid).
 *
 * What it checks: switch_side() sends a new E obstacle with the side it
 * now drives (S.byp); t_block runs over Blocked and its rescans, so a box
 * that reads D_stop or more at rest still pauses the mission; a re-plan
 * during a bypass that finds the way clear goes back to Avoiding (motion
 * resumes the bypass), so its looks are answered.
 * The whole Statechart runs in scan_task_run(); every scan_if_wait()
 * calls the scenario's hook, and the run ends at a fake time. */

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/obstacle_task/obstacle_task.c"

/* ---- checks ---------------------------------------------------------- */
static int g_checks;
static int g_failures;
#define CHECK(c)                                                              \
	do {                                                                  \
		g_checks++;                                                   \
		if (!(c)) {                                                   \
			g_failures++;                                         \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);   \
		}                                                             \
	} while (0)

/* ---- fakes: clock, console ------------------------------------------- */
static UW g_now_us;			/* TIMER_TIMERAWL */
UW in_w(UW addr) { (void)addr; return g_now_us; }
ER tk_dly_tsk(TMO tmo) { g_now_us += (UW)tmo * 1000U; return E_OK; }
ER tk_slp_tsk(TMO tmo) { (void)tmo; return E_OK; }

static char g_log[65536];
static void log_add(const char *p_text)
{
	size_t len = strlen(g_log);

	(void)snprintf(g_log + len, sizeof(g_log) - len, "%s", p_text);
}
INT tm_printf(const UB *format, ...)
{
	char	line[256];
	va_list	ap;

	va_start(ap, format);
	(void)vsnprintf(line, sizeof(line), (const char *)format, ap);
	va_end(ap);
	log_add(line);
	return 0;
}
INT tm_sprintf(UB *str, const UB *format, ...)
{
	va_list	ap;
	int	len;

	va_start(ap, format);
	len = vsprintf((char *)str, (const char *)format, ap);
	va_end(ap);
	return len;
}

/* ---- fakes: the world (servo, HC-SR04, motion's distance) ------------ */
static INT      g_servo_deg;
static uint16_t g_world_mm[SL_SLOTS];	/* mm; SL_FAR = out of range */
static int      g_close_pings;		/* next pings ahead read 180 mm */
static uint32_t g_ds_word;		/* motion_ds: dist_mm field only */
static uint32_t g_ds_step;		/* mm added per read (car moving) */

void servo_init(void) { g_servo_deg = 0; }
UW   servo_set_angle(INT angle_deg) { g_servo_deg = angle_deg; return 1500U; }
UW   servo_move_to(INT angle_deg) { g_servo_deg = angle_deg; return 1500U; }
void us_init(void) { }
void us_use_flag(ID flgid, UINT echo_bit) { (void)flgid; (void)echo_bit; }
us_status_t us_ping(UW *p_mm, UW *p_width_us)
{
	uint16_t mm = SL_FAR;

	*p_width_us = 0U;
	if ((0 < g_close_pings) && (0 == g_servo_deg)) {
		g_close_pings--;
		mm = 180U;			/* something close, then gone */
	} else if ((g_servo_deg >= -SL_LIMIT_DEG) && (g_servo_deg <= SL_LIMIT_DEG)) {
		mm = g_world_mm[SL_IDX(g_servo_deg)];
	}
	*p_mm = mm;
	return (SL_FAR == mm) ? US_OUT_OF_RANGE : US_OK;
}
uint32_t motion_ds_read(void)
{
	g_ds_word = (g_ds_word + g_ds_step) & MOTION_DS_MM_MASK;
	return g_ds_word;
}

static void world_all(uint16_t mm)
{
	int idx;

	for (idx = 0; idx < SL_SLOTS; idx++) {
		g_world_mm[idx] = mm;
	}
}

/* ---- fakes: scan_if (motion_task, nav_ctrl_task, comms_task) --------- */
static UINT	g_scan_flg;		/* bits motion_task has set */
static bool	g_stop;			/* STOP_SCAN */
static bool	g_running;		/* mission Running (nav_ctrl_task) */
static int	g_blocked_n;		/* BLOCKED raised to nav */
static UW	g_blocked_at_us;	/* ... first time */
static avoid_cmd_t g_cmds[32];		/* commands placed */
static int	g_cmd_n;
static int	g_cmd_full;		/* avoid_cmd_mbf full */
static uint32_t	g_tele_width[16];	/* E obstacle records */
static uint32_t	g_tele_bypass[16];
static int	g_tele_n;

void scan_if_bind(const scan_if_bind_t *p_bind) { (void)p_bind; }
ER   scan_if_init(void) { return E_OK; }
ID   scan_if_flag(void) { return 1; }
void scan_if_clear(UINT bits) { g_scan_flg &= ~bits; }
void scan_if_stop_set(void) { g_stop = true; }
void scan_if_stop_clear(void) { g_stop = false; }
bool scan_if_send_cmd(const avoid_cmd_t *p_cmd)
{
	if (0 != g_cmd_full) {
		return false;
	}
	g_cmds[g_cmd_n % 32] = *p_cmd;
	g_cmd_n++;
	return true;
}
void scan_if_set_blocked(void)
{
	if (0 == g_blocked_n) {
		g_blocked_at_us = g_now_us;
	}
	g_blocked_n++;
	g_running = false;			/* nav_ctrl_task pauses the mission */
}
void scan_if_send_profile(uint32_t width_mm, uint32_t bypass)
{
	g_tele_width[g_tele_n % 16] = width_mm;
	g_tele_bypass[g_tele_n % 16] = bypass;
	g_tele_n++;
}
void scan_if_sensor_lost(bool lost) { (void)lost; }
bool scan_if_mission_running(void) { return g_running; }
bool scan_if_hump_active(void) { return false; }
bool scan_if_standalone(void) { return false; }

/* scan_task_run() is endless: it runs until the fake clock reaches
   g_end_us, calling the scenario's hook on every wait. */
static jmp_buf	g_task_out;
static int	g_in_task;		/* inside run_task() */
static UW	g_end_us;
static void	(*g_hook)(void);

/* scan_task's waits: one 60 ms TICK each, or a bit motion_task set. */
UINT scan_if_wait(UINT bits, TMO tmout)
{
	UINT seen;

	(void)tmout;
	if (NULL != g_hook) {
		g_hook();
	}
	if ((0 != g_in_task) && (g_now_us >= g_end_us)) {
		longjmp(g_task_out, 1);
	}
	seen = g_scan_flg & bits;
	if (0U == seen) {
		g_now_us += (UW)SCAN_TICK_MS * 1000U;
		seen = bits & SCAN_FLG_TICK;
	}
	g_scan_flg &= ~seen;
	return seen;
}

/* ---- helpers --------------------------------------------------------- */
static void reset(void)
{
	g_now_us = 1000000U;
	g_log[0] = '\0';
	g_servo_deg = 0;
	world_all(SL_FAR);
	g_close_pings = 0;
	g_ds_word = 0U;
	g_ds_step = 0U;
	g_scan_flg = 0U;
	g_stop = false;
	g_running = true;
	g_blocked_n = 0;
	g_blocked_at_us = 0U;
	g_cmd_n = 0;
	g_cmd_full = 0;
	g_tele_n = 0;
	g_hook = NULL;
}

/* Run the real scan_task for run_ms of fake time. */
static void run_task(UW run_ms)
{
	g_end_us = g_now_us + run_ms * 1000U;
	g_in_task = 1;
	if (0 == setjmp(g_task_out)) {
		scan_task_run();
	}
	g_in_task = 0;
}

/* times text appears in the log */
static int log_count(const char *p_text)
{
	const char *p_at = g_log;
	int         count = 0;

	while (NULL != (p_at = strstr(p_at, p_text))) {
		count++;
		p_at++;
	}
	return count;
}

/* A context in Avoiding after a bypass planned from this sweep: the 90 mm
   box on the car, trials_batch2 04:36:17 (both sides open). */
static void ctx_avoiding(scan_ctx_t *p_ctx, sl_action_t action)
{
	memset(p_ctx, 0, sizeof(*p_ctx));
	memset(p_ctx->grid, 0xFF, sizeof(p_ctx->grid));	/* not measured */
	p_ctx->grid[SL_IDX(-45)] = 1332; p_ctx->grid[SL_IDX(-40)] = 1342;
	p_ctx->grid[SL_IDX(-35)] = 275;  p_ctx->grid[SL_IDX(-25)] = 252;
	p_ctx->grid[SL_IDX(0)]   = 253;  p_ctx->grid[SL_IDX(10)]  = 261;
	p_ctx->grid[SL_IDX(15)]  = SL_FAR; p_ctx->grid[SL_IDX(20)] = 1540;
	p_ctx->grid[SL_IDX(25)]  = 1451; p_ctx->grid[SL_IDX(45)]  = 1138;
	sl_profile(p_ctx->grid, SL_D_OBS_MM, &p_ctx->prof);
	p_ctx->action = (uint8_t)action;
	ctx_set_state(p_ctx, ST_AVOIDING);
}

/* ---- scenarios ------------------------------------------------------- */

/* O7: the lane of the side planned is blocked and switch_side() takes the
   other one: a new E obstacle says so (S.byp showed the abandoned side). */
static void switch_side_reports(void)
{
	scan_ctx_t ctx;

	reset();
	ctx_avoiding(&ctx, SL_CONTINUE);
	make_profile(&ctx);			/* the scan's record, as before */
	CHECK(SL_BYPASS_RIGHT == ctx.action);
	CHECK((1 == g_tele_n) && (SCAN_TELE_BYPASS_RIGHT == g_tele_bypass[0]) &&
	      ((uint32_t)ctx.prof.width_mm == g_tele_width[0]));
	switch_side(&ctx);			/* the right lane was blocked */
	CHECK(ST_AVOIDING == ctx_state(&ctx));
	CHECK((1 == g_cmd_n) && (AVOID_BYPASS == g_cmds[0].type) &&
	      (AVOID_SIDE_LEFT == g_cmds[0].side) &&
	      (ctx.prof.offset_left_mm == (int16_t)g_cmds[0].dist_mm));
	CHECK(2 == g_tele_n);
	CHECK(SCAN_TELE_BYPASS_LEFT == g_tele_bypass[1]);
	CHECK((uint32_t)ctx.prof.width_mm == g_tele_width[1]);
	CHECK(!g_stop);				/* cleared after the command */
	CHECK(NULL != strstr(g_log, "BYPASS RIGHT lane blocked: trying BYPASS LEFT"));

	/* left first, then right */
	reset();
	ctx_avoiding(&ctx, SL_BYPASS_LEFT);
	switch_side(&ctx);
	CHECK((1 == g_tele_n) && (SCAN_TELE_BYPASS_RIGHT == g_tele_bypass[0]));

	/* the other side was tried already: Blocked, no new record */
	reset();
	ctx_avoiding(&ctx, SL_BYPASS_RIGHT);
	ctx_set_flag(&ctx, CTX_SIDE_SWITCHED, true);
	switch_side(&ctx);
	CHECK(ST_BLOCKED == ctx_state(&ctx));
	CHECK((0 == g_cmd_n) && (0 == g_tele_n) && g_stop);

	/* the command could not be placed: no record of a side not taken */
	reset();
	ctx_avoiding(&ctx, SL_BYPASS_RIGHT);
	g_cmd_full = 1;
	switch_side(&ctx);
	CHECK((0 == g_tele_n) && g_stop);
}

/* O5 world: the car stops at 240 mm from a wall it cannot pass, then at
   rest the forward ping reads 255 mm (>= D_stop 250: "clear"), as on the
   car (rest 223-252 mm, trials_batch2:873-876: Blocked left after 290 ms).
   Later the wall goes away (CONTINUE) and comes back. */
static UW  g_first_blocked_us;		/* first "-> BLOCKED" of the run */
static UW  g_second_blocked_us;		/* after the wall came back */
static int g_phase;
static void hook_blocked_at_rest(void)
{
	if ((0 == g_phase) && (1 <= log_count("-> BLOCKED"))) {
		g_first_blocked_us = g_now_us;
		world_all(255);			/* at rest: reads D_stop or more */
		g_phase = 1;
	}
}

/* t_block from the first Blocked, over the rescans: BLOCKED to nav. */
static void blocked_at_rest_pauses(void)
{
	/* the box stays close: Blocked never rescans (worked before) */
	reset();
	world_all(240);
	g_phase = 0;
	run_task(30000U);
	CHECK(1 == log_count("-> BLOCKED"));
	CHECK(1 == g_blocked_n);

	/* the box reads 255 mm at rest: rescans every few seconds */
	reset();
	world_all(240);				/* stops, scans: no side passable */
	g_phase = 0;
	g_hook = hook_blocked_at_rest;
	run_task(30000U);
	CHECK(3 <= log_count("readings clear; rescan while STOP_SCAN remains set"));
	CHECK(NULL != strstr(g_log, "[scan] still blocked: "));
	CHECK(1 <= g_blocked_n);		/* was never raised */
	CHECK((g_blocked_at_us - g_first_blocked_us) >= (UW)T_BLOCK_MS * 1000U);
	CHECK((g_blocked_at_us - g_first_blocked_us) < (UW)(T_BLOCK_MS + 4000) * 1000U);
	CHECK(1 == g_blocked_n);		/* the mission paused: Idle */
}

/* After a rescan finds the way clear the car drives on: a later Blocked
   counts its own t_block. */
static int g_blocked_seen;		/* "-> BLOCKED" count when the wall came back */
static void hook_blocked_again(void)
{
	if ((0 == g_phase) && (1 <= log_count("-> BLOCKED"))) {
		g_first_blocked_us = g_now_us;
		world_all(255);			/* at rest: reads D_stop or more */
		g_phase = 1;
	} else if ((1 == g_phase) && ((g_now_us - g_first_blocked_us) >= 6000000U)) {
		world_all(SL_FAR);		/* the wall is taken away */
		g_phase = 2;
	} else if ((2 == g_phase) && (1 <= log_count("PLANNING -> MONITORING"))) {
		world_all(240);			/* and put back */
		g_blocked_seen = log_count("-> BLOCKED");
		g_phase = 3;
	} else if ((3 == g_phase) && (log_count("-> BLOCKED") > g_blocked_seen)) {
		g_second_blocked_us = g_now_us;
		world_all(255);
		g_phase = 4;
	} else {
		/* nothing to change */
	}
}

static void blocked_run_restarts(void)
{
	reset();
	world_all(240);
	g_phase = 0;
	g_second_blocked_us = 0U;
	g_hook = hook_blocked_again;
	run_task(40000U);
	CHECK(1 == log_count("PLANNING -> MONITORING"));	/* the way was clear */
	CHECK(0U != g_second_blocked_us);
	CHECK(1 == g_blocked_n);
	/* from the second Blocked, not the first */
	CHECK((g_blocked_at_us - g_second_blocked_us) >= (UW)T_BLOCK_MS * 1000U);
	CHECK((g_blocked_at_us - g_second_blocked_us) < (UW)(T_BLOCK_MS + 4000) * 1000U);
}

/* O3 world: a 90 mm-ish box 200 mm ahead (hits -10..+10 deg): stop,
   scan, BYPASS.  On the bypass leg two pings read 180 mm and then the
   way is clear (the re-plan's scan sees nothing: CONTINUE).  Then motion
   asks for a look, and finds the line. */
static int g_look_n;			/* AVOID_LOOK commands seen */
static void hook_replan_clear(void)
{
	int idx;

	g_look_n = 0;
	for (idx = 0; (idx < g_cmd_n) && (idx < 32); idx++) {
		if (AVOID_LOOK == g_cmds[idx].type) {
			g_look_n++;
		}
	}
	if ((0 == g_phase) && (1 <= log_count("PLANNING -> AVOIDING"))) {
		world_all(SL_FAR);		/* the box is beside the car now */
		g_ds_step = 5U;			/* a straight leg: D_stop applies */
		g_close_pings = 2;		/* something on the leg, then gone */
		g_phase = 1;
	} else if ((1 == g_phase) && (2 <= log_count("-> AVOIDING"))) {
		g_ds_step = 0U;			/* a probe pivot */
		g_scan_flg |= SCAN_FLG_LOOK;	/* motion asks to look */
		g_phase = 2;
	} else if ((2 == g_phase) && (1 <= g_look_n)) {
		g_scan_flg |= SCAN_FLG_LINE_FOUND;	/* back on the line */
		g_phase = 3;
	} else if ((3 == g_phase) && (1 <= log_count("AVOIDING -> MONITORING"))) {
		g_close_pings = 2;		/* a false alarm later on */
		g_phase = 4;
	} else {
		/* nothing to change */
	}
}

static void replan_clear_resumes_bypass(void)
{
	int idx;

	reset();
	for (idx = SL_IDX(-10); idx <= SL_IDX(10); idx++) {
		g_world_mm[idx] = 200U;
	}
	g_phase = 0;
	g_hook = hook_replan_clear;
	run_task(20000U);
	CHECK(4 == g_phase);
	CHECK(AVOID_BYPASS == g_cmds[0].type);
	CHECK(1 == log_count("AVOIDING -> COARSE_SCAN"));	/* the re-plan */
	CHECK(1 == log_count("[scan] way clear: the bypass carries on"));
	CHECK(log_count("PLANNING -> AVOIDING") == 2);	/* not Monitoring */
	CHECK(1 == g_look_n);				/* the look is answered */
	CHECK(NULL != strstr(g_log, "[scan] side look"));
	CHECK(1 == log_count("AVOIDING -> MONITORING"));	/* LINE_FOUND ends it */
	/* the later false alarm, outside any avoidance: Monitoring as before */
	CHECK(1 == log_count("[scan] false alarm / not in the path: continue"));
	CHECK(1 == log_count("PLANNING -> MONITORING"));
	CHECK(!g_stop);
}

/* A false alarm with no bypass under way goes back to Monitoring. */
static void false_alarm_monitors(void)
{
	reset();
	g_close_pings = 2;
	run_task(5000U);
	CHECK(1 == log_count("[scan] false alarm / not in the path: continue"));
	CHECK(1 == log_count("PLANNING -> MONITORING"));
	CHECK(0 == log_count("way clear"));
	CHECK((0 == g_cmd_n) && !g_stop);
}

int main(void)
{
	switch_side_reports();
	blocked_at_rest_pauses();
	blocked_run_restarts();
	replan_clear_resumes_bypass();
	false_alarm_monitors();
	printf("scan_task: %d checks, %d failures\n", g_checks, g_failures);
	return (0 == g_failures) ? 0 : 1;
}
