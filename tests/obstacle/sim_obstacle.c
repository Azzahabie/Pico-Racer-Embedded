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
 * now drives (S.byp). */

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
	if ((g_servo_deg >= -SL_LIMIT_DEG) && (g_servo_deg <= SL_LIMIT_DEG)) {
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
}
void scan_if_send_profile(uint32_t width_mm, uint32_t bypass)
{
	g_tele_width[g_tele_n % 16] = width_mm;
	g_tele_bypass[g_tele_n % 16] = bypass;
	g_tele_n++;
}
void scan_if_sensor_lost(bool lost) { (void)lost; }
bool scan_if_mission_running(void) { return true; }
bool scan_if_hump_active(void) { return false; }
bool scan_if_standalone(void) { return false; }

/* scan_task's waits: one 60 ms TICK each, or a bit motion_task set. */
UINT scan_if_wait(UINT bits, TMO tmout)
{
	UINT seen;

	(void)tmout;
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
	g_ds_word = 0U;
	g_ds_step = 0U;
	g_scan_flg = 0U;
	g_stop = false;
	g_blocked_n = 0;
	g_blocked_at_us = 0U;
	g_cmd_n = 0;
	g_cmd_full = 0;
	g_tele_n = 0;
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

int main(void)
{
	switch_side_reports();
	printf("scan_task: %d checks, %d failures\n", g_checks, g_failures);
	return (0 == g_failures) ? 0 : 1;
}
