/*
 *	sim_motion.c
 *	Runs the real motor_task.c on a PC against a simulated car: two
 *	motors with a dead band and lag, two encoders, the button, the 1 MHz
 *	timer and the kernel calls motion_task makes.  Checks the bench test
 *	end to end before anything is flashed:
 *
 *	    make -C tests/motion_task_sim          (V=1 prints the console)
 *
 *	motor.c, encoder.c, motion_wdt.c and gpio_irq.c are replaced by the
 *	fakes below; motion_lib.c is the real one.
 */

#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* The unit under test, statics included. */
#include "../../app_program/motor_task/motor_task.c"
_Static_assert((__alignof__(g_motion_stack) >= 8) && (__alignof__(g_report_stack) >= 8),
	       "task stacks must be 8-byte aligned (AAPCS)");

/* ------------------------------------------------------------------ */

static int g_checks;
static int g_failures;
static int g_verbose;

#define CHECK(cond)							\
	do {								\
		g_checks++;						\
		if (!(cond)) {						\
			g_failures++;					\
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		}							\
	} while (0)

/* ------------------------------------------------------------------ *
 *  Simulated car
 * ------------------------------------------------------------------ */

#define SIM_DEADBAND	(120)		/* per-mille below which nothing turns */
#define SIM_GAIN	(0.15)		/* counts per tick per per-mille       */
#define SIM_LAG		(0.3)

typedef struct
{
	double		speed;		/* counts per tick, + = car forward    */
	double		pos;		/* physical position in counts         */
	int		b_motor_inv;	/* + duty turns this wheel backward    */
	int		b_enc_inv;	/* encoder counts down going forward   */
	int		b_enc_dead;	/* whole encoder dead: no edges at all */
	int		dead_phase;	/* 1 = A wire, 2 = B wire unplugged    */
	int		b_blocked;
	long		enc_pos;	/* position the encoder has followed   */
	uint8_t		ab;		/* (A << 1) | B seen on the pins       */
	motion_quad_t	quad;		/* the real decoder, as in encoder.c   */
	uint16_t	edges_a;	/* edges seen on each phase            */
	uint16_t	edges_b;
	uint16_t	count;		/* what its encoder reports            */
} sim_wheel_t;

static sim_wheel_t	g_wheel[2];
static int32_t		g_duty[2];
static int32_t		g_duty_max_abs;
static int		g_enc_swap;	/* left encoder plugged into right port */
static uint32_t		g_time_us;
static uint32_t		g_gpio_in;
static int		g_gpio_in_reads;	/* times firmware read GPIO_IN   */
static int		g_gp20_setups;	/* times firmware configured GP20  */
static int		g_fail_ticks;	/* tk_wai_flg times out this many times */

/* The Robo Pico user button is on GP20 whether or not motion uses it. */
#define SIM_PIN_GP20	(20u)
#if MOTION_BENCH
_Static_assert(MOTION_PIN_BUTTON == SIM_PIN_GP20, "bench button is GP20");
#endif
static UINT		g_flag;

/* Pin levels for an encoder position: forward is 00 10 11 01 (A leads),
   as in motion_lib.c.  An unplugged wire reads high (pull-up). */
static uint8_t
sim_ab (long c, int dead_phase)
{
	unsigned m = (unsigned)(c & 3L);
	uint32_t a = ((m == 1u) || (m == 2u)) ? 1u : 0u;
	uint32_t b = ((m == 2u) || (m == 3u)) ? 1u : 0u;

	if (dead_phase == 1) { a = 1u; }
	if (dead_phase == 2) { b = 1u; }
	return motion_quad_ab(a, b, false);
}

static void
plant_step (void)
{
	int w;

	for (w = 0; w < 2; w++) {
		sim_wheel_t *p = &g_wheel[w];
		double d = (double)(p->b_motor_inv ? -g_duty[w] : g_duty[w]);
		double target = 0.0;

		if (fabs(d) >= SIM_DEADBAND) {
			target = (d - ((d > 0) ? SIM_DEADBAND : -SIM_DEADBAND)) * SIM_GAIN;
		}
		if (p->b_blocked) {
			target = 0.0;
			p->speed = 0.0;
		}
		p->speed += (target - p->speed) * SIM_LAG;
		p->pos   += p->speed;
	}

	for (w = 0; w < 2; w++) {
		sim_wheel_t *p_src = &g_wheel[g_enc_swap ? (1 - w) : w];
		sim_wheel_t *p_enc = &g_wheel[w];
		long c = lround(p_src->pos) * (p_enc->b_enc_inv ? -1 : 1);

		/* Walk the quadrature signals one count at a time, as the
		   real pins would change, through the real decoder. */
		while (!p_enc->b_enc_dead && (p_enc->enc_pos != c)) {
			uint8_t ab;
			p_enc->enc_pos += (c > p_enc->enc_pos) ? 1 : -1;
			ab = sim_ab(p_enc->enc_pos, p_enc->dead_phase);
			if (ab != p_enc->ab) {
				if (((ab ^ p_enc->ab) & 2u) != 0u) { p_enc->edges_a++; }
				if (((ab ^ p_enc->ab) & 1u) != 0u) { p_enc->edges_b++; }
				p_enc->ab = ab;
				(void)motion_quad_step(&p_enc->quad, ab);
			}
		}
		p_enc->count = p_enc->quad.count;
	}
}

/* Unplug one encoder signal wire: the pin's pull-up takes it high, which
   is one more edge (and one decoder step) if the pin was low. */
static void
sim_unplug_phase (int w, int phase)
{
	sim_wheel_t *p = &g_wheel[w];
	uint8_t ab = sim_ab(p->enc_pos, phase);

	p->dead_phase = phase;
	if (ab != p->ab) {
		if (((ab ^ p->ab) & 2u) != 0u) { p->edges_a++; }
		if (((ab ^ p->ab) & 1u) != 0u) { p->edges_b++; }
		p->ab = ab;
		(void)motion_quad_step(&p->quad, ab);
		p->count = p->quad.count;
	}
}

/* ------------------------------------------------------------------ *
 *  Fakes: motor.h, encoder.h, gpio_irq.h (Buddy 5's dispatcher)
 * ------------------------------------------------------------------ */

static int		g_fail_init;	/* motor_init reports a failure */

ER motor_init(void) { g_duty[0] = g_duty[1] = 0; return g_fail_init ? E_OBJ : E_OK; }

void motor_set(motor_side_t side, int32_t duty_pm)
{
	if ((duty_pm != 0) && ((g_flag & MOTION_EVT_STOPS) != 0u)) {
		CHECK(!"a wheel driven while a STOP bit is set");
	}
	g_duty[side] = motion_clamp(duty_pm, -MOTION_DUTY_FULL, MOTION_DUTY_FULL);
	if (motion_abs(g_duty[side]) > g_duty_max_abs) {
		g_duty_max_abs = motion_abs(g_duty[side]);
	}
}

static int		g_brake_calls;	/* motor_brake_all() calls */
static int		g_probe;	/* time the brake and the first report */
static uint32_t		g_brake_t_us;
static uint32_t		g_rep_t_us;

void motor_brake_all(void)
{
	g_duty[0] = g_duty[1] = 0;
	g_brake_calls++;
	if (g_probe) { g_brake_t_us = g_time_us; g_time_us += 3u; }	/* 3 us to brake */
}
int32_t motor_get_duty(motor_side_t side) { return g_duty[side]; }

ER encoder_init(void) { return E_OK; }
void encoder_read_counts(uint16_t *p_left, uint16_t *p_right)
{
	*p_left = g_wheel[0].count;
	*p_right = g_wheel[1].count;
}
uint32_t encoder_invalid_packed(void)
{
	return motion_pack_counts(g_wheel[0].quad.invalid, g_wheel[1].quad.invalid);
}
bool encoder_left_enabled(void) { return true; }
uint32_t encoder_read_edges_left(void)
{
	return motion_pack_counts(g_wheel[0].edges_a, g_wheel[0].edges_b);
}
uint32_t encoder_read_edges_right(void)
{
	return motion_pack_counts(g_wheel[1].edges_a, g_wheel[1].edges_b);
}

ER gpio_irq_init(void) { return E_OK; }

/* motion_wdt.h: counts what motion_task does with the watchdog. */
static motion_boot_t	g_boot;		/* cause the next start-up reads */
static int		g_wdt_starts;
static int		g_wdt_feeds;
static int		g_wdt_feeds_at_start;	/* armed before any feed? */

motion_boot_t motion_wdt_boot_cause(void) { return g_boot; }
void motion_wdt_start(void) { g_wdt_starts++; g_wdt_feeds_at_start = g_wdt_feeds; }
void motion_wdt_feed(void) { g_wdt_feeds++; }

#if MOTION_BENCH
static int		g_hang_marker;	/* SCRATCH0 holds the test record */
static uint32_t		g_hang_us;
static jmp_buf		g_hang_jmp;
static uint32_t		g_hang_fed_us;

/* The real one never returns: jump back to the scenario. */
void motion_wdt_hang(uint32_t fed_us) { g_hang_fed_us = fed_us; longjmp(g_hang_jmp, 1); }
bool motion_wdt_take_hang_us(uint32_t *p_us)
{
	int b_found = g_hang_marker;

	if (b_found) { *p_us = g_hang_us; g_hang_marker = 0; }
	return b_found != 0;
}
#endif
uint32_t encoder_irq_count(void) { return 0u; }
uint32_t encoder_take_irq_max_us(void) { return 3u; }

/* ------------------------------------------------------------------ *
 *  Fakes: registers, kernel, monitor
 * ------------------------------------------------------------------ */

UW in_w(UW addr)
{
	UW v = 0u;
	if (addr == TIMER_RAWL) { v = g_time_us; }
	else if (addr == GPIO_IN) { g_gpio_in_reads++; v = g_gpio_in; }
	return v;
}
void out_w(UW a, UW d) { (void)a; (void)d; }
void set_w(UW a, UW d) { (void)a; (void)d; }
void clr_w(UW a, UW d) { (void)a; (void)d; }
ER gpio_set_pin(UINT no, UINT mode)
{
	(void)mode;
	if (no == SIM_PIN_GP20) { g_gp20_setups++; }
	return E_OK;
}

#define SIM_QUEUE_LEN	((int)REPORT_QUEUE_LEN)

static motion_report_t	g_queue[SIM_QUEUE_LEN];
static int		g_q_count;
static int		g_hold_reports;

ID tk_cre_flg(CONST T_CFLG *p) { (void)p; return 1; }
ER tk_set_flg(ID id, UINT ptn) { (void)id; g_flag |= ptn; return E_OK; }
ER tk_clr_flg(ID id, UINT ptn) { (void)id; g_flag &= ptn; return E_OK; }
/* As the kernel does it (eventflag.c): wake if any waited bit is set,
   return the whole pattern from before the clear, and with TWF_BITCLR
   clear every waited bit. */
static UINT		g_last_waiptn;
static UINT		g_last_wmode;

static jmp_buf		g_loop_jmp;	/* leaves motor_task()'s for (;;) */
static int		g_loop_wakes = -1;	/* waits allowed before the jump */

ER tk_wai_flg(ID id, UINT waiptn, UINT mode, UINT *p_ptn, TMO tmo)
{
	(void)id; (void)tmo;
	if (g_loop_wakes == 0) { longjmp(g_loop_jmp, 1); }
	if (g_loop_wakes > 0) { g_loop_wakes--; }
	g_last_waiptn = waiptn;
	g_last_wmode = mode;
	if (g_fail_ticks > 0) { g_fail_ticks--; return E_TMOUT; }
	if ((g_flag & waiptn) == 0u) {
		CHECK(!"motion_cycle called with nothing to wake it (it would block)");
		return E_TMOUT;
	}
	*p_ptn = g_flag;
	if ((mode & TWF_BITCLR) != 0u) { g_flag &= ~waiptn; }
	return E_OK;
}
ID tk_cre_mbf(CONST T_CMBF *p)
{
	CHECK(((p->mbfatr & TA_USERBUF) != 0u) && (p->bufptr != NULL));
	if (p->maxmsz == (INT)sizeof(motion_report_t)) {
		/* Report queue: Week 13 RAM evidence. */
		CHECK(p->bufsz == (SZ)(REPORT_QUEUE_LEN * (sizeof(motion_report_t) + 4u)));
		return 2;
	}
	if (p->maxmsz == (INT)sizeof(uint16_t)) {
		CHECK(p->bufsz == (SZ)(AVOID_CMD_QUEUE_LEN * (sizeof(uint16_t) + 4u)));
		return 6;
	}
	CHECK(!"unexpected message buffer definition");
	return E_PAR;
}
ER tk_snd_mbf(ID id, void *msg, INT size, TMO tmo)
{
	(void)id;
	CHECK(tmo == TMO_POL);			/* motion_task never blocks */
	CHECK(size == (INT)sizeof(motion_report_t));
	if (g_probe && (g_rep_t_us == 0u)) { g_rep_t_us = g_time_us; g_time_us += 5u; }
	if (g_q_count >= SIM_QUEUE_LEN) { return E_TMOUT; }
	memcpy(&g_queue[g_q_count++], msg, sizeof(motion_report_t));
	return E_OK;
}
INT tk_rcv_mbf(ID id, void *msg, TMO tmo) { (void)id; (void)msg; (void)tmo; return E_TMOUT; }
ID tk_cre_tsk(CONST T_CTSK *p)
{
	CHECK((p->tskatr & TA_USERBUF) != 0u);	/* stack is ours, not the heap */
	CHECK((p->bufptr != NULL) && (((uintptr_t)p->bufptr % 8u) == 0u));
	CHECK((p->stksz % 8) == 0);
	if (p->task == (FP)motor_task) {
		CHECK(p->itskpri == 11);
		CHECK((p->tskatr & TA_ASSPRC) != 0u);
		CHECK(p->assprc == TP_PRC2);	/* core 1 */
#if MOTION_BENCH && MOTION_TEST_STOP
	} else if (p->task == (FP)stop_standin_task) {
		CHECK(p->itskpri == 12);	/* imu_task's, below motion */
		CHECK(p->assprc == TP_PRC2);	/* core 1, like scan and imu */
#endif
	} else {
		CHECK(p->assprc == TP_PRC1);	/* reporter on core 0 */
	}
	return 3;
}
ER tk_sta_tsk(ID id, INT stacd) { (void)id; (void)stacd; return E_OK; }
ID tk_cre_cyc(CONST T_CCYC *p) { CHECK(p->cyctim == 10u); return 4; }

/* distance_state mutex (MOTION_PACKED 0 only).  g_mtx_held = 1 plays a
   reader that holds it. */
#define SIM_MTXID	(5)
static int		g_mtx_held;
static int		g_mtx_pol_locks;	/* writer: never waits */
static int		g_mtx_wait_locks;	/* reader: waits */

ID tk_cre_mtx(CONST T_CMTX *p) { CHECK(p->mtxatr == TA_INHERIT); return SIM_MTXID; }
ER tk_loc_mtx(ID id, TMO tmo)
{
	CHECK(id == SIM_MTXID);
	if (tmo == TMO_POL) { g_mtx_pol_locks++; }
	else { g_mtx_wait_locks++; }
	if (g_mtx_held) {
		CHECK(tmo == TMO_POL);		/* a wait here would never end */
		return E_TMOUT;
	}
	g_mtx_held = 1;
	return E_OK;
}
ER tk_unl_mtx(ID id) { CHECK(id == SIM_MTXID); CHECK(g_mtx_held); g_mtx_held = 0; return E_OK; }
ER tk_dly_tsk(TMO tmo) { (void)tmo; return E_OK; }
ER tk_slp_tsk(TMO tmo) { (void)tmo; return E_OK; }
INT tk_get_prc(void) { return 2; }

static char	g_log[1 << 20];
static size_t	g_log_len;

INT tm_printf(const UB *format, ...)
{
	char	buf[512];
	va_list	ap;
	int	n;

	va_start(ap, format);
	n = vsnprintf(buf, sizeof buf, (const char *)format, ap);
	va_end(ap);
	if (g_verbose) { fputs(buf, stdout); }
	if (g_log_len + (size_t)n + 1u < sizeof g_log) {
		memcpy(&g_log[g_log_len], buf, (size_t)n);
		g_log_len += (size_t)n;
		g_log[g_log_len] = '\0';
	}
	return n;
}

INT tm_sprintf(UB *str, const UB *format, ...)
{
	va_list	ap;
	int	n;

	va_start(ap, format);
	n = vsprintf((char *)str, (const char *)format, ap);
	va_end(ap);
	CHECK(n < REPORT_LINE_LEN);		/* line buffer never overflows */
	return n;
}

/* ------------------------------------------------------------------ *
 *  Driving the simulation
 * ------------------------------------------------------------------ */

#define MAX_SEEN	(512)

static motion_report_t	g_seen[MAX_SEEN];
static int		g_n_seen;

static void
drain_reports (void)
{
	int i;

	if (g_hold_reports) { return; }
	for (i = 0; i < g_q_count; i++) {
		print_report(&g_queue[i]);
		if (g_n_seen < MAX_SEEN) { g_seen[g_n_seen++] = g_queue[i]; }
	}
	g_q_count = 0;
}

static int	g_tick;			/* ticks since sim_reset          */
static int	g_first_drive;		/* first tick with a duty, or -1  */

static void
sim_tick (void)
{
	g_time_us += 10000u;
	g_tick++;
	plant_step();
	if (g_fail_ticks == 0) { motion_cyclic_isr(NULL); }	/* the 10 ms handler */
	motion_cycle(&g_ctx);
	if ((g_first_drive < 0) && ((g_duty[0] != 0) || (g_duty[1] != 0))) {
		g_first_drive = g_tick;
	}
	drain_reports();
}

static void
sim_ticks (int n)
{
	while (n-- > 0) { sim_tick(); }
}

#define BUTTON_BIT	(1u << SIM_PIN_GP20)

static void
press_button (void)
{
	g_gpio_in &= ~BUTTON_BIT;
	sim_ticks(6);
	g_gpio_in |= BUTTON_BIT;
	sim_ticks(6);
}

static void
sim_reset (const char *p_name)
{
	if (p_name != NULL) { printf("-- %s\n", p_name); }
	memset(&g_ctx, 0, sizeof g_ctx);
	memset(g_wheel, 0, sizeof g_wheel);
	g_duty[0] = g_duty[1] = 0;
	g_duty_max_abs = 0;
	g_enc_swap = 0;
	g_fail_ticks = 0;
	g_q_count = 0;
	g_hold_reports = 0;
	g_n_seen = 0;
	g_log_len = 0;
	g_log[0] = '\0';
	g_tick = 0;
	g_first_drive = -1;
	g_gpio_in = 0xFFFFFFFFu;		/* pull-ups: button released */
	g_gpio_in_reads = 0;
	g_gp20_setups = 0;
	g_wdt_starts = 0;
	g_wdt_feeds = 0;
	g_flag = 0u;
	g_brake_calls = 0;
	g_probe = 0;
	g_brake_t_us = 0u;
	g_rep_t_us = 0u;
	memset((void *)g_stop_req_us, 0, sizeof g_stop_req_us);
#if MOTION_BENCH && MOTION_TEST_STOP
	gb_st_ready = false;
#endif
#if MOTION_PACKED
	g_ds_word = MOTION_DS_WORD_ZERO;
#else
	memset(&g_ds, 0, sizeof g_ds);
	g_mtx_held = 0;
	g_mtx_pol_locks = 0;
	g_mtx_wait_locks = 0;
#endif
	CHECK(motion_start_up(&g_ctx, 2) == E_OK);
	g_boot = MOTION_BOOT_POWER_ON;		/* one start-up only */
	drain_reports();
}

/* The same, for scenarios that run many short cases. */
static void
sim_reset_quiet (void)
{
	sim_reset(NULL);
}

static int
count_kind (report_kind_t kind, int code)
{
	int i, n = 0;

	for (i = 0; i < g_n_seen; i++) {
		if ((g_seen[i].kind == (uint8_t)kind)
		    && ((code < 0) || (g_seen[i].code == (uint8_t)code))) {
			n++;
		}
	}
	return n;
}

static motion_report_t const *
find_kind (report_kind_t kind, int code, int nth)
{
	int i;

	for (i = 0; i < g_n_seen; i++) {
		if ((g_seen[i].kind == (uint8_t)kind)
		    && ((code < 0) || (g_seen[i].code == (uint8_t)code))) {
			if (nth-- == 0) { return &g_seen[i]; }
		}
	}
	return NULL;
}

/* Run until an event with this code arrives; returns ticks taken or -1. */
static int
run_until_event (report_event_t evt, int limit)
{
	int t;

	for (t = 1; t <= limit; t++) {
		sim_tick();
		if (count_kind(REP_EVENT, (int)evt) > 0) { return t; }
	}
	return -1;
}

/* A stall must be called exactly [t_stall] after the motors were first
   driven (the stall check judges the duty of the tick it measures, so
   one extra tick is allowed).  Printed as a simulation result. */
static void
check_stall_delay (void)
{
	int ticks = g_tick - g_first_drive;

	printf("   fault after %d ms of driving (simulation)\n", ticks * 10);
	CHECK(g_first_drive > 0);
	CHECK(ticks >= (int)MOTION_STALL_TICKS && ticks <= (int)MOTION_STALL_TICKS + 1);
}

/* ------------------------------------------------------------------ *
 *  Scenarios: distance_state (motion_ds.h), wheels moved by hand
 * ------------------------------------------------------------------ */

#define DS_MM_PER_COUNT	(M_PI * MOTION_WHEEL_DIA_MM / MOTION_COUNTS_PER_REV)

/* Turn both wheels by hand (counts, + = forward) and run one tick. */
static void
ds_move_tick (double left, double right)
{
	g_wheel[0].pos += left;
	g_wheel[1].pos += right;
	sim_tick();
}

/* Expected dist_mm for both wheels at this many counts. */
static long
ds_mm_at (double counts)
{
	return lround(counts * DS_MM_PER_COUNT);
}

/* Turn rate for k counts per tick of difference, over the window. */
static long
ds_turn_at (double diff_per_tick)
{
	double diff = diff_per_tick * MOTION_DS_WINDOW_TICKS;
	double s = MOTION_DS_WINDOW_TICKS * MOTION_TICK_MS / 1000.0;

	return lround(diff * DS_MM_PER_COUNT / MOTION_WHEELBASE_MM * (180.0 / M_PI) / s);
}

static void
scenario_ds_straight (void)
{
	uint32_t w0, prev, now = 0u;
	int i, b_every_tick = 1;

	sim_reset("distance_state: straight line, one publish per tick, accel then steady");
	w0 = motion_ds_read();
	CHECK(w0 == MOTION_DS_WORD_ZERO);	/* nothing published before a tick */
	prev = w0;
	for (i = 1; i <= 100; i++) {
		ds_move_tick(13.0, 13.0);
		now = motion_ds_read();
		if (motion_ds_updates(now, prev) != 1u) { b_every_tick = 0; }
		if (i == 1) { CHECK(motion_ds_state(now) == MOTION_DS_ACCEL); }
		if (i == (int)MOTION_DS_HIST) { CHECK(motion_ds_state(now) == MOTION_DS_STEADY); }
		prev = now;
	}
	CHECK(b_every_tick);
	CHECK(motion_ds_delta_mm(now, w0) == ds_mm_at(1300.0));	/* 103 mm */
	CHECK(motion_ds_turn_dps(now) == 0);
	CHECK(motion_ds_state(now) == MOTION_DS_STEADY);
	CHECK(count_kind(REP_DIST, -1) >= 4);
	CHECK(strstr(g_log, "[motion] distance_state 0x") != NULL);
	CHECK(strstr(g_log, "STEADY") != NULL);

	/* Stop: decelerating, then steady once the history is still. */
	ds_move_tick(0.0, 0.0);
	CHECK(motion_ds_state(motion_ds_read()) == MOTION_DS_DECEL);
	sim_ticks((int)MOTION_DS_HIST);
	CHECK(motion_ds_state(motion_ds_read()) == MOTION_DS_STEADY);
	CHECK(motion_ds_delta_mm(motion_ds_read(), w0) == ds_mm_at(1300.0));
}

static void
scenario_ds_turn_and_reverse (void)
{
	uint32_t w0, now;
	int i;

	sim_reset("distance_state: left turn +, right turn -, reversing falls");
	w0 = motion_ds_read();
	for (i = 0; i < 20; i++) { ds_move_tick(-13.0, 13.0); }	/* spin left */
	now = motion_ds_read();
	printf("   spin at 13 counts/tick per wheel: %d deg/s (simulation)\n",
	       (int)motion_ds_turn_dps(now));
	CHECK(motion_ds_turn_dps(now) == ds_turn_at(26.0));	/* about +93 */
	CHECK(motion_ds_turn_dps(now) > 0);
	CHECK(motion_ds_delta_mm(now, w0) == 0);		/* on the spot */
	CHECK(motion_ds_state(now) == MOTION_DS_STEADY);

	for (i = 0; i < 20; i++) { ds_move_tick(13.0, -13.0); }	/* spin right */
	now = motion_ds_read();
	CHECK(motion_ds_turn_dps(now) == -ds_turn_at(26.0));

	w0 = now;
	for (i = 0; i < 50; i++) { ds_move_tick(-13.0, -13.0); }	/* reverse */
	now = motion_ds_read();
	CHECK(motion_ds_delta_mm(now, w0) == -ds_mm_at(650.0));
	CHECK(motion_ds_turn_dps(now) == 0);
}

static void
scenario_ds_lost_tick (void)
{
	uint32_t w1, w2, w3;
	int i;

	sim_reset("distance_state: a lost tick publishes nothing and loses nothing");
	for (i = 0; i < 10; i++) { ds_move_tick(13.0, 13.0); }
	w1 = motion_ds_read();
	g_fail_ticks = 1;
	ds_move_tick(13.0, 13.0);			/* the car moves, no tick */
	w2 = motion_ds_read();
	CHECK(w2 == w1);
	CHECK(motion_ds_updates(w2, w1) == 0u);	/* reader: nothing new */
	ds_move_tick(13.0, 13.0);
	w3 = motion_ds_read();
	CHECK(motion_ds_updates(w3, w2) == 1u);
	CHECK(motion_ds_delta_mm(w3, w1) == ds_mm_at(156.0) - ds_mm_at(130.0));
}

static void
scenario_ds_wrap (void)
{
	uint32_t prev, now;
	long long summed = 0;
	int i;

	sim_reset("distance_state: past 32.768 m the wrapped differences still add up");
	prev = motion_ds_read();
	for (i = 0; i < 500; i++) {
		ds_move_tick(1000.0, 1000.0);
		now = motion_ds_read();
		summed += motion_ds_delta_mm(now, prev);
		prev = now;
	}
	printf("   driven %lld mm, dist_mm field now %u (simulation)\n",
	       summed, (unsigned)motion_ds_mm(prev));
	CHECK(summed > 32768);
	CHECK(summed == ds_mm_at(500000.0));
	CHECK(summed == g_ctx.ds_mm);
}

/* The status line prints the word imu_task reads and each state name.
   Lines come every 20 ticks: start at tick 15 so the one at tick 20 falls
   while accelerating, and stop at tick 38 so the one at 40 falls while
   decelerating. */
static void
scenario_ds_status_line (void)
{
	char want[64];
	int i;

	sim_reset("distance_state: status line shows the word and the state");
	for (i = 1; i <= 40; i++) {
		if ((i >= 15) && (i < 38)) { ds_move_tick(13.0, 13.0); }
		else { sim_tick(); }
		if ((i == 20) || (i == 40)) {
			(void)snprintf(want, sizeof want, "distance_state 0x%08x:",
				       (unsigned)motion_ds_read());
			CHECK(strstr(g_log, want) != NULL);
		}
	}
	CHECK(count_kind(REP_DIST, -1) == 2);
	CHECK(strstr(g_log, "ACCELERATING") != NULL);
	CHECK(strstr(g_log, "DECELERATING") != NULL);
	CHECK(strstr(g_log, "update 4, turn +0 deg/s") != NULL);	/* 20 mod 8 */
}

#if !MOTION_PACKED
/* Week 6 form: the writer never waits, so a busy mutex costs one
   publish, not any distance; the reader sees a repeated read. */
static void
scenario_ds_week6_mutex (void)
{
	uint32_t w1, w3;
	int i;

	sim_reset("distance_state, Week 6 form: busy mutex skips a publish, loses nothing");
	for (i = 0; i < 10; i++) { ds_move_tick(13.0, 13.0); }
	CHECK(g_mtx_pol_locks == 10);
	w1 = motion_ds_read();
	CHECK(g_mtx_wait_locks == 1);

	g_mtx_held = 1;				/* a reader holds it */
	ds_move_tick(13.0, 13.0);
	CHECK(g_mtx_pol_locks == 11);
	g_mtx_held = 0;
	CHECK(motion_ds_read() == w1);		/* that tick was skipped */
	CHECK(motion_ds_updates(motion_ds_read(), w1) == 0u);

	ds_move_tick(13.0, 13.0);
	w3 = motion_ds_read();
	CHECK(motion_ds_updates(w3, w1) == 1u);
	CHECK(motion_ds_delta_mm(w3, w1) == ds_mm_at(156.0) - ds_mm_at(130.0));
	CHECK(!g_mtx_held);
}
#endif

/* ------------------------------------------------------------------ *
 *  Scenarios: stop path (NFR-18)
 * ------------------------------------------------------------------ */

#define STOP_WAKE_US	(17u)	/* simulated request -> wake time */

/* A STOP sender (scan_task or imu_task, core 1, below motion) between two
   ticks: optionally stamp, set the bit, and motion wakes on it. */
static void
stop_request (UINT bit, int b_stamp)
{
	g_time_us += 1234u;		/* somewhere between two ticks */
	if (b_stamp) { motion_stop_stamp(bit); }
	(void)tk_set_flg(g_flgid, bit);
	g_time_us += STOP_WAKE_US;
	motion_cycle(&g_ctx);		/* the wake on the STOP bit */
	drain_reports();
}

static void
scenario_stop_scan (void)
{
	uint32_t ticks0;
	int feeds0;

	sim_reset("STOP_SCAN: brake at once, stay braked, scan_task clears it");
	CHECK(g_flgid > 0);
	CHECK(motor_task_flag() == (MOTION_BENCH ? 0 : g_flgid));	/* bench: scan_task print-only */
	sim_ticks(5);
	CHECK((g_last_waiptn & MOTION_EVT_STOPS) == MOTION_EVT_STOPS);
	CHECK((g_last_wmode & TWF_BITCLR) == 0u);	/* never clears a STOP */

	ticks0 = g_ctx.ticks;
	feeds0 = g_wdt_feeds;
	stop_request(MOTION_EVT_STOP_SCAN, 1);
	CHECK(g_brake_calls == 1);			/* the first write */
	CHECK(g_ctx.ticks == ticks0);		/* a STOP wake does no tick work */
	CHECK(g_wdt_feeds == feeds0);
	CHECK((g_flag & MOTION_EVT_STOP_SCAN) != 0u);	/* only scan_task clears it */
	CHECK(strstr(g_log, "[motion] STOP_SCAN set: brake written 17 us after the request.") != NULL);

	/* While stopped the STOP bits are left out of the wait, so motion
	   cannot spin on a bit that stays set; it checks them every tick. */
	sim_ticks(50);
	CHECK((g_last_waiptn & MOTION_EVT_STOPS) == 0u);
	CHECK((g_last_wmode & TWF_BITCLR) != 0u);
	CHECK((g_flag & MOTION_EVT_TICK) == 0u);	/* each tick consumed */
	CHECK((g_flag & MOTION_EVT_STOP_SCAN) != 0u);
	CHECK(g_ctx.ticks == ticks0 + 50u);
	CHECK(g_wdt_feeds == feeds0 + 50);
	CHECK(count_kind(REP_EVENT, EVT_STOP) == 1);

	(void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_STOP_SCAN);
	sim_tick();
	CHECK(g_ctx.stop_bits == 0u);
	CHECK(strstr(g_log, "every STOP bit clear, after 510 ms braked.") != NULL);	/* 51 ticks */
	CHECK(strstr(g_log, "Motors stay braked until a command.") != NULL);
	sim_tick();
	CHECK((g_last_waiptn & MOTION_EVT_STOPS) == MOTION_EVT_STOPS);
	CHECK(g_duty_max_abs == 0);
}

static void
scenario_stop_imu_and_scan (void)
{
	sim_reset("STOP_IMU unstamped, then STOP_SCAN too: braked until both clear");
	sim_ticks(3);
	stop_request(MOTION_EVT_STOP_IMU, 0);
	CHECK(g_brake_calls == 1);
	CHECK(strstr(g_log, "[motion] STOP_IMU set: motors braked (the sender did not stamp the request).") != NULL);

	sim_tick();
	g_time_us += 500u;
	motion_stop_stamp(MOTION_EVT_STOP_SCAN);
	(void)tk_set_flg(g_flgid, MOTION_EVT_STOP_SCAN);
	sim_tick();				/* seen on the next tick */
	CHECK(g_brake_calls == 1);		/* already braked */
	CHECK(strstr(g_log, "[motion] STOP_SCAN set: seen") != NULL);
	CHECK(strstr(g_log, "(already braked)") != NULL);

	(void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_STOP_SCAN);
	sim_ticks(3);
	CHECK(g_ctx.stop_bits == MOTION_EVT_STOP_IMU);	/* STOP_IMU stays */
	CHECK(count_kind(REP_EVENT, EVT_GO) == 0);
	CHECK((g_flag & MOTION_EVT_STOP_IMU) != 0u);	/* motion did not clear it */

	(void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_STOP_IMU);	/* FAULT_CLEAR(IMU), later */
	sim_tick();
	CHECK(count_kind(REP_EVENT, EVT_GO) == 1);
	CHECK(g_ctx.stop_bits == 0u);
}

/* A STOP set before motor_task() reaches its loop survives the start-up
   clear (only TICK is cleared) and brakes on the first wake. */
static void
scenario_stop_before_first_tick (void)
{
	sim_reset("STOP_SCAN set before motor_task's loop: braked on the first wake");
	g_flag = MOTION_EVT_STOP_SCAN | MOTION_EVT_TICK;	/* a stale tick too */
	g_loop_wakes = 1;			/* one wake, then leave the loop */
	if (setjmp(g_loop_jmp) == 0) {
		motor_task(0, NULL);
	}
	g_loop_wakes = -1;
	drain_reports();
	CHECK((g_flag & MOTION_EVT_STOP_SCAN) != 0u);
	CHECK(g_ctx.stop_bits == MOTION_EVT_STOP_SCAN);
	CHECK(g_brake_calls >= 1);
	CHECK(count_kind(REP_EVENT, EVT_STOP) == 1);
	CHECK(g_ctx.ticks == 0u);		/* the stale tick was cleared */
}

/* In the wake that sees a STOP the brake comes first: before any report,
   and the time is taken after it. */
static void
scenario_stop_order (void)
{
	sim_reset("STOP wake: brake first, then the time, then the report");
	sim_ticks(3);
	g_probe = 1;
	stop_request(MOTION_EVT_STOP_SCAN, 1);
	g_probe = 0;
	CHECK(g_brake_t_us != 0u);
	CHECK(g_rep_t_us > g_brake_t_us);
	CHECK(strstr(g_log, "brake written 20 us after the request") != NULL);	/* 17 + 3 */
}

#if MOTION_BENCH && !MOTION_TEST_STOP
/* A STOP during a bench test brakes the wheels in that wake and ends the
   test; it does not start again when the STOP clears, nor on the button
   while a STOP is set. */
static void
scenario_stop_during_bench (void)
{
	sim_reset("STOP_SCAN during the open-loop test: brake at once, test ends");
	press_button();
	sim_ticks(20);
	CHECK(g_duty[0] != 0);			/* L fwd is driving */
	stop_request(MOTION_EVT_STOP_SCAN, 1);
	CHECK((g_duty[0] == 0) && (g_duty[1] == 0));	/* in the same wake */
	CHECK(g_ctx.phase == PHASE_IDLE);
	CHECK(strstr(g_log, "the bench test ended on the STOP") != NULL);

	sim_ticks(30);
	press_button();				/* STOP still set: no start */
	CHECK(g_ctx.phase == PHASE_IDLE);
	CHECK(count_kind(REP_EVENT, EVT_START) == 1);
	(void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_STOP_SCAN);
	sim_ticks(30);
	CHECK((g_duty[0] == 0) && (g_duty[1] == 0));
	CHECK(g_ctx.phase == PHASE_IDLE);

	/* A tick and a STOP in the same wake while driving: the STOP comes
	   first, so the tick never drives (the motor_set fake checks it). */
	sim_reset(NULL);
	press_button();
	sim_ticks(20);
	CHECK(g_duty[0] != 0);
	(void)tk_set_flg(g_flgid, MOTION_EVT_STOP_SCAN);
	sim_tick();
	CHECK((g_duty[0] == 0) && (g_duty[1] == 0));
	CHECK(g_ctx.phase == PHASE_IDLE);
	CHECK(count_kind(REP_EVENT, EVT_STOP) == 1);
}
#endif

#if MOTION_BENCH && MOTION_TEST_STOP
/* The whole NFR-18 bench test: the stand-in runs one step after every
   tick (it is below motion_task on the same core), and a STOP it sets
   wakes motion at once. */
static void
scenario_stop_test (void)
{
	standin_t st = { 0u, 0u, 0u };
	int t, done = 0, b_spin = 0;
	uint32_t k = 0u, src, drove = 0u, drove_min = UINT32_MAX;
	uint32_t want_min[2] = { UINT32_MAX, UINT32_MAX }, want_max[2] = { 0u, 0u };
	uint32_t want_sum[2] = { 0u, 0u };
	char want[160];

	sim_reset("stop test: 100 stops per source from the stand-in, timed");
	press_button();
	CHECK(g_ctx.phase == PHASE_STOPTEST);
	CHECK(strstr(g_log, "START stop test (NFR-18)") != NULL);
	CHECK(strstr(g_log, "test=stop (NFR-18)") != NULL);
	drove = g_ctx.st_drive;			/* it started driving in the press */

	for (t = 0; (t < 20000) && !done; t++) {
		UINT before = g_flag & MOTION_EVT_STOPS;
		UINT added;

		sim_tick();
		if ((g_duty[0] != 0) && (g_duty[1] != 0)) { drove++; }
		standin_step(&st);
		added = g_flag & MOTION_EVT_STOPS & ~before;
		if (added != 0u) {
			/* a different wake time each stop, so min, mean and max differ */
			uint32_t w = 10u + ((k * 7u) % 23u);

			src = (added == MOTION_EVT_STOP_SCAN) ? 0u : 1u;
			if (w < want_min[src]) { want_min[src] = w; }
			if (w > want_max[src]) { want_max[src] = w; }
			want_sum[src] += w;
			k++;
			if (drove < drove_min) { drove_min = drove; }
			drove = 0u;
			g_time_us += w;
			motion_cycle(&g_ctx);
			drain_reports();
			if ((g_duty[0] != 0) || (g_duty[1] != 0)) { b_spin = 1; }
		}
		done = (g_ctx.phase == PHASE_IDLE);
	}
	printf("   stop test took %d ticks (simulation)\n", t);
	CHECK(done);
	CHECK(!b_spin);				/* braked in every STOP wake */
	CHECK(drove_min >= MOTION_TEST_STOP_DRIVE);	/* wheels turning at each stop */
	for (src = 0u; src < 2u; src++) {
		(void)snprintf(want, sizeof want,
			       "STOP TEST %s: 100 stops, request to brake min %u / mean %u / max %u us",
			       (src == 0u) ? "STOP_SCAN" : "STOP_IMU", (unsigned)want_min[src],
			       (unsigned)(want_sum[src] / 100u), (unsigned)want_max[src]);
		CHECK(want_min[src] < want_max[src]);
		CHECK(strstr(g_log, want) != NULL);
	}
	CHECK(g_ctx.st_n[STOP_SRC_SCAN] == MOTION_TEST_STOP_TRIALS);
	CHECK(g_ctx.st_n[STOP_SRC_IMU] == MOTION_TEST_STOP_TRIALS);
	CHECK(g_duty_max_abs == MOTION_TEST_DUTY);	/* it drove between stops */
	CHECK((g_duty[0] == 0) && (g_duty[1] == 0));
	CHECK(g_ctx.faults == 0u);

	/* A STOP from elsewhere (unstamped) is braked but never counted, and
	   a stamp made while a bit is already set is never charged later. */
	sim_reset(NULL);
	press_button();
	sim_ticks((int)MOTION_TEST_STOP_DRIVE + 1);
	CHECK(gb_st_ready);
	stop_request(MOTION_EVT_STOP_SCAN, 0);		/* scan_task, say */
	CHECK(!gb_st_ready);				/* the stand-in waits */
	motion_stop_stamp(MOTION_EVT_STOP_SCAN);	/* stamp for a set bit */
	(void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_STOP_SCAN);
	sim_ticks(5);
	stop_request(MOTION_EVT_STOP_SCAN, 0);
	CHECK(g_ctx.st_n[STOP_SRC_SCAN] == 0u);
	CHECK(strstr(g_log, "brake written") == NULL);	/* the old stamp was dropped */
	(void)tk_clr_flg(g_flgid, ~(UINT)MOTION_EVT_STOP_SCAN);
	press_button();					/* abort */

	/* Idle afterwards: the stand-in sets nothing more. */
	for (t = 0; t < 200; t++) { sim_tick(); standin_step(&st); }
	CHECK((g_flag & MOTION_EVT_STOPS) == 0u);
}
#endif

/* ------------------------------------------------------------------ *
 *  Scenarios
 * ------------------------------------------------------------------ */

#if !MOTION_TEST_PID

static void
scenario_idle (void)
{
	sim_reset("idle: motors stay braked, timing line every 5 s");
	sim_ticks(600);
	CHECK(g_duty_max_abs == 0);
	CHECK(count_kind(REP_STATUS, -1) == 0);
	CHECK(count_kind(REP_TIMING, -1) == 1);
	CHECK(strstr(g_log, "tick period 10000..10000 us") != NULL);
}

static void
scenario_hand_turn (void)
{
	int i;

	sim_reset("hand turn while idle: counts print, motors stay braked");
	for (i = 0; i < 100; i++) {
		g_wheel[0].pos += 2.0;		/* left wheel turned forward */
		sim_tick();
	}
	CHECK(g_duty_max_abs == 0);
	CHECK(count_kind(REP_STATUS, -1) >= 4);
	{
		motion_report_t const *p = find_kind(REP_STATUS, -1, count_kind(REP_STATUS, -1) - 1);
		CHECK((p != NULL) && (p->val_l >= 180) && (p->val_r == 0));
	}
}

/* Armed once at start-up, before the first feed; fed once per completed
   tick and never on a lost one; the timing line shows the gap. */
static void
scenario_watchdog_feed (void)
{
	sim_reset("watchdog: armed at start-up, fed every completed tick");
	CHECK(g_wdt_starts == 1);
	CHECK(g_wdt_feeds_at_start == 0);
	CHECK(strstr(g_log, "last reset: power-on or RUN pin") != NULL);
	CHECK(strstr(g_log, "watchdog armed: the chip resets if a tick does not complete within 30 ms") != NULL);
	sim_ticks(600);
	CHECK(g_wdt_feeds == 600);
	CHECK(strstr(g_log, "longest gap between feeds 10000 us, resets at 30000 us") != NULL);

	/* No tick, no feed.  On the car the chip resets 30 ms after the
	   last feed, before this 50 ms timeout could ever fire. */
	g_fail_ticks = 3;
	sim_ticks(3);
	CHECK(g_wdt_feeds == 600);
	sim_tick();
	CHECK(g_wdt_feeds == 601);
}

/* After a watchdog reset the car says so and does not start on its own. */
static void
scenario_boot_after_watchdog (void)
{
	g_boot = MOTION_BOOT_WATCHDOG;
	sim_reset("start-up after a watchdog reset: reported, motors stay braked");
	CHECK(strstr(g_log, "last reset: WATCHDOG") != NULL);
	CHECK(strstr(g_log, "watchdog test") == NULL);
	CHECK(g_wdt_starts == 1);
	sim_ticks(300);
	CHECK(g_duty_max_abs == 0);
	CHECK(count_kind(REP_EVENT, EVT_START) == 0);

	g_boot = MOTION_BOOT_REBOOT;
	sim_reset("start-up after the boot ROM's reboot: not a fault");
	CHECK(strstr(g_log, "not a fault") != NULL);
	CHECK(strstr(g_log, "WATCHDOG") == NULL);
}

/* A car that failed its set-up stays braked and is never armed, so it
   does not reboot over and over. */
static void
scenario_init_fail_not_armed (void)
{
	printf("-- init fails: watchdog not armed, motors braked\n");
	memset(&g_ctx, 0, sizeof g_ctx);
	g_n_seen = 0; g_q_count = 0; g_log_len = 0; g_log[0] = '\0';
	g_wdt_starts = 0;
	g_fail_init = 1;
	CHECK(motion_start_up(&g_ctx, 2) == E_OBJ);
	g_fail_init = 0;
	drain_reports();
	CHECK(g_wdt_starts == 0);
	CHECK(strstr(g_log, "init FAILED") != NULL);
	CHECK(strstr(g_log, "watchdog NOT armed") != NULL);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}

#if !MOTION_BENCH
/* Integration build: GP20 is Buddy 5's start button, so motion must
   never configure or read it, and no press may drive a motor. */
static void
scenario_bench_off (void)
{
	sim_reset("bench off: GP20 never touched, the button cannot start a motor");
	CHECK(strstr(g_log, "bench OFF (MOTION_BENCH=0): motors stay braked") != NULL);
	press_button();
	press_button();
	g_gpio_in &= ~BUTTON_BIT;		/* held down */
	sim_ticks(300);
	g_gpio_in |= BUTTON_BIT;
	sim_ticks(10);
	CHECK(g_gp20_setups == 0);
	CHECK(g_gpio_in_reads == 0);
	CHECK(count_kind(REP_EVENT, EVT_START) == 0);
	CHECK(g_duty_max_abs == 0);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}
#endif

static void
scenario_button_held_at_boot (void)
{
	g_gpio_in = 0xFFFFFFFFu & ~BUTTON_BIT;	/* held before init */
	printf("-- button held at boot does not start a test\n");
	memset(&g_ctx, 0, sizeof g_ctx);
	g_n_seen = 0; g_q_count = 0; g_duty_max_abs = 0;
	CHECK(motion_init(&g_ctx) == E_OK);
	sim_ticks(200);
	CHECK(count_kind(REP_EVENT, EVT_START) == 0);
	CHECK(g_duty_max_abs == 0);
}

static void
scenario_open_loop (void)
{
	static const int8_t sign_l[] = { 1, -1, 0, 0, 1 };
	static const int8_t sign_r[] = { 0, 0, 1, -1, 1 };
	motion_report_t const *p;
	int i, t;

	sim_reset("open-loop bench test, healthy car");
	press_button();
	CHECK(count_kind(REP_EVENT, EVT_START) == 1);
	t = run_until_event(EVT_DONE, 5000);
	CHECK(t > 0);
	CHECK(count_kind(REP_EVENT, EVT_FAULT) == 0);
	CHECK(count_kind(REP_STEP, -1) == 5);

	for (i = 0; i < 5; i++) {
		p = find_kind(REP_STEP, -1, i);
		CHECK(p != NULL);
		if (p == NULL) { continue; }
		CHECK((sign_l[i] > 0) ? (p->val_l > 100) : (sign_l[i] < 0) ? (p->val_l < -100) : (p->val_l == 0));
		CHECK((sign_r[i] > 0) ? (p->val_r > 100) : (sign_r[i] < 0) ? (p->val_r < -100) : (p->val_r == 0));
		CHECK(motion_abs(p->duty_l) == ((sign_l[i] != 0) ? MOTION_TEST_DUTY : 0));
	}

	p = find_kind(REP_EVENT, EVT_RAMP, 0);
	CHECK(p != NULL);
	if (p != NULL) {
		printf("   ramp found L=%d R=%d (sim dead band %d)\n",
		       (int)p->val_l, (int)p->val_r, SIM_DEADBAND);
		CHECK(p->val_l > SIM_DEADBAND && p->val_l <= SIM_DEADBAND + 50);
		CHECK(p->val_r > SIM_DEADBAND && p->val_r <= SIM_DEADBAND + 50);
	}
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
	CHECK(g_duty_max_abs <= MOTION_RAMP_MAX_DUTY);
	CHECK(strstr(g_log, "WARNING") == NULL);
	CHECK(strstr(g_log, "opposite way") == NULL);

	/* Runs again on the next press. */
	g_n_seen = 0;
	press_button();
	CHECK(count_kind(REP_EVENT, EVT_START) == 1);
}

static void
scenario_dead_encoder (void)
{
	int t;

	sim_reset("left encoder dead: FR-30 stall fault within [t_stall]");
	g_wheel[0].b_enc_dead = 1;
	press_button();
	t = run_until_event(EVT_FAULT, 400);
	CHECK(t > 0);
	check_stall_delay();
	CHECK((g_ctx.faults & MOTION_FAULT_STALL_L) != 0u);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
	CHECK(g_ctx.phase == PHASE_IDLE);
	CHECK(strstr(g_log, "left wheel stalled") != NULL);
}

/* FR-30's own test: one encoder signal wire unplugged.  The other phase
   keeps toggling, so the count only steps back and forth. */
static void
scenario_unplugged_phase (void)
{
	int t;

	sim_reset("left encoder A wire unplugged: encoder fault within [t_stall]");
	sim_unplug_phase(0, 1);
	press_button();
	t = run_until_event(EVT_FAULT, 400);
	CHECK(t > 0);
	check_stall_delay();
	CHECK(g_ctx.faults == MOTION_FAULT_ENC_L);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
	CHECK(g_ctx.phase == PHASE_IDLE);
	CHECK(strstr(g_log, "left encoder fault") != NULL);
	CHECK(strstr(g_log, "Check the A and B wires") != NULL);
}

/* The wire comes off while the wheel is turning: the window already holds
   edges from before, and the pull-up adds one more edge if the pin was
   low.  Still an encoder fault, every time. */
static void
scenario_unplug_mid_run (void)
{
	static const int offsets[] = { 3, 8, 17, 26, 41, 59 };
	int phase, i, t, n = 0, n_enc = 0;

	printf("-- left encoder wire pulled mid-step (A and B, 6 moments each)\n");
	for (phase = 1; phase <= 2; phase++) {
		for (i = 0; i < (int)(sizeof offsets / sizeof offsets[0]); i++) {
			sim_reset_quiet();
			press_button();
			sim_ticks(offsets[i]);
			CHECK(g_duty[0] != 0);			/* still in L fwd */
			sim_unplug_phase(0, phase);
			t = run_until_event(EVT_FAULT, 400);
			CHECK(t > 0 && t <= (int)MOTION_STALL_TICKS + 2);
			CHECK(g_ctx.faults == MOTION_FAULT_ENC_L);
			CHECK(g_duty[0] == 0 && g_duty[1] == 0);
			n++;
			if (g_ctx.faults == MOTION_FAULT_ENC_L) { n_enc++; }
		}
	}
	printf("   encoder fault in %d of %d runs\n", n_enc, n);
}

static void
scenario_blocked_right (void)
{
	sim_reset("right wheel blocked: left steps pass, R fwd faults");
	g_wheel[1].b_blocked = 1;
	press_button();
	CHECK(run_until_event(EVT_FAULT, 2000) > 0);
	CHECK(g_ctx.faults == MOTION_FAULT_STALL_R);
	CHECK(count_kind(REP_STEP, -1) == 2);		/* L fwd, L rev done */
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}

static void
scenario_abort (void)
{
	sim_reset("second press aborts and brakes");
	press_button();
	sim_ticks(60);
	CHECK(g_duty[0] != 0);
	press_button();
	CHECK(count_kind(REP_EVENT, EVT_ABORT) == 1);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}

static void
scenario_tick_lost (void)
{
	sim_reset("tick lost during a test: brake and fault");
	press_button();
	sim_ticks(20);
	g_fail_ticks = 1;
	sim_tick();
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
	CHECK((g_ctx.faults & MOTION_FAULT_TICK) != 0u);
	CHECK(count_kind(REP_EVENT, EVT_FAULT) == 1);
}

static void
scenario_swapped_plugs (void)
{
	sim_reset("encoder plugs swapped: warning printed");
	g_enc_swap = 1;
	press_button();
	(void)run_until_event(EVT_DONE, 5000);
	CHECK(strstr(g_log, "WARNING: the right encoder moved on a left-only step") != NULL);
}

static void
scenario_inverted_motor (void)
{
	sim_reset("left motor wired backwards: hint printed, no fault");
	g_wheel[0].b_motor_inv = 1;
	press_button();
	(void)run_until_event(EVT_DONE, 5000);
	CHECK(count_kind(REP_EVENT, EVT_FAULT) == 0);	/* open loop: |delta| */
	CHECK(strstr(g_log, "opposite way") != NULL);
}

static void
scenario_reports_never_block (void)
{
	sim_reset("report queue full: records dropped, control keeps running");
	g_hold_reports = 1;
	press_button();
	sim_ticks(400);
	CHECK(g_ctx.rep_dropped > 0u);
	CHECK(g_ctx.step >= 2u);			/* sequence kept going */
	g_hold_reports = 0;
	drain_reports();
}

#if MOTION_TEST_WDT
/* Bench watchdog test: motion_task hangs with the left wheel turning and
   never feeds again.  The car's reset and its timing are checked on the
   car; here, that the hang comes where the README says, and that the next
   start-up prints the record and keeps the motors braked. */
static void
scenario_wdt_hang (void)
{
	int feeds;

	sim_reset("watchdog test: hang in L fwd, then the reboot");
	if (setjmp(g_hang_jmp) == 0) {
		press_button();
		sim_ticks(200);
		CHECK(0);			/* must have hung */
	}
	CHECK(g_ctx.step == 0u);		/* L fwd */
	CHECK(g_ctx.step_ticks == MOTION_TEST_WDT_TICKS);
	CHECK(g_duty[0] == MOTION_TEST_DUTY);	/* wheel still driven */
	CHECK(g_hang_fed_us == g_ctx.wdt_fed_us);
	feeds = g_wdt_feeds;
	CHECK(feeds > (int)MOTION_TEST_WDT_TICKS);	/* fed until the hang */

	/* The chip resets; SCRATCH0..2 survive with the record. */
	g_boot = MOTION_BOOT_WATCHDOG;
	g_hang_marker = 1;
	g_hang_us = 30004u;
	sim_reset(NULL);
	CHECK(strstr(g_log, "last reset: WATCHDOG") != NULL);
	CHECK(strstr(g_log, "watchdog test: the reset came 30004 us after the last feed (set to 30000 us)") != NULL);
	CHECK(g_hang_marker == 0);		/* reported once */
	sim_ticks(300);
	CHECK(g_duty_max_abs == 0);		/* waits for the button */
}
#endif

#else /* MOTION_TEST_PID */

static void
scenario_pid (void)
{
	double sum = 0.0;
	int i;

	sim_reset("closed-loop speed test reaches target");
	press_button();
	for (i = 0; i < 280; i++) {
		sim_tick();
		if (i >= 180) { sum += g_wheel[0].speed + g_wheel[1].speed; }
	}
	printf("   mean speed over last second: %.2f counts/tick (target %d)\n",
	       sum / 200.0, MOTION_PID_TARGET);
	CHECK(fabs(sum / 200.0 - MOTION_PID_TARGET) <= 0.1 * MOTION_PID_TARGET);
	CHECK(g_duty_max_abs <= MOTION_PID_OUT_LIMIT);
	CHECK(run_until_event(EVT_DONE, 200) > 0);
	CHECK(count_kind(REP_EVENT, EVT_FAULT) == 0);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}

static void
scenario_pid_wrong_polarity (void)
{
	sim_reset("closed loop with a backwards motor: stall fault, no runaway");
	g_wheel[0].b_motor_inv = 1;
	press_button();
	CHECK(run_until_event(EVT_FAULT, 300) > 0);
	CHECK((g_ctx.faults & MOTION_FAULT_STALL_L) != 0u);
	CHECK(g_duty_max_abs <= MOTION_PID_OUT_LIMIT);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}

/* The dangerous case: closed loop with one wire unplugged.  The PID sees
   no speed and pushes that wheel to its limit; the stall check must end
   it within [t_stall], not let it run. */
static void
scenario_pid_unplugged_phase (void)
{
	int t;

	sim_reset("closed loop, right encoder B wire unplugged: encoder fault, no runaway");
	sim_unplug_phase(1, 2);
	press_button();
	t = run_until_event(EVT_FAULT, 300);
	CHECK(t > 0);
	check_stall_delay();
	CHECK(g_ctx.faults == MOTION_FAULT_ENC_R);
	CHECK(g_duty_max_abs <= MOTION_PID_OUT_LIMIT);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}

/* Closed loop, wire pulled mid-run at the bench target speed. */
static void
scenario_pid_unplug_mid_run (void)
{
	int t;

	sim_reset("closed loop, right encoder A wire pulled mid-run: encoder fault, no runaway");
	press_button();
	sim_ticks(120);
	sim_unplug_phase(1, 1);
	t = run_until_event(EVT_FAULT, 300);
	CHECK(t > 0 && t <= (int)MOTION_STALL_TICKS + 2);
	CHECK(g_ctx.faults == MOTION_FAULT_ENC_R);
	CHECK(g_duty_max_abs <= MOTION_PID_OUT_LIMIT);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
}

#endif

int
main (int argc, char **argv)
{
	g_verbose = (argc > 1) && (strcmp(argv[1], "-v") == 0);

	CHECK(motor_task_start() == E_OK);
	CHECK(sizeof(motion_report_t) == 52u);

#if MOTION_TEST_WDT
	scenario_wdt_hang();
#elif !MOTION_BENCH
	scenario_idle();
	scenario_hand_turn();
	scenario_watchdog_feed();
	scenario_boot_after_watchdog();
	scenario_init_fail_not_armed();
	scenario_bench_off();
	scenario_ds_straight();
	scenario_ds_turn_and_reverse();
	scenario_ds_lost_tick();
	scenario_ds_wrap();
	scenario_ds_status_line();
#if !MOTION_PACKED
	scenario_ds_week6_mutex();
#endif
	scenario_stop_scan();
	scenario_stop_imu_and_scan();
	scenario_stop_before_first_tick();
	scenario_stop_order();
#elif MOTION_TEST_STOP
	scenario_stop_test();
#elif !MOTION_TEST_PID
	scenario_idle();
	scenario_hand_turn();
	scenario_watchdog_feed();
	scenario_boot_after_watchdog();
	scenario_init_fail_not_armed();
	scenario_button_held_at_boot();
	scenario_open_loop();
	scenario_dead_encoder();
	scenario_unplugged_phase();
	scenario_unplug_mid_run();
	scenario_blocked_right();
	scenario_abort();
	scenario_tick_lost();
	scenario_swapped_plugs();
	scenario_inverted_motor();
	scenario_reports_never_block();
	scenario_stop_scan();
	scenario_stop_during_bench();
#else
	scenario_pid();
	scenario_pid_wrong_polarity();
	scenario_pid_unplugged_phase();
	scenario_pid_unplug_mid_run();
#endif

	printf("motion_task sim (%s%s): %d checks, %d failures\n",
	       !MOTION_BENCH ? "bench off"
	       : MOTION_TEST_WDT ? "watchdog test"
	       : MOTION_TEST_STOP ? "stop test"
	       : (MOTION_TEST_PID ? "closed-loop" : "open-loop"),
	       MOTION_PACKED ? "" : ", Week 6 unpacked",
	       g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
