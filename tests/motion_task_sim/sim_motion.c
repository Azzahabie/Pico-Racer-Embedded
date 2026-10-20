/*
 *	sim_motion.c
 *	Runs the real motor_task.c on a PC against a simulated car: two
 *	motors with a dead band and lag, two encoders, the button, the 1 MHz
 *	timer and the kernel calls motion_task makes.  Checks the bench test
 *	end to end before anything is flashed:
 *
 *	    make -C tests/motion_task_sim          (V=1 prints the console)
 *
 *	motor.c, encoder.c and gpio_irq.c are replaced by the fakes below;
 *	motion_lib.c is the real one.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* The unit under test, statics included. */
#include "../../app_program/motor_task/motor_task.c"

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
	int		b_enc_dead;
	int		b_blocked;
	uint16_t	count;		/* what its encoder reports            */
} sim_wheel_t;

static sim_wheel_t	g_wheel[2];
static int32_t		g_duty[2];
static int32_t		g_duty_max_abs;
static int		g_enc_swap;	/* left encoder plugged into right port */
static uint32_t		g_time_us;
static uint32_t		g_gpio_in;
static int		g_fail_ticks;	/* tk_wai_flg times out this many times */
static UINT		g_flag;

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

		if (!p_enc->b_enc_dead) {
			p_enc->count = (uint16_t)((unsigned long)c & 0xFFFFu);
		}
	}
}

/* ------------------------------------------------------------------ *
 *  Fakes: motor.h, encoder.h, gpio_irq.h
 * ------------------------------------------------------------------ */

ER motor_init(void) { g_duty[0] = g_duty[1] = 0; return E_OK; }

void motor_set(motor_side_t side, int32_t duty_pm)
{
	g_duty[side] = motion_clamp(duty_pm, -MOTION_DUTY_FULL, MOTION_DUTY_FULL);
	if (motion_abs(g_duty[side]) > g_duty_max_abs) {
		g_duty_max_abs = motion_abs(g_duty[side]);
	}
}

void motor_brake_all(void) { g_duty[0] = g_duty[1] = 0; }
int32_t motor_get_duty(motor_side_t side) { return g_duty[side]; }

ER encoder_init(void) { return E_OK; }
uint32_t encoder_read_packed(void)
{
	return motion_pack_counts(g_wheel[0].count, g_wheel[1].count);
}
uint32_t encoder_invalid_packed(void) { return 0u; }
bool encoder_left_enabled(void) { return true; }

ER gpio_irq_init(INT level) { (void)level; return E_OK; }
ER gpio_irq_attach(uint32_t m, uint32_t e, gpio_irq_cb_t cb) { (void)m; (void)e; (void)cb; return E_OK; }
uint32_t gpio_irq_count(void) { return 0u; }
uint32_t gpio_irq_take_max_us(void) { return 3u; }

/* ------------------------------------------------------------------ *
 *  Fakes: registers, kernel, monitor
 * ------------------------------------------------------------------ */

UW in_w(UW addr)
{
	UW v = 0u;
	if (addr == TIMER_RAWL) { v = g_time_us; }
	else if (addr == GPIO_IN) { v = g_gpio_in; }
	return v;
}
void out_w(UW a, UW d) { (void)a; (void)d; }
void set_w(UW a, UW d) { (void)a; (void)d; }
void clr_w(UW a, UW d) { (void)a; (void)d; }
ER gpio_set_pin(UINT no, UINT mode) { (void)no; (void)mode; return E_OK; }

#define SIM_QUEUE_LEN	((int)REPORT_QUEUE_LEN)

static motion_report_t	g_queue[SIM_QUEUE_LEN];
static int		g_q_count;
static int		g_hold_reports;

ID tk_cre_flg(CONST T_CFLG *p) { (void)p; return 1; }
ER tk_set_flg(ID id, UINT ptn) { (void)id; g_flag |= ptn; return E_OK; }
ER tk_clr_flg(ID id, UINT ptn) { (void)id; g_flag &= ptn; return E_OK; }
ER tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *p_ptn, TMO tmo)
{
	(void)id; (void)mode; (void)tmo;
	if (g_fail_ticks > 0) { g_fail_ticks--; return E_TMOUT; }
	*p_ptn = ptn;
	return E_OK;
}
ID tk_cre_mbf(CONST T_CMBF *p)
{
	/* The real buffer must hold REPORT_QUEUE_LEN records exactly. */
	CHECK(p->bufsz == (SZ)(REPORT_QUEUE_LEN * (sizeof(motion_report_t) + 4u)));
	CHECK(p->maxmsz == (INT)sizeof(motion_report_t));
	return 2;
}
ER tk_snd_mbf(ID id, void *msg, INT size, TMO tmo)
{
	(void)id;
	CHECK(tmo == TMO_POL);			/* motion_task never blocks */
	CHECK(size == (INT)sizeof(motion_report_t));
	if (g_q_count >= SIM_QUEUE_LEN) { return E_TMOUT; }
	memcpy(&g_queue[g_q_count++], msg, sizeof(motion_report_t));
	return E_OK;
}
INT tk_rcv_mbf(ID id, void *msg, TMO tmo) { (void)id; (void)msg; (void)tmo; return E_TMOUT; }
ID tk_cre_tsk(CONST T_CTSK *p)
{
	if (p->task == (FP)motor_task) {
		CHECK(p->itskpri == 11);
		CHECK((p->tskatr & TA_ASSPRC) != 0u);
		CHECK(p->assprc == TP_PRC2);	/* core 1 */
	} else {
		CHECK(p->assprc == TP_PRC1);	/* reporter on core 0 */
	}
	return 3;
}
ER tk_sta_tsk(ID id, INT stacd) { (void)id; (void)stacd; return E_OK; }
ID tk_cre_cyc(CONST T_CCYC *p) { CHECK(p->cyctim == 10u); return 4; }
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

static void
sim_tick (void)
{
	g_time_us += 10000u;
	plant_step();
	motion_cycle(&g_ctx);
	drain_reports();
}

static void
sim_ticks (int n)
{
	while (n-- > 0) { sim_tick(); }
}

#define BUTTON_BIT	(1u << MOTION_PIN_BUTTON)

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
	printf("-- %s\n", p_name);
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
	g_gpio_in = 0xFFFFFFFFu;		/* pull-ups: button released */
	CHECK(motion_init(&g_ctx) == E_OK);
	report_event(&g_ctx, EVT_READY, E_OK, 2);
	drain_reports();
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
	printf("   fault after %d ms of driving\n", (t + 6) * 10);
	CHECK(t > 0 && (t + 6) <= (int)MOTION_STALL_TICKS + 3);
	CHECK((g_ctx.faults & MOTION_FAULT_STALL_L) != 0u);
	CHECK(g_duty[0] == 0 && g_duty[1] == 0);
	CHECK(g_ctx.phase == PHASE_IDLE);
	CHECK(strstr(g_log, "left wheel stalled") != NULL);
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

#endif

int
main (int argc, char **argv)
{
	g_verbose = (argc > 1) && (strcmp(argv[1], "-v") == 0);

	CHECK(motor_task_start() == E_OK);
	CHECK(sizeof(motion_report_t) == 52u);

#if !MOTION_TEST_PID
	scenario_idle();
	scenario_hand_turn();
	scenario_button_held_at_boot();
	scenario_open_loop();
	scenario_dead_encoder();
	scenario_blocked_right();
	scenario_abort();
	scenario_tick_lost();
	scenario_swapped_plugs();
	scenario_inverted_motor();
	scenario_reports_never_block();
#else
	scenario_pid();
	scenario_pid_wrong_polarity();
#endif

	printf("motion_task sim (%s): %d checks, %d failures\n",
	       MOTION_TEST_PID ? "closed-loop" : "open-loop", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
