/*
 *	test_encoder_dispatch.c
 *	Host test: motion's encoder driver through Buddy 5's real GPIO
 *	dispatcher (app_program/common/gpio_irq.c, compiled unchanged), on a
 *	fake IO_BANK0 with two cores.  It follows the start-up order on the
 *	car: motion on core 1, then scan_task's ECHO pin on core 1, then
 *	nav_ctrl_task's buttons on core 0.   make -C tests/encoder_dispatch
 *
 *	Opt-in, not part of make -C tests: -DTEST_ATTACH_RACE (make race)
 *	shows that gpio_irq_attach() can lose motion's encoder enables when
 *	scan_task is preempted inside its own attach.  Each run preempts after
 *	a different PROC1_INTE access (1 to 4), never while DI is held, so
 *	only an attach that is safe at every point passes them all.  It fails
 *	until the attach stops doing an unprotected read-modify-write.
 *
 *	This build also reads motion_cfg.h with no -DMOTION_BENCH, so it is
 *	where the committed value of MOTION_BENCH is checked.
 */

#include <stdio.h>
#include <stdlib.h>
#include <tk/tkernel.h>

#include "gpio_irq.h"
#include "motion_cfg.h"
#include "motion_lib.h"
#include "encoder.h"

_Static_assert(MOTION_BENCH == 0,
	       "motion_cfg.h: MOTION_BENCH is committed as 1; bench images only, never commit it");

static int g_checks, g_failures;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failures++; \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

#define NUM_REGS	4u
#define INTNO_BANK0	13u
#define EDGE_LOW	0x4u	/* per-pin event bits, as in gpio_irq.h */
#define EDGE_HIGH	0x8u

#define REG_INTR	(IO_BANK0_BASE + 0x0F0u)
#define REG_INTE0	(IO_BANK0_BASE + 0x100u)	/* core 0 */
#define REG_INTE1	(IO_BANK0_BASE + 0x130u)	/* core 1 */
#define REG_INTS0	(IO_BANK0_BASE + 0x120u)
#define REG_INTS1	(IO_BANK0_BASE + 0x150u)

/* Fake chip state. */
static uint32_t	g_gpio_in;
static uint32_t	g_intr[NUM_REGS];		/* raw latched edges */
static uint32_t	g_inte[2][NUM_REGS];		/* per-core enables */
static uint32_t	g_timer;
static unsigned	g_core;				/* core running now */
static FP	g_handler;
static int	g_def_int_calls;
static int	g_enable_level[2];

/* What the other subsystems' callbacks saw. */
static int	g_echo_calls;
static UW	g_echo_now;
static int	g_btn_calls;

static int	reg_of(UW addr, UW base)
{
	return ((addr >= base) && (addr < base + 4u * NUM_REGS)) ? (int)((addr - base) / 4u) : -1;
}

#ifdef TEST_ATTACH_RACE
static int	g_in_scan_attach;	/* scan_task is inside its attach */
static int	g_preempt_at;		/* preempt after this PROC1_INTE access */
static int	g_inte1_accesses;
static int	g_masked;		/* DI nesting (race_hooks.h) */
static int	g_pending;		/* preemption held off by DI */
static int	g_motion_done;
static void	motion_attach_now(void);

void race_di(void) { g_masked++; }
void race_ei(void)
{
	g_masked--;
	if ((0 == g_masked) && g_pending) {	/* motion runs when DI ends */
		g_pending = 0;
		motion_attach_now();
	}
}
#endif

/* Race test only: motion_task preempts scan_task right after scan's
   g_preempt_at-th access to PROC1_INTE, or at its EI if DI is held. */
static void after_inte1_access(void)
{
#ifdef TEST_ATTACH_RACE
	if (g_in_scan_attach && (++g_inte1_accesses == g_preempt_at)) {
		if (g_masked > 0) {
			g_pending = 1;
		} else {
			motion_attach_now();
		}
	}
#endif
}

UW in_w(UW addr)
{
	int r;

	if (addr == GPIO_IN) { return g_gpio_in; }
	if (addr == TIMER_TIMERAWL) { return ++g_timer; }	/* 1 us per read */
	if ((r = reg_of(addr, REG_INTR)) >= 0) { return g_intr[r]; }
	if ((r = reg_of(addr, REG_INTS0)) >= 0) { return g_intr[r] & g_inte[0][r]; }
	if ((r = reg_of(addr, REG_INTS1)) >= 0) { return g_intr[r] & g_inte[1][r]; }
	if ((r = reg_of(addr, REG_INTE0)) >= 0) { return g_inte[0][r]; }
	if ((r = reg_of(addr, REG_INTE1)) >= 0) {
		UW v = g_inte[1][r];
		after_inte1_access();
		return v;
	}
	return 0u;
}

void out_w(UW addr, UW data)
{
	int r;

	if ((r = reg_of(addr, REG_INTR)) >= 0) { g_intr[r] &= ~(data & 0xCCCCCCCCu); }	/* W1C */
	else if ((r = reg_of(addr, REG_INTE0)) >= 0) { g_inte[0][r] = data; }
	else if ((r = reg_of(addr, REG_INTE1)) >= 0) { g_inte[1][r] = data; after_inte1_access(); }
}

/* set_w / clr_w model the RP2040's atomic set and clear register aliases. */
void set_w(UW addr, UW data)
{
	int r;

	if ((r = reg_of(addr, REG_INTE0)) >= 0) { g_inte[0][r] |= data; }
	else if ((r = reg_of(addr, REG_INTE1)) >= 0) { g_inte[1][r] |= data; after_inte1_access(); }
}
void clr_w(UW addr, UW data)
{
	int r;

	if ((r = reg_of(addr, REG_INTE0)) >= 0) { g_inte[0][r] &= ~data; }
	else if ((r = reg_of(addr, REG_INTE1)) >= 0) { g_inte[1][r] &= ~data; after_inte1_access(); }
}
ER gpio_set_pin(UINT no, UINT mode) { (void)no; (void)mode; return E_OK; }
ER tk_dly_tsk(TMO t) { (void)t; return E_OK; }
INT tk_get_prc(void) { return (INT)g_core + 1; }		/* SMP: 1 = core 0 */
ER tk_def_int(UINT intno, CONST T_DINT *p)
{
	CHECK(intno == INTNO_BANK0);
	g_def_int_calls++;
	g_handler = p->inthdr;
	return E_OK;
}
void EnableInt(UINT intno, INT level)
{
	CHECK(intno == INTNO_BANK0);
	g_enable_level[g_core] = level;
}

/* Set new pin levels; every pin that changed latches its edge. */
static void set_pins(uint32_t levels)
{
	uint32_t changed = g_gpio_in ^ levels;
	unsigned pin;

	for (pin = 0u; pin < 30u; pin++) {
		if ((changed & (1u << pin)) != 0u) {
			uint32_t ev = ((levels >> pin) & 1u) ? EDGE_HIGH : EDGE_LOW;
			g_intr[pin / 8u] |= ev << (4u * (pin % 8u));
		}
	}
	g_gpio_in = levels;
}

/* The IO_BANK0 interrupt taken on one core. */
static void fire(unsigned core)
{
	unsigned saved = g_core;
	g_core = core;
	((void (*)(UINT))g_handler)(INTNO_BANK0);
	g_core = saved;
}

static void echo_isr(UINT pin, UW events, UW now_us)
{
	(void)events;
	CHECK(pin == 5u);
	g_echo_calls++;
	g_echo_now = now_us;
}

static void btn_isr(UINT pin, UW events, UW now_us)
{
	(void)now_us;
	CHECK((pin == 20u) || (pin == 21u));
	CHECK(events == EDGE_LOW);
	g_btn_calls++;
}

/* Forward order of (A, B): 00 10 11 01. */
static const uint32_t FWD_A[4] = { 0u, 1u, 1u, 0u };
static const uint32_t FWD_B[4] = { 0u, 0u, 1u, 1u };

static uint32_t with_step(uint32_t levels, unsigned pin_a, unsigned pin_b, int phase)
{
	uint32_t mask = (1u << pin_a) | (1u << pin_b);
	return (levels & ~mask) | (FWD_A[phase & 3] << pin_a) | (FWD_B[phase & 3] << pin_b);
}

static void step(unsigned pin_a, unsigned pin_b, int phase)
{
	set_pins(with_step(g_gpio_in, pin_a, pin_b, phase));
	fire(1u);
}

#ifdef TEST_ATTACH_RACE

static void motion_attach_now(void)
{
	int in_scan = g_in_scan_attach;

	g_in_scan_attach = 0;		/* motion's own accesses do not count */
	CHECK(gpio_irq_init() == E_OK);
	CHECK(encoder_init() == E_OK);
	g_motion_done = 1;
	g_in_scan_attach = in_scan;
}

int main(int argc, char **argv)
{
	/* scan_task (priority 13) attaches GP5 on core 1, and motion_task
	   (priority 11) preempts it after its k-th PROC1_INTE access to
	   attach GP0-GP3.  If the attach makes fewer than k accesses, motion
	   simply attaches afterwards. */
	g_preempt_at = (argc > 1) ? atoi(argv[1]) : 1;
	g_core = 1u;
	CHECK(gpio_irq_init() == E_OK);
	g_in_scan_attach = 1;
	CHECK(gpio_irq_attach(5u, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, echo_isr) == E_OK);
	g_in_scan_attach = 0;
	if (!g_motion_done) {
		motion_attach_now();
	}
	CHECK(0 == g_masked);
	CHECK(((g_inte[1][0] >> 20) & 0xFu) == 0xCu);		/* ECHO enabled */
	if ((g_inte[1][0] & 0xFFFFu) != 0xCCCCu) {
		printf("attach race (preempted after INTE access %d): motion's "
		       "encoder enables were lost (PROC1_INTE0 = 0x%08x)\n",
		       g_preempt_at, (unsigned)g_inte[1][0]);
	}
	CHECK((g_inte[1][0] & 0xFFFFu) == 0xCCCCu);

	printf("encoder_dispatch attach race, preempt at %d: %d checks, %d failures\n",
	       g_preempt_at, g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

#else

int main(void)
{
	uint32_t enc_mask;
	uint32_t n0, t0;
	uint16_t c0;
	int i;

#if TM_CONSOLE_USB_CDC
	enc_mask = 0xCCCCu;			/* GP0..GP3, both edges */
#else
	enc_mask = 0xCC00u;			/* GP2, GP3; GP0/GP1 are the UART */
#endif

	/* 1. Motion on core 1 (motion_init: gpio_irq_init, then encoder_init). */
	g_core = 1u;
	CHECK(gpio_irq_init() == E_OK);
	CHECK(encoder_init() == E_OK);
	CHECK(g_def_int_calls == 1);
	CHECK(g_enable_level[1] == 2);
	CHECK((g_inte[1][0] & 0xFFFFu) == enc_mask);
	CHECK(g_inte[0][0] == 0u && g_inte[0][2] == 0u);

	/* 2. scan_task on core 1: init again, then GP5 both edges. */
	CHECK(gpio_irq_init() == E_OK);
	CHECK(g_def_int_calls == 1);				/* no second handler */
	CHECK(gpio_irq_attach(5u, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, echo_isr) == E_OK);
	CHECK(((g_inte[1][0] >> 20) & 0xFu) == 0xCu);
	CHECK((g_inte[1][0] & 0xFFFFu) == enc_mask);		/* encoder kept */

	/* 3. nav_ctrl_task on core 0: GP20/GP21 falling edges. */
	g_core = 0u;
	CHECK(gpio_irq_init() == E_OK);
	CHECK(g_def_int_calls == 1);
	CHECK(g_enable_level[0] == 2);
	CHECK(gpio_irq_attach(20u, GPIO_IRQ_EDGE_FALL, btn_isr) == E_OK);
	CHECK(gpio_irq_attach(21u, GPIO_IRQ_EDGE_FALL, btn_isr) == E_OK);
	CHECK(g_inte[0][2] == 0x00440000u);
	CHECK(g_inte[1][2] == 0u);
	g_core = 1u;

	/* 4. Quadrature through the real handler: left +10, right -6. */
	g_gpio_in |= (1u << 20) | (1u << 21);			/* buttons released */
	for (i = 1; i <= 10; i++) step(MOTION_PIN_ENC_L_A, MOTION_PIN_ENC_L_B, i);
	for (i = 1; i <= 6; i++) step(MOTION_PIN_ENC_R_A, MOTION_PIN_ENC_R_B, 4 - (i % 4));
#if TM_CONSOLE_USB_CDC
	CHECK(motion_unpack_left(encoder_read_packed()) == 10u);
	CHECK(encoder_irq_count() == 16u);
	CHECK((g_intr[0] & 0xFFFFu) == 0u);			/* all edges cleared */
#else
	CHECK(motion_unpack_left(encoder_read_packed()) == 0u);
	CHECK(encoder_irq_count() == 6u);
	CHECK((g_intr[0] & 0x00FFu) != 0u);		/* GP0/GP1 not ours: left latched */
	CHECK((g_intr[0] & 0xFF00u) == 0u);
#endif
	CHECK(motion_unpack_right(encoder_read_packed()) == (uint16_t)(0u - 6u));
	CHECK(encoder_invalid_packed() == 0u);
	CHECK(motion_unpack_left(encoder_read_edges_right()) == 3u);
	CHECK(motion_unpack_right(encoder_read_edges_right()) == 3u);

	/* 5. Encoder and ECHO edges in one interrupt: both delivered, and the
	   entry timestamp is taken before any callback. */
	n0 = encoder_irq_count();
	c0 = motion_unpack_right(encoder_read_packed());
	t0 = g_timer;
	/* The right wheel ended case 4 at phase 2; one step forward is 3. */
	set_pins(with_step(g_gpio_in, MOTION_PIN_ENC_R_A, MOTION_PIN_ENC_R_B, 3) ^ (1u << 5));
	fire(1u);
	CHECK(g_echo_calls == 1);
	CHECK(g_echo_now == t0 + 1u);			/* the first timer read */
	CHECK(encoder_irq_count() - n0 == 1u);
	CHECK(motion_unpack_right(encoder_read_packed()) != c0);	/* moved one step */
	CHECK(((g_intr[0] >> 8) & 0xFFu) == 0u);	/* GP2, GP3 cleared */
	CHECK(((g_intr[0] >> 20) & 0xFu) == 0u);	/* GP5 cleared */

	/* 6. Both right pins flip in one interrupt: one missed edge, count
	   unchanged, one interrupt, two callbacks (longest run 2 us here). */
	(void)encoder_take_irq_max_us();
	n0 = encoder_irq_count();
	c0 = motion_unpack_right(encoder_read_packed());
	set_pins(g_gpio_in ^ ((1u << MOTION_PIN_ENC_R_A) | (1u << MOTION_PIN_ENC_R_B)));
	fire(1u);
	CHECK(motion_unpack_right(encoder_invalid_packed()) == 1u);
	CHECK(motion_unpack_right(encoder_read_packed()) == c0);
	CHECK(encoder_irq_count() - n0 == 1u);

	/* 8. Longest run: entry stamp to the end of the second callback. */
	CHECK(encoder_take_irq_max_us() == 2u);
	CHECK(encoder_take_irq_max_us() == 0u);

	/* 7. A GP20 press is not core 1's: not delivered there and not
	   cleared; core 0 takes it. */
	set_pins(g_gpio_in & ~(1u << 20));
	fire(1u);
	CHECK(g_btn_calls == 0);
	CHECK(((g_intr[2] >> 16) & 0xFu) == EDGE_LOW);
	fire(0u);
	CHECK(g_btn_calls == 1);
	CHECK(((g_intr[2] >> 16) & 0xFu) == 0u);

	printf("encoder_dispatch (%s console): %d checks, %d failures\n",
	       TM_CONSOLE_USB_CDC ? "USB" : "UART", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

#endif /* TEST_ATTACH_RACE */
