/*
 *	test_encoder.c
 *	Host test for app_program/motor_task/encoder.c: pin set-up, the per-pin
 *	edge callback decoding GP0..GP3 into the packed counts, the timing
 *	counters, and the UART-build rule that leaves the left encoder off.
 *	The fake dispatcher below calls back like Buddy 5's gpio_irq.c: one
 *	call per attached pin that changed, lowest pin first, all with the same
 *	entry timestamp.   make -C tests/encoder
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/motor_task/encoder.c"

static int g_checks, g_failures;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failures++; \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

/* The steps below turn each wheel in its encoder's own (uninverted)
   sense; the configured INVERT flag flips the sign of the count. */
#define L_SIGN	((MOTION_ENC_L_INVERT != 0) ? -1 : 1)
#define R_SIGN	((MOTION_ENC_R_INVERT != 0) ? -1 : 1)

#define NUM_PINS	30u
#define RUN_US		3u	/* fake callback run time */

static uint32_t		g_gpio_in;
static uint32_t		g_attached_mask;
static gpio_isr_t	g_isr[NUM_PINS];
static UW		g_edges[NUM_PINS];
static int		g_attach_calls;
static int		g_attach_errors;
static UW		g_now_us;
static uint32_t		g_dispatches;	/* interrupts that reached an encoder pin */
static uint32_t		g_pue, g_pde, g_inputs;

UW in_w(UW addr)
{
	UW v = 0u;
	if (addr == GPIO_IN) { v = g_gpio_in; }
	else if (addr == TIMER_TIMERAWL) { v = g_now_us + RUN_US; }
	return v;
}
void out_w(UW a, UW d) { (void)a; (void)d; }
void set_w(UW a, UW d) { if (d & GPIO_PUE) g_pue |= 1u << ((a - GPIO(0)) / 4u); }
void clr_w(UW a, UW d) { if (d & GPIO_PDE) g_pde |= 1u << ((a - GPIO(0)) / 4u); }
ER gpio_set_pin(UINT no, UINT mode) { if (mode == GPIO_MODE_IN) g_inputs |= 1u << no; return E_OK; }
ER tk_dly_tsk(TMO t) { (void)t; return E_OK; }

/* Buddy 5's prototype: one pin per call. */
ER gpio_irq_attach(UINT pin, UW edges, gpio_isr_t p_isr)
{
	g_attach_calls++;
	if ((pin >= NUM_PINS) || (p_isr == NULL) || (g_isr[pin] != NULL)) {
		g_attach_errors++;
		return E_PAR;
	}
	g_isr[pin] = p_isr;
	g_edges[pin] = edges;
	g_attached_mask |= 1u << pin;
	return E_OK;
}

/* One interrupt: call back every attached pin that changed since old. */
static void dispatch(uint32_t old)
{
	uint32_t changed = (old ^ g_gpio_in) & g_attached_mask;
	UINT pin;

	g_now_us += 100u;
	if (changed != 0u) {
		g_dispatches++;
	}
	for (pin = 0u; pin < NUM_PINS; pin++) {
		if ((changed & (1u << pin)) != 0u) {
			UW ev = ((g_gpio_in >> pin) & 1u) ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
			g_isr[pin](pin, ev, g_now_us);
		}
	}
}

/* Forward order of (A, B): 00 10 11 01. */
static const uint32_t FWD_A[4] = { 0u, 1u, 1u, 0u };
static const uint32_t FWD_B[4] = { 0u, 0u, 1u, 1u };

static void step(uint32_t pin_a, uint32_t pin_b, int phase)
{
	uint32_t mask = (1u << pin_a) | (1u << pin_b);
	uint32_t old = g_gpio_in;
	g_gpio_in = (g_gpio_in & ~mask) | (FWD_A[phase & 3] << pin_a) | (FWD_B[phase & 3] << pin_b);
	dispatch(old);
}

/* Toggle one pin only (a live phase while the other wire is unplugged). */
static void toggle(uint32_t pin)
{
	uint32_t old = g_gpio_in;
	g_gpio_in ^= 1u << pin;
	dispatch(old);
}

int main(void)
{
	uint32_t p;
	uint32_t enc_pins;
	UINT pin;
	int i;

	g_gpio_in = 0u;
	CHECK(encoder_init() == E_OK);
	CHECK(g_attach_errors == 0);
#if TM_CONSOLE_USB_CDC
	CHECK(encoder_left_enabled());
	CHECK(g_attach_calls == 4);
	CHECK(g_attached_mask == 0xFu);
	CHECK(g_inputs == 0xFu && g_pue == 0xFu && g_pde == 0xFu);
	enc_pins = 0xFu;
#else
	CHECK(!encoder_left_enabled());
	CHECK(g_attach_calls == 2);
	CHECK(g_attached_mask == 0xCu);		/* GP0/GP1 stay the console */
	CHECK(g_inputs == 0xCu && g_pue == 0xCu);
	enc_pins = 0xCu;
#endif
	for (pin = 0u; pin < NUM_PINS; pin++) {
		if ((enc_pins & (1u << pin)) != 0u) {
			CHECK(g_edges[pin] == (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE));
			CHECK(g_isr[pin] == encoder_edge_isr);
		}
	}
	CHECK(encoder_read_packed() == 0u);
	CHECK(encoder_irq_count() == 0u);

	/* Left forward 10 steps, right backward 6 steps. */
	for (i = 1; i <= 10; i++) step(MOTION_PIN_ENC_L_A, MOTION_PIN_ENC_L_B, i);
	for (i = 1; i <= 6; i++) step(MOTION_PIN_ENC_R_A, MOTION_PIN_ENC_R_B, 4 - (i % 4));
	p = encoder_read_packed();
#if TM_CONSOLE_USB_CDC
	CHECK(motion_unpack_left(p) == (uint16_t)(L_SIGN * 10));
	CHECK(encoder_irq_count() == 16u);
#else
	CHECK(motion_unpack_left(p) == 0u);
	CHECK(encoder_irq_count() == 6u);	/* left pins never interrupt */
#endif
	CHECK(encoder_irq_count() == g_dispatches);
	CHECK(motion_unpack_right(p) == (uint16_t)(R_SIGN * -6));
	CHECK(encoder_invalid_packed() == 0u);

	/* Longest run: timer read at the end of the callback minus now_us. */
	CHECK(encoder_take_irq_max_us() == RUN_US);
	CHECK(encoder_take_irq_max_us() == 0u);

	/* Per-phase edges: each quadrature step moves exactly one phase. */
#if TM_CONSOLE_USB_CDC
	CHECK(motion_unpack_left(encoder_read_edges_left()) == 5u);	/* A */
	CHECK(motion_unpack_right(encoder_read_edges_left()) == 5u);	/* B */
#else
	CHECK(encoder_read_edges_left() == 0u);		/* left is off */
#endif
	CHECK(motion_unpack_left(encoder_read_edges_right()) == 3u);
	CHECK(motion_unpack_right(encoder_read_edges_right()) == 3u);

#if TM_CONSOLE_USB_CDC
	/* Left B wire unplugged: only A toggles.  Edges on left A only, the
	   left count goes nowhere, and the right wheel is untouched. */
	{
		uint32_t e0 = encoder_read_edges_left();
		uint32_t r0 = encoder_read_edges_right();
		uint16_t c0 = motion_unpack_left(encoder_read_packed());
		for (i = 0; i < 20; i++) toggle(MOTION_PIN_ENC_L_A);
		CHECK((uint16_t)(motion_unpack_left(encoder_read_edges_left())
				 - motion_unpack_left(e0)) == 20u);
		CHECK((uint16_t)(motion_unpack_right(encoder_read_edges_left())
				 - motion_unpack_right(e0)) == 0u);
		CHECK(motion_unpack_left(encoder_read_packed()) == c0);
		CHECK(encoder_read_edges_right() == r0);
		CHECK(encoder_invalid_packed() == 0u);
	}
#endif

	/* Right B wire unplugged: only A toggles.  Edges on A only, and the
	   count just steps back and forth (no net movement). */
	{
		uint32_t e0 = encoder_read_edges_right();
		uint16_t c0 = motion_unpack_right(encoder_read_packed());
		uint32_t n0 = encoder_irq_count();
		for (i = 0; i < 20; i++) toggle(MOTION_PIN_ENC_R_A);
		CHECK((uint16_t)(motion_unpack_left(encoder_read_edges_right())
				 - motion_unpack_left(e0)) == 20u);
		CHECK((uint16_t)(motion_unpack_right(encoder_read_edges_right())
				 - motion_unpack_right(e0)) == 0u);
		CHECK(motion_unpack_right(encoder_read_packed()) == c0);
		CHECK(encoder_invalid_packed() == 0u);
		CHECK(encoder_irq_count() - n0 == 20u);
	}

	/* Both right channels flip in one interrupt: two callbacks, one missed
	   edge, count unchanged, one more edge on each phase, one interrupt. */
	{
		uint32_t e0 = encoder_read_edges_right();
		uint32_t n0 = encoder_irq_count();
		uint32_t old = g_gpio_in;
		g_gpio_in ^= (1u << MOTION_PIN_ENC_R_A) | (1u << MOTION_PIN_ENC_R_B);
		dispatch(old);
		CHECK(motion_unpack_right(encoder_invalid_packed()) == 1u);
		CHECK(motion_unpack_right(encoder_read_packed()) == (uint16_t)(R_SIGN * -6));
		CHECK((uint16_t)(motion_unpack_left(encoder_read_edges_right())
				 - motion_unpack_left(e0)) == 1u);
		CHECK((uint16_t)(motion_unpack_right(encoder_read_edges_right())
				 - motion_unpack_right(e0)) == 1u);
		CHECK(encoder_irq_count() - n0 == 1u);
	}

	printf("encoder (%s console): %d checks, %d failures\n",
	       TM_CONSOLE_USB_CDC ? "USB" : "UART", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
