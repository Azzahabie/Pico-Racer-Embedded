/*
 *	test_encoder.c
 *	Host test for app_program/motor_task/encoder.c: pin set-up, the edge
 *	callback decoding GP0..GP3 into the packed counts, and the UART-build
 *	rule that leaves the left encoder off.   make -C tests/encoder
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/motor_task/encoder.c"

static int g_checks, g_failures;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failures++; \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

static uint32_t		g_gpio_in;
static uint32_t		g_attached_mask;
static uint32_t		g_attached_edges;
static gpio_irq_cb_t	g_cb;
static uint32_t		g_pue, g_pde, g_inputs;

UW in_w(UW addr) { return (addr == GPIO_IN) ? g_gpio_in : 0u; }
void out_w(UW a, UW d) { (void)a; (void)d; }
void set_w(UW a, UW d) { if (d & GPIO_PUE) g_pue |= 1u << ((a - GPIO(0)) / 4u); }
void clr_w(UW a, UW d) { if (d & GPIO_PDE) g_pde |= 1u << ((a - GPIO(0)) / 4u); }
ER gpio_set_pin(UINT no, UINT mode) { if (mode == GPIO_MODE_IN) g_inputs |= 1u << no; return E_OK; }
ER tk_dly_tsk(TMO t) { (void)t; return E_OK; }
ER gpio_irq_attach(uint32_t m, uint32_t e, gpio_irq_cb_t cb)
{
	g_attached_mask = m; g_attached_edges = e; g_cb = cb;
	return E_OK;
}

/* Forward order of (A, B): 00 10 11 01. */
static const uint32_t FWD_A[4] = { 0u, 1u, 1u, 0u };
static const uint32_t FWD_B[4] = { 0u, 0u, 1u, 1u };

static void step(uint32_t pin_a, uint32_t pin_b, int phase)
{
	uint32_t keep = g_gpio_in & ~((1u << pin_a) | (1u << pin_b));
	g_gpio_in = keep | (FWD_A[phase & 3] << pin_a) | (FWD_B[phase & 3] << pin_b);
	g_cb((1u << pin_a) | (1u << pin_b), g_gpio_in);
}

int main(void)
{
	uint32_t p;
	int i;

	g_gpio_in = 0u;
	CHECK(encoder_init() == E_OK);
#if TM_CONSOLE_USB_CDC
	CHECK(encoder_left_enabled());
	CHECK(g_attached_mask == 0xFu);
	CHECK(g_inputs == 0xFu && g_pue == 0xFu && g_pde == 0xFu);
#else
	CHECK(!encoder_left_enabled());
	CHECK(g_attached_mask == 0xCu);		/* GP0/GP1 stay the console */
	CHECK(g_inputs == 0xCu && g_pue == 0xCu);
#endif
	CHECK(g_attached_edges == GPIO_IRQ_EDGE_BOTH);
	CHECK(encoder_read_packed() == 0u);

	/* Left forward 10 steps, right backward 6 steps. */
	for (i = 1; i <= 10; i++) step(MOTION_PIN_ENC_L_A, MOTION_PIN_ENC_L_B, i);
	for (i = 1; i <= 6; i++) step(MOTION_PIN_ENC_R_A, MOTION_PIN_ENC_R_B, 4 - (i % 4));
	p = encoder_read_packed();
#if TM_CONSOLE_USB_CDC
	CHECK(motion_unpack_left(p) == 10u);
#else
	CHECK(motion_unpack_left(p) == 0u);
#endif
	CHECK(motion_unpack_right(p) == (uint16_t)(0u - 6u));
	CHECK(encoder_invalid_packed() == 0u);

	/* Both right channels flip at once: one missed edge, count unchanged. */
	g_gpio_in ^= (1u << MOTION_PIN_ENC_R_A) | (1u << MOTION_PIN_ENC_R_B);
	g_cb(0xCu, g_gpio_in);
	CHECK(motion_unpack_right(encoder_invalid_packed()) == 1u);
	CHECK(motion_unpack_right(encoder_read_packed()) == (uint16_t)(0u - 6u));

	printf("encoder (%s console): %d checks, %d failures\n",
	       TM_CONSOLE_USB_CDC ? "USB" : "UART", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
