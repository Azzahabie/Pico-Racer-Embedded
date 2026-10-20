/*
 *	test_motor.c
 *	Host test for app_program/motor_task/motor.c against fake PWM and
 *	reset registers: 10 kHz set-up, the CC layout (A low half, B high
 *	half), brake, clamping and inversion.   make -C tests/motor
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/motor_task/motor.c"

static int g_checks, g_failures;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failures++; \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

#define SLICE_REGS	(5u)
static uint32_t g_pwm[8][SLICE_REGS];	/* CSR DIV CTR CC TOP per slice */
static uint32_t g_reset = 0xFFFFFFFFu;	/* everything held in reset */
static uint32_t g_pwm_pins;
static uint32_t g_cc_writes;

static uint32_t *fake_pwm_reg(UW addr)
{
	uint32_t off;

	if (addr < PWM_BASE || addr >= PWM_BASE + 8u * 0x14u) return NULL;
	off = addr - PWM_BASE;
	return &g_pwm[off / 0x14u][(off % 0x14u) / 4u];
}

UW in_w(UW addr)
{
	uint32_t *p = fake_pwm_reg(addr);
	if (addr == RESETS_RESET_DONE) return ~g_reset;
	return (p != NULL) ? *p : 0u;
}
void out_w(UW a, UW d)
{
	uint32_t *p = fake_pwm_reg(a);
	if (p != NULL) {
		if ((a - PWM_BASE) % 0x14u == PWM_CHx_CC) g_cc_writes++;
		*p = d;
	}
}
void set_w(UW a, UW d) { uint32_t *p = fake_pwm_reg(a); if (p != NULL) *p |= d; }
void clr_w(UW a, UW d) { if (a == RESETS_RESET) g_reset &= ~d; }
ER pwm_set_pin(UINT no) { g_pwm_pins |= 1u << no; return E_OK; }

#define CC(slice)	(g_pwm[slice][3])

int main(void)
{
	uint32_t before;

	/* PWM writes before the reset is released would be lost on hardware:
	   motor_init must release it. */
	g_pwm[4][3] = 0xDEADu;
	CHECK(motor_init() == E_OK);
	CHECK((g_reset & RESETS_RESET_PWM) == 0u);
	CHECK(g_pwm_pins == 0xF00u);			/* GP8..GP11 */

	/* Slices 4 and 5: 125 MHz / 1 / 12500 = 10 kHz, braked, enabled. */
	CHECK(g_pwm[4][1] == 0x10u && g_pwm[5][1] == 0x10u);	/* DIV = 1.0 */
	CHECK(g_pwm[4][4] == 12499u && g_pwm[5][4] == 12499u);	/* TOP */
	CHECK(CC(4) == 0u && CC(5) == 0u);
	CHECK(g_pwm[4][0] == PWM_CH_CSR_EN && g_pwm[5][0] == PWM_CH_CSR_EN);
	CHECK(g_pwm[6][0] == 0u);			/* servo slice untouched */

	/* Forward = A (low half), reverse = B (high half), 0 = brake. */
	motor_set(MOTOR_LEFT, 350);
	CHECK(CC(4) == 4375u);
	motor_set(MOTOR_LEFT, -350);
	CHECK(CC(4) == (4375u << 16));
	motor_set(MOTOR_RIGHT, 1000);
	CHECK(CC(5) == 12500u);			/* > TOP: always high */
	motor_set(MOTOR_RIGHT, -2000);			/* clamped */
	CHECK(CC(5) == (12500u << 16));
	CHECK(motor_get_duty(MOTOR_RIGHT) == -1000);
	motor_set(MOTOR_LEFT, 0);
	CHECK(CC(4) == 0u);

	/* One store per update, so both inputs change together. */
	before = g_cc_writes;
	motor_set(MOTOR_LEFT, 500);
	CHECK(g_cc_writes == before + 1u);

	/* Inversion swaps A and B, and get_duty reports the request. */
	g_motor[MOTOR_LEFT].b_invert = true;
	motor_set(MOTOR_LEFT, 200);
	CHECK(CC(4) == (2500u << 16));
	CHECK(motor_get_duty(MOTOR_LEFT) == 200);

	motor_brake_all();
	CHECK(CC(4) == 0u && CC(5) == 0u);
	CHECK(motor_get_duty(MOTOR_LEFT) == 0 && motor_get_duty(MOTOR_RIGHT) == 0);

	/* Out-of-range side is ignored. */
	motor_set(MOTOR_COUNT, 500);
	CHECK(CC(4) == 0u && CC(5) == 0u);

	printf("motor: %d checks, %d failures\n", g_checks, g_failures);
	return (g_failures == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
