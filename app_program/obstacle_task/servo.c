/*
 *	servo.c
 *	SG90 on GP12 using the RP2040 hardware PWM, uT-Kernel port.
 *
 *	Uses the repo's BSP calls (pwm_set_pin / pwm_set_wrap / pwm_set_cc /
 *	pwm_set_enabled). The BSP has no clock-divider call, so the divider
 *	register is written directly.
 *
 *	Lab 4 maths (system clock is 125 MHz in this port, cpu_clock.c):
 *	  divider 125   -> 125 MHz / 125 = 1 MHz, so 1 count = 1 us
 *	  wrap  19999   -> 20 000 counts = 20 ms = 50 Hz
 *	  level 1500    -> 1.5 ms pulse = centre
 *	Slice 6 is the servo's own slice, so 50 Hz here cannot change the
 *	motors' PWM frequency (slices 4 and 5).
 *
 *	Barr-C (safety subset): braces on every body, constant on the left of
 *	==/!=, names of 3+ characters. TRUE is the kernel BOOL used by the BSP
 *	call pwm_set_enabled(). Tabs/brace placement follow the repo.
 */

#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>

#include "servo.h"

#define SERVO_SLICE		6U
#define SERVO_DIV_REG		(PWM_BASE + PWM_CHx_DIV + (SERVO_SLICE * 0x14))
#define SERVO_DIV_INT		125U	/* DIV register: integer part in bits 11:4 */

EXPORT void servo_init(void)
{
	/* The kernel only takes the PWM block out of reset when USE_PTMR is set
	 * (kernel/sysdepend/pico_rp2040/hw_setting.c), and this repo does not
	 * set it. Release it here. Clear only -- never pulse the reset -- so a
	 * PWM another task already configured (e.g. the motors) is not wiped. */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while(0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) {
		;
	}

	pwm_set_pin(SERVO_PIN);
	out_w(SERVO_DIV_REG, SERVO_DIV_INT << 4);	/* 1 MHz count, no fraction */
	pwm_set_wrap(SERVO_PIN, 19999);			/* 50 Hz frame */
	pwm_set_cc(SERVO_PIN, SERVO_CENTRE_US + SERVO_TRIM_US);	/* start at centre */
	pwm_set_enabled(SERVO_PIN, TRUE);
}

EXPORT UW servo_set_angle(INT angle_deg)
{
	INT pulse_us;

	if(angle_deg > SERVO_LIMIT_DEG) {
		angle_deg = SERVO_LIMIT_DEG;
	}
	if(angle_deg < -SERVO_LIMIT_DEG) {
		angle_deg = -SERVO_LIMIT_DEG;
	}

	/* Negative angle = car's LEFT, positive = car's RIGHT (after SERVO_DIR). */
	pulse_us = SERVO_CENTRE_US + SERVO_TRIM_US
		 + (SERVO_DIR * angle_deg * SERVO_US_PER_DEG);

	if(pulse_us < SERVO_MIN_US) {
		pulse_us = SERVO_MIN_US;
	}
	if(pulse_us > SERVO_MAX_US) {
		pulse_us = SERVO_MAX_US;
	}

	pwm_set_cc(SERVO_PIN, (UW)pulse_us);
	return (UW)pulse_us;
}
