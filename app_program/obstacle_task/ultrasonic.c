/*
 *	ultrasonic.c
 *	HC-SR04 driver, uT-Kernel port of the bench-test driver.
 *
 *	Uses only what this repo already provides:
 *	  - gpio_set_pin / gpio_set_val / gpio_get_val   (bsp/libbsp.h)
 *	  - in_w / out_w and the RP2040 register names     (tk/syslib.h, sysdef.h)
 *	The Pico SDK is NOT used here (the app build does not link it).
 *
 *	Lab links:
 *	  Lab 3 - timeouts so a missing echo can never hang the task.
 *	  Lab 6 - real microsecond delay from the hardware timer, not an empty
 *	          loop the compiler may delete; integer maths only (no FPU).
 */

#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>

#include "ultrasonic.h"

/* 1 MHz free-running hardware timer, low 32 bits. Wraps every ~71 min;
 * unsigned subtraction (now - start) stays correct across the wrap. */
LOCAL UW now_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

LOCAL void wait_us(UW us)
{
	UW start = now_us();
	while((now_us() - start) < us) {
		;	/* reads a hardware register each pass: cannot be optimised away */
	}
}

EXPORT void us_init(void)
{
	UW pad;

	gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
	gpio_set_val(US_TRIG_PIN, 0);

	gpio_set_pin(US_ECHO_PIN, GPIO_MODE_IN);

	/* libbsp has no pull-resistor call, so set the pad register directly:
	 * pull-down on, pull-up off, input enabled. ECHO then reads 0 if the
	 * wire falls out, instead of floating. */
	pad = in_w(GPIO(US_ECHO_PIN));
	pad &= ~GPIO_PUE;
	pad |= (GPIO_PDE | GPIO_IE);
	out_w(GPIO(US_ECHO_PIN), pad);
}

EXPORT US_STATUS us_ping(UW *p_mm, UW *p_width_us)
{
	UW t0, rise, fall, width;

	/* 10 us trigger pulse */
	gpio_set_val(US_TRIG_PIN, 1);
	wait_us(10);
	gpio_set_val(US_TRIG_PIN, 0);

	/* Wait for ECHO to go high */
	t0 = now_us();
	while(gpio_get_val(US_ECHO_PIN) == 0) {
		if((now_us() - t0) > US_RISE_TIMEOUT_US) {
			return US_NO_ECHO;		/* never answered: fault */
		}
	}
	rise = now_us();

	/* Time how long ECHO stays high */
	while(gpio_get_val(US_ECHO_PIN) != 0) {
		if((now_us() - rise) > US_FALL_TIMEOUT_US) {
			return US_FAR;			/* still high: nothing near */
		}
	}
	fall = now_us();

	width = fall - rise;
	*p_width_us = width;

	if(width >= US_FAR_WIDTH_US) {
		return US_FAR;
	}

	/* mm = width / 5.8 = width * 10/58.  10/58 * 65536 = 11299, so
	 * multiply then shift instead of dividing (no divide instruction).
	 * Max 38000 * 11299 = 429 million: fits in 32 bits. */
	*p_mm = (width * 11299U) >> 16;
	return US_OK;
}
