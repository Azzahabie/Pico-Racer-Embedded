/*
 *	ultrasonic.c
 *	HC-SR04 driver, uT-Kernel port.
 *
 *	IRQ version (US_USE_IRQ = 1, the design):
 *	  ECHO_ISR (us_echo_isr) runs from the shared GPIO dispatcher. It only
 *	  stores the edge timestamps and, on the falling edge, wakes the task
 *	  that is waiting in us_ping(). The task sleeps meanwhile, so other
 *	  tasks run, and a task cutting in can no longer stretch the reading
 *	  (the problem seen with polling: up to 38 mm spread vs 9 mm bench).
 *
 *	Lab links:
 *	  Lab 3 - interrupt instead of a blocking wait; volatile shared data;
 *	          no printf and no maths in the ISR.
 *	  Lab 5 - deferred interrupt handling (ISR wakes the task).
 *	  Lab 6 - real 10 us delay; integer maths only (no FPU, no divide).
 *
 *	Barr-C (safety subset): braces on every body, constant on the left of
 *	==/!=, lowercase _t type names, names of 3+ characters, g_ globals,
 *	_isr suffix, bool from stdbool.h. Tabs/brace placement follow the repo.
 */

#include <stdbool.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>

#include "ultrasonic.h"
#if US_USE_IRQ
#include "../common/gpio_irq.h"
#endif

/* 1 MHz free-running hardware timer, low 32 bits. Unsigned subtraction
 * stays correct across the ~71 min wrap. Needs commit 993ebc7 or later
 * (Buddy 1's timer fix) or the timer runs 43x slow after power-on. */
LOCAL UW now_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

LOCAL void wait_us(UW delay_us)
{
	UW start_us = now_us();

	while((now_us() - start_us) < delay_us) {
		;	/* hardware register read: cannot be optimised away */
	}
}

LOCAL UW mm_from_width(UW width_us)
{
	/* mm = width / 5.8 = width * 10/58; 10/58 * 65536 = 11299.
	 * Max 38000 * 11299 = 429 million: fits in 32 bits. */
	return (width_us * 11299U) >> 16;
}

#if US_USE_IRQ
/* ---- shared between ECHO_ISR and the task (32-bit: no torn reads) ---- */
LOCAL volatile UW	g_rise_us;
LOCAL volatile UW	g_fall_us;
LOCAL volatile bool	g_rise_seen;
LOCAL volatile bool	g_fall_seen;
LOCAL volatile ID	g_waiter;		/* task sleeping in us_ping() */
LOCAL ID		g_echo_flgid = 0;	/* scan_flg, 0 = use wake-up  */
LOCAL UINT		g_echo_bit = 0U;

/* ECHO_ISR: timestamps only. t_us was read first by the dispatcher. */
LOCAL void us_echo_isr(UINT pin, UW events, UW t_us)
{
	(void)pin;

	if(0U != (events & GPIO_IRQ_EDGE_RISE)) {
		g_rise_us = t_us;
		g_rise_seen = true;
	}
	if((0U != (events & GPIO_IRQ_EDGE_FALL)) && g_rise_seen) {
		g_fall_us = t_us;
		g_fall_seen = true;
		if(0 < g_echo_flgid) {
			(void)tk_set_flg(g_echo_flgid, g_echo_bit);	/* design */
		} else if(0 < g_waiter) {
			(void)tk_wup_tsk(g_waiter);
		} else {
			/* nobody waiting */
		}
	}
}
#endif

EXPORT void us_init(void)
{
	UW pad;

	gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
	gpio_set_val(US_TRIG_PIN, 0);

	gpio_set_pin(US_ECHO_PIN, GPIO_MODE_IN);

	/* libbsp has no pull-resistor call: pull-down on, pull-up off,
	 * input enabled. ECHO reads 0 if the wire falls out -> US_NO_ECHO. */
	pad = in_w(GPIO(US_ECHO_PIN));
	pad &= ~GPIO_PUE;
	pad |= (GPIO_PDE | GPIO_IE);
	out_w(GPIO(US_ECHO_PIN), pad);

#if US_USE_IRQ
	g_waiter = tk_get_tid();
	(void)gpio_irq_init();		/* this core (core 1 in the design) */
	(void)gpio_irq_attach(US_ECHO_PIN,
			      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
			      us_echo_isr);
#endif
}

EXPORT void us_use_flag(ID flgid, UINT echo_bit)
{
#if US_USE_IRQ
	g_echo_bit = echo_bit;
	g_echo_flgid = flgid;		/* set last: the ISR checks the ID */
#else
	(void)flgid;
	(void)echo_bit;
#endif
}

EXPORT us_status_t us_ping(UW *p_mm, UW *p_width_us)
{
	UW width_us;

#if US_USE_IRQ
	UINT flgptn;

	g_rise_seen = false;
	g_fall_seen = false;
	if(0 < g_echo_flgid) {
		(void)tk_clr_flg(g_echo_flgid, ~g_echo_bit);	/* forget old ECHO */
	} else {
		(void)tk_can_wup(TSK_SELF);	/* forget any old wake-up */
	}

	gpio_set_val(US_TRIG_PIN, 1);
	wait_us(10);
	gpio_set_val(US_TRIG_PIN, 0);

	/* Sleep until ECHO_ISR signals the falling edge, or time out.
	 * A signal that arrives before we wait is kept, not lost. Only the
	 * ECHO bit is cleared, so a TICK arriving meanwhile stays set. */
	if(0 < g_echo_flgid) {
		(void)tk_wai_flg(g_echo_flgid, g_echo_bit, TWF_ORW | TWF_BITCLR,
				 &flgptn, US_WAIT_MS);
	} else {
		(void)tk_slp_tsk(US_WAIT_MS);
	}

	if(!g_rise_seen) {
		return US_NO_ECHO;		/* never answered */
	}
	if(!g_fall_seen) {
		return US_OUT_OF_RANGE;		/* still high */
	}
	width_us = g_fall_us - g_rise_us;
#else
	UW start_us;
	UW rise_us;

	gpio_set_val(US_TRIG_PIN, 1);
	wait_us(10);
	gpio_set_val(US_TRIG_PIN, 0);

	start_us = now_us();
	while(0U == gpio_get_val(US_ECHO_PIN)) {
		if((now_us() - start_us) > US_RISE_TIMEOUT_US) {
			return US_NO_ECHO;
		}
	}
	rise_us = now_us();
	while(0U != gpio_get_val(US_ECHO_PIN)) {
		if((now_us() - rise_us) > US_FALL_TIMEOUT_US) {
			return US_OUT_OF_RANGE;
		}
	}
	width_us = now_us() - rise_us;
#endif

	*p_width_us = width_us;
	if(width_us >= US_FAR_WIDTH_US) {
		return US_OUT_OF_RANGE;
	}

	*p_mm = mm_from_width(width_us);
	return US_OK;
}
