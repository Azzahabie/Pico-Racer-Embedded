/*
 *	gpio_irq.c
 *	Shared GPIO interrupt dispatcher. See gpio_irq.h.
 *
 *	RP2040 registers (datasheet 2.19.6), IO_BANK0 at 0x40014000:
 *	  INTR0..3        0x0F0  raw status; write 1 to clear an EDGE bit
 *	  PROC0_INTE0..3  0x100  enable, core 0
 *	  PROC0_INTS0..3  0x120  status, core 0
 *	  PROC1_INTE0..3  0x130  enable, core 1
 *	  PROC1_INTS0..3  0x150  status, core 1
 *	Each register covers 8 pins, 4 bits per pin:
 *	  bit 0 level low, bit 1 level high, bit 2 edge low, bit 3 edge high.
 *	IO_IRQ_BANK0 is interrupt number 13.
 */

#include <stdbool.h>
#include <tk/tkernel.h>		/* also defines NULL (tk/typedef.h); do not add
				   <stddef.h>: its size_t clashes with tk/syslib.h */
#include <tk/syslib.h>
#include <sys/sysdef.h>

#include "gpio_irq.h"

#define INTNO_IO_BANK0		13
#define GPIO_IRQ_LEVEL		2	/* below INTPRI_MAX_EXTINT_PRI (1), above SysTick (3) */

#define IOB_INTR(reg)		(IO_BANK0_BASE + 0x0F0 + (4 * (reg)))
#define IOB_INTE(core, reg)	(IO_BANK0_BASE + ((core) ? 0x130 : 0x100) + (4 * (reg)))
#define IOB_INTS(core, reg)	(IO_BANK0_BASE + ((core) ? 0x150 : 0x120) + (4 * (reg)))

#define NUM_PINS		30
#define PINS_PER_REG		8
#define NUM_REGS		4

LOCAL gpio_isr_t	g_pin_isr[NUM_PINS];
LOCAL bool		g_is_defined = false;

/* 0 = core 0, 1 = core 1. tk_get_prc() counts from 1 in SMP builds. */
LOCAL UINT this_core(void)
{
#if TK_SUPPORT_SMP
	return (2 == tk_get_prc()) ? 1U : 0U;
#else
	return 0U;
#endif
}

/* The one IO_IRQ_BANK0 handler: timestamps first, then per-pin callbacks. */
LOCAL void gpio_bank0_isr(UINT intno)
{
	UW	now_us = in_w(TIMER_TIMERAWL);	/* FIRST: before any work */
	UINT	core = this_core();
	UINT	reg_idx;
	UINT	pin_idx;
	UINT	pin;
	UW	status;
	UW	events;

	(void)intno;

	for(reg_idx = 0; reg_idx < NUM_REGS; reg_idx++) {
		status = in_w(IOB_INTS(core, reg_idx));

		for(pin_idx = 0; (0U != status) && (pin_idx < PINS_PER_REG); pin_idx++) {
			events = (status >> (4 * pin_idx)) & 0xFU;
			if(0U != events) {
				pin = (reg_idx * PINS_PER_REG) + pin_idx;

				/* Clear edge bits (level bits clear themselves). */
				out_w(IOB_INTR(reg_idx), (events & 0xCU) << (4 * pin_idx));

				if((pin < NUM_PINS) && (NULL != g_pin_isr[pin])) {
					g_pin_isr[pin](pin, events, now_us);
				}
			}
		}
	}
}

EXPORT ER gpio_irq_init(void)
{
	T_DINT	dint;
	ER	ercd;

	if(!g_is_defined) {
		dint.intatr = TA_HLNG;
		dint.inthdr = (FP)gpio_bank0_isr;
		ercd = tk_def_int(INTNO_IO_BANK0, &dint);
		if(E_OK > ercd) {
			return ercd;
		}
		g_is_defined = true;
	}
	EnableInt(INTNO_IO_BANK0, GPIO_IRQ_LEVEL);	/* this core's NVIC */
	return E_OK;
}

EXPORT ER gpio_irq_attach(UINT pin, UW edges, gpio_isr_t p_isr)
{
	UINT	core = this_core();
	UINT	reg_idx = pin / PINS_PER_REG;
	UINT	shift = 4 * (pin % PINS_PER_REG);

	if((pin >= NUM_PINS) || (NULL == p_isr)) {
		return E_PAR;
	}

	g_pin_isr[pin] = p_isr;
	out_w(IOB_INTR(reg_idx), 0xCU << shift);		/* drop stale edges */

	/* Two single writes to the atomic clear and set aliases, not a
	 * read-modify-write: another task attaching a pin in the same
	 * register on this core can preempt us between any two lines
	 * without losing its enables (or ours). */
	clr_w(IOB_INTE(core, reg_idx), 0xFU << shift);
	set_w(IOB_INTE(core, reg_idx), (edges & 0xCU) << shift);
	return E_OK;
}
