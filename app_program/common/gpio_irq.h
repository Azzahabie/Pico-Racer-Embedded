/*
 *	gpio_irq.h
 *	One shared GPIO interrupt dispatcher (IO_IRQ_BANK0).
 *
 *	The RP2040 has ONE interrupt line for all GPIO edges per core. If two
 *	drivers each install a handler, the last one wins and the other driver
 *	silently stops working. This dispatcher owns the line and calls a
 *	per-pin callback instead.
 *
 *	Design (Week 6 + Tribunal 1, shared-resource row "GPIO interrupt
 *	callback on core 1"): one dispatcher on core 1 for ENC_ISR (GP0-GP3,
 *	Buddy 2) and ECHO_ISR (GP5, Buddy 5); the timer is read FIRST so a
 *	burst on another pin cannot delay the timestamp. BTN_ISR stays on
 *	core 0. Owner: Buddy 2 (motion side), agreed with Buddy 5.
 *
 *	Call gpio_irq_init() and gpio_irq_attach() FROM A TASK RUNNING ON THE
 *	CORE THAT SHOULD HANDLE THE INTERRUPT (core 1 for ENC/ECHO).
 *
 *	Barr-C (safety subset): braces on every body, constant on the left of
 *	==/!=, lowercase _t type names, names of 3+ characters, g_ globals,
 *	_isr suffix, bool from stdbool.h. Tabs/brace placement follow the repo.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <tk/tkernel.h>

#define GPIO_IRQ_EDGE_FALL	0x4U	/* RP2040 event bits, per pin */
#define GPIO_IRQ_EDGE_RISE	0x8U

/* now_us = microsecond timer, read once at entry, before any callback. */
typedef void (*gpio_isr_t)(UINT pin, UW events, UW now_us);

IMPORT ER gpio_irq_init(void);					/* once per core */
IMPORT ER gpio_irq_attach(UINT pin, UW edges, gpio_isr_t p_isr);	/* per pin */

#endif /* GPIO_IRQ_H */
