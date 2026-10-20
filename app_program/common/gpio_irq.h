/*
 *  gpio_irq.h
 *  The one owner of the RP2040 GPIO bank interrupt (IO_IRQ_BANK0).
 *
 *  Owner: Buddy 2 (motion).  Agreed in the Week 6 design: "one core 1
 *  GPIO dispatcher owned by Buddy 2, agreed with Buddy 5".
 *
 *  Why a dispatcher: every GPIO edge interrupt on the chip arrives as the
 *  same interrupt number, which has one slot in the vector table, and
 *  both cores share that table.  If two tasks each call tk_def_int() for
 *  it, the second silently replaces the first and the first subsystem
 *  stops seeing edges.  So nobody calls tk_def_int(13, ...) directly:
 *  register a callback here instead.
 *
 *  Per core: each core that wants GPIO interrupts calls gpio_irq_init()
 *  once from one of its tasks, and a callback runs on the core whose task
 *  attached it.  motion_task (core 1) does this for the encoders; a core 0
 *  button handler would call gpio_irq_init() and gpio_irq_attach() from a
 *  core 0 task.
 *
 *  Callbacks run in interrupt context: keep them short (read pins, store
 *  a timestamp, tk_set_flg) and never block.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Edge selection for gpio_irq_attach(). */
#define GPIO_IRQ_EDGE_FALL (0x4u)
#define GPIO_IRQ_EDGE_RISE (0x8u)
#define GPIO_IRQ_EDGE_BOTH (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

#define GPIO_IRQ_MAX_CLIENTS (4u) /* per core */

/* pins: bit n set = GPn had an enabled edge (only this client's pins).
   gpio_in: all pin levels, read after the edges were acknowledged. */
typedef void (*gpio_irq_cb_t)(uint32_t pins, uint32_t gpio_in);

/* Take the GPIO interrupt on the calling core.  level is the NVIC
   priority (INTPRI_MAX_EXTINT_PRI is the highest allowed).  Once per
   core, from a task.  E_OBJ if this core already did it. */
ER gpio_irq_init(INT level);

/* Deliver edges on the pins in pin_mask (bit n = GPn) to p_callback, on the
   calling core.  Each pin can belong to one client only (E_OBJ
   otherwise).  Call from task context at start-up, after gpio_irq_init()
   on the same core, with the pins already set up as inputs. */
ER gpio_irq_attach(uint32_t pin_mask, uint32_t edges, gpio_irq_cb_t p_callback);

/* Number of times the interrupt has run on the calling core. */
uint32_t gpio_irq_count(void);

/* Longest single run of the handler on the calling core since the last
   call, in us (1 us resolution, so a 2-3 us handler reads as 2..4). */
uint32_t gpio_irq_take_max_us(void);

#endif /* GPIO_IRQ_H */
