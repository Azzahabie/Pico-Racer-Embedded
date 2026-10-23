/*
 *  encoder.h
 *  Both wheel encoders: quadrature, x4 decoding, GPIO edge interrupts.
 *
 *  Counts are published as one packed 32-bit word (left in bits 15:0,
 *  right in bits 31:16), written only by the GPIO interrupt with a single
 *  store.  Any task on either core reads both counts with one load, so
 *  there is no lock and no torn pair.  Use motion_unpack_left/right() and
 *  motion_count_delta() from motion_lib.h.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* Set GP0..GP3 to inputs with pull-ups and attach them, one pin at a time,
   to Buddy 5's shared GPIO dispatcher (common/gpio_irq.h; gpio_irq_init()
   must already have run on this core, core 1).
   On a UART-console build GP0/GP1 carry the console, so the left encoder
   is left off and only the right one runs (encoder_left_enabled() says
   which).  The approved pin map uses the USB console for this reason. */
ER encoder_init(void);

uint32_t encoder_read_packed(void);    /* one load, both counts       */
uint32_t encoder_invalid_packed(void); /* missed-edge counts, same packing */
bool     encoder_left_enabled(void);

/* Edges seen on each phase of one wheel: phase A in bits 15:0, phase B
   in bits 31:16, both wrapping (16-bit).  Take the change since the last
   read with (uint16_t)(now - prev) on each half.  The stall check uses
   them to tell a dead encoder wire (edges on one phase only) from a
   stopped wheel (no edges).  Written only by the interrupt, one store. */
uint32_t encoder_read_edges_left(void);
uint32_t encoder_read_edges_right(void);

/* Timing evidence: interrupts that reached the encoder callback (wrapping
   count), and the longest dispatcher-entry-to-callback-end time in us
   since the last take (a lower bound: it misses the exception entry).
   Call the take only from motion_task, on the encoder's core. */
uint32_t encoder_irq_count(void);
uint32_t encoder_take_irq_max_us(void);

#endif /* ENCODER_H */
